/**
 * @file foc_gimbal.c
 * @brief 云台应用层 — 速度环（电压模式 + Iq 前馈）
 *
 * 控制链（1 kHz 慢环）：
 *   ω_ref ──→ 速度 PI ──→ Iq_ref（限幅 ±MOTOR_IQ_MAX_A）
 *                              ↓
 *              Uq = R·Iq_ref + ω·Ke   （Ud = 0，无电流环）
 *                              ↓
 *              斜率限制 → 写入 motor->foc_var.U_qd.q
 *
 * 快环（5 kHz）读 ElAngle 并 SVPWM，Uqd 由本模块每 ms 更新。
 */

#include "foc_gimbal.h"
#include "foc_config.h"
#include "foc_pid.h"
#include "foc_math.h"

#include <math.h>

/** 斜率限制后的实际 Uq（用于调试打印） */
static float    s_vq_applied;
/** 上一拍 PI 输出的 Iq_ref（A） */
static float    s_iq_ref;
/** 本模块内部预热计数（ms）；为 0 表示不额外预热 */
static uint16_t s_loop_warmup_ms;

/* -------------------------------------------------------------------------- */

/** 机械转速 RPM → 机械角速度 rad/s */
static float Gimbal_RpmToRad(float rpm)
{
    return rpm * _2PI / 60.0f;
}

/**
 * @brief 电阻 + 反电势前馈：Uq = R·Iq + ω·Ke
 * @param iq_ref  速度 PI 输出的期望力矩电流 (A)
 * @param rpm_fb  反馈转速 (RPM)，用于 ω·Ke 项
 */
static float Gimbal_FeedforwardVq(float iq_ref, float rpm_fb)
{
    float omega = Gimbal_RpmToRad(rpm_fb);

    return MOTOR_R_OHM * iq_ref + omega * MOTOR_KE_VS_RAD;
}

/**
 * @brief Uq 斜率限制，避免力矩阶跃引起 overshoot
 * @param vq_target  本拍目标电压
 */
static float Gimbal_SlewVq(float vq_target)
{
    float dv = vq_target - s_vq_applied;

    if (dv > FOC_SPEED_VQ_SLEW_V)
    {
        vq_target = s_vq_applied + FOC_SPEED_VQ_SLEW_V;
    }
    else if (dv < -FOC_SPEED_VQ_SLEW_V)
    {
        vq_target = s_vq_applied - FOC_SPEED_VQ_SLEW_V;
    }

    s_vq_applied = vq_target;
    return vq_target;
}

/** 输出零力矩并复位斜率状态 */
static void Gimbal_OutputZero(FOC_MOTOR_t *motor)
{
    motor->foc_var.U_qd.q = 0.0f;
    motor->foc_var.U_qd.d = 0.0f;
    s_vq_applied          = 0.0f;
    s_iq_ref              = 0.0f;
}

/* -------------------------------------------------------------------------- */

/** 初始化云台应用状态；PID 系数在 FOC_Init / SpeedArm 配置 */
void FOC_Gimbal_Init(FOC_MOTOR_t *motor)
{
    motor->target_speed   = 0.0f;
    motor->foc_var.U_qd.q = 0.0f;
    motor->foc_var.U_qd.d = 0.0f;
    s_vq_applied          = 0.0f;
    s_iq_ref              = 0.0f;
    s_loop_warmup_ms      = 0u;
}

/**
 * @brief 速度环启动前调用：清 PID 积分、Uq 斜率、Iq 状态
 *        测试层在 RUN 预热 500 ms 结束后调用一次即可
 */
void FOC_Gimbal_SpeedArm(FOC_MOTOR_t *motor)
{
    PID_Clear(&motor->pid_speed);
    s_vq_applied     = 0.0f;
    s_iq_ref         = 0.0f;
    s_loop_warmup_ms = FOC_GIMBAL_SPEED_WARMUP_MS;
    Gimbal_OutputZero(motor);
}

/** 调试：上一拍 Iq_ref (A) */
float FOC_Gimbal_GetIqRef(void)
{
    return s_iq_ref;
}

/** 调试：斜率限制后的 Uq (V) */
float FOC_Gimbal_GetUqApplied(void)
{
    return s_vq_applied;
}

/** 上一拍 L1：Iq 输出被 PI 限幅 */
uint8_t FOC_Gimbal_GetPiSatIqFrom(FOC_MOTOR_t *motor)
{
    return motor->pid_speed.sat_iq;
}

/** 上一拍 L2：Vq/斜率饱和触发积分回算 */
uint8_t FOC_Gimbal_GetPiSatVqFrom(FOC_MOTOR_t *motor)
{
    return motor->pid_speed.sat_vq;
}

/**
 * @brief 速度环主体（TIM4 1 kHz，在 Speed_Calc 之后调用）
 *
 * 1. 测速无效或预热中 → Uq=0
 * 2. 速度 PI：误差 = target_speed - AvrMecSpeed → Iq_ref
 * 3. 前馈 Uq = R·Iq + ω·Ke
 * 4. 限幅 + 斜率 → 写入 U_qd
 *
 * 积分抗饱和（两级）：
 *   L1 foc_pid：Iq 输出顶到 ±MOTOR_IQ_MAX_A 时回算 SumError
 *   L2 本文件：Vq 顶到 VQ_MAX 或被斜率限制时，按 (Vq_ap-ωKe)/R 反算 Iq_eff 再回算
 */
void FOC_Gimbal_SpeedLoop(FOC_MOTOR_t *motor)
{
    float rpm_fb;
    float vq;
    float vq_ff;
    float vq_tgt;
    float iq_eff;

    /* 测速尚未 valid：保持零力矩（避免 SPI/滤波未就绪时乱转） */
    if (!motor->enc.speed_valid)
    {
        Gimbal_OutputZero(motor);
        PID_Clear(&motor->pid_speed);
        return;
    }

    /* 可选额外预热（默认 0 ms；测试层已做 500 ms 时可不再设） */
    if (s_loop_warmup_ms > 0u)
    {
        s_loop_warmup_ms--;
        Gimbal_OutputZero(motor);
        return;
    }

    rpm_fb = motor->enc.speed.AvrMecSpeed;

    /* 目标为 0：清积分，零力矩 */
    if (motor->target_speed == 0.0f)
    {
        PID_Clear(&motor->pid_speed);
        Gimbal_OutputZero(motor);
        return;
    }

    /* 速度 PI → Iq_ref；L1 抗饱和在 PID_Position_ctrl 内 */
    motor->pid_speed.SetPoint = motor->target_speed;
    s_iq_ref = PID_Position_ctrl(&motor->pid_speed, rpm_fb);

    /* 前馈 + Vq 限幅 + 斜率 */
    vq_ff  = Gimbal_FeedforwardVq(s_iq_ref, rpm_fb);
    vq_tgt = LIMIT_RANGE(vq_ff, -VQ_MAX, VQ_MAX);
    vq     = Gimbal_SlewVq(vq_tgt);

    /*
     * L2 抗饱和：Uq 送不到前馈要求（母线限幅或斜率）时，电压环等效饱和，
     * 反算有效 Iq 使积分不再继续增大。
     *   iq_eff ≈ (Vq_applied - ω·Ke) / R
     */
    if (fabsf(vq - vq_ff) > FOC_SPEED_AW_VQ_THRESH_V)
    {
        float omega = Gimbal_RpmToRad(rpm_fb);

        iq_eff = (vq - omega * MOTOR_KE_VS_RAD) / MOTOR_R_OHM;
        iq_eff = LIMIT_RANGE(iq_eff, -MOTOR_IQ_MAX_A, MOTOR_IQ_MAX_A);
        PID_IntegratorBackCalc(&motor->pid_speed, iq_eff);
        s_iq_ref = motor->pid_speed.ActualValue;
    }

    motor->foc_var.U_qd.q = vq;
    motor->foc_var.U_qd.d = 0.0f;
    motor->foc_var.speed  = motor->enc.speed;
}
