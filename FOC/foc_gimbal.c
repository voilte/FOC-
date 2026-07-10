/**
 * @file foc_gimbal.c
 * @brief 云台应用层 — 位置环 + 速度环（电压模式 + Iq 前馈）
 *
 * 控制链：
 *   Pitch 目标(deg) ──→ 位置 P @200Hz ──→ ω_ref (RPM)
 *                                              ↓
 *   ω_ref ──→ 速度 PI @1kHz ──→ Iq_ref ──→ Uq = R·Iq + ω·Ke ──→ SVPWM
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
/** 位置环输出的 ω_ref（RPM） */
static float    s_omega_ref;
/** 上一拍 Pitch 反馈（deg） */
static float    s_pitch_fb;
/** 本模块内部预热计数（ms）；为 0 表示不额外预热 */
static uint16_t s_loop_warmup_ms;
/** 位置环已挂接 */
static uint8_t  s_pos_armed;
/** 1=Pitch 误差已在死区内（允许速度环完全停转） */
static uint8_t  s_pos_in_deadband;
/** TIM4 1kHz 分频，跑位置环 */
static uint8_t  s_pos_div_cnt;

/* -------------------------------------------------------------------------- */

/** 角度 wrap 到 (-180, 180] */
static float Gimbal_WrapDeg180(float deg)
{
    while (deg > 180.0f)  { deg -= 360.0f; }
    while (deg <= -180.0f) { deg += 360.0f; }
    return deg;
}

/** mec_deg → Pitch（相对 FOC_PITCH_ZERO_DEG，wrap ±180°） */
float FOC_Gimbal_PitchDegFromEnc(const FOC_AS5048A_t *enc)
{
    float mec_deg = enc->speed.MecAngle * 360.0f / _2PI;

    while (mec_deg < 0.0f)    { mec_deg += 360.0f; }
    while (mec_deg >= 360.0f) { mec_deg -= 360.0f; }

    return Gimbal_WrapDeg180(mec_deg - FOC_PITCH_ZERO_DEG);
}

/** 机械转速 RPM → 机械角速度 rad/s */
static float Gimbal_RpmToRad(float rpm)
{
    return rpm * _2PI / 60.0f;
}

/**
 * @brief 电阻 + 反电势前馈：Uq = R·Iq + ω·Ke
 * @param iq_ref       速度 PI 输出的 Iq_ref (A)
 * @param rpm_for_ke   ω·Ke 项用的转速 (RPM)，推荐 target 防超调
 */
static float Gimbal_FeedforwardVq(float iq_ref, float rpm_for_ke)
{
    float omega = Gimbal_RpmToRad(rpm_for_ke);

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
    motor->target_pitch   = 0.0f;
    motor->foc_var.U_qd.q = 0.0f;
    motor->foc_var.U_qd.d = 0.0f;
    s_vq_applied          = 0.0f;
    s_iq_ref              = 0.0f;
    s_omega_ref           = 0.0f;
    s_pitch_fb            = 0.0f;
    s_loop_warmup_ms      = 0u;
    s_pos_armed           = 0u;
    s_pos_in_deadband     = 0u;
    s_pos_div_cnt         = 0u;
}

/**
 * @brief 退出位置环：仅速度环测试时调用
 */
void FOC_Gimbal_PosDisarm(FOC_MOTOR_t *motor)
{
    (void)motor;
    s_pos_armed       = 0u;
    s_pos_in_deadband = 0u;
    s_pos_div_cnt     = 0u;
    s_omega_ref       = 0.0f;
}

/**
 * @brief 位置环启动：挂速度环 + 清位置 PID，设定 Pitch 目标
 */
void FOC_Gimbal_PosArm(FOC_MOTOR_t *motor, float target_pitch_deg)
{
    FOC_Gimbal_SpeedArm(motor);
    PID_Clear(&motor->pid_pos);
    motor->pid_pos.P = FOC_POS_PI_P;
    motor->pid_pos.I = FOC_POS_PI_I;
    motor->pid_pos.D = FOC_POS_PI_D;
    FOC_Gimbal_SetTargetPitch(motor, target_pitch_deg);
    s_pos_armed       = 1u;
    s_pos_in_deadband = 0u;
    s_pos_div_cnt     = 0u;
    s_omega_ref   = 0.0f;
    s_pitch_fb    = FOC_Gimbal_PitchDegFromEnc(&motor->enc);
}

void FOC_Gimbal_SetTargetPitch(FOC_MOTOR_t *motor, float pitch_deg)
{
    float old_tgt = motor->target_pitch;

    motor->target_pitch = LIMIT_RANGE(pitch_deg,
                                      -FOC_PITCH_LIMIT_DEG,
                                      FOC_PITCH_LIMIT_DEG);
    motor->pid_pos.SetPoint = motor->target_pitch;

    /* 仅大阶跃清速度积分；连续斜坡每 0.5° 清一次会导致内环永远积不上 */
    if (s_pos_armed &&
        (fabsf(motor->target_pitch - old_tgt) > FOC_POS_TARGET_JUMP_DEG))
    {
        s_pos_in_deadband = 0u;
        PID_Clear(&motor->pid_speed);
    }
}

float FOC_Gimbal_GetPitchFb(void)
{
    return s_pitch_fb;
}

float FOC_Gimbal_GetOmegaRef(void)
{
    return s_omega_ref;
}

uint8_t FOC_Gimbal_IsPosArmed(void)
{
    return s_pos_armed;
}

/**
 * @brief 速度环启动前调用：清 PID 积分、Uq 斜率、Iq 状态，并退出位置环
 */
void FOC_Gimbal_SpeedArm(FOC_MOTOR_t *motor)
{
    FOC_Gimbal_PosDisarm(motor);
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

/** 调试：斜率限制后的 Uq (V)，含 FOC_MOTOR_UQ_SIGN */
float FOC_Gimbal_GetUqApplied(void)
{
    return FOC_MOTOR_UQ_SIGN * s_vq_applied;
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
 * @brief 位置环（TIM4 1 kHz）：Pitch P → ω_ref → target_speed
 *
 * 误差取最短路径 wrap；SetPoint 做等价变换使 PID 内部 Error = wrap(目标-反馈)
 */
void FOC_Gimbal_PosLoop(FOC_MOTOR_t *motor)
{
    float pitch_fb;
    float err_wrap;
    float sp_wrap;
    float omega_ref;

    if (!s_pos_armed)
    {
        return;
    }

    if (!motor->enc.angle_valid)
    {
        motor->target_speed   = 0.0f;
        s_omega_ref           = 0.0f;
        s_pos_in_deadband     = 0u;
        return;
    }

    s_pos_div_cnt++;
    if (s_pos_div_cnt < (uint8_t)FOC_POS_LOOP_DIV)
    {
        motor->target_speed = s_omega_ref;
        return;
    }
    s_pos_div_cnt = 0u;

    pitch_fb       = FOC_Gimbal_PitchDegFromEnc(&motor->enc);
    s_pitch_fb     = pitch_fb;
    err_wrap       = Gimbal_WrapDeg180(motor->target_pitch - pitch_fb);
    sp_wrap        = pitch_fb + err_wrap;
    motor->pid_pos.SetPoint = sp_wrap;

    omega_ref = PID_Position_ctrl(&motor->pid_pos, pitch_fb);
    omega_ref = LIMIT_RANGE(omega_ref,
                            -FOC_POS_OMEGA_MAX_RPM,
                            FOC_POS_OMEGA_MAX_RPM);

    /* 死区：纯 P 时直接停；有 I 时允许低速精调 */
    if (fabsf(err_wrap) < FOC_POS_ERR_DEADBAND_DEG)
    {
        s_pos_in_deadband = 1u;

        if (motor->pid_pos.I > 0.0f)
        {
            omega_ref = LIMIT_RANGE(omega_ref,
                                    -FOC_POS_OMEGA_FINE_RPM,
                                    FOC_POS_OMEGA_FINE_RPM);
            if (fabsf(omega_ref) < 0.3f)
            {
                omega_ref = 0.0f;
            }
        }
        else
        {
            omega_ref = 0.0f;
        }
    }
    else
    {
        s_pos_in_deadband = 0u;

        if ((FOC_POS_OMEGA_MIN_RPM > 0.0f) &&
            (fabsf(err_wrap) > FOC_POS_MIN_OMEGA_ERR_DEG) &&
            (fabsf(omega_ref) < FOC_POS_OMEGA_MIN_RPM))
        {
            omega_ref = (err_wrap > 0.0f) ? FOC_POS_OMEGA_MIN_RPM
                                          : -FOC_POS_OMEGA_MIN_RPM;
        }
    }

    s_omega_ref           = omega_ref;
    motor->target_speed   = omega_ref;
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

    /* 到位停力：仅 ω_ref≈0 时；死区内 I 精调仍允许非零 target_speed */
    if (s_pos_armed && s_pos_in_deadband &&
        (fabsf(rpm_fb) < FOC_POS_STOP_RPM) &&
        (fabsf(motor->target_speed) < 0.5f))
    {
        PID_Clear(&motor->pid_speed);
        Gimbal_OutputZero(motor);
        return;
    }

    /* 目标为 0 且非位置环：清积分，零力矩（恒速测试停转） */
    if ((motor->target_speed == 0.0f) && !s_pos_armed)
    {
        PID_Clear(&motor->pid_speed);
        Gimbal_OutputZero(motor);
        return;
    }

    /* 速度 PI → Iq_ref；L1 抗饱和在 PID_Position_ctrl 内 */
    motor->pid_speed.SetPoint = motor->target_speed;
    s_iq_ref = PID_Position_ctrl(&motor->pid_speed, rpm_fb);

    /* 超调退积分：仅恒速测试；位置环负速目标时 rpm=0 会误判为超调 */
    if (!s_pos_armed)
    {
        float spd_overshoot = rpm_fb - motor->target_speed;

        if ((spd_overshoot > FOC_SPEED_OVERSHOOT_RPM) &&
            (motor->pid_speed.I > 0.0f))
        {
            motor->pid_speed.SumError *= FOC_SPEED_OVERSHOOT_DECAY;
        }
        else if ((spd_overshoot < -FOC_SPEED_OVERSHOOT_RPM) &&
                 (motor->pid_speed.I < 0.0f))
        {
            motor->pid_speed.SumError *= FOC_SPEED_OVERSHOOT_DECAY;
        }
    }

    /* 前馈 + Vq 限幅 + 斜率
     * 位置环：Ke 用 rpm_fb（静止时 Ke≈0）；恒速测试仍可用 target 防超调 */
#if FOC_SPEED_FF_KE_USE_TARGET
    if (s_pos_armed)
    {
        vq_ff = Gimbal_FeedforwardVq(s_iq_ref, rpm_fb);
    }
    else
    {
        vq_ff = Gimbal_FeedforwardVq(s_iq_ref, motor->target_speed);
    }
#else
    vq_ff = Gimbal_FeedforwardVq(s_iq_ref, rpm_fb);
#endif
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

    motor->foc_var.U_qd.q = FOC_MOTOR_UQ_SIGN * vq;
    motor->foc_var.U_qd.d = 0.0f;
    motor->foc_var.speed  = motor->enc.speed;
}
