/**
 * @file foc_motor.c
 * @brief 电机状态机 + FOC 快环 + PhaseShift 校准（底层框架）
 *
 * TIM2 快环：读编码器 → 逆 Park + SVPWM
 * TIM4 慢环：FOC_Motor_SlowLoop() 由应用层重写
 */

#include "foc_motor.h"
#include "foc_config.h"
#include "foc_gimbal.h"
#include "foc_hw.h"
#include "foc_math.h"
#include "foc_uart.h"

#include <stdio.h>
#include <string.h>

/* ---- 内部辅助 ---- */

/** 机械角误差（deg），结果归一化到 (-180, 180]；仅 Vd-lock well 检查用 */
#if FOC_CALIB_MEC_REF_ENABLE
static float Motor_MecDegErr(float mec_rad, float ref_deg)
{
    float mec_deg = mec_rad * 360.0f / _2PI;
    float err     = mec_deg - ref_deg;

    while (err > 180.0f)  { err -= 360.0f; }
    while (err < -180.0f) { err += 360.0f; }
    return err;
}
#endif

/** PhaseShift 校准失败时递增重试计数并复位 tick */
static bool Motor_CalibRetry(FOC_MOTOR_t *motor)
{
    motor->calib_retry++;
    motor->calib_tick = 0u;
    return (motor->calib_retry < FOC_CALIB_MAX_RETRY);
}

/** 切换电机状态 */
static void Motor_NextState(FOC_MOTOR_t *motor, MOTOR_State_t s)
{
    motor->motor_state = s;
}

/** 清零 Uqd、速度 PID 积分，关 PWM 与驱动使能 */
static void Motor_Clear(FOC_MOTOR_t *motor)
{
    motor->foc_var.U_qd.q = 0.0f;
    motor->foc_var.U_qd.d = 0.0f;
    PID_Clear(&motor->pid_speed);
    FOC_PWM_StopALL(&motor->pwm);
    FOC_HW_DriverEnable(false);
}

/* ---- 对外接口 ---- */

/** 复位电机运行时状态（不清硬件） */
void FOC_Motor_Init(FOC_MOTOR_t *motor)
{
    motor->motor_state = MOTOR_IDLE;
    motor->calib_tick  = 0;
    motor->calib_retry = 0;
}

/** 停止电机：关 PWM、驱动，进入 IDLE */
void FOC_Motor_Stop(FOC_MOTOR_t *motor)
{
    Motor_Clear(motor);
    Motor_NextState(motor, MOTOR_IDLE);
}

/** 从 READY/IDLE 进入 GIMBAL_RUN，开启 PWM 与驱动 */
void FOC_Motor_StartGimbal(FOC_MOTOR_t *motor)
{
    if ((motor->motor_state == MOTOR_READY) || (motor->motor_state == MOTOR_IDLE))
    {
        FOC_PWM_StartAll(&motor->pwm);
        FOC_HW_DriverEnable(true);
        Motor_NextState(motor, MOTOR_GIMBAL_RUN);
    }
}

/**
 * @brief FOC 快环（TIM2 ~5kHz）
 *        读 AS5048A 机械角 → 电角度 → 逆 Park + SVPWM
 */
void Motor_Gimbal_Voltage_RUN(FOC_MOTOR_t *motor)
{
    float ElAngle = FOC_AS5048A_Angle_Update(&motor->enc);

    FOC_PWM_Run(&motor->pwm, motor->foc_var.U_qd, ElAngle);
    motor->foc_var.speed = motor->enc.speed;
}

/**
 * @brief PhaseShift 校准：开环 Vd 锁定 d 轴（El=0），统计稳定段 cum_count 均值
 * @return true 校准成功；false 仍在进行中或需重试
 */
static bool Motor_Enc_PhaseCalib(FOC_MOTOR_t *motor)
{
    static int64_t  s_calib_sum;
    static uint32_t s_calib_n;
    static int32_t  s_calib_cum_min;
    static int32_t  s_calib_cum_max;
    static bool     s_settle_init;

    uint32_t hold_ticks = (uint32_t)(FOC_CALIB_HOLD_MS * FOC_PWM_FREQ_HZ / 1000u);
    uint32_t ramp_ticks = (uint32_t)(FOC_CALIB_RAMP_MS * FOC_PWM_FREQ_HZ / 1000u);
    uint32_t settle_start;
    int32_t  avg_cum;
    float    mec;
    float    mec_deg;
#if FOC_CALIB_MEC_REF_ENABLE
    float    mec_err;
#endif
    float    vd;
    float    spread_deg;
    qd_t     Uqd;
    char     buf[128];

    if (motor->calib_tick == 0u)
    {
        s_calib_sum     = 0;
        s_calib_n       = 0u;
        s_settle_init   = false;
    }

    if (ramp_ticks == 0u) { ramp_ticks = 1u; }
    if (motor->calib_tick < ramp_ticks)
    {
        vd = FOC_CALIB_VD * (float)motor->calib_tick / (float)ramp_ticks;
    }
    else
    {
        vd = FOC_CALIB_VD;
    }
    Uqd.q = 0.0f;
    Uqd.d = vd;

    FOC_AS5048A_Angle_Update(&motor->enc);
    FOC_PWM_Run(&motor->pwm, Uqd, 0.0f);

    motor->calib_tick++;

    /* 取 hold 窗口后 40% 段做 cum_count 平均 */
    settle_start = (hold_ticks * 6u) / 10u;
    if (motor->calib_tick > settle_start)
    {
        int32_t c = motor->enc.cum_count;

        if (!s_settle_init)
        {
            s_calib_cum_min = c;
            s_calib_cum_max = c;
            s_settle_init   = true;
        }
        else
        {
            if (c < s_calib_cum_min) { s_calib_cum_min = c; }
            if (c > s_calib_cum_max) { s_calib_cum_max = c; }
        }

        s_calib_sum += (int64_t)c;
        s_calib_n++;
    }

    if (motor->calib_tick < hold_ticks)
    {
        return false;
    }

    spread_deg = (float)(s_calib_cum_max - s_calib_cum_min)
               / (float)AS5048A_RESOLUTION * 360.0f;

    if (spread_deg > FOC_CALIB_MAX_MEC_SPREAD_DEG)
    {
        if (Motor_CalibRetry(motor))
        {
            return false;
        }
        motor->calib_retry = 0u;
        motor->calib_tick  = 0u;
        return false;
    }

    avg_cum = (s_calib_n > 0u) ? (int32_t)(s_calib_sum / (int64_t)s_calib_n)
                               : motor->enc.cum_count;
    avg_cum = avg_cum % (int32_t)AS5048A_RESOLUTION;
    if (avg_cum < 0) { avg_cum += (int32_t)AS5048A_RESOLUTION; }
    mec     = (float)avg_cum / (float)AS5048A_RESOLUTION * _2PI;
    mec_deg = mec * 360.0f / _2PI;

#if (FOC_CALIB_MEC_REF_ENABLE)
    mec_err = Motor_MecDegErr(mec, FOC_CALIB_MEC_REF_DEG);
    if ((mec_err < -FOC_CALIB_MEC_TOLERANCE_DEG)
        || (mec_err > FOC_CALIB_MEC_TOLERANCE_DEG))
    {
        if (Motor_CalibRetry(motor))
        {
            return false;
        }
        motor->calib_retry = 0u;
        motor->calib_tick  = 0u;
        return false;
    }
#endif

    motor->enc.calib_mec_rad = mec;
    motor->enc.speed.PhaseShift = Limit_Angle(-mec * (float)motor->pole_pairs);
    motor->enc.speed.ElAngle    = Limit_Angle(motor->enc.speed.MecAngle
                                              * (float)motor->pole_pairs
                                              + motor->enc.speed.PhaseShift);
    motor->calib_tick      = 0;
    motor->calib_retry     = 0;
    motor->foc_var.U_qd.q  = 0.0f;
    motor->foc_var.U_qd.d  = 0.0f;

    (void)snprintf(buf, sizeof(buf),
                   "[CALIB] Vd-lock  PhaseShift=%.2f deg  mec=%.2f deg  spread=%.2f deg\r\n",
                   motor->enc.speed.PhaseShift * 360.0f / _2PI,
                   mec_deg,
                   spread_deg);
    FOC_UART_Write((const uint8_t *)buf, (uint16_t)strlen(buf));
    return true;
}

/**
 * @brief 电机主状态机（TIM2 快环入口 FOC_Motor_Run 调用）
 */
void FOC_Motor_Run(FOC_MOTOR_t *motor)
{
    switch (motor->motor_state)
    {
        case MOTOR_IDLE:
            Motor_Clear(motor);
            break;

        case MOTOR_ENC_CALIB:
            if (motor->calib_tick == 0u)
            {
                FOC_PWM_StartAll(&motor->pwm);
                FOC_HW_DriverEnable(true);
            }
            if (Motor_Enc_PhaseCalib(motor))
            {
                FOC_HW_DriverEnable(false);
                FOC_PWM_StopALL(&motor->pwm);
                Motor_NextState(motor, MOTOR_READY);
            }
            break;

        case MOTOR_READY:
            break;

        case MOTOR_GIMBAL_RUN:
            Motor_Gimbal_Voltage_RUN(motor);
            break;

        case MOTOR_FAULT:
            Motor_Clear(motor);
            break;

        default:
            break;
    }
}

/**
 * @brief 慢环入口（TIM4 1kHz）：测速 + 速度环
 */
void FOC_Motor_SlowLoop(FOC_MOTOR_t *motor)
{
    if (motor->motor_state != MOTOR_GIMBAL_RUN)
    {
        return;
    }

    FOC_AS5048A_Speed_Calc(&motor->enc, 1.0f / (float)FOC_SPEED_LOOP_HZ);
    FOC_Gimbal_SpeedLoop(motor);
}
