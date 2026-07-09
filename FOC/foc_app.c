/**
 * @file foc_app.c
 * @brief FOC 系统初始化与中断入口
 */

#include "foc_app.h"
#include "foc_config.h"
#include "foc_gimbal.h"
#include "foc_hw.h"
#include "foc_pid.h"
#include "foc_as5048a.h"
#include "foc_pwm.h"
#include "foc_motor.h"
#include "tim.h"

/** 全局电机实例 */
FOC_MOTOR_t FOC_MOTOR = {
    .pole_pairs   = MOTOR_POLE_PAIRS,
    .power        = MOTOR_BUS_VOLTAGE,
    .target_speed = 0.0f,
    .pid_speed = {
        .P                   = FOC_SPEED_PI_P,
        .I                   = FOC_SPEED_PI_I,
        .D                   = 0.0f,
        .SetPoint_limit      = FOC_SPEED_MAX_RPM,
        .ActualValue_limit   = MOTOR_IQ_MAX_A,
        .SumError_limit      = (MOTOR_IQ_MAX_A / (FOC_SPEED_PI_I * (1.0f / (float)FOC_SPEED_LOOP_HZ)) * 2.0f),
        .change_limit        = MOTOR_IQ_MAX_A,
    },
};

/**
 * @brief 系统初始化
 *
 * OTP 方案 B（FOC_OTP_ZERO_BURNED=1）：
 *   自检 → PhaseShift=0 → MOTOR_READY（无 Vd-lock、无 Flash）
 * 仅 OTP 未烧且未模拟时才会进 MOTOR_ENC_CALIB。
 */
void FOC_Init(void)
{
    PID_Init(&FOC_MOTOR.pid_speed, (float)FOC_SPEED_LOOP_HZ);
    FOC_AS5048A_Init(&FOC_MOTOR.enc, FOC_MOTOR.pole_pairs);
    FOC_PWM_Init(&FOC_MOTOR.pwm, FOC_PWM_PERIOD, FOC_MOTOR.power);
    FOC_Gimbal_Init(&FOC_MOTOR);
    FOC_Motor_Init(&FOC_MOTOR);

    FOC_MOTOR.pole_pairs = MOTOR_POLE_PAIRS;
    FOC_MOTOR.power      = MOTOR_BUS_VOLTAGE;

    if (FOC_AS5048A_SelfTest(&FOC_MOTOR.enc))
    {
        FOC_MOTOR.enc.speed.PhaseShift = 0.0f;

        if (FOC_AS5048A_OtpSkipBootCalib())
        {
            FOC_MOTOR.motor_state = MOTOR_READY;
        }
        else
        {
            FOC_MOTOR.motor_state = MOTOR_ENC_CALIB;
        }
    }
    else
    {
        FOC_MOTOR.motor_state = MOTOR_FAULT;
    }

    FOC_PWM_HW_Init();
}

/** TIM2 快环回调入口 */
void FOC_App_Run(void)
{
    FOC_Motor_Run(&FOC_MOTOR);
}
