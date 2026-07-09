#ifndef _FOC_MOTOR_H
#define _FOC_MOTOR_H

/**
 * @file foc_motor.h
 * @brief 电机对象、状态机与 FOC 快/慢环入口
 */

#include <stdint.h>
#include <stdbool.h>

#include "foc_pwm.h"
#include "foc_type.h"
#include "foc_pid.h"
#include "foc_as5048a.h"

/** 电机运行状态 */
typedef enum
{
    MOTOR_IDLE = 0,         /**< 空闲，PWM 关闭 */
    MOTOR_ENC_CALIB,        /**< PhaseShift 校准中 */
    MOTOR_READY,            /**< 校准完成，等待启动 */
    MOTOR_GIMBAL_RUN,       /**< 正常运行（FOC 快环） */
    MOTOR_FAULT,            /**< 故障锁死 */
} MOTOR_State_t;

/** 单轴电机 FOC 对象 */
typedef struct
{
    MOTOR_State_t   motor_state;
    uint32_t        pole_pairs;
    float           power;          /**< 母线电压 (V) */

    FOC_AS5048A_t   enc;
    FOC_PID_t       pid_speed;      /**< 速度环 PID（应用层配置） */
    float           target_speed;   /**< 速度给定 (RPM)，应用层使用 */

    uint32_t        calib_tick;
    uint8_t         calib_retry;

    FOC_PWM_t       pwm;
    FOC_VAR_t       foc_var;
} FOC_MOTOR_t;

void FOC_Motor_Init(FOC_MOTOR_t *motor);
void FOC_Motor_Run(FOC_MOTOR_t *motor);            /**< 快环 TIM2 */
void FOC_Motor_SlowLoop(FOC_MOTOR_t *motor);       /**< 慢环 TIM4，应用层待重写 */
void FOC_Motor_StartGimbal(FOC_MOTOR_t *motor);
void FOC_Motor_Stop(FOC_MOTOR_t *motor);

#endif /* _FOC_MOTOR_H */
