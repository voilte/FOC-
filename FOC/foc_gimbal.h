#ifndef FOC_GIMBAL_H
#define FOC_GIMBAL_H

/**
 * @file foc_gimbal.h
 * @brief 云台应用层：速度环（Iq_ref + 前馈 + 两级抗饱和）
 */

#include "foc_motor.h"
#include <stdint.h>

void    FOC_Gimbal_Init(FOC_MOTOR_t *motor);
void    FOC_Gimbal_SpeedArm(FOC_MOTOR_t *motor);
void    FOC_Gimbal_SpeedLoop(FOC_MOTOR_t *motor);

float   FOC_Gimbal_GetIqRef(void);
float   FOC_Gimbal_GetUqApplied(void);
uint8_t FOC_Gimbal_GetPiSatIqFrom(FOC_MOTOR_t *motor);
uint8_t FOC_Gimbal_GetPiSatVqFrom(FOC_MOTOR_t *motor);

#endif /* FOC_GIMBAL_H */
