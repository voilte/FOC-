#ifndef FOC_GIMBAL_H
#define FOC_GIMBAL_H

/**
 * @file foc_gimbal.h
 * @brief 云台应用层：位置环 → 速度环（Iq_ref + 前馈 + 两级抗饱和）
 */

#include "foc_motor.h"
#include <stdint.h>

void    FOC_Gimbal_Init(FOC_MOTOR_t *motor);
void    FOC_Gimbal_SpeedArm(FOC_MOTOR_t *motor);
void    FOC_Gimbal_PosArm(FOC_MOTOR_t *motor, float target_pitch_deg);
void    FOC_Gimbal_PosDisarm(FOC_MOTOR_t *motor);
void    FOC_Gimbal_SetTargetPitch(FOC_MOTOR_t *motor, float pitch_deg);
void    FOC_Gimbal_PosLoop(FOC_MOTOR_t *motor);
void    FOC_Gimbal_SpeedLoop(FOC_MOTOR_t *motor);

float   FOC_Gimbal_PitchDegFromEnc(const FOC_AS5048A_t *enc);
float   FOC_Gimbal_GetPitchFb(void);
float   FOC_Gimbal_GetOmegaRef(void);
uint8_t FOC_Gimbal_IsPosArmed(void);

float   FOC_Gimbal_GetIqRef(void);
float   FOC_Gimbal_GetUqApplied(void);
uint8_t FOC_Gimbal_GetPiSatIqFrom(FOC_MOTOR_t *motor);
uint8_t FOC_Gimbal_GetPiSatVqFrom(FOC_MOTOR_t *motor);

#endif /* FOC_GIMBAL_H */
