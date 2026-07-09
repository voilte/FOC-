#ifndef __FOC_APP_H
#define __FOC_APP_H

/**
 * @file foc_app.h
 * @brief FOC 系统初始化与中断入口
 */

#include "foc_motor.h"

extern FOC_MOTOR_t FOC_MOTOR;

void FOC_Init(void);
void FOC_App_Run(void);   /**< TIM2 快环入口 */

#endif /* __FOC_APP_H */
