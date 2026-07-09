#ifndef FOC_HW_H
#define FOC_HW_H

/**
 * @file foc_hw.h
 * @brief FOC 硬件层：TIM2 三相 PWM (PA0/1/2)、驱动使能
 */

#include <stdbool.h>
#include "tim.h"

#define U_H_SET_PWM(x)       (htim2.Instance->CCR1 = (uint32_t)(x))
#define V_H_SET_PWM(x)       (htim2.Instance->CCR2 = (uint32_t)(x))
#define W_H_SET_PWM(x)       (htim2.Instance->CCR3 = (uint32_t)(x))

void FOC_PWM_HW_Init(void);
void FOC_PWM_HW_DeInit(void);
void FOC_PWM_HW_ON_OFF(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N);
void FOC_HW_DriverEnable(bool en);

#endif
