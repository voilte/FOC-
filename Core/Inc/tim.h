/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file    tim.h

  * @brief   This file contains all the function prototypes for

  *          the tim.c file

  ******************************************************************************

  */

/* USER CODE END Header */

#ifndef __TIM_H__

#define __TIM_H__



#ifdef __cplusplus

extern "C" {

#endif



#include "main.h"



extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim4;



/* Center-aligned TIM2 PWM: f = 72MHz / (2*(ARR+1)) = 10kHz */

#define FOC_PWM_TIM_ARR     3599u

#define FOC_PWM_PERIOD      (2u * (FOC_PWM_TIM_ARR + 1u))



/* Legacy names used in foc_pwm.c */

#define TIM4_ARR            FOC_PWM_TIM_ARR



void MX_TIM2_Init(void);

void MX_TIM4_Init(void);



#ifdef __cplusplus

}

#endif



#endif /* __TIM_H__ */


