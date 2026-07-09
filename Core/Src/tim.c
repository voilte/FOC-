/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file    tim.c

  * @brief   TIM2: 3-phase PWM 10kHz center-aligned (PA0/PA1/PA2)

  *          TIM4: 1kHz speed-loop tick (no pin output)

  ******************************************************************************

  */

/* USER CODE END Header */

#include "tim.h"



void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);



TIM_HandleTypeDef htim2;

TIM_HandleTypeDef htim4;



/* TIM2: 72MHz / (2*3600) = 10kHz, center-aligned mode 1 */

void MX_TIM2_Init(void)

{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  TIM_OC_InitTypeDef sConfigOC = {0};



  htim2.Instance = TIM2;

  htim2.Init.Prescaler = 0;

  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;

  htim2.Init.Period = FOC_PWM_TIM_ARR;

  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)

  {

    Error_Handler();

  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)

  {

    Error_Handler();

  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)

  {

    Error_Handler();

  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)

  {

    Error_Handler();

  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;

  sConfigOC.Pulse = 0;

  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;

  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)

  {

    Error_Handler();

  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)

  {

    Error_Handler();

  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)

  {

    Error_Handler();

  }

  HAL_TIM_MspPostInit(&htim2);

}



/* TIM4: 72MHz / 72 / 1000 = 1kHz (speed loop tick, no pin output) */

void MX_TIM4_Init(void)

{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  TIM_MasterConfigTypeDef sMasterConfig = {0};



  htim4.Instance = TIM4;

  htim4.Init.Prescaler = 71;

  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim4.Init.Period = 999;

  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)

  {

    Error_Handler();

  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)

  {

    Error_Handler();

  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)

  {

    Error_Handler();

  }

}

