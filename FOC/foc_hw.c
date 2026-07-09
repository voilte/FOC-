/**
 * @file foc_hw.c
 * @brief FOC 硬件抽象：TIM2 三相 PWM、TIM4 慢环、驱动使能、中断分发
 */

#include "foc_hw.h"
#include "foc_app.h"
#include "main.h"

/** 启动 TIM2 PWM 与更新中断、TIM4 慢环中断，CCR 清零 */
void FOC_PWM_HW_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim2);

    HAL_TIM_Base_Start_IT(&htim4);

    U_H_SET_PWM(0);
    V_H_SET_PWM(0);
    W_H_SET_PWM(0);
}

/** 停止 TIM2/TIM4 中断与 PWM，关闭驱动 */
void FOC_PWM_HW_DeInit(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_Stop_IT(&htim4);
    FOC_HW_DriverEnable(false);
}

/**
 * @brief 三相 PWM 通道启停（L6234 仅用上桥 PWM，下桥互补由硬件处理）
 * @note  A_N/B_N/C_N 参数保留兼容接口，本板未使用
 */
void FOC_PWM_HW_ON_OFF(bool A, bool A_N, bool B, bool B_N, bool C, bool C_N)
{
    (void)A_N;
    (void)B_N;
    (void)C_N;

    if (A)
    {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }

    if (B)
    {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    }

    if (C)
    {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    }
}

/** L6234 驱动芯片使能脚（高=使能） */
void FOC_HW_DriverEnable(bool en)
{
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin,
                      en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief HAL 定时器周期回调
 *        TIM2 中心对齐：仅在向上计数半周期触发 FOC 快环（实际 ~5kHz）
 *        TIM4：1kHz 慢环
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        if (htim->Instance->CR1 & TIM_CR1_DIR)
        {
            return;
        }
        FOC_App_Run();
    }
    else if (htim->Instance == TIM4)
    {
        FOC_Motor_SlowLoop(&FOC_MOTOR);
    }
}
