/**
 ****************************************************************************************************
 * @file        foc_pid.c
 * @author      ????????-Rebron????
 * @version     V0.0
 * @date        2025-01-11
 * @brief       PID????
 * @license     MIT License
 *              Copyright (c) 2025 Reborn????
 *              ???????????????????????????????????b????????????
 ****************************************************************************************************
 */

#include "foc_pid.h"
#include "foc_config.h"
#include "foc_math.h"
#include <math.h>

/**
 * @brief PID?????
 * 
 * @param foc_pid 
 * @param pid_freq PID??????
 */
void PID_Init(FOC_PID_t *foc_pid, float pid_freq)
{
    foc_pid->ActualValue    = 0.0f;
    foc_pid->Error          = 0.0f;
    foc_pid->LastError      = 0.0f;
    foc_pid->PrevError      = 0.0f;
    foc_pid->SumError       = 0.0f;
    foc_pid->Ts             = 1.0f/pid_freq;
}

/**
 * @brief ?????PID????
 * 
 * @param PID 
 * @param real_val ?????
 * @return float PID???
 */
float PID_Increment_ctrl(FOC_PID_t *PID, float real_val)
{
    float P = PID->P;
    float I = PID->I;
    float D = PID->D;

    if (isnan(real_val) || isinf(real_val) ||
        isnan(PID->P) || isinf(PID->P) ||
        isnan(PID->I) || isinf(PID->I) ||
        isnan(PID->D) || isinf(PID->D) ||
        isnan(PID->Ts) || isinf(PID->Ts)) {
        // ??????????PID??????????????0?????????
        return 0.0;
    }

    PID->SetPoint = LIMIT_RANGE(PID->SetPoint, -PID->SetPoint_limit, PID->SetPoint_limit);

    PID->In = real_val;
    PID->Error = PID->SetPoint - real_val;                                                  /*???????*/

    PID->Up = P *(PID->Error - PID->LastError);                                             /*????????*/
    PID->Ui = I *(PID->Error)* PID->Ts;                                                     /*???????*/
    PID->Ud = D *(PID->Error -2*PID->LastError +PID->PrevError)* PID->Ts;                   /*??????*/

    float increment = PID->Up + PID->Ui + PID->Ud;
    increment = LIMIT_RANGE(increment, -PID->change_limit, PID->change_limit);              /*????????*/
    
    PID->ActualValue += increment;
    PID->PrevError = PID->LastError;
    PID->LastError = PID->Error;

    PID->ActualValue = LIMIT_RANGE(PID->ActualValue,  -PID->ActualValue_limit, PID->ActualValue_limit);/*???????*/

    if(PID->ActualValue > PID->OutMax){
        PID->OutMax = PID->ActualValue;
    }
    if(PID->ActualValue < PID->OutMin){
        PID->OutMin = PID->ActualValue;
    }

    return PID->ActualValue;
}


/**
 * @brief 位置式 PID + 积分抗饱和（第一级：Iq 限幅）
 */
float PID_Position_ctrl(FOC_PID_t *PID, float real_val)
{
    float out_unsat;
    float out_sat;

    PID->sat_iq = 0u;
    PID->sat_vq = 0u;

    if (PID->I == 0.0f)
    {
        PID->Ui       = 0.0f;
        PID->SumError = 0.0f;
    }

    if (isnan(real_val) || isinf(real_val) ||
        isnan(PID->P) || isinf(PID->P) ||
        isnan(PID->I) || isinf(PID->I) ||
        isnan(PID->D) || isinf(PID->D) ||
        isnan(PID->Ts) || isinf(PID->Ts))
    {
        return 0.0f;
    }

    if (PID->flag)
    {
        PID->SumError = 0.0f;
        PID->flag     = 0;
    }

    PID->SetPoint = LIMIT_RANGE(PID->SetPoint, -PID->SetPoint_limit, PID->SetPoint_limit);
    PID->In       = real_val;
    PID->Error    = PID->SetPoint - real_val;
    PID->Up       = PID->P * PID->Error;
    PID->Ui       = PID->I * PID->SumError * PID->Ts;

    out_unsat = PID->Up + PID->Ui;
    out_sat   = LIMIT_RANGE(out_unsat, -PID->ActualValue_limit, PID->ActualValue_limit);
    PID->ActualValue = out_sat;

    if (PID->SetPoint == 0.0f)
    {
        PID->SumError = 0.0f;
    }
    else if (PID->I > 0.0f)
    {
        if (fabsf(out_sat - out_unsat) > 1e-6f)
        {
            PID->SumError += FOC_PID_AW_GAIN * (out_sat - out_unsat)
                             / (PID->I * PID->Ts);
            PID->sat_iq    = 1u;
        }
        else
        {
            bool allow = true;

            if ((PID->Error > 0.0f) && (out_sat >= PID->ActualValue_limit - 1e-6f))
            {
                allow = false;
            }
            if ((PID->Error < 0.0f) && (out_sat <= -PID->ActualValue_limit + 1e-6f))
            {
                allow = false;
            }
            if (allow)
            {
                PID->SumError += PID->Error;
            }
        }
    }

    PID->SumError = LIMIT_RANGE(PID->SumError, -PID->SumError_limit, PID->SumError_limit);

    if (PID->ActualValue > PID->OutMax) { PID->OutMax = PID->ActualValue; }
    if (PID->ActualValue < PID->OutMin) { PID->OutMin = PID->ActualValue; }

    return PID->ActualValue;
}

/**
 * @brief 第二级抗饱和：Vq 限幅/斜率后反算有效 Iq，回灌 SumError
 */
void PID_IntegratorBackCalc(FOC_PID_t *PID, float out_sat)
{
    float out_unsat;
    float diff;

    if (PID->I <= 0.0f)
    {
        return;
    }

    out_unsat = PID->P * PID->Error + PID->I * PID->SumError * PID->Ts;
    diff      = out_sat - out_unsat;

    if (fabsf(diff) < 1e-6f)
    {
        return;
    }

    PID->SumError += FOC_PID_AW_GAIN * diff / (PID->I * PID->Ts);
    PID->SumError  = LIMIT_RANGE(PID->SumError, -PID->SumError_limit, PID->SumError_limit);

    PID->Ui          = PID->I * PID->SumError * PID->Ts;
    PID->ActualValue = LIMIT_RANGE(PID->Up + PID->Ui,
                                   -PID->ActualValue_limit,
                                   PID->ActualValue_limit);
    PID->sat_vq      = 1u;
}

/**
 * @brief PID计算参数清除
 * 
 * @param PID 
 */
void PID_Clear(FOC_PID_t *PID)
{
    PID->SumError    = 0.0f;
    PID->Up          = 0.0f;
    PID->Ui          = 0.0f;
    PID->Ud          = 0.0f;
    PID->Error       = 0.0f;
    PID->LastError   = 0.0f;
    PID->PrevError   = 0.0f;
    PID->IngMin      = 0.0f;
    PID->IngMax      = 0.0f;
    PID->OutMin      = 0.0f;
    PID->OutMax      = 0.0f;
    PID->ActualValue = 0.0f;
    PID->sat_iq      = 0u;
    PID->sat_vq      = 0u;
}



