
#ifndef __PID_H
#define __PID_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 位置式 / 增量式 PID 状态
 *
 * 速度环用法：ActualValue_limit = MOTOR_IQ_MAX_A（PI 输出 Iq_ref，不是 Vq）
 * SumError 为误差累加和；Ui = I * SumError * Ts
 */
typedef struct
{
    float ratio;
    float ratio_limit;

    volatile float  SetPoint;
    volatile float  SetPoint_limit;
    volatile float  In;
    volatile float  ActualValue;
    volatile float  ActualValue_limit;   /* PI 输出限幅（速度环=Iq_max） */
    volatile float  SumError;            /* 积分状态（误差累加） */
    volatile float  SumError_limit;
    volatile float  Up;
    volatile float  Ui;
    volatile float  Ud;
    volatile float  P;
    volatile float  I;
    volatile float  D;
    volatile float  Error;
    volatile float  LastError;
    volatile float  PrevError;
    volatile float  IngMin;
    volatile float  IngMax;
    volatile float  OutMin;
    volatile float  OutMax;
    volatile float  Ts;
    volatile float  change_limit;
    volatile bool   flag;

    /** 抗饱和：1=本拍 PI 输出被 ActualValue_limit 钳位 */
    volatile uint8_t sat_iq;
    /** 抗饱和：1=本拍由 Vq/斜率回算触发了积分回退 */
    volatile uint8_t sat_vq;

} FOC_PID_t;

typedef struct
{
    float point;
    float P;
    float I;
    float D;
    float steady_coefficient;
    float ratio_limit;
} FOC_PID_PARAMETER_t;

void PID_Init(FOC_PID_t *foc_pid, float pid_freq);
void PID_Clear(FOC_PID_t *foc_pid);
float PID_Increment_ctrl(FOC_PID_t *foc_pid, float real_val);
float PID_Position_ctrl(FOC_PID_t *foc_pid, float real_val);

/**
 * @brief 下游饱和后的积分回算（Back-calculation）
 * @param out_sat 经 Vq 限幅/斜率后反算得到的有效 Iq_ref (A)
 */
void PID_IntegratorBackCalc(FOC_PID_t *foc_pid, float out_sat);

#endif
