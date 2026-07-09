#ifndef _FOC_PWM_H
#define _FOC_PWM_H

/**
 * @file foc_pwm.h
 * @brief SVPWM 调制器
 */

#include <stdint.h>
#include "foc_type.h"

typedef struct
{
    float         svpwm_val1;
    float         svpwm_val2;
    float         power;
    uint32_t      Tpwm;
    uint8_t       sector;
    float         K;
    qd_t          Uqd;
    alphabeta_t   alpha_beta;
    abc_t         _output;
    abc_t         T_abc;
} FOC_PWM_t;

void FOC_PWM_Init(FOC_PWM_t *foc_pwm, uint32_t Tpwm, float power);
void FOC_PWM_Run(FOC_PWM_t *foc_pwm, qd_t Uqd, float ElAngle);
void FOC_PWM_StartAll(FOC_PWM_t *foc_pwm);
void FOC_PWM_StopALL(FOC_PWM_t *foc_pwm);

#endif /* _FOC_PWM_H */
