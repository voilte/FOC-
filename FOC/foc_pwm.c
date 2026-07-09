/**
 * @file foc_pwm.c
 * @brief SVPWM：Uqd + 电角度 → 三相占空比 → TIM2 CCR
 */

#include "foc_pwm.h"
#include "foc_hw.h"
#include "foc_math.h"
#include "tim.h"

/** 将 CCR 钳位到 [0, FOC_PWM_TIM_ARR] */
static uint32_t PWM_ClampCcr(int32_t ticks)
{
    if (ticks < 0) { return 0u; }
    if ((uint32_t)ticks > FOC_PWM_TIM_ARR) { return FOC_PWM_TIM_ARR; }
    return (uint32_t)ticks;
}

/**
 * @brief 扇区判断：N = sign(A)+2*sign(B)+4*sign(C) → sector 1..6
 * @return N 编码值
 */
static uint8_t Sector_Judgment(FOC_PWM_t *foc_pwm, alphabeta_t ab, uint8_t *sector)
{
    float A, B, C;
    uint8_t N = 0;

    foc_pwm->svpwm_val1 = ab.alpha * SQRT_3_DIV_2;
    foc_pwm->svpwm_val2 = ab.beta / 2.0f;

    A = ab.beta;
    B = foc_pwm->svpwm_val1 - foc_pwm->svpwm_val2;
    C = -foc_pwm->svpwm_val1 - foc_pwm->svpwm_val2;

    if (A > 0) { N += 1; }
    if (B > 0) { N += 2; }
    if (C > 0) { N += 4; }

    switch (N)
    {
        case 3:  *sector = 1; break;
        case 1:  *sector = 2; break;
        case 5:  *sector = 3; break;
        case 4:  *sector = 4; break;
        case 6:  *sector = 5; break;
        case 2:  *sector = 6; break;
        default: *sector = 0; break;
    }
    return N;
}

/**
 * @brief 计算相邻矢量作用时间并写入 TIM2 CCR（中心对齐 SVPWM）
 * @param Tpwm  定时器周期 ticks（= FOC_PWM_PERIOD）
 * @param Udc   母线电压
 */
static void VectorActionTime(FOC_PWM_t *foc_pwm, uint8_t sector, alphabeta_t ab,
                             uint32_t Tpwm, float Udc)
{
    float K, X, Y, Z;
    int32_t t4 = 0, t6 = 0;
    int32_t t1, t2, t3, ta, tb, tc, sum, tmax;

    Udc = Udc * 1.5f;
    K = (float)Tpwm * SQRT_3 / Udc;

    X = K * ab.beta;
    Y = K * (foc_pwm->svpwm_val1 + foc_pwm->svpwm_val2);
    Z = K * (-foc_pwm->svpwm_val1 + foc_pwm->svpwm_val2);

    switch (sector)
    {
        case 1: t4 = (int32_t)Z;       t6 = (int32_t)Y;        break;
        case 2: t4 = (int32_t)Y;       t6 = (int32_t)(-X);     break;
        case 3: t4 = (int32_t)(-Z);    t6 = (int32_t)X;        break;
        case 4: t4 = (int32_t)(-X);    t6 = (int32_t)Z;        break;
        case 5: t4 = (int32_t)X;       t6 = (int32_t)(-Y);     break;
        case 6: t4 = (int32_t)(-Y);    t6 = (int32_t)(-Z);     break;
        default: t4 = 0; t6 = 0; break;
    }

    tmax = (int32_t)((float)Tpwm * foc_pwm->K);
    sum = t4 + t6;
    if (sum < 0) { sum = 0; }
    if ((sum > tmax) && (sum > 0))
    {
        t4 = (int32_t)((int64_t)t4 * (int64_t)tmax / (int64_t)sum);
        t6 = (int32_t)((int64_t)t6 * (int64_t)tmax / (int64_t)sum);
        sum = t4 + t6;
    }

    ta = ((int32_t)Tpwm - sum) / 4;
    tb = ta + t4 / 2;
    tc = tb + t6 / 2;

    switch (sector)
    {
        case 1: t1 = tb; t2 = ta; t3 = tc; break;
        case 2: t1 = ta; t2 = tc; t3 = tb; break;
        case 3: t1 = ta; t2 = tb; t3 = tc; break;
        case 4: t1 = tc; t2 = tb; t3 = ta; break;
        case 5: t1 = tc; t2 = ta; t3 = tb; break;
        case 6: t1 = tb; t2 = tc; t3 = ta; break;
        default: t1 = ta; t2 = ta; t3 = ta; break;
    }

    foc_pwm->T_abc.a = (float)t1;
    foc_pwm->T_abc.b = (float)t2;
    foc_pwm->T_abc.c = (float)t3;

    U_H_SET_PWM(PWM_ClampCcr(t1));
    V_H_SET_PWM(PWM_ClampCcr(t2));
    W_H_SET_PWM(PWM_ClampCcr(t3));
}

/** 逆 Park → αβ → 扇区 → 矢量时间 → CCR */
static void FOC_RUN_SVPWM(FOC_PWM_t *foc_pwm, qd_t U_qd, float angle_el)
{
    alphabeta_t ab;

    ab = FOC_Rev_Park(U_qd, angle_el);
    foc_pwm->alpha_beta = ab;
    (void)Sector_Judgment(foc_pwm, ab, &foc_pwm->sector);
    VectorActionTime(foc_pwm, foc_pwm->sector, ab, foc_pwm->Tpwm, foc_pwm->power);
}

/** SVPWM 主入口：保存 Uqd 并执行一次调制 */
void FOC_PWM_Run(FOC_PWM_t *foc_pwm, qd_t Uqd, float ElAngle)
{
    foc_pwm->Uqd = Uqd;
    FOC_RUN_SVPWM(foc_pwm, Uqd, ElAngle);
}

/** 初始化 PWM 结构：周期、母线电压、调制系数 K=1 */
void FOC_PWM_Init(FOC_PWM_t *foc_pwm, uint32_t Tpwm, float power)
{
    foc_pwm->Tpwm  = Tpwm;
    foc_pwm->power = power;
    foc_pwm->K     = 1.0f;
}

/** 开启三相 PWM 通道 */
void FOC_PWM_StartAll(FOC_PWM_t *foc_pwm)
{
    (void)foc_pwm;
    FOC_PWM_HW_ON_OFF(true, true, true, true, true, true);
}

/** 关闭 PWM 并将 CCR 清零 */
void FOC_PWM_StopALL(FOC_PWM_t *foc_pwm)
{
    (void)foc_pwm;
    FOC_PWM_HW_ON_OFF(false, false, false, false, false, false);
    U_H_SET_PWM(0);
    V_H_SET_PWM(0);
    W_H_SET_PWM(0);
}
