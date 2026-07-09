/**
 ****************************************************************************************************
 * @file        foc_math.c
 * @author      哔哩哔哩-Rebron大侠
 * @version     V0.0
 * @date        2025-01-11
 * @brief       FOC的数学运算
 * @license     MIT License
 *              Copyright (c) 2025 Reborn大侠
 *              允许任何人使用、复制、修改和分发该代码，但需保留此版权声明。
 ****************************************************************************************************
 */

#include "foc_math.h"

#include <math.h>
#include <stdlib.h>

/**
 * @brief 符号判断
 * 
 * @param val 
 * @return int8_t 
 */
int8_t FOC_SIGN_Judgment(float val)
{
    if(val>=0)return 1;
    else return 0;
}


/**
 * @brief 克拉克变换
 * 
 * @param input 三相输入
 * @return alphabeta_t 返回αβ轴
 */
alphabeta_t FOC_Clarke(abc_t input)
{
    alphabeta_t output = {0};

    /* qIalpha = qIas*/
    /*qIbeta = -(2*qIbs+qIas)/sqrt(3)*/

    /*没有等幅值变换*/
    // output.alpha = 1.5f * input.a;
    // output.beta = SQRT_3 *(0.5f*input.a + input.b);

    /*等幅值变换 */
    output.alpha = input.a;
    output.beta = ((input.a + 2.0f*input.b)/SQRT_3);

    return output;
}


/**
 * @brief 帕克变换
 * 
 * @param input αβ轴输入
 * @param ElAngle 电角度
 * @return qd_t 返回qd轴
 */
qd_t Foc_Park(alphabeta_t input, float ElAngle)
{
    qd_t output = {0};

    float _cos = arm_cos_f32(ElAngle);
    float _sin = arm_sin_f32(ElAngle);

    output.q = _cos*input.beta -  _sin*input.alpha;
    output.d = _cos*input.alpha + _sin*input.beta;

    return output;
}

/**
 * @brief 帕克逆变换
 * 
 * @param input qd轴
 * @param ElAngle 电角度
 * @return alphabeta_t 返回αβ轴
 */
alphabeta_t FOC_Rev_Park(qd_t input, float ElAngle)
{
    alphabeta_t output = {0};

    float _cos = arm_cos_f32(ElAngle);
    float _sin = arm_sin_f32(ElAngle);

    /*派克逆变换
    Iα = Id*cosθ - Iq*sinθ
    Iβ = Iq*cosθ + Id*sinθ 我们取Id为0的情况*/
    output.alpha = _cos* input.d - _sin* input.q;
    output.beta  = _cos* input.q + _sin* input.d;

    return output;
}

/*克拉克逆变换*/

/**
 * @brief 克拉克逆变换
 * 
 * @param input αβ轴
 * @return abc_t 返回三相
 */
abc_t FOC_Rev_Clarke(alphabeta_t input)
{
    abc_t output = {0};
/*
    Ia = Iα;
    Ib = (√3*Iβ-Iα)/2
    Ic = (-Iα-√3*Iβ)/2 
*/
    output.a = input.alpha;
    output.b = (SQRT_3*(input.beta) -input.alpha)/2.0f;
    output.c = (-input.alpha - SQRT_3*(input.beta))/2.0f;

    return output;
}



/**
 * @brief 限制角度在0-2PI之间
 * 
 * @param ElAngle 输入角度
 * @return float 返回角度
 */
float Limit_Angle(float ElAngle)
{
    while(ElAngle>_2PI||ElAngle<0){
        if(ElAngle>_2PI){
            ElAngle -= _2PI;
        }else if(ElAngle<0){
            ElAngle += _2PI;
        }
    }

    return ElAngle;
}



/**
 * @brief 机械角度转换为电角度
 * 
 * @param MecAngle 机械角度
 * @param pole_pairs pole_pairs：极对数
 * @return float 电角度
 */
float _MecAngle_to_ElAngle(float MecAngle, uint32_t pole_pairs)
{
    return (float)(MecAngle * pole_pairs);
}


/**
 * @brief 计算电角度差
 * 
 * @param ElAngle1 角度1
 * @param ElAngle2 角度2
 * @return float  返回角度差
 */
float FOC_ERR(float ElAngle1, float ElAngle2)
{
    float err = Limit_Angle(ElAngle1 - ElAngle2 + _2PI);

    if (err > PI)           // 限制误差到 [-π, π]
        err -= _2PI;
    return err;
}




