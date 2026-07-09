#ifndef _FOC_TYPE_H
#define _FOC_TYPE_H

/**
 * @file foc_type.h
 * @brief FOC 基础数据类型
 */

#include <stdint.h>
#include <stdbool.h>

/** 三相 abc */
typedef struct { float a, b, c; } abc_t;

/** 静止坐标系 αβ */
typedef struct { float alpha, beta; } alphabeta_t;

/** 旋转坐标系 qd（q=力矩轴, d=励磁轴） */
typedef struct { float q, d; } qd_t;

/** 编码器 / 速度状态 */
typedef struct
{
    float    ElAngle;
    float    PhaseShift;
    float    MecAngle;
    float    AvrMecSpeed;
    float    Max_MecSpeed;
    uint32_t pole_pairs;
} SPEED_t;

/** FOC 运行时变量 */
typedef struct
{
    qd_t          U_qd;
    alphabeta_t   U_alphabeta;
    SPEED_t       speed;
} FOC_VAR_t;

#endif /* _FOC_TYPE_H */
