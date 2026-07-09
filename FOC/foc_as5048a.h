#ifndef FOC_AS5048A_H
#define FOC_AS5048A_H

/**
 * @file foc_as5048a.h
 * @brief AS5048A 绝对式磁编码器（SPI）
 *
 * cum_count 为解包后的连续计数（真值源）。
 * OTP 已烧或 FOC_OTP_AZ_SIMULATE 时：angle 已对齐 d∥A，PhaseShift=0。
 */

#include <stdint.h>
#include <stdbool.h>
#include "foc_type.h"

typedef struct
{
    SPEED_t   speed;
    int32_t   cum_count;
    int32_t   speed_last_cum;     /* 慢环测速：上一拍 cum_count */
    int16_t   last_dr;
    float     last_mec_angle;
    uint16_t  raw;
    uint16_t  last_raw;
    uint8_t   speed_warmup;       /* 测速预热计数 */
    bool      angle_valid;
    bool      speed_valid;
    bool      speed_init;
    float     calib_mec_rad;
} FOC_AS5048A_t;

void    FOC_AS5048A_Init(FOC_AS5048A_t *enc, uint32_t pole_pairs);
bool    FOC_AS5048A_SelfTest(FOC_AS5048A_t *enc);
float   FOC_AS5048A_Angle_Update(FOC_AS5048A_t *enc);
void    FOC_AS5048A_Speed_Calc(FOC_AS5048A_t *enc, float Ts);
void    FOC_AS5048A_Speed_Reset(FOC_AS5048A_t *enc);

/** OTP 方案 B：已烧 OTP 或软件模拟时，上电跳过 Vd-lock，PhaseShift 固定 0 */
bool    FOC_AS5048A_OtpSkipBootCalib(void);

#endif /* FOC_AS5048A_H */
