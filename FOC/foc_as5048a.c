/**
 * @file foc_as5048a.c
 * @brief AS5048A 磁编码器 SPI 驱动 + 角度解包（底层）
 *
 * 快环（TIM2）：FOC_AS5048A_Angle_Update() 读 SPI、解包 cum_count、算 ElAngle
 * 慢环（TIM4）：FOC_AS5048A_Speed_Calc() cum_count 差分测速
 */

#include "foc_as5048a.h"
#include "foc_config.h"
#include "foc_math.h"
#include "spi.h"
#include "main.h"

#include <math.h>

/* ---------- 内部辅助 ---------- */

/** raw 相对 ref 的最短有符号计数差（-8192..8191） */
static int16_t AS5048A_RawDelta(uint16_t raw, uint16_t ref)
{
    int32_t d = (int32_t)raw - (int32_t)ref;

    if (d > (int32_t)(AS5048A_RESOLUTION / 2u))  { d -= (int32_t)AS5048A_RESOLUTION; }
    if (d < -(int32_t)(AS5048A_RESOLUTION / 2u)) { d += (int32_t)AS5048A_RESOLUTION; }
    return (int16_t)d;
}

/** cum_count 取模得到 0..16383 原始计数 */
static uint16_t AS5048A_CumToRaw(int32_t cum)
{
    int32_t r = cum % (int32_t)AS5048A_RESOLUTION;

    if (r < 0) { r += (int32_t)AS5048A_RESOLUTION; }
    return (uint16_t)r;
}

/** 每 tick 最大允许计数步进（来自 FOC_ANGLE_GLITCH_MAX_RAD） */
static int16_t AS5048A_MaxStepCounts(void)
{
    return (int16_t)(FOC_ANGLE_GLITCH_MAX_RAD / _2PI * (float)AS5048A_RESOLUTION + 0.5f);
}

/** 方向滤波死区（计数） */
static int16_t AS5048A_DirBandCounts(void)
{
    return (int16_t)(FOC_ANGLE_DIR_BAND_RAD / _2PI * (float)AS5048A_RESOLUTION + 0.5f);
}

/** 由 cum_count 同步 speed.MecAngle（0～2π，供显示/Pitch 用） */
static void AS5048A_SyncMecAngle(FOC_AS5048A_t *enc)
{
    enc->speed.MecAngle = (float)AS5048A_CumToRaw(enc->cum_count)
                          / (float)AS5048A_RESOLUTION * _2PI;
}

/**
 * @brief 用连续 cum_count 算电角度（FOC 用）
 *
 * 旧：Limit(MecAngle×pp)，MecAngle 每圈 0/2π 跳变 → ElAngle 突变 → 特定位置“卡住”
 * 新：cum_count 连续递增 → ElAngle 在 2π 边界平滑（6.28→0 为同一点）
 */
static float AS5048A_ElAngleFromCum(const FOC_AS5048A_t *enc)
{
    float el = (float)enc->cum_count * _2PI / (float)AS5048A_RESOLUTION
               * (float)enc->speed.pole_pairs
               + enc->speed.PhaseShift;

    return Limit_Angle(el);
}

/** cum_count 减掉整圈数，保持差分/电角不变，防 int32 膨胀与 float 精度损失 */
static void AS5048A_WrapCumRevolutions(FOC_AS5048A_t *enc)
{
    const int32_t mod  = (int32_t)AS5048A_RESOLUTION;
    const int32_t span = mod * 512;   /* 超过 ±512 圈则回绕 */
    int32_t       strip;

    if (enc->cum_count > span)
    {
        strip = (enc->cum_count / mod) * mod;
        enc->cum_count      -= strip;
        enc->speed_last_cum -= strip;
    }
    else if (enc->cum_count < -span)
    {
        strip = ((-enc->cum_count / mod) + 1) * mod;
        enc->cum_count      += strip;
        enc->speed_last_cum += strip;
    }
}

/** 校验 SPI 帧：非 0/0xFFFF 且 raw 在 14-bit 范围内 */
static bool AS5048A_FrameValid(uint16_t frame, uint16_t raw)
{
    if ((frame == 0u) || (frame == 0xFFFFu)) { return false; }
    if (raw >= AS5048A_RESOLUTION) { return false; }
    return true;
}

/**
 * @brief OTP 零位：软件模拟减 A_z（烧前验收）；OTP 已烧时芯片已减，原样返回
 */
static uint16_t AS5048A_ApplyOtpZero(uint16_t raw)
{
#if FOC_OTP_AZ_SIMULATE
    int32_t r = (int32_t)raw - (int32_t)FOC_OTP_AZ_VALUE;

    r = r % (int32_t)AS5048A_RESOLUTION;
    if (r < 0) { r += (int32_t)AS5048A_RESOLUTION; }
    return (uint16_t)r;
#else
    (void)raw;
    /* FOC_OTP_ZERO_BURNED=1：angle 寄存器输出已含 OTP ZPOS 偏移 */
    return raw;
#endif
}

/** OTP 方案 B：无需上电 Vd-lock 对齐 */
bool FOC_AS5048A_OtpSkipBootCalib(void)
{
#if (FOC_OTP_ZERO_BURNED || FOC_OTP_AZ_SIMULATE)
    return true;
#else
    return false;
#endif
}

/* ---------- SPI ---------- */

/** 16-bit SPI 全双工传输（CS 软件控制） */
static uint16_t AS5048A_Transfer(uint16_t cmd)
{
    uint8_t tx[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFFu) };
    uint8_t rx[2] = { 0, 0 };

    HAL_GPIO_WritePin(AS5048A_CS_GPIO_Port, AS5048A_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 50);
    HAL_GPIO_WritePin(AS5048A_CS_GPIO_Port, AS5048A_CS_Pin, GPIO_PIN_SET);

    return ((uint16_t)rx[0] << 8) | rx[1];
}

/** 读角度寄存器（需两次传输：首帧丢弃，次帧有效） */
static uint16_t FOC_AS5048A_ReadFrame(void)
{
    AS5048A_Transfer(AS5048A_READ_CMD);
    return AS5048A_Transfer(AS5048A_READ_CMD);
}

/* ---------- 对外接口 ---------- */

/** 初始化编码器状态与极对数 */
void FOC_AS5048A_Init(FOC_AS5048A_t *enc, uint32_t pole_pairs)
{
    enc->speed.pole_pairs    = pole_pairs;
    enc->speed.PhaseShift    = 0.0f;
    enc->speed.MecAngle      = 0.0f;
    enc->speed.ElAngle       = 0.0f;
    enc->speed.AvrMecSpeed   = 0.0f;
    enc->speed.Max_MecSpeed  = FOC_SPEED_MAX_RPM;
    enc->cum_count           = 0;
    enc->speed_last_cum      = 0;
    enc->last_dr             = 0;
    enc->last_mec_angle      = 0.0f;
    enc->raw                 = 0u;
    enc->last_raw            = 0u;
    enc->angle_valid         = false;
    enc->speed_valid         = false;
    enc->speed_init          = false;
    enc->speed_warmup        = 0u;
    enc->speed_still_ms      = 0u;
    enc->calib_mec_rad       = 0.0f;
}

/** 上电自检：连续读两帧，检查 raw 合法 */
bool FOC_AS5048A_SelfTest(FOC_AS5048A_t *enc)
{
    uint16_t f1 = FOC_AS5048A_ReadFrame();
    uint16_t f2 = FOC_AS5048A_ReadFrame();
    uint16_t r1 = f1 & 0x3FFFu;
    uint16_t r2 = f2 & 0x3FFFu;

    if (r1 >= AS5048A_RESOLUTION || r2 >= AS5048A_RESOLUTION) { return false; }
    if (((f1 == 0u) && (f2 == 0u)) || ((f1 == 0xFFFFu) && (f2 == 0xFFFFu))) { return false; }

    enc->raw              = AS5048A_ApplyOtpZero(r2);
    enc->last_raw         = enc->raw;
    enc->cum_count        = (int32_t)enc->raw;
    enc->last_dr          = 0;
    AS5048A_SyncMecAngle(enc);
    enc->last_mec_angle   = enc->speed.MecAngle;
    enc->angle_valid      = true;
    enc->speed_valid      = false;
    enc->speed_init       = false;
    return true;
}

/**
 * @brief 快环角度更新：SPI 读 → 解包 cum_count → 计算 ElAngle
 * @return 当前电角度 (rad)
 */
float FOC_AS5048A_Angle_Update(FOC_AS5048A_t *enc)
{
    uint16_t frame = FOC_AS5048A_ReadFrame();
    uint16_t raw   = AS5048A_ApplyOtpZero(frame & 0x3FFFu);

    if (AS5048A_FrameValid(frame, raw))
    {
        int16_t max_step = AS5048A_MaxStepCounts();
        int16_t dir_band = AS5048A_DirBandCounts();
        int16_t dr;
        bool    accept;

        enc->last_raw = enc->raw;
        enc->raw      = raw;

        if (!enc->angle_valid)
        {
            enc->cum_count   = (int32_t)raw;
            enc->last_dr     = 0;
            enc->angle_valid = true;
        }
        else
        {
            dr = AS5048A_RawDelta(raw, AS5048A_CumToRaw(enc->cum_count));

            accept = ((dr >= -max_step) && (dr <= max_step))
                  || ((enc->last_dr < -max_step) || (enc->last_dr > max_step));

            if (accept && enc->speed_init)
            {
                float rpm = enc->speed.AvrMecSpeed;

                if ((rpm > FOC_ANGLE_DIR_MIN_RPM) && (dr < -dir_band))
                {
                    accept = false;
                }
                else if ((rpm < -FOC_ANGLE_DIR_MIN_RPM) && (dr > dir_band))
                {
                    accept = false;
                }
            }

            if (accept)
            {
                enc->cum_count += (int32_t)dr;
                enc->last_dr    = dr;
            }
            /* 异常帧：丢弃，不推进 cum_count（旧逻辑 last_dr 累加会导致停转后假速度） */
        }

        AS5048A_WrapCumRevolutions(enc);
        AS5048A_SyncMecAngle(enc);
    }

    enc->speed.ElAngle = AS5048A_ElAngleFromCum(enc);
    return enc->speed.ElAngle;
}

/** 清零速度滤波状态（启动 RUN 预热前调用） */
void FOC_AS5048A_Speed_Reset(FOC_AS5048A_t *enc)
{
    enc->speed.AvrMecSpeed = 0.0f;
    enc->speed_last_cum    = enc->cum_count;
    enc->last_mec_angle    = enc->speed.MecAngle;
    enc->last_raw          = enc->raw;
    enc->last_dr           = 0;
    enc->speed_warmup      = 0u;
    enc->speed_still_ms    = 0u;
    enc->speed_valid       = false;
    enc->speed_init        = false;
}

/**
 * @brief 速度测量：cum_count 整数差分 + 一阶低通（方案 B）
 * @param Ts 采样周期 (s)，调用方传 1/FOC_SPEED_LOOP_HZ
 *
 * dr = cum_count[k] - cum_count[k-1]
 * rpm_inst = dr / 16384 / Ts * 60
 * AvrMecSpeed += (Ts/τ) * (rpm_inst - AvrMecSpeed)
 */
void FOC_AS5048A_Speed_Calc(FOC_AS5048A_t *enc, float Ts)
{
    int32_t dr;
    int32_t abs_dr;
    float   rpm_inst;
    float   alpha;

    if (!enc->angle_valid || (Ts <= 0.0f))
    {
        return;
    }

    if (enc->speed_warmup < FOC_SPEED_WARMUP_TICKS)
    {
        enc->speed_last_cum = enc->cum_count;
        enc->speed_warmup++;
        return;
    }

    if (!enc->speed_valid)
    {
        enc->speed_valid = true;
    }

    dr     = enc->cum_count - enc->speed_last_cum;
    abs_dr = (dr >= 0) ? dr : -dr;

    /* 静止检测：cum 几乎不变 → 强制 rpm=0，快速清零 Avr */
    if (abs_dr <= (int32_t)FOC_SPEED_ZERO_DR_COUNTS)
    {
        if (enc->speed_still_ms < 255u)
        {
            enc->speed_still_ms++;
        }

        enc->speed_last_cum = enc->cum_count;
        rpm_inst            = 0.0f;

        if (enc->speed_still_ms >= (uint8_t)FOC_SPEED_STATIC_ZERO_MS)
        {
            enc->speed.AvrMecSpeed = 0.0f;
            enc->speed_init        = true;
            return;
        }
    }
    else
    {
        enc->speed_still_ms = 0u;
    }

    /* 坏样本：dr / rpm_inst 超限 → 丢弃本拍，不污染 Avr（旧逻辑会先推进 last_cum） */
    rpm_inst = (float)dr / (float)AS5048A_RESOLUTION / Ts * 60.0f;

    if ((abs_dr > (int32_t)FOC_SPEED_MAX_DR_COUNTS) ||
        (fabsf(rpm_inst) > FOC_SPEED_INST_MAX_RPM))
    {
        enc->speed_last_cum = enc->cum_count;
        return;
    }

    enc->speed_last_cum = enc->cum_count;

    rpm_inst = LIMIT_RANGE(rpm_inst, -FOC_SPEED_MAX_RPM, FOC_SPEED_MAX_RPM);

    alpha = (FOC_SPEED_LPF_TAU > 0.0f) ? (Ts / FOC_SPEED_LPF_TAU) : 1.0f;
    if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    if (!enc->speed_init)
    {
        enc->speed.AvrMecSpeed = rpm_inst;
        enc->speed_init        = true;
    }
    else
    {
        enc->speed.AvrMecSpeed += alpha * (rpm_inst - enc->speed.AvrMecSpeed);
    }

    enc->speed.AvrMecSpeed = LIMIT_RANGE(enc->speed.AvrMecSpeed,
                                         -FOC_SPEED_MAX_RPM,
                                         FOC_SPEED_MAX_RPM);

    if (fabsf(enc->speed.AvrMecSpeed) < FOC_SPEED_DEADZONE_RPM)
    {
        enc->speed.AvrMecSpeed = 0.0f;
    }
}
