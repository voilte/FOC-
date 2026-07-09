#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* ============================================================
 *  GM4108 云台 FOC 配置
 * ============================================================ */

/* ---- 电机 GM4108（规格表 + 实测修正） ---- */
#define MOTOR_POLE_PAIRS        11u
#define MOTOR_R_OHM             11.2f
#define MOTOR_L_H               0.003f
#define MOTOR_BUS_VOLTAGE       14.8f
#define MOTOR_I_ABS_MAX_A       1.5f
#define MOTOR_KE_VS_RAD         0.340f
#define MOTOR_IQ_MAX_A          0.15f

/* ---- 编码器 AS5048A ---- */
#define AS5048A_RESOLUTION      16384u
#define AS5048A_READ_CMD        0xFFFFu

/* 方案 B：OTP ZeroPosition（量产烧一次，上电无需 PhaseShift / Vd-lock）
 * OTP 已烧：芯片 angle 输出已减 ZPOS，固件 PhaseShift=0
 * 烧前验收：FOC_OTP_AZ_SIMULATE=1 软件减 A_z，同样 PhaseShift=0 */
#define FOC_OTP_AZ_VALUE            14693u
#define FOC_OTP_AZ_SIMULATE         0
#define FOC_OTP_ZERO_BURNED         1       /* 1=OTP 已烧，跳过 boot 对齐 */

/* OTP 烧写（量产工装；OTP 已烧则保持 BURN_ENABLE=0） */
#define FOC_TEST_OTP_BURN_ENABLE    0
#define FOC_TEST_OTP_RAW_TOLERANCE  80u
#define FOC_TEST_OTP_BURN_DELAY_MS  5000u

/* ---- 环路频率 ---- */
#define FOC_PWM_FREQ_HZ         10000u
#define FOC_SPEED_LOOP_HZ       1000u

/* ---- 电压限幅 ---- */
#define VQ_MAX                  (MOTOR_BUS_VOLTAGE * 0.85f)

/* ---- 速度测量（cum_count 差分 + LPF） ---- */
#define FOC_SPEED_LPF_TAU           0.04f
#define FOC_SPEED_DEADZONE_RPM      1.0f
#define FOC_SPEED_INST_MAX_RPM      500.0f
#define FOC_SPEED_MAX_RPM           300.0f
#define FOC_SPEED_WARMUP_TICKS      2u

/* ---- 速度环 PI + 前馈（1 kHz，输出 Iq_ref → Uq） ----
 * PI 输出单位：A（力矩电流给定）；P 约 0.003 → 10 RPM 误差 ≈ 0.03 A
 * 带宽目标 2~10 Hz，需实机微调 */
#define FOC_SPEED_PI_P              0.003f
#define FOC_SPEED_PI_I              0.001f
#define FOC_SPEED_VQ_SLEW_V         0.5f    /* 每 1 ms 最大 |ΔUq| (V) */
#define FOC_GIMBAL_SPEED_WARMUP_MS  0u

/* ---- PID 积分抗饱和 ---- */
#define FOC_PID_AW_GAIN             1.0f    /* 回算增益；1=标准 back-calculation */
#define FOC_SPEED_AW_VQ_THRESH_V    0.05f   /* |Vq_applied-Vq_ff| 超此值触发 Vq 级抗饱和 */

/* ---- 速度环测试（foc_test.c） ---- */
#define FOC_TEST_TARGET_RPM         30.0f   /* 恒速给定，CCW 为正 */

/* ---- Vd-lock 校准（仅 OTP 未烧且未模拟时 boot 使用；OTP 方案下不启用） ---- */
#define FOC_CALIB_VD                1.5f
#define FOC_CALIB_HOLD_MS           1000u
#define FOC_CALIB_RAMP_MS           300u
#define FOC_CALIB_MAX_MEC_SPREAD_DEG 2.0f
#define FOC_CALIB_MAX_RETRY         8u
#define FOC_CALIB_MEC_REF_DEG       145.0f
#define FOC_CALIB_MEC_TOLERANCE_DEG 20.0f
#define FOC_CALIB_MEC_REF_ENABLE    0u      /* OTP 已烧可关；1=启用 Vd-lock well 检查 */

/* ---- 角度跟踪 ---- */
#define FOC_ANGLE_GLITCH_MAX_RAD  0.006f
#define FOC_ANGLE_DIR_MIN_RPM     3.0f
#define FOC_ANGLE_DIR_BAND_RAD    0.0015f

/* ---- 测速验证测试（foc_test.c） ---- */
#define FOC_TEST_READY_DELAY_MS     1000u
#define FOC_TEST_RUN_WARMUP_MS      500u
#define FOC_TEST_PRINT_MS           50u

#endif /* FOC_CONFIG_H */
