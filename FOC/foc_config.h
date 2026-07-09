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
#define MOTOR_IQ_MAX_A          0.50f

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
#define FOC_SPEED_INST_MAX_RPM      100.0f  /* 单拍瞬时 RPM 上限，超则丢弃本拍 */
#define FOC_SPEED_MAX_RPM           300.0f
#define FOC_SPEED_WARMUP_TICKS      2u
/* 1 ms 内最大合理计数差（按 FOC_SPEED_MAX_RPM 估算 + 余量） */
#define FOC_SPEED_MAX_DR_COUNTS     90
#define FOC_SPEED_ZERO_DR_COUNTS    2       /* |dr|<=此值视为静止 */
#define FOC_SPEED_STATIC_ZERO_MS    30u     /* 连续静止超过此时间 → Avr=0 */

/* ---- 电机转向（正目标 RPM = CCW = 机械角增大） ----
 * 正目标时若电机反转、测速为负 → FOC_MOTOR_UQ_SIGN 改为 -1 */
#define FOC_MOTOR_UQ_SIGN           (-1.0f)

/* ---- 速度环 PI + 前馈（1 kHz，输出 Iq_ref → Uq） ----
 * P 单位 A/RPM；0.01 → 20 RPM 误差约 0.2 A（限幅 0.15 A） */
#define FOC_SPEED_PI_P              0.0505f
#define FOC_SPEED_PI_I              0.0020f
#define FOC_SPEED_VQ_SLEW_V         0.5f    /* 每 1 ms 最大 |ΔUq| (V) */
#define FOC_GIMBAL_SPEED_WARMUP_MS  0u

/* ---- PID 积分抗饱和 ---- */
#define FOC_PID_AW_GAIN             1.0f    /* 回算增益；1=标准 back-calculation */
#define FOC_SPEED_AW_VQ_THRESH_V    0.05f   /* |Vq_applied-Vq_ff| 超此值触发 Vq 级抗饱和 */
/* 前馈 ω·Ke 用目标转速（1=推荐，防超调后 Ke·ω_fb 继续加压） */
#define FOC_SPEED_FF_KE_USE_TARGET  1
/* 超调退积分：实测 > 目标 + 此值 (RPM) 时 SumError 每 ms 衰减 */
#define FOC_SPEED_OVERSHOOT_RPM     2.0f
#define FOC_SPEED_OVERSHOOT_DECAY   0.95f

/* ---- 测试模式（foc_test.c） ----
 * 0 = 速度环联调  1 = 只读机械角  2 = 位置环联调 */
#define FOC_TEST_MODE_SPEED         0
#define FOC_TEST_MODE_ANGLE         1
#define FOC_TEST_MODE_POSITION      2
#define FOC_TEST_MODE               FOC_TEST_MODE_SPEED

/* 兼容旧宏名 */
#define FOC_TEST_MODE_ANGLE_ONLY    (FOC_TEST_MODE == FOC_TEST_MODE_ANGLE)

/* ---- 测速 / 速度环测试（FOC_TEST_MODE_SPEED） ---- */
#define FOC_TEST_TARGET_RPM         30.0f   /* FOC_TEST_SPEED_STEP_ENABLE=0 时恒速给定 */
#define FOC_TEST_SPEED_STEP_ENABLE  1       /* 1=阶梯给定  0=恒速 */
#define FOC_TEST_SPEED_STEP_MIN_RPM 20.0f   /* 阶梯起始 RPM */
#define FOC_TEST_SPEED_STEP_MAX_RPM 40.0f   /* 阶梯结束 RPM */
#define FOC_TEST_SPEED_STEP_DELTA   1.0f    /* 每级递增 RPM */
#define FOC_TEST_SPEED_STEP_HOLD_MS 3000u   /* 每级保持时间 (ms) */

/* ---- 位置环测试（FOC_TEST_MODE_POSITION） ---- */
#define FOC_TEST_TARGET_PITCH_DEG   0.0f    /* Pitch 目标角 (deg)，相对水平零点 */

/* ---- Vd-lock 校准（仅 OTP 未烧且未模拟时 boot 使用；OTP 方案下不启用） ---- */
#define FOC_CALIB_VD                1.5f
#define FOC_CALIB_HOLD_MS           1000u
#define FOC_CALIB_RAMP_MS           300u
#define FOC_CALIB_MAX_MEC_SPREAD_DEG 2.0f
#define FOC_CALIB_MAX_RETRY         8u
#define FOC_CALIB_MEC_REF_DEG       145.0f
#define FOC_CALIB_MEC_TOLERANCE_DEG 20.0f
#define FOC_CALIB_MEC_REF_ENABLE    0u      /* OTP 已烧可关；1=启用 Vd-lock well 检查 */

/* ---- Pitch 轴（位置环零点，方案 B） ----
 * 水平姿态时 VOFA mec_deg 实测值；位置环用 pitch = mec_deg - ZERO，wrap ±180° */
#define FOC_PITCH_ZERO_DEG          182.0f
#define FOC_PITCH_LIMIT_DEG         60.0f   /* 软限位 ±60° */

/* ---- 位置环 P → ω_ref（200 Hz，TIM4 5 分频） ----
 * 输出 RPM 给速度环；P 单位 RPM/deg，初值需实机微调 */
#define FOC_POS_LOOP_HZ             200u
#define FOC_POS_LOOP_DIV            (FOC_SPEED_LOOP_HZ / FOC_POS_LOOP_HZ)
#if (FOC_POS_LOOP_DIV * FOC_POS_LOOP_HZ) != FOC_SPEED_LOOP_HZ
#error FOC_POS_LOOP_HZ must divide FOC_SPEED_LOOP_HZ evenly
#endif
#define FOC_POS_PI_P                0.02f    /* 位置环 P：RPM/deg，增大=响应快、易振荡 */
#define FOC_POS_PI_I                0.0f    /* 位置环 I：暂为 0，消除静差时可试 0.01~0.05 */
#define FOC_POS_PI_D                0.0f    /* 位置环 D：暂为 0 */
#define FOC_POS_OMEGA_MAX_RPM       30.0f   /* 位置环输出 ω_ref 限幅 (RPM)，先保守 */
#define FOC_POS_ERR_DEADBAND_DEG    1.5f    /* |Pitch 误差| < 此值 → ω_ref=0，防抖动 */
#define FOC_POS_STOP_RPM            3.0f    /* 到位：误差在死区内且 |rpm| < 此值 → 完全停转 */

/* ---- 角度跟踪 ---- */
#define FOC_ANGLE_GLITCH_MAX_RAD  0.006f
#define FOC_ANGLE_DIR_MIN_RPM     3.0f
#define FOC_ANGLE_DIR_BAND_RAD    0.0015f

/* ---- 测速验证测试（foc_test.c） ---- */
#define FOC_TEST_READY_DELAY_MS     1000u
#define FOC_TEST_RUN_WARMUP_MS      500u
#define FOC_TEST_PRINT_MS           50u

#endif /* FOC_CONFIG_H */
