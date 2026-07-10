GM4108 云台 FOC
平台：STM32F103C8T6 + GM4108 + AS5048A + L6234
最后更新：2026-07-09

1. 项目目标
项目	说明
电机	iFlight GM4108 云台电机（22 极 → 11 对极）
编码器	AS5048A，14位，SPI
控制器  STM32F103C8T6
控制	电压模式 FOC（位置→速度→Iq_ref→前馈 Uq）;
参考	Reborn大侠 库移植STM32G473CBT6_FOC
3. 电机参数（foc_config.h）
参数	值
MOTOR_POLE_PAIRS	11
MOTOR_R_OHM	11.2 Ω（规格 11 Ω ±5%）
MOTOR_L_H	0.003 H（待测）
MOTOR_BUS_VOLTAGE	14.8 V（4S）
MOTOR_KE_VS_RAD	0.340 V·s/rad（20V 空载 ~540RPM 估算）
MOTOR_IQ_MAX_A	±0.15 A（速度环 Iq_ref 限幅）
MOTOR_I_ABS_MAX_A	1.5 A（规格绝对上限）
VQ_MAX	Vbus × 0.85
电压模式理由：GM4108 大内阻 + 小电感，低速时 Iq≈Vq/R
