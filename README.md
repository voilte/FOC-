GM4108 云台 FOC — 项目上下文与需求规格
用途：在新工作区恢复对话上下文，作为应用层/测试层开发依据。
平台：STM32F103C8T6 + GM4108 + AS5048A + L6234
最后更新：2026-07-09

1. 项目目标
项目	说明
电机	iFlight GM4108 云台电机（22 极 → 11 对极）
编码器	AS5048A，14位，SPI
漫威电影宇宙	STM32F103C8T6
控制	电压模式 FOC（位置→速度→Iq_ref→前馈 Uq）;效果不好再升级电流环
参考	Reborn大侠 库移植STM32G473CBT6_FOC
2. 电机参数（foc_config.h）
参数	值
MOTOR_POLE_PAIRS	11
MOTOR_R_OHM	11.2 Ω（规格 11 Ω ±5%）
MOTOR_L_H	0.003 H（待测）
MOTOR_BUS_VOLTAGE	14.8 V（4S）
MOTOR_KE_VS_RAD	0.340 V·s/rad（20V 空载 ~540RPM 估算）
MOTOR_IQ_MAX_A	±0.15 A（速度环 Iq_ref 限幅）
MOTOR_I_ABS_MAX_A	1.5 A（规格绝对上限）
VQ_MAX	Vbus × 0.85
电压模式理由：GM4108 大内阻 + 小电感，低速时 Iq≈Vq/R，第一版可省略电流环。

3. 控制架构（已定）
位置参考 → 位置 PID → ω_ref
                          ↓
                     速度 PID → Iq_ref（力矩电流给定，限幅 ±I_max）
                          ↓
              ┌── 前馈（核心，无电流环仍适用）──┐
              │  Uq_ff = R·Iq_ref + ω·Ke      │  Ke=0.340 V·s/rad
              │  Ud_ff = 0  （低速忽略 -ω·L·Iq）│
              └──────────────────────────────┘
                          ↓
              Uq = Uq_ff (+ 可选小电压修正)
              Ud = 0
                          ↓
              逆 Park → SVPWM → TIM2 PWM
                          ↓
              GM4108 ← AS5048A（SPI）
