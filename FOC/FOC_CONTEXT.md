# GM4108 云台 FOC — 项目上下文与需求规格

> 用途：在新工作区恢复对话上下文，作为应用层/测试层开发依据。  
> 平台：STM32F103C8T6 + GM4108 + AS5048A + L6234  
> 最后更新：2026-07-09

---

## 1. 项目目标

| 项目 | 说明 |
|------|------|
| 电机 | iFlight GM4108 云台电机（22 极 → **11 对极**） |
| 编码器 | AS5048A，14-bit，SPI |
| MCU | STM32F103C8T6 |
| 控制 | **电压模式 FOC**（位置→速度→**Iq_ref**→前馈 **Uq**）；效果不好再升级电流环 |
| 参考 | Reborn大侠 `STM32G473CBT6_FOC` 库移植 |

---

## 2. 电机参数（foc_config.h）

| 参数 | 值 |
|------|-----|
| `MOTOR_POLE_PAIRS` | 11 |
| `MOTOR_R_OHM` | 11.2 Ω（规格 11 Ω ±5%） |
| `MOTOR_L_H` | 0.003 H（待实测） |
| `MOTOR_BUS_VOLTAGE` | 14.8 V（4S，改实测） |
| `MOTOR_KE_VS_RAD` | **0.340 V·s/rad**（20V 空载 ~540RPM 估算） |
| `MOTOR_IQ_MAX_A` | **±0.15 A**（速度环 Iq_ref 限幅） |
| `MOTOR_I_ABS_MAX_A` | 1.5 A（规格绝对上限） |
| `VQ_MAX` | `Vbus × 0.85` |

**电压模式理由**：GM4108 大内阻 + 小电感，低速时 Iq≈Vq/R，第一版可省略电流环。

---

## 3. 控制架构（已定）

```
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
```

| 环 | 频率 | 定时器 |
|----|------|--------|
| FOC / SVPWM | ~5 kHz | TIM2 中心对齐 10 kHz，半周期触发 |
| 速度环 + 测速 | 1 kHz | TIM4 |
| 位置环 | 200 Hz 建议 | TIM4 内 5 分频 |

- **速度 PI 输出 Iq_ref**，限幅 **±MOTOR_IQ_MAX_A（0.15 A）**
- **前馈**：`Uq = R·Iq_ref + ω·Ke`，`Ud = 0`（ω 为机械角速度 rad/s）
- **Id = 0**：正常运行
- **正方向**：+Vq = CCW = 机械角增大
- **速度环带宽**：2~10 Hz（云台）

### 3.1 力矩 / 前馈参数（已定）

| 项 | 值 |
|----|-----|
| `Iq_ref` 限幅 | **±0.15 A** |
| `Ke` | **0.340 V·s/rad**（可实测微调） |
| 速度 PI 输出 | **绝对 Iq_ref** → `Uq = R·Iq + ω·Ke` |

---

## 4. 速度测量（方案 B，**已实现**）

**cum_count 整数差分 + 一阶低通**，`FOC_AS5048A_Speed_Calc()`，TIM4 1 kHz 在 `FOC_Motor_SlowLoop()` 调用。

```c
dr = cum_count[k] - cum_count[k-1];
rpm_inst = dr / 16384.0f / Ts * 60.0f;
// 限幅 / LPF / 死区 → AvrMecSpeed
```

| 参数 | 宏名建议 | 值 |
|------|----------|-----|
| 采样周期 Ts | — | 1 ms（FOC_SPEED_LOOP_HZ=1000） |
| 低通时间常数 τ | `FOC_SPEED_LPF_TAU` | **0.04 s**（40 ms） |
| 死区 | `FOC_SPEED_DEADZONE_RPM` | **1.0 RPM** |
| 瞬时限幅 | `FOC_SPEED_INST_MAX_RPM` | **500 RPM**（超限丢弃本拍） |
| 输出限幅 | `FOC_SPEED_MAX_RPM` | **300 RPM** |
| 预热 tick | `FOC_SPEED_WARMUP_TICKS` | **2**（第 3 拍起 `speed_valid`） |

---

## 5. 电角度零位：AS5048A OTP 方案 B（已定）

| 场景 | 行为 |
|------|------|
| **OTP 已烧**（`FOC_OTP_ZERO_BURNED=1`） | 芯片 angle 已减 ZPOS；**PhaseShift=0**；**无 boot Vd-lock** |
| 烧前验收 | `FOC_OTP_AZ_SIMULATE=1` 软件减 `FOC_OTP_AZ_VALUE`；同样 PhaseShift=0 |
| OTP 未烧且未模拟 | 才走 `MOTOR_ENC_CALIB` Vd-lock（备用） |
| 量产烧 OTP | `FOC_TEST_OTP_BURN_ENABLE=1` 测试模式（OTP 已烧则保持 0） |

| 宏 | 当前值 |
|----|--------|
| `FOC_OTP_AZ_VALUE` | 14693 |
| `FOC_OTP_ZERO_BURNED` | **1** |
| `FOC_OTP_AZ_SIMULATE` | 0 |

**不再使用** MCU Flash 存 PhaseShift；θ_mech 来自 AS5048A，θ_elec = θ_mech × P，offset = 0。

---

## 6. 状态机与启动流程（已定）

### 6.1 状态说明

| 状态 | PWM | 含义 |
|------|-----|------|
| `MOTOR_IDLE` | 关 | 初始化占位；**STOP 后进入此状态** |
| `MOTOR_ENC_CALIB` | 开(Vd) | PhaseShift Vd-lock 校准 |
| `MOTOR_READY` | 关 | 校准完成，零力矩待机 |
| `MOTOR_GIMBAL_RUN` | 开 | FOC + 三环运行 |
| `MOTOR_FAULT` | 关 | 故障锁死 |

### 6.2 上电流程

```
复位 → HAL/外设 Init → FOC_UART_Init → FOC_Init()
  → AS5048A 自检
      失败 → MOTOR_FAULT
      成功 → OTP 已烧：PhaseShift=0 → MOTOR_READY（无 Vd-lock）
  → 启动 TIM2/TIM4 中断

MOTOR_READY 持续 1 s → 自动 FOC_Motor_StartGimbal() → MOTOR_GIMBAL_RUN
  → 预热 500 ms：Vq=0，Speed_Reset，等 speed_valid
  → 三环闭环工作
```

| 参数 | 值 |
|------|-----|
| READY → RUN 延时 | **1000 ms** |
| RUN 预热 | **500 ms**（Vq=0） |

### 6.3 STOP 行为（已定）

```
GIMBAL_RUN → FOC_Motor_Stop() → MOTOR_IDLE
```

- 关 PWM、清 PID、Uqd=0
- **串口打印说明**：已进入 IDLE，需 **复位或重新 FOC_Init()** 才能再次完整启动
- **不**自动回到 READY/RUN（防误重启）

### 6.4 FAULT 恢复（已定）

- 仅 **硬件复位** 或 **重新 FOC_Init()**
- 无串口 soft-reset

---

## 7. 代码分层（当前工程）

### 底层（已实现，已注释）

| 文件 | 作用 |
|------|------|
| foc_math, foc_pwm, foc_pid | Clarke/Park/SVPWM/PID |
| foc_hw | TIM2 PWM、TIM4、L6234 EN、中断 |
| foc_as5048a | SPI 读角、cum_count 解包、**Speed_Calc 已实现** |
| foc_uart | 非阻塞调试 TX |
| foc_motor | 状态机、PhaseShift 校准、FOC 快环 |
| foc_app | FOC_Init、快环入口 |
| foc_config | 电机/校准/角度跟踪参数 |

### 应用层（待写）

| 文件 | 内容 |
|------|------|
| foc_gimbal.c/h | 位置环、速度环、Iq 前馈 Uq |
| foc_motor SlowLoop | **测速已实现**；速度/位置环待加 |
| Flash 读写 | PhaseShift 持久化 |
| main / 应用 Poll | READY 1s 自动 RUN、STOP 打印 |

### 测试层（讨论结束后由用户指定）

- PhaseShift 验证、开环/闭环测试、VOFA 等

---

## 8. 硬件（当前 CubeMX）

| 项目 | 配置 |
|------|------|
| PWM | TIM2 CH1/2/3 → PA0/PA1/PA2，10 kHz 中心对齐 |
| 慢环 | TIM4 1 kHz |
| 编码器 | SPI1 + CS |
| 驱动 | L6234，DRV_EN GPIO |
| 调试 | USART1 |

---

## 9. 需求确认清单（全部已定）

- [x] 三环：位置 → 速度 → **Iq_ref → 前馈 Uq**（电压模式，无电流环）
- [x] 效果不好再上电流环
- [x] Ke / Iq 限幅写入 foc_config.h
- [x] 测速 cum_count 差分（foc_as5048a.c + SlowLoop 调用）
- [x] 零位：AS5048A OTP 已烧，PhaseShift=0，无 boot 对齐
- [x] READY 后 **1 s 自动 RUN**
- [x] RUN 预热 **500 ms**
- [x] STOP → **IDLE** + 串口提示需复位/Init
- [x] FAULT → 复位 / FOC_Init
- [ ] 测试代码内容（用户后续指定）

---

## 10. 进度

- [x] 底层框架移植 F103
- [x] AS5048A SPI + cum_count 角度跟踪
- [x] SVPWM + PhaseShift Vd-lock 校准
- [x] 需求讨论完成
- [x] 测速方案 B（cum_count 差分 + LPF）
- [x] AS5048A OTP 零位 + OtpSkipBootCalib
- [ ] OTP 烧写测试模式（需时从工装分支恢复 BurnZeroPosition）
- [ ] 应用层（速度环 Iq 前馈 + 位置环 + 启动序列）
- [ ] 测试代码
