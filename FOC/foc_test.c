/**
 * @file foc_test.c
 * @brief 速度环联调测试
 *
 * 流程：
 *   1. FOC_Init() → OTP 零位 → MOTOR_READY
 *   2. READY 1 s → GIMBAL_RUN
 *   3. 预热 500 ms：Vq=0（Speed_Reset）
 *   4. SpeedArm + target=FOC_TEST_TARGET_RPM → 速度环闭环
 *   5. 每 50 ms 打印 tgt / avr / Iq / Uq
 */

#include "foc_test.h"
#include "foc_app.h"
#include "foc_config.h"
#include "foc_gimbal.h"
#include "foc_math.h"
#include "foc_motor.h"
#include "foc_as5048a.h"
#include "foc_uart.h"
#include "main.h"

#include <stdio.h>
#include <stdarg.h>

typedef enum
{
    TEST_WAIT_READY = 0,
    TEST_WAIT_RUN_DELAY,
    TEST_RUN_WARMUP,
    TEST_SPEED_LOOP,      /**< 速度环闭环运行 */
    TEST_FAULT_HOLD,
} FOC_TestPhase_t;

static FOC_TestPhase_t s_phase;
static uint32_t        s_phase_since_ms;
static uint32_t        s_last_print_ms;
static uint8_t         s_started_run;

/* -------------------------------------------------------------------------- */

void FOC_Test_Printf(const char *fmt, ...)
{
    char    buf[192];
    va_list args;
    int     len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0) { return; }
    if ((size_t)len >= sizeof(buf)) { len = (int)sizeof(buf) - 1; }
    FOC_UART_Write((const uint8_t *)buf, (uint16_t)len);
}

static void Test_PrintBanner(void)
{
    FOC_Test_Printf("\r\n=== FOC Speed Loop Test ===\r\n");
    FOC_Test_Printf("Target=%.1f RPM  P=%.4f I=%.4f  Iq_max=%.2fA\r\n",
                    FOC_TEST_TARGET_RPM,
                    FOC_SPEED_PI_P,
                    FOC_SPEED_PI_I,
                    MOTOR_IQ_MAX_A);
    FOC_Test_Printf("VOFA: spd:tgt,avr,iq,vq,valid\r\n\r\n");
}

static void Test_StartRun(void)
{
    if (s_started_run) { return; }

    FOC_AS5048A_Speed_Reset(&FOC_MOTOR.enc);
    FOC_Motor_StartGimbal(&FOC_MOTOR);

    s_started_run    = 1u;
    s_phase          = TEST_RUN_WARMUP;
    s_phase_since_ms = HAL_GetTick();

    FOC_Test_Printf("[TEST] RUN — warmup %u ms Vq=0\r\n",
                    (unsigned)FOC_TEST_RUN_WARMUP_MS);
}

/** 预热结束：挂速度环并给目标转速 */
static void Test_ArmSpeedLoop(void)
{
    FOC_Gimbal_SpeedArm(&FOC_MOTOR);
    FOC_MOTOR.target_speed = FOC_TEST_TARGET_RPM;
    FOC_MOTOR.pid_speed.P  = FOC_SPEED_PI_P;
    FOC_MOTOR.pid_speed.I  = FOC_SPEED_PI_I;

    FOC_Test_Printf("[TEST] Speed loop ON  target=%.1f RPM\r\n", FOC_TEST_TARGET_RPM);
}

static void Test_PrintSpeedLoop(void)
{
    const FOC_AS5048A_t *enc = &FOC_MOTOR.enc;

    FOC_Test_Printf("[SPD] tgt=%+.1f avr=%+.2f iq=%+.4f vq=%+.2f sat_iq=%u sat_vq=%u\r\n",
                    FOC_MOTOR.target_speed,
                    enc->speed.AvrMecSpeed,
                    FOC_Gimbal_GetIqRef(),
                    FOC_Gimbal_GetUqApplied(),
                    (unsigned)FOC_Gimbal_GetPiSatIqFrom(&FOC_MOTOR),
                    (unsigned)FOC_Gimbal_GetPiSatVqFrom(&FOC_MOTOR));

    FOC_Test_Printf("spd:%.2f,%.2f,%.4f,%.2f,%u,%u,%u\n",
                    FOC_MOTOR.target_speed,
                    enc->speed.AvrMecSpeed,
                    FOC_Gimbal_GetIqRef(),
                    FOC_Gimbal_GetUqApplied(),
                    (unsigned)enc->speed_valid,
                    (unsigned)FOC_Gimbal_GetPiSatIqFrom(&FOC_MOTOR),
                    (unsigned)FOC_Gimbal_GetPiSatVqFrom(&FOC_MOTOR));
}

void FOC_Test_Init(void)
{
    s_phase           = TEST_WAIT_READY;
    s_phase_since_ms  = 0u;
    s_last_print_ms   = 0u;
    s_started_run     = 0u;

    Test_PrintBanner();
    FOC_Init();
}

void FOC_Test_Poll(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_phase)
    {
        case TEST_WAIT_READY:
            if (FOC_MOTOR.motor_state == MOTOR_FAULT)
            {
                s_phase = TEST_FAULT_HOLD;
                FOC_Test_Printf("[TEST] FAULT — reset or FOC_Init()\r\n");
                break;
            }
            if (FOC_MOTOR.motor_state == MOTOR_READY)
            {
                s_phase          = TEST_WAIT_RUN_DELAY;
                s_phase_since_ms = now;
                FOC_Test_Printf("[TEST] READY — RUN in %u ms\r\n",
                                (unsigned)FOC_TEST_READY_DELAY_MS);
            }
            break;

        case TEST_WAIT_RUN_DELAY:
            if (FOC_MOTOR.motor_state != MOTOR_READY) { break; }
            if ((now - s_phase_since_ms) >= (uint32_t)FOC_TEST_READY_DELAY_MS)
            {
                Test_StartRun();
            }
            break;

        case TEST_RUN_WARMUP:
            /* 预热阶段仍强制零力矩（速度环尚未 Arm） */
            FOC_MOTOR.foc_var.U_qd.q = 0.0f;
            FOC_MOTOR.foc_var.U_qd.d = 0.0f;

            if (FOC_MOTOR.motor_state != MOTOR_GIMBAL_RUN) { break; }

            if ((now - s_phase_since_ms) >= (uint32_t)FOC_TEST_RUN_WARMUP_MS)
            {
                Test_ArmSpeedLoop();
                s_phase          = TEST_SPEED_LOOP;
                s_phase_since_ms = now;
                s_last_print_ms  = now;
            }
            break;

        case TEST_SPEED_LOOP:
            if ((now - s_last_print_ms) >= (uint32_t)FOC_TEST_PRINT_MS)
            {
                s_last_print_ms = now;
                Test_PrintSpeedLoop();
            }
            break;

        case TEST_FAULT_HOLD:
        default:
            break;
    }
}
