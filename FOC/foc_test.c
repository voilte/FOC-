/**

 * @file foc_test.c

 * @brief 联调测试

 *

 * FOC_TEST_MODE_SPEED：速度环（可选阶梯给定 20→40 RPM 循环）

 * FOC_TEST_MODE_ANGLE：只读机械角

 * FOC_TEST_MODE_POSITION：位置环 + 速度环（可选 Pitch 线性斜坡）

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

#if (FOC_TEST_MODE == FOC_TEST_MODE_ANGLE)

    TEST_ANGLE_ONLY,

#elif (FOC_TEST_MODE == FOC_TEST_MODE_POSITION)

    TEST_RUN_WARMUP,

    TEST_POS_LOOP,

#else

    TEST_RUN_WARMUP,

    TEST_SPEED_LOOP,

#endif

    TEST_FAULT_HOLD,

} FOC_TestPhase_t;



static FOC_TestPhase_t s_phase;

static uint32_t        s_phase_since_ms;

static uint32_t        s_last_print_ms;

static uint8_t         s_started_run;



#if (FOC_TEST_MODE == FOC_TEST_MODE_SPEED) && FOC_TEST_SPEED_STEP_ENABLE

static float     s_step_target_rpm;
static uint32_t  s_step_since_ms;

#endif

#if (FOC_TEST_MODE == FOC_TEST_MODE_POSITION) && FOC_TEST_POS_RAMP_ENABLE

static float     s_ramp_pitch;
static uint32_t  s_ramp_last_ms;
static int8_t    s_ramp_dir;

#endif



/* -------------------------------------------------------------------------- */



void FOC_Test_Printf(const char *fmt, ...)

{

    char    buf[256];

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

#if (FOC_TEST_MODE == FOC_TEST_MODE_ANGLE)

    FOC_Test_Printf("\r\n=== FOC Angle Test (manual) ===\r\n");

    FOC_Test_Printf("VOFA FireWater, channel: mec_deg\r\n\r\n");

#elif (FOC_TEST_MODE == FOC_TEST_MODE_POSITION)

    FOC_Test_Printf("\r\n=== FOC Position Loop Test ===\r\n");

    FOC_Test_Printf("Pitch tgt=%.1f  zero=%.1f  limit=+/-%.0f\r\n",

                    FOC_TEST_TARGET_PITCH_DEG,

                    FOC_PITCH_ZERO_DEG,

                    FOC_PITCH_LIMIT_DEG);

    FOC_Test_Printf("Pos PID  P=%.2f I=%.4f  w_max=%.0f RPM\r\n",
                    FOC_POS_PI_P,
                    FOC_POS_PI_I,
                    FOC_POS_OMEGA_MAX_RPM);
#if FOC_TEST_POS_RAMP_ENABLE
    FOC_Test_Printf("Ramp pitch: %.0f -> %.0f deg @ %.1f deg/s (ping-pong)\r\n",
                    FOC_TEST_POS_RAMP_MIN_DEG,
                    FOC_TEST_POS_RAMP_MAX_DEG,
                    FOC_TEST_POS_RAMP_RATE_DPS);
#else
    FOC_Test_Printf("Const pitch target: %.1f deg\r\n", FOC_TEST_TARGET_PITCH_DEG);
#endif
    FOC_Test_Printf("VOFA 2ch @ %u ms: I0=tgt  I1=pitch_fb\r\n\r\n",
                    (unsigned)FOC_TEST_POS_PRINT_MS);

#else

    FOC_Test_Printf("\r\n=== FOC Speed Loop Test ===\r\n");

    FOC_Test_Printf("Speed PI  P=%.4f I=%.4f\r\n",

                    FOC_SPEED_PI_P,

                    FOC_SPEED_PI_I);

#if FOC_TEST_SPEED_STEP_ENABLE

    FOC_Test_Printf("Step target: %.0f -> %.0f RPM, +%.0f every %u ms\r\n",

                    FOC_TEST_SPEED_STEP_MIN_RPM,

                    FOC_TEST_SPEED_STEP_MAX_RPM,

                    FOC_TEST_SPEED_STEP_DELTA,

                    (unsigned)FOC_TEST_SPEED_STEP_HOLD_MS);

#else

    FOC_Test_Printf("Const target: %.1f RPM\r\n", FOC_TEST_TARGET_RPM);

#endif

    FOC_Test_Printf("VOFA FireWater: I0=目标RPM  I1=实测RPM\r\n");
    FOC_Test_Printf("  (每行格式: tgt,avr  例: 20.00,19.85)\r\n\r\n");

#endif

}



static float Test_MecDeg360(const FOC_AS5048A_t *enc)

{

    float deg = enc->speed.MecAngle * 360.0f / _2PI;



    while (deg < 0.0f)    { deg += 360.0f; }

    while (deg >= 360.0f) { deg -= 360.0f; }

    return deg;

}



static void Test_PrintMecDegVofa(const FOC_AS5048A_t *enc)

{

    FOC_Test_Printf("mec_deg:%.2f\n", Test_MecDeg360(enc));

}



static void Test_PrintPitchFbVofa(const FOC_AS5048A_t *enc)

{

    FOC_Test_Printf("pitch_fb:%.2f\n", FOC_Gimbal_PitchDegFromEnc(enc));

}



static void Test_PrintMecAndPitchFb(void)

{

    Test_PrintMecDegVofa(&FOC_MOTOR.enc);

    Test_PrintPitchFbVofa(&FOC_MOTOR.enc);

}



static void Test_ForceZeroTorque(void)

{

    FOC_MOTOR.target_speed   = 0.0f;

    FOC_MOTOR.foc_var.U_qd.q = 0.0f;

    FOC_MOTOR.foc_var.U_qd.d = 0.0f;

}



static void Test_StartRun(void)

{

    if (s_started_run) { return; }



    FOC_MOTOR.target_speed = 0.0f;

    FOC_Motor_StartGimbal(&FOC_MOTOR);



    s_started_run    = 1u;

    s_phase_since_ms = HAL_GetTick();

    s_last_print_ms  = s_phase_since_ms;



#if (FOC_TEST_MODE == FOC_TEST_MODE_ANGLE)

    s_phase = TEST_ANGLE_ONLY;

    FOC_Test_Printf("[TEST] Angle ON — Uq=0, rotate by hand\r\n");

#else

    FOC_AS5048A_Speed_Reset(&FOC_MOTOR.enc);

    s_phase = TEST_RUN_WARMUP;

    FOC_Test_Printf("[TEST] RUN — warmup %u ms Vq=0\r\n",

                    (unsigned)FOC_TEST_RUN_WARMUP_MS);

#endif

}



#if (FOC_TEST_MODE == FOC_TEST_MODE_SPEED)



static void Test_ApplySpeedTarget(float rpm)

{

    FOC_MOTOR.target_speed = rpm;

}



#if FOC_TEST_SPEED_STEP_ENABLE



static void Test_StepTargetInit(void)

{

    s_step_target_rpm = FOC_TEST_SPEED_STEP_MIN_RPM;

    s_step_since_ms   = HAL_GetTick();

    Test_ApplySpeedTarget(s_step_target_rpm);

}



/** 阶梯给定：20→21→…→40→20 循环 */

static void Test_StepTargetPoll(void)

{

    uint32_t now = HAL_GetTick();



    if ((now - s_step_since_ms) < (uint32_t)FOC_TEST_SPEED_STEP_HOLD_MS)

    {

        return;

    }



    s_step_since_ms = now;

    s_step_target_rpm += FOC_TEST_SPEED_STEP_DELTA;



    if (s_step_target_rpm > FOC_TEST_SPEED_STEP_MAX_RPM)

    {

        s_step_target_rpm = FOC_TEST_SPEED_STEP_MIN_RPM;

    }



    Test_ApplySpeedTarget(s_step_target_rpm);

    FOC_Test_Printf("[TEST] Step target -> %.0f RPM\r\n", s_step_target_rpm);

}



#endif /* FOC_TEST_SPEED_STEP_ENABLE */



static void Test_ArmSpeedLoop(void)

{

    FOC_Gimbal_SpeedArm(&FOC_MOTOR);

    FOC_MOTOR.pid_speed.P = FOC_SPEED_PI_P;

    FOC_MOTOR.pid_speed.I = FOC_SPEED_PI_I;



#if FOC_TEST_SPEED_STEP_ENABLE

    Test_StepTargetInit();

    FOC_Test_Printf("[TEST] Speed loop ON  step %.0f->%.0f RPM\r\n",

                    FOC_TEST_SPEED_STEP_MIN_RPM,

                    FOC_TEST_SPEED_STEP_MAX_RPM);

#else

    Test_ApplySpeedTarget(FOC_TEST_TARGET_RPM);

    FOC_Test_Printf("[TEST] Speed loop ON  target=%.1f RPM\r\n", FOC_TEST_TARGET_RPM);

#endif

}



static void Test_PrintSpeedLoop(void)
{
    const FOC_AS5048A_t *enc = &FOC_MOTOR.enc;
    float tgt = FOC_MOTOR.target_speed;
    float avr = enc->speed.AvrMecSpeed;

    /* VOFA FireWater 默认双通道：I0=目标  I1=实测（勿加通道名前缀） */
    FOC_Test_Printf("%.2f,%.2f\n", tgt, avr);
}



#endif /* FOC_TEST_MODE_SPEED */



#if (FOC_TEST_MODE == FOC_TEST_MODE_POSITION)



static void Test_ApplyPitchTarget(float pitch_deg)
{
    FOC_Gimbal_SetTargetPitch(&FOC_MOTOR, pitch_deg);
}

#if FOC_TEST_POS_RAMP_ENABLE

static void Test_RampPitchInit(void)
{
    s_ramp_pitch     = FOC_TEST_POS_RAMP_MIN_DEG;
    s_ramp_last_ms   = HAL_GetTick();
    s_ramp_dir       = 1;
    Test_ApplyPitchTarget(s_ramp_pitch);
}

/** 线性斜坡（main 轮询，FOC_TEST_POS_RAMP_UPDATE_MS 更新目标，与位置环频率无关） */
static void Test_RampPitchPoll(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms;
    float    dt_s;
    float    delta;

    dt_ms = now - s_ramp_last_ms;
    if (dt_ms < (uint32_t)FOC_TEST_POS_RAMP_UPDATE_MS)
    {
        return;
    }

    s_ramp_last_ms = now;
    dt_s           = (float)dt_ms * 0.001f;
    delta          = FOC_TEST_POS_RAMP_RATE_DPS * dt_s * (float)s_ramp_dir;
    s_ramp_pitch  += delta;

    if (s_ramp_pitch >= FOC_TEST_POS_RAMP_MAX_DEG)
    {
        s_ramp_pitch = FOC_TEST_POS_RAMP_MAX_DEG;
        s_ramp_dir   = -1;
    }
    else if (s_ramp_pitch <= FOC_TEST_POS_RAMP_MIN_DEG)
    {
        s_ramp_pitch = FOC_TEST_POS_RAMP_MIN_DEG;
        s_ramp_dir   = 1;
    }

    Test_ApplyPitchTarget(s_ramp_pitch);
}

#endif /* FOC_TEST_POS_RAMP_ENABLE */

static void Test_ArmPositionLoop(void)

{

    FOC_Gimbal_PosArm(&FOC_MOTOR, FOC_TEST_TARGET_PITCH_DEG);

    FOC_MOTOR.pid_speed.P = FOC_SPEED_PI_P;

    FOC_MOTOR.pid_speed.I = FOC_SPEED_PI_I;

#if FOC_TEST_POS_RAMP_ENABLE
    Test_RampPitchInit();
    FOC_Test_Printf("[TEST] Position loop ON  ramp %.0f->%.0f @ %.1f deg/s\r\n",
                    FOC_TEST_POS_RAMP_MIN_DEG,
                    FOC_TEST_POS_RAMP_MAX_DEG,
                    FOC_TEST_POS_RAMP_RATE_DPS);
#else
    FOC_Test_Printf("[TEST] Position loop ON  pitch tgt=%.1f deg\r\n",
                    FOC_MOTOR.target_pitch);
#endif

}



static void Test_PrintPositionLoop(void)

{

    float pitch_fb = FOC_Gimbal_GetPitchFb();
    float tgt      = FOC_MOTOR.target_pitch;

    FOC_Test_Printf("%.2f,%.2f\n", tgt, pitch_fb);

}



#endif /* FOC_TEST_MODE_POSITION */



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



#if (FOC_TEST_MODE == FOC_TEST_MODE_ANGLE)



        case TEST_ANGLE_ONLY:

            Test_ForceZeroTorque();

            if (FOC_MOTOR.motor_state != MOTOR_GIMBAL_RUN) { break; }

            if ((now - s_last_print_ms) >= (uint32_t)FOC_TEST_PRINT_MS)

            {

                s_last_print_ms = now;

                Test_PrintMecDegVofa(&FOC_MOTOR.enc);

            }

            break;



#elif (FOC_TEST_MODE == FOC_TEST_MODE_POSITION)



        case TEST_RUN_WARMUP:

            Test_ForceZeroTorque();

            if (FOC_MOTOR.motor_state != MOTOR_GIMBAL_RUN) { break; }



            if ((now - s_phase_since_ms) >= (uint32_t)FOC_TEST_RUN_WARMUP_MS)

            {

                Test_ArmPositionLoop();

                s_phase         = TEST_POS_LOOP;

                s_last_print_ms = now;

            }

            break;



        case TEST_POS_LOOP:

#if FOC_TEST_POS_RAMP_ENABLE
            Test_RampPitchPoll();
#endif
            if ((now - s_last_print_ms) >= (uint32_t)FOC_TEST_POS_PRINT_MS)

            {

                s_last_print_ms = now;

                Test_PrintPositionLoop();

            }

            break;



#else /* FOC_TEST_MODE_SPEED */



        case TEST_RUN_WARMUP:

            Test_ForceZeroTorque();

            if (FOC_MOTOR.motor_state != MOTOR_GIMBAL_RUN) { break; }



            if ((now - s_phase_since_ms) >= (uint32_t)FOC_TEST_RUN_WARMUP_MS)

            {

                Test_ArmSpeedLoop();

                s_phase         = TEST_SPEED_LOOP;

                s_last_print_ms = now;

            }

            break;



        case TEST_SPEED_LOOP:

#if FOC_TEST_SPEED_STEP_ENABLE

            Test_StepTargetPoll();

#endif

            if ((now - s_last_print_ms) >= (uint32_t)FOC_TEST_PRINT_MS)

            {

                s_last_print_ms = now;

                Test_PrintSpeedLoop();

            }

            break;



#endif



        case TEST_FAULT_HOLD:

        default:

            break;

    }

}

