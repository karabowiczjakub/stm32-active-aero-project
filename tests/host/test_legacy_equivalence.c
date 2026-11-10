#include "main.h"
#include "wing_control.h"
#include "imu.h"
#include "servo.h"
#include "motion_fx_app.h"
#include "motion_fx.h"
#include <string.h>

extern MFX_output_t host_mfx_output;

int main(int argc, char **argv)
{
    uint32_t origin = argc > 1 && !strcmp(argv[1], "wrap") ? UINT32_MAX - 6000U : 0U;
    uint32_t random = 1327U;
    MotionFxApp_Init();
    Servo_SetWingState(WING_NORMAL);
    for (uint32_t t = 0U; t <= 60000U; t += 10U)
    {
        host_tick = origin + t;
        uint32_t phase = t % 12000U;
        random = random * 1664525U + 1013904223U;
        if (t % 20U == 0U && !(phase >= 8000U && phase < 8300U))
        {
            host_mfx_output.linear_acceleration[1] = phase < 2000U ? -0.4f : 0.2f;
            host_mfx_output.linear_acceleration[0] = phase >= 3000U && phase < 3400U ? -0.3f : 0;
            Host_FeedIMU(phase >= 4000U && phase < 4600U ? 5000 : 0);
        }
        if (t % 50U == 0U && !(phase >= 6500U && phase < 7300U) &&
            !(phase >= 9000U && phase < 9600U && t % 150U != 0U))
        {
            uint8_t speed = phase < 2500U ? 60U : phase < 3500U ?
                (uint8_t)(80U - (phase - 2500U) / 50U) : phase < 5000U ? 70U :
                phase < 6000U ? 45U : phase < 10000U ? 65U : (uint8_t)(random % 90U);
            Host_QueueSpeed(speed);
        }
        Host_MainLoopOnce();
        Control_Diag_t d;
        Control_GetDiagSnapshot(&d);
        printf("EQUIV,%lu,%u,%u,%u,%u,%u,%ld,%ld,%lu,%u,%u,%lu,%u,%lu,%u,%lu\n",
               (unsigned long)t, d.wing_state, d.motion_state, d.brake_detected,
               d.rate_valid, d.failsafe_flags, (long)d.rate_x10, (long)d.accel_mg,
               (unsigned long)d.filter_span_ms, d.filter_samples, d.open_confirm_active,
               (unsigned long)d.open_remaining_ms, d.post_brake_hold_active,
               (unsigned long)d.post_remaining_ms, Servo_GetCurrentPulseUs(),
               (unsigned long)host_pwm_writes);
    }
    return 0;
}
