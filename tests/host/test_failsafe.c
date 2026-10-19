#include "main.h"
#include "app_config.h"
#include "wing_control.h"
#include "imu.h"
#include "obd.h"
#include "servo.h"
#include <assert.h>
#include <string.h>

static uint32_t origin;

static Control_Diag_t control(void)
{
    Control_Diag_t out;
    Control_GetDiagSnapshot(&out);
    return out;
}

static OBD_Diag_t obd(void)
{
    OBD_Diag_t out;
    OBD_GetDiagSnapshot(&out);
    return out;
}

static void at(uint32_t elapsed)
{
    host_tick = origin + elapsed;
}

static void sample(uint32_t elapsed, uint8_t speed, uint8_t imu)
{
    at(elapsed);
    if (imu) Host_FeedIMU(0);
    Host_QueueSpeed(speed);
    Host_MainLoopOnce();
}

static void normal(void)
{
    Control_Diag_t c = control();
    assert(c.wing_state == WING_NORMAL);
    assert(Servo_GetCurrentPulseUs() == 1600U);
    assert(!c.open_confirm_active && !c.post_brake_hold_active);
}

static void open_wing(void)
{
    for (uint32_t t = 0; t <= 1250U; t += 50U) sample(t, 60U, 1U);
    assert(control().wing_state == WING_OPEN && control().rate_valid);
    assert(Servo_GetCurrentPulseUs() == 1900U);
    assert(!control().failsafe_flags);
}

static void brake_wing(void)
{
    for (uint32_t t = 0; t <= 500U; t += 50U)
        sample(t, (uint8_t)(80U - t / 50U), 1U);
    assert(control().wing_state == WING_AIR_BRAKE && control().rate_valid);
    assert(Servo_GetCurrentPulseUs() == 1100U);
}

static void no_obd(void)
{
    uint32_t writes = 0U;
    for (uint32_t t = 0; t <= 2000U; t += 10U)
    {
        at(t);
        if (t % 20U == 0U) Host_FeedIMU(0);
        if (t % 50U == 0U) HAL_TIM_PeriodElapsedCallback(&htim6);
        Host_MainLoopOnce();
        normal();
        assert(control().failsafe_flags & CONTROL_FAULT_OBD_MISSING);
        if (t == 0U) writes = host_pwm_writes;
        assert(host_pwm_writes == writes);
    }
    assert(!obd().speed_valid && obd().response_timeout_count > 0U);
    assert(writes > 0U);
}

static void obd_loss(const char *state)
{
    uint32_t last = 500U;
    if (!strcmp(state, "open")) { open_wing(); last = 1250U; }
    else if (!strcmp(state, "brake")) brake_wing();
    else for (uint32_t t = 0; t <= last; t += 50U) sample(t, 40U, 1U);
    WingState_t previous = control().wing_state;
    at(last + APP_OBD_SPEED_MAX_AGE_MS);
    Host_FeedIMU(0);
    Host_MainLoopOnce();
    assert(control().wing_state == previous);
    at(last + APP_OBD_SPEED_MAX_AGE_MS + 1U);
    Host_FeedIMU(0);
    Host_MainLoopOnce();
    normal();
    assert(control().failsafe_flags == (CONTROL_FAULT_OBD_STALE | CONTROL_FAULT_RATE));
    assert(!control().rate_valid && !control().filter_samples);
    uint32_t writes = host_pwm_writes;
    for (unsigned k = 0; k < 10U; k++) Host_MainLoopOnce();
    assert(host_pwm_writes == writes);
}

static void queued_frame(uint32_t delay)
{
    at(0U);
    Host_QueueSpeed(80U);
    at(delay);
    Host_FeedIMU(0);
    Host_MainLoopOnce();
    normal();
    if (delay > APP_OBD_SPEED_MAX_AGE_MS)
    {
        assert(!obd().speed_valid && !obd().speed_rx_count);
        assert(control().failsafe_flags & CONTROL_FAULT_OBD_MISSING);
    }
    else
    {
        assert(obd().speed_valid && obd().speed_age_ms == delay);
        assert(obd().speed_rx_count == 1U);
    }
}

static void stale_frame_in_open(void)
{
    open_wing();
    uint32_t count = obd().speed_rx_count;
    at(1300U);
    Host_QueueSpeed(0U);
    at(1300U + APP_OBD_SPEED_MAX_AGE_MS + 1U);
    Host_FeedIMU(0);
    Host_MainLoopOnce();
    normal();
    assert(obd().speed_rx_count == count && obd().speed_kmh == 60U);
    assert(obd().speed_age_ms == APP_OBD_SPEED_MAX_AGE_MS + 51U);
}

static void queued_timestamps(void)
{
    for (uint32_t t = 0; t <= 1250U; t += 50U)
    {
        uint32_t delay = ((t / 50U) % 5U) * 2U;
        at(t); Host_QueueSpeed(60U);
        at(t + delay); Host_FeedIMU(0); Host_MainLoopOnce();
        assert(obd().speed_age_ms == delay);
        if (t >= 300U) assert(control().filter_span_ms == 300U);
        assert(control().wing_state == (t < 1250U ? WING_NORMAL : WING_OPEN));
    }
    assert(control().filter_span_ms == 300U && control().filter_samples == 7U);
}

static void no_imu(void)
{
    for (uint32_t t = 0; t <= 2000U; t += 50U)
    {
        sample(t, (uint8_t)(100U - t / 50U), 0U);
        normal();
        assert(control().failsafe_flags & CONTROL_FAULT_IMU_MISSING);
    }
    assert(obd().speed_valid && !control().filter_samples);
}

static void imu_loss(uint8_t corner)
{
    open_wing();
    uint32_t last = 1250U;
    if (corner)
    {
        for (uint32_t t = 1270U; t <= 1310U; t += 20U)
        { at(t); Host_FeedIMU(16384); Host_MainLoopOnce(); }
        assert(IMU_IsCornerDetected());
        normal();
        last = 1310U;
    }
    sample(last + APP_IMU_MAX_AGE_MS, 60U, 0U);
    assert(!(control().failsafe_flags & CONTROL_FAULT_IMU_STALE));
    sample(last + APP_IMU_MAX_AGE_MS + 1U, 60U, 0U);
    normal();
    assert(control().failsafe_flags == (CONTROL_FAULT_IMU_STALE | CONTROL_FAULT_RATE));
    sample(last + APP_IMU_MAX_AGE_MS + 51U, 40U, 0U);
    normal();
}

static void invalid_rate(uint8_t brake)
{
    if (brake) brake_wing(); else open_wing();
    uint32_t last = brake ? 500U : 1250U;
    uint8_t speed = brake ? 70U : 60U;
    for (uint32_t t = last + 1U; t <= last + 8U; t++) sample(t, speed, 1U);
    normal();
    assert(!control().rate_valid && control().failsafe_flags == CONTROL_FAULT_RATE);
    for (uint32_t t = last + 58U; t <= last + 258U; t += 50U) sample(t, speed, 1U);
    assert(control().rate_valid && control().open_confirm_active);
    assert(!control().failsafe_flags);
}

static void confirmation_gap(uint8_t long_gap)
{
    for (uint32_t t = 0; t <= 250U; t += 50U) sample(t, 60U, 1U);
    assert(control().open_confirm_active);
    uint32_t resume = 600U;
    if (long_gap)
    {
        at(250U + APP_OBD_SPEED_MAX_AGE_MS + 1U);
        Host_FeedIMU(0); Host_MainLoopOnce();
        normal();
        resume = 250U + APP_OBD_SPEED_MAX_AGE_MS + 100U;
    }
    sample(resume, 60U, 1U);
    normal();
    assert(!control().rate_valid);
    for (uint32_t t = resume + 50U; t <= resume + 250U; t += 50U) sample(t, 60U, 1U);
    assert(control().rate_valid && control().open_confirm_active);
    for (uint32_t t = resume + 300U; t < resume + 1250U; t += 50U)
    { sample(t, 60U, 1U); assert(control().wing_state == WING_NORMAL); }
    sample(resume + 1250U, 60U, 1U);
    assert(control().wing_state == WING_OPEN);
}

static void recovery(uint8_t imu_lost)
{
    open_wing();
    uint32_t resume;
    if (imu_lost)
    {
        sample(1250U + APP_IMU_MAX_AGE_MS + 1U, 60U, 0U);
        resume = 1250U + APP_IMU_MAX_AGE_MS + 50U;
    }
    else
    {
        at(1250U + APP_OBD_SPEED_MAX_AGE_MS + 1U);
        Host_FeedIMU(0); Host_MainLoopOnce();
        resume = 1250U + APP_OBD_SPEED_MAX_AGE_MS + 50U;
    }
    normal();
    for (uint32_t t = resume; t <= resume + 250U; t += 50U)
    {
        sample(t, 60U, 1U);
        assert(control().rate_valid == (t == resume + 250U));
        assert(control().wing_state == WING_NORMAL);
    }
    assert(control().filter_span_ms == 250U && control().filter_samples == 6U);
    assert(control().open_confirm_active && !control().failsafe_flags);
    for (uint32_t t = resume + 300U; t < resume + 1250U; t += 50U)
    { sample(t, 60U, 1U); assert(control().wing_state == WING_NORMAL); }
    sample(resume + 1250U, 60U, 1U);
    assert(control().wing_state == WING_OPEN);
}

static void imu_corner_recovery(void)
{
    for (uint32_t t = 0; t <= 40U; t += 20U)
    { at(t); Host_FeedIMU(16384); Host_MainLoopOnce(); }
    assert(IMU_IsCornerDetected());
    at(40U + APP_IMU_MAX_AGE_MS + 1U); Host_MainLoopOnce();
    at(40U + APP_IMU_MAX_AGE_MS + 20U); Host_FeedIMU(0);
    assert(!IMU_IsCornerDetected());
    IMU_Diag_t i; IMU_GetDiagSnapshot(&i);
    assert(i.lateral_filtered_mg == 0 && !i.corner_on_count && !i.corner_off_count);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *name = argv[1];
    Servo_SetWingState(WING_NORMAL);
    if (!strcmp(name, "obd_wrap") || !strcmp(name, "recovery_wrap")) origin = UINT32_MAX - 1300U;
    if (!strcmp(name, "imu_wrap")) origin = UINT32_MAX - 1350U;
    if (!strcmp(name, "queue_wrap_stale")) origin = UINT32_MAX - 20U;
    if (!strcmp(name, "no_obd")) no_obd();
    else if (!strcmp(name, "no_imu")) no_imu();
    else if (!strcmp(name, "obd_normal")) obd_loss("normal");
    else if (!strcmp(name, "obd_open") || !strcmp(name, "obd_wrap")) obd_loss("open");
    else if (!strcmp(name, "obd_brake")) obd_loss("brake");
    else if (!strcmp(name, "queue_stale") || !strcmp(name, "queue_wrap_stale")) queued_frame(APP_OBD_SPEED_MAX_AGE_MS + 1U);
    else if (!strcmp(name, "queue_limit")) queued_frame(APP_OBD_SPEED_MAX_AGE_MS);
    else if (!strcmp(name, "queue_wrap_fresh")) { origin = UINT32_MAX - 20U; queued_frame(30U); }
    else if (!strcmp(name, "stale_frame_in_open")) stale_frame_in_open();
    else if (!strcmp(name, "queued_timestamps")) queued_timestamps();
    else if (!strcmp(name, "imu_straight") || !strcmp(name, "imu_wrap")) imu_loss(0U);
    else if (!strcmp(name, "imu_corner")) imu_loss(1U);
    else if (!strcmp(name, "invalid_open")) invalid_rate(0U);
    else if (!strcmp(name, "invalid_brake")) invalid_rate(1U);
    else if (!strcmp(name, "confirmation_gap")) confirmation_gap(0U);
    else if (!strcmp(name, "confirmation_long_gap")) confirmation_gap(1U);
    else if (!strcmp(name, "recovery_obd") || !strcmp(name, "recovery_wrap")) recovery(0U);
    else if (!strcmp(name, "recovery_imu")) recovery(1U);
    else if (!strcmp(name, "imu_corner_recovery")) imu_corner_recovery();
    else assert(!"unknown test");
    printf("PASS: %s\n", name);
    return 0;
}
