#include "main.h"
#include "imu.h"
#include "motion_fx_app.h"
#include "motion_fx.h"
#include "diag_log.h"
#include "wing_control.h"
#include "servo.h"
#include "obd.h"
#include "uart_log.h"
#include <assert.h>
#include <float.h>
#include <string.h>

extern MFX_output_t host_mfx_output;
extern uint32_t host_mfx_calls, host_mfx_initializations;
extern size_t host_mfx_size;

static uint8_t maximum_log;
void __real_OBD_GetDiagSnapshot(OBD_Diag_t *);
void __real_IMU_GetDiagSnapshot(IMU_Diag_t *);
void __real_Control_GetDiagSnapshot(Control_Diag_t *);
uint8_t __real_MotionFxApp_GetLatest(MotionFxApp_Result_t *);

void __wrap_OBD_GetDiagSnapshot(OBD_Diag_t *out)
{
    if (!maximum_log) { __real_OBD_GetDiagSnapshot(out); return; }
    memset(out, 0xff, sizeof(*out));
    out->speed_age_ms = 0x80000000U;
}

void __wrap_IMU_GetDiagSnapshot(IMU_Diag_t *out)
{
    if (!maximum_log) { __real_IMU_GetDiagSnapshot(out); return; }
    memset(out, 0xff, sizeof(*out));
    out->age_ms = 0x80000000U;
    out->ax_mg = out->ay_mg = out->az_mg = INT32_MIN;
    out->gx_mdps = out->gy_mdps = out->gz_mdps = INT32_MIN;
    out->lateral_raw_mg = out->lateral_filtered_mg = INT32_MIN;
}

void __wrap_Control_GetDiagSnapshot(Control_Diag_t *out)
{
    if (!maximum_log) { __real_Control_GetDiagSnapshot(out); return; }
    memset(out, 0xff, sizeof(*out));
    out->rate_x10 = -100000;
    out->accel_mg = INT32_MIN;
    out->wing_state = WING_AIR_BRAKE;
}

uint8_t __wrap_MotionFxApp_GetLatest(MotionFxApp_Result_t *out)
{
    if (!maximum_log) return __real_MotionFxApp_GetLatest(out);
    *out = (MotionFxApp_Result_t){
        .gravity_g = {-2000000, -2000000, -2000000},
        .linear_g = {-2000000, -2000000, -2000000},
        .gyro_dps = {-2000000, -2000000, -2000000},
        .yaw_deg = -2000000, .pitch_deg = -2000000, .roll_deg = -2000000,
        .gyro_y_integral_deg = -2000000,
        .acc_longitudinal_g = -2000000, .acc_lateral_g = -2000000,
        .acc_vertical_g = -2000000,
        .acc_longitudinal_filtered_g = -2000000, .acc_lateral_filtered_g = -2000000,
        .timestamp_ms = host_tick, .sample_id = UINT32_MAX, .valid = 1U
    };
    return 1U;
}

static MotionFxApp_Result_t latest(void)
{
    MotionFxApp_Result_t out;
    MotionFxApp_GetLatest(&out);
    return out;
}

static void output(float x, float y, float z)
{
    host_mfx_output.linear_acceleration[0] = x;
    host_mfx_output.linear_acceleration[1] = y;
    host_mfx_output.linear_acceleration[2] = z;
}

static void sample(uint32_t tick)
{
    host_tick = tick;
    MotionFxApp_Process(0, 0, 1, 0, 0, 0, tick);
}

static void start_fx(void)
{
    assert(MotionFxApp_Init());
    output(0.5f, 1.0f, 0.25f);
    sample(0U);
    assert(!latest().valid);
    sample(20U);
    assert(latest().valid);
}

static void raw_mapping(void)
{
    IMU_Acceleration_t out;
    assert(!IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &out));
    assert(!out.sample_id && out.age_ms == UINT32_MAX);
    host_tick = 42U;
    Host_FeedIMU(-16384);
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &out));
    assert(out.acc_longitudinal_mg == 61 && out.acc_lateral_mg == -999);
    assert(out.acc_vertical_mg == 999 && out.source == ACC_SOURCE_RAW_IMU);
    assert(out.timestamp_ms == 42U && out.age_ms == 0U && out.sample_id == 1U);
    for (unsigned i = 0; i < 10U; i++)
    {
        IMU_Task();
        assert(IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &out));
        assert(out.sample_id == 1U);
    }
    host_tick = 42U + APP_IMU_MAX_AGE_MS;
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &out));
    host_tick++;
    assert(!IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &out));
    assert(out.sample_id == 1U && out.timestamp_ms == 42U);
    assert(out.age_ms == APP_IMU_MAX_AGE_MS + 1U && !out.acc_lateral_mg);
    Host_FeedIMU(16384);
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &out));
    assert(out.sample_id == 2U && out.acc_lateral_mg == 999);
    assert(!IMU_GetAccelerationSnapshot(NULL));
    assert(!IMU_GetAccelerationFromSource(255U, &out) && !out.valid);
}

static void fx_mapping(void)
{
    start_fx();
    IMU_Acceleration_t out;
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
    assert(out.acc_longitudinal_mg == 1000 && out.acc_lateral_mg == -500);
    assert(out.acc_vertical_mg == 250 && out.timestamp_ms == 20U);
    assert(out.source == ACC_SOURCE_MOTIONFX && out.sample_id == 1U);
    MotionFxApp_Result_t fx = latest();
    assert(fx.acc_longitudinal_g == 1.0f && fx.acc_lateral_g == -0.5f);
    assert(fx.acc_longitudinal_filtered_g == 1.0f);
    assert(fx.acc_lateral_filtered_g == -0.5f);
    assert(fx.linear_g[0] == 0.5f && fx.linear_g[1] == 1.0f);
}

static void filter(void)
{
    start_fx();
    output(-0.5f, -1.0f, 0.75f);
    sample(40U);
    IMU_Acceleration_t out;
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
#if !MOTIONFX_FILTER_ENABLE
    assert(out.acc_longitudinal_mg == -1000 && out.acc_lateral_mg == 500);
#elif MOTIONFX_FILTER_DIVISOR == 4
    assert(out.acc_longitudinal_mg == 500 && out.acc_lateral_mg == -250);
#elif MOTIONFX_FILTER_DIVISOR == 2
    assert(out.acc_longitudinal_mg == 0 && out.acc_lateral_mg == 0);
#else
#error "missing filter expectation"
#endif
    assert(out.acc_vertical_mg == 750);
    MotionFxApp_Result_t before = latest();
    for (unsigned i = 0; i < 100U; i++)
    {
        assert(IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
        MotionFxApp_Result_t after = latest();
        assert(after.sample_id == before.sample_id && after.timestamp_ms == 40U);
        assert(after.acc_longitudinal_filtered_g == before.acc_longitudinal_filtered_g);
        assert(after.acc_lateral_filtered_g == before.acc_lateral_filtered_g);
    }
    assert(host_mfx_calls == 2U);
}

static void selected_source(void)
{
    start_fx();
    host_tick = 40U;
    Host_FeedIMU(-16384);
    IMU_Acceleration_t selected, explicit;
    assert(IMU_GetAccelerationSnapshot(&selected));
    assert(IMU_GetAccelerationFromSource(APP_ACC_SOURCE, &explicit));
    assert(selected.source == APP_ACC_SOURCE && selected.sample_id == explicit.sample_id);
    assert(selected.acc_longitudinal_mg == explicit.acc_longitudinal_mg);
    assert(selected.acc_lateral_mg == explicit.acc_lateral_mg);
}

static void stale_recovery(void)
{
    start_fx();
    output(-0.5f, -1.0f, 0);
    sample(40U);
    host_tick = 40U + APP_MOTIONFX_MAX_AGE_MS;
    IMU_Acceleration_t out;
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
    host_tick++;
    assert(!IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
    assert(out.sample_id == 2U && out.timestamp_ms == 40U);
    assert(out.age_ms == APP_MOTIONFX_MAX_AGE_MS + 1U && !out.acc_longitudinal_mg);
    sample(host_tick + 20U);
    assert(!latest().valid && latest().sample_id == 2U);
    sample(host_tick + 20U);
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
    assert(out.sample_id == 3U && out.acc_longitudinal_mg == -1000);
    assert(out.acc_lateral_mg == 500 && host_mfx_initializations == 1U);
}

static void invalid_recovery(void)
{
    start_fx();
    const float bad[] = {NAN, INFINITY, -INFINITY, FLT_MAX, -2147483.75f};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
    {
        MotionFxApp_Result_t before = latest();
        output(bad[i], 0.0f, 0.0f);
        sample(host_tick + 20U);
        assert(!latest().valid && latest().sample_id == before.sample_id);
        assert(latest().timestamp_ms == before.timestamp_ms);
        output(-0.5f, -1.0f, 0);
        sample(host_tick + 20U);
        assert(latest().valid && latest().acc_longitudinal_filtered_g == -1.0f);
        assert(latest().acc_lateral_filtered_g == 0.5f);
    }
}

static void reinitialize(void)
{
    start_fx();
    uint32_t previous_id = latest().sample_id;
    host_mfx_size = 5000U;
    assert(!MotionFxApp_Init());
    IMU_Acceleration_t out;
    assert(!IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
    assert(!out.sample_id && out.age_ms == UINT32_MAX);
    host_mfx_size = 100U;
    assert(MotionFxApp_Init());
    output(-0.5f, -1.0f, 0);
    sample(40U);
    assert(!latest().valid);
    sample(60U);
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
    assert(out.sample_id != previous_id && out.acc_longitudinal_mg == -1000);
    assert(out.acc_lateral_mg == 500);
}

static void duplicate(void)
{
    start_fx();
    sample(20U);
    assert(!latest().valid && latest().sample_id == 1U && host_mfx_calls == 1U);
    output(-0.5f, -1.0f, 0);
    sample(40U);
    assert(latest().valid && latest().sample_id == 2U);
    assert(latest().acc_longitudinal_filtered_g == -1.0f);
}

static void imu_once(void)
{
    assert(MotionFxApp_Init());
    host_tick = 0U; Host_FeedIMU(0);
    host_tick = 20U; Host_FeedIMU(0);
    assert(host_mfx_calls == 1U && latest().sample_id == 1U);
    for (unsigned i = 0; i < 20U; i++) IMU_Task();
    assert(host_mfx_calls == 1U && latest().sample_id == 1U);
}

static void unavailable(void)
{
    host_mfx_size = 5000U;
    assert(!MotionFxApp_Init());
    host_tick = 100U; Host_FeedIMU(0);
    IMU_Acceleration_t out;
    assert(!IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
    assert(!out.sample_id && out.age_ms == UINT32_MAX && !out.acc_lateral_mg);
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &out));
    uint8_t valid = IMU_GetAccelerationSnapshot(&out);
    assert(out.source == APP_ACC_SOURCE);
    assert(valid == (APP_ACC_SOURCE == ACC_SOURCE_RAW_IMU));
}

static void tick_wrap(void)
{
    assert(MotionFxApp_Init());
    output(0.5f, 1.0f, 0);
    sample(UINT32_MAX - 29U);
    sample(UINT32_MAX - 9U);
    sample(10U);
    IMU_Acceleration_t out;
    host_tick = 110U;
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
    assert(out.age_ms == 100U && out.sample_id == 2U);
    host_tick++;
    assert(!IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &out));
    host_tick = UINT32_MAX - 2U; Host_FeedIMU(0);
    host_tick = 97U;
    assert(IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &out) && out.age_ms == 100U);
    host_tick++;
    assert(!IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &out));
}

static void control_unchanged(void)
{
    MotionFxApp_Init();
    output(1.0f, -1.0f, 0);
    for (uint32_t t = 0U; t <= 1400U; t += 50U)
    {
        host_tick = t; Host_FeedIMU(0); Host_QueueSpeed(60U); Host_MainLoopOnce();
    }
    Control_Diag_t ctrl;
    Control_GetDiagSnapshot(&ctrl);
    assert(ctrl.wing_state == WING_OPEN && !IMU_IsCornerDetected());
    assert(Servo_GetCurrentPulseUs() == 1900U);
    for (uint32_t t = 1450U; t <= 1850U; t += 50U)
    {
        host_tick = t; Host_FeedIMU(0);
        Host_QueueSpeed((uint8_t)(60U - (t - 1400U) / 50U)); Host_MainLoopOnce();
    }
    Control_GetDiagSnapshot(&ctrl);
    assert(ctrl.wing_state == WING_AIR_BRAKE && Servo_GetCurrentPulseUs() == 1100U);
    output(0, 0, 0);
    for (uint32_t t = 1870U; t <= 2270U; t += 20U)
    {
        host_tick = t; Host_FeedIMU(16384); Host_QueueSpeed(60U); Host_MainLoopOnce();
    }
    Control_GetDiagSnapshot(&ctrl);
    assert(IMU_IsCornerDetected() && ctrl.wing_state == WING_NORMAL);
    assert(Servo_GetCurrentPulseUs() == 1600U);
}

static void logger(void)
{
    host_uart_mirror = 1U;
    start_fx();
    host_tick = 100U; Host_FeedIMU(-16384); DiagLog_Task();
    for (unsigned i = 0; i < 10U; i++) DiagLog_Task();
    Host_UART_Drain();
    host_tick = 201U; DiagLog_Task(); Host_UART_Drain();
    UARTLog_Diag_t d;
    UARTLog_GetDiagSnapshot(&d);
    assert(!d.dropped_count && !d.error_count);
}

static void logger_maximum(void)
{
    host_uart_mirror = 1U;
    host_tick = UINT32_MAX;
    Host_FeedIMU(0);
    maximum_log = 1U;
    DiagLog_Task();
    Host_UART_Drain();
    UARTLog_Diag_t d;
    UARTLog_GetDiagSnapshot(&d);
    assert(!d.dropped_count && !d.error_count);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *name = argv[1];
    if (!strcmp(name, "raw_mapping")) raw_mapping();
    else if (!strcmp(name, "fx_mapping")) fx_mapping();
    else if (!strcmp(name, "filter")) filter();
    else if (!strcmp(name, "selected_source")) selected_source();
    else if (!strcmp(name, "stale_recovery")) stale_recovery();
    else if (!strcmp(name, "invalid_recovery")) invalid_recovery();
    else if (!strcmp(name, "reinitialize")) reinitialize();
    else if (!strcmp(name, "duplicate")) duplicate();
    else if (!strcmp(name, "imu_once")) imu_once();
    else if (!strcmp(name, "unavailable")) unavailable();
    else if (!strcmp(name, "tick_wrap")) tick_wrap();
    else if (!strcmp(name, "control_unchanged")) control_unchanged();
    else if (!strcmp(name, "logger")) logger();
    else if (!strcmp(name, "logger_maximum")) logger_maximum();
    else assert(0);
    fputs("PASS: ", stdout); fputs(name, stdout); fputc('\n', stdout);
    return 0;
}
