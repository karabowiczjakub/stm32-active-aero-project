#include "main.h"
#include "app_config.h"
#include "wing_control.h"
#include "imu.h"
#include "servo.h"
#include "obd.h"
#include "lap_sim.h"
#include "lap_data.h"
#include <assert.h>
#include <math.h>
#include <string.h>

static Control_Diag_t control(void)
{
    Control_Diag_t result;
    Control_GetDiagSnapshot(&result);
    return result;
}

static void normal_trace(void)
{
    Servo_SetWingState(WING_NORMAL);
    for (uint32_t t = 0U; t <= 10000U; t += 10U)
    {
        host_tick = t;
        if (t % 20U == 0U) Host_FeedIMU(t >= 3000U && t < 3600U ? 16384 : 0);
        if (t % 50U == 0U)
        {
            uint8_t speed = t < 2000U ? 80U : t < 3000U ?
                (uint8_t)(80U - (t-2000U)/50U) : t < 7000U ? 60U : 40U;
            Host_QueueSpeed(speed);
            HAL_TIM_PeriodElapsedCallback(&htim6);
        }
        Host_MainLoopOnce();
        Control_Diag_t c = control();
        IMU_Diag_t imu;
        IMU_GetDiagSnapshot(&imu);
        printf("TRACE,%u,%u,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%d\n", t,
            c.wing_state, c.rate_x10, c.accel_mg, c.rate_valid, c.filter_span_ms,
            c.filter_samples, c.open_confirm_active, c.open_remaining_ms,
            c.post_brake_hold_active, c.post_remaining_ms, imu.corner,
            imu.lateral_filtered_mg);
    }
    assert(host_can_tx > 0U && host_fx_calls == 501U);
}

#if APP_LAP_SIM_ENABLED
static LapSim_Diag_t simulation(void)
{
    LapSim_Diag_t result;
    LapSim_GetDiagSnapshot(&result);
    return result;
}
static void reset_control(void)
{
    Control_Reset();
    IMU_ResetCornerDetection();
}
static void speed_at(uint8_t speed, uint32_t sample, uint32_t actual_delay)
{
    host_tick = sample + actual_delay;
    Control_Update_From_SpeedAt(speed, sample);
}
static void open_at_1250(uint32_t delay)
{
    for (uint32_t t=0U; t<=1250U; t+=50U)
    {
        speed_at(60U, t, delay);
        Control_Diag_t c = control();
        assert(c.rate_valid == (t >= 250U));
        assert(c.wing_state == (t < 1250U ? WING_NORMAL : WING_OPEN));
        if (t >= 300U) assert(c.filter_span_ms == 300U && c.filter_samples == 7U);
    }
    assert(Servo_GetCurrentPulseUs() == 1900U);
}
static void regression(void)
{
    reset_control();
    for (uint32_t t=0U; t<=500U; t+=50U)
    {
        speed_at((uint8_t)(80U-t/50U),t,7U);
        if (t >= 250U)
        {
            Control_Diag_t c=control();
            assert(c.rate_valid && c.rate_x10 == -200);
            assert(c.wing_state == WING_AIR_BRAKE && c.brake_detected);
            assert(Servo_GetCurrentPulseUs() == 1100U);
        }
    }
}
static void hysteresis(void)
{
    reset_control();
    open_at_1250(7U);
    /* Test the 45/55 speed boundaries with an already valid zero trend. */
    reset_control();
    for (uint32_t t=0U;t<=2000U;t+=50U) speed_at(54U,t,0U);
    assert(control().wing_state == WING_NORMAL);
    reset_control();
    for (uint32_t t=0U;t<=1250U;t+=50U) speed_at(55U,t,0U);
    assert(control().wing_state == WING_OPEN);
    /* Descend slowly enough to stay above the brake threshold. */
    for (uint32_t t=1300U;t<=6250U;t+=50U)
        speed_at((uint8_t)(55U-(t-1300U)/500U),t,0U);
    assert(control().wing_state == WING_OPEN);
    speed_at(45U,6300U,0U);
    assert(control().wing_state == WING_NORMAL);
}
static void brake_end(void)
{
    reset_control();
    for (uint32_t t=0U;t<=500U;t+=50U) speed_at((uint8_t)(80U-t/50U),t,0U);
    assert(control().wing_state == WING_AIR_BRAKE);
    uint32_t exit_ms=0U;
    for (uint32_t t=550U;t<=1000U;t+=50U)
    {
        speed_at(70U,t,0U);
        if (control().wing_state == WING_NORMAL) { exit_ms=t; break; }
    }
    assert(exit_ms && control().post_brake_hold_active);
    for (uint32_t t=exit_ms+50U;t<=exit_ms+1000U;t+=50U)
    {
        speed_at(70U,t,0U);
        assert(control().wing_state == WING_NORMAL);
        assert(!control().open_confirm_active);
    }
    assert(!control().post_brake_hold_active);
    for (uint32_t t=exit_ms+1050U;t<=exit_ms+2000U;t+=50U)
    {
        speed_at(70U,t,0U);
        assert(control().wing_state == WING_NORMAL);
    }
    speed_at(70U,exit_ms+2050U,0U);
    assert(control().wing_state == WING_OPEN);
}
static void brake_hysteresis(void)
{
    reset_control();
    const uint8_t initial[] = {80U,80U,79U,79U,79U,79U,78U};
    for (uint32_t i=0U;i<7U;i++) speed_at(initial[i],i*50U,0U);
    assert(control().rate_x10 == -57 && control().wing_state == WING_AIR_BRAKE);
    for (uint32_t t=350U;t<=500U;t+=50U)
    {
        speed_at(78U,t,0U);
        assert(control().wing_state == WING_AIR_BRAKE);
    }
    assert(control().rate_x10 == -35);
    speed_at(78U,550U,0U);
    assert(control().rate_x10 == -21 && control().wing_state == WING_NORMAL);
    assert(control().post_brake_hold_active);

    reset_control();
    for (uint32_t t=0U;t<=250U;t+=50U) speed_at((uint8_t)(35U-t/50U),t,0U);
    assert(control().wing_state == WING_AIR_BRAKE);
    speed_at(29U,300U,0U);
    assert(control().wing_state == WING_NORMAL && !control().brake_detected);
}
static void corner(void)
{
    reset_control();
    open_at_1250(0U);
    for (unsigned i=0;i<3;i++) IMU_ProcessLateralSample_mG(-1000);
    assert(IMU_IsCornerDetected() && IMU_TakeCornerDetectedEvent());
    Control_ForceNormalForCorner();
    assert(control().wing_state == WING_NORMAL && Servo_GetCurrentPulseUs() == 1600U);
    for (uint32_t t=1300U;t<=1800U;t+=50U) speed_at((uint8_t)(80U-(t-1300U)/50U),t,0U);
    assert(control().wing_state == WING_NORMAL);
    IMU_ResetCornerDetection();
    regression();
    for (unsigned i=0;i<3;i++) IMU_ProcessLateralSample_mG(1000);
    Control_ForceNormalForCorner();
    assert(control().wing_state == WING_NORMAL);
    /* Exact thresholds and 3/5 confirmation, with a fresh first-sample filter. */
    IMU_ResetCornerDetection();
    IMU_ProcessLateralSample_mG(200);
    IMU_ProcessLateralSample_mG(200);
    assert(!IMU_IsCornerDetected());
    IMU_ProcessLateralSample_mG(200);
    assert(IMU_IsCornerDetected());
    for (unsigned i=0;i<5;i++) IMU_ProcessLateralSample_mG(0);
    assert(IMU_IsCornerDetected());
    IMU_ProcessLateralSample_mG(0);
    assert(!IMU_IsCornerDetected());
    IMU_ResetCornerDetection();
    IMU_ProcessLateralSample_mG(-2101);
    IMU_Diag_t d;
    IMU_GetDiagSnapshot(&d);
    assert(d.lateral_raw_mg == -2101 && d.lateral_filtered_mg == -2101);
}
static void physical_isolation(void)
{
    reset_control();
    IMU_ProcessLateralSample_mG(0);
    for (unsigned i=0;i<12;i++) { host_tick+=20U; Host_FeedIMU(16384); }
    assert(!IMU_IsCornerDetected());
    IMU_Diag_t d;
    IMU_GetDiagSnapshot(&d);
    assert(d.ay_mg == 999 && d.lateral_filtered_mg == 0 && d.valid);
    assert(host_fx_calls == 12U);
    assert(fabsf(host_fx_acc[1]-0.999f)<0.00001f);
    assert(fabsf(host_fx_gyro[0]-0.875f)<0.00001f);
    for (unsigned i=0;i<3;i++) IMU_ProcessLateralSample_mG(1000);
    for (unsigned i=0;i<12;i++) { host_tick+=20U; Host_FeedIMU(0); }
    assert(IMU_IsCornerDetected());
    assert(host_fx_calls == 24U && host_fx_acc[1] == 0.0f);
}
static void start_button(uint32_t origin)
{
    host_tick=origin;
    LapSim_Init();
    assert(simulation().state == LAP_SIM_WAITING && control().wing_state == WING_NORMAL);
    Host_MainLoopOnce();
    assert(simulation().lateral_samples == 0U);
    BSP_PB_Callback(BUTTON_USER);
    assert(simulation().state == LAP_SIM_WAITING);
    Host_MainLoopOnce();
    assert(simulation().state == LAP_SIM_RUNNING);
    assert(simulation().lateral_samples == 1U && simulation().speed_samples == 1U);
}
static void full_lap(uint32_t origin)
{
    start_button(origin);
    for (uint32_t t=0U;t<=LAP_DATA_END_MS;t++)
    {
        host_tick=origin+t;
        if (t == 500U) BSP_PB_Callback(BUTTON_USER);
        if (t%50U == 0U) HAL_TIM_PeriodElapsedCallback(&htim6);
        Host_MainLoopOnce();
        LapSim_Diag_t d=simulation();
        assert(d.lateral_samples == t/20U+1U && d.speed_samples == t/50U+1U);
        assert(d.lateral_sample_ms == (t/20U)*20U);
        assert(d.speed_sample_ms == (t/50U)*50U);
        assert(d.speed_kmh == lap_data[d.speed_sample_ms/20U].speed_kmh);
        assert(d.lateral_mg == lap_data[t/20U].lateral_mg);
        assert(d.longitudinal_mg == lap_data[t/20U].longitudinal_mg);
        assert(d.quality == ((lap_data[t/20U].quality & 2U) |
                            (lap_data[d.speed_sample_ms/20U].quality & 1U)));
        assert(!d.timing_error && d.lateness_ms == 0U);
        Host_MainLoopOnce();
        assert(simulation().lateral_samples == d.lateral_samples);
        assert(simulation().speed_samples == d.speed_samples);
    }
    LapSim_Diag_t d=simulation();
    assert(d.state == LAP_SIM_FINISHED && d.lateral_samples == 5226U && d.speed_samples == 2091U);
    assert(control().wing_state == WING_NORMAL && !control().rate_valid);
    assert(!IMU_IsCornerDetected() && Servo_GetCurrentPulseUs() == 1600U);
    BSP_PB_Callback(BUTTON_USER);
    host_tick+=500U;
    Host_MainLoopOnce();
    assert(simulation().state == LAP_SIM_FINISHED && simulation().lateral_samples == 5226U);
    assert(host_can_tx == 0U && host_fx_calls == 0U);
}
static void small_delay(uint32_t delay)
{
    start_button(1000U);
    for (uint32_t t=10U;t<=3000U;t+=10U)
    {
        host_tick=1000U+t+delay;
        Host_MainLoopOnce();
        LapSim_Diag_t d=simulation();
        assert(d.state == LAP_SIM_RUNNING && !d.timing_error);
        assert(d.lateral_samples == t/20U+1U && d.speed_samples == t/50U+1U);
        assert(d.lateness_ms == (t < 20U ? 0U : delay));
        Control_Diag_t c=control();
        printf("DELAY_TRACE,%u,%u,%d,%u,%u,%u,%u,%u,%u\n",t,c.wing_state,
               c.rate_x10,c.rate_valid,c.filter_span_ms,c.filter_samples,
               c.open_confirm_active,c.post_brake_hold_active,IMU_IsCornerDetected());
    }
    assert(control().rate_valid);
}
static void timing_error(uint32_t lateness)
{
    start_button(1000U);
    for (uint32_t t=1U;t<10000U && control().wing_state != WING_OPEN;t++)
    { host_tick=1000U+t; Host_MainLoopOnce(); }
    assert(control().wing_state == WING_OPEN);
    LapSim_Diag_t before=simulation();
    uint32_t next_lat=before.lateral_samples*20U;
    uint32_t next_speed=before.speed_samples*50U;
    uint32_t next=next_lat<next_speed ? next_lat : next_speed;
    uint32_t writes=host_pwm_writes;
    host_tick=1000U+next+lateness;
    Host_MainLoopOnce();
    LapSim_Diag_t after=simulation();
    assert(after.state == LAP_SIM_FINISHED && after.timing_error);
    assert(after.lateral_samples == before.lateral_samples && after.speed_samples == before.speed_samples);
    assert(control().wing_state == WING_NORMAL && Servo_GetCurrentPulseUs() == 1600U);
    assert(host_pwm_writes == writes+1U);
    host_tick+=200U;
    BSP_PB_Callback(BUTTON_USER);
    Host_MainLoopOnce();
    assert(simulation().state == LAP_SIM_FINISHED);
}
static void timing_limit(void)
{
    start_button(1000U);
    host_tick=1020U+APP_LAP_SIM_MAX_LATENESS_MS;
    Host_MainLoopOnce();
    assert(simulation().state == LAP_SIM_RUNNING && simulation().lateral_samples == 2U);
    assert(simulation().lateness_ms == APP_LAP_SIM_MAX_LATENESS_MS);
}
#endif

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1],"normal_trace")) normal_trace();
#if APP_LAP_SIM_ENABLED
    else if (!strcmp(argv[1],"regression")) regression();
    else if (!strcmp(argv[1],"hysteresis")) hysteresis();
    else if (!strcmp(argv[1],"brake_end")) brake_end();
    else if (!strcmp(argv[1],"brake_hysteresis")) brake_hysteresis();
    else if (!strcmp(argv[1],"corner")) corner();
    else if (!strcmp(argv[1],"physical_isolation")) physical_isolation();
    else if (!strcmp(argv[1],"full_lap")) full_lap(1234U);
    else if (!strcmp(argv[1],"tick_wrap")) full_lap(UINT32_MAX-500U);
    else if (!strcmp(argv[1],"small_delay")) small_delay(7U);
    else if (!strcmp(argv[1],"no_delay")) small_delay(0U);
    else if (!strcmp(argv[1],"timing_error")) timing_error(APP_LAP_SIM_MAX_LATENESS_MS+1U);
    else if (!strcmp(argv[1],"large_delay")) timing_error(200U);
    else if (!strcmp(argv[1],"timing_limit")) timing_limit();
#endif
    else assert(!"unknown test");
    printf("PASS: %s\n",argv[1]);
    return 0;
}
