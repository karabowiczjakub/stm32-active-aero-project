#include "main.h"
#include "wing_control.h"
#include "motion_fx_app.h"
#include "imu.h"
#include "servo.h"
#include "diag_log.h"
#include "uart_log.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static MotionFxApp_Result_t injected;
static uint32_t t, origin, sequence, fx_period=20U;
static uint8_t speed=60U, raw_enabled=1U, obd_enabled=1U, fx_enabled=1U, frozen, slowing, accelerating;
static float longitudinal, lateral;
static int16_t raw_lateral;

#ifdef HOST_COMPARE_MAX
static uint8_t maximum_diag;
void __real_Control_GetCompareSnapshot(Control_CompareDiag_t *out);
void __wrap_Control_GetCompareSnapshot(Control_CompareDiag_t *out)
{
    __real_Control_GetCompareSnapshot(out);
    if (!maximum_diag) return;
    out->wing_legacy=out->wing_hybrid=WING_AIR_BRAKE;
    out->brake_legacy=out->brake_hybrid=out->corner_raw=1U;
    out->faults_legacy=out->faults_hybrid=out->faults_fx=UINT8_MAX;
    out->phase=CONTROL_HYBRID_OBD;
    out->fx_armed=out->on_pending=out->off_pending=1U;
    out->wait_remaining_ms=out->fx_early_count=out->obd_confirm_count=out->fx_reject_count=UINT32_MAX;
}
void __real_UARTLog_GetDiagSnapshot(UARTLog_Diag_t *out);
void __wrap_UARTLog_GetDiagSnapshot(UARTLog_Diag_t *out)
{
    __real_UARTLog_GetDiagSnapshot(out);
    if (!maximum_diag) return;
    out->dropped_count=out->error_count=out->busy_count=UINT32_MAX;
    out->queued=UART_LOG_QUEUE_SIZE;
}
#endif

uint8_t __wrap_MotionFxApp_GetLatest(MotionFxApp_Result_t *out)
{
    *out=injected;
#if !MOTIONFX_DIAG_ENABLE
    out->valid=0U;
#endif
    return out->valid;
}

static Control_CompareDiag_t compare(void)
{
    Control_CompareDiag_t d;
    Control_GetCompareSnapshot(&d);
    return d;
}

static void output(void)
{
    Control_Diag_t actual;
    Control_GetDiagSnapshot(&actual);
    Control_CompareDiag_t d=compare();
#if APP_WING_CONTROL_MODE == WING_CONTROL_HYBRID_ACTIVE
    assert(actual.wing_state==d.wing_hybrid);
#else
    assert(actual.wing_state==d.wing_legacy);
#endif
    assert(Servo_GetCurrentPulseUs()==(actual.wing_state==WING_OPEN ? 1900U :
                                     actual.wing_state==WING_AIR_BRAKE ? 1100U : 1600U));
}

static void tick(void)
{
    host_tick=origin+t;
    if (raw_enabled && t%20U==0U) Host_FeedIMU(raw_lateral);
    if (fx_enabled && t%fx_period==0U && !frozen)
    {
        if (++sequence==0U) sequence=1U;
        injected=(MotionFxApp_Result_t){.linear_g={-lateral,longitudinal,0},
            .acc_longitudinal_g=longitudinal,.acc_lateral_g=lateral,
            .acc_longitudinal_filtered_g=longitudinal,.acc_lateral_filtered_g=lateral,
            .timestamp_ms=host_tick,.sample_id=sequence,.valid=1U};
    }
    if (obd_enabled && t%50U==0U)
    {
        if (slowing) speed=(uint8_t)(60U-(t-600U)/100U);
        if (accelerating) speed=(uint8_t)(60U+(t-600U)/100U);
        Host_QueueSpeed(speed);
    }
    Host_MainLoopOnce();
    output();
}

static void until(uint32_t end)
{
    while(t<end) {t++;tick();}
}

static void boot(void)
{
    Servo_SetWingState(WING_NORMAL);tick();until(500U);
}

static void early(void)
{
    assert(compare().fx_armed);
    uint32_t entry=520U+HYBRID_FX_ON_CONFIRM_MS;
    longitudinal=-.2f;until(entry-1U);
    assert(compare().wing_hybrid==WING_NORMAL);
    until(entry);
    assert(compare().wing_hybrid==WING_AIR_BRAKE);
    assert(compare().phase==CONTROL_HYBRID_WAIT_OBD);
    assert(compare().wing_legacy==WING_NORMAL);
}

static void confirmed(void)
{
    early();slowing=1U;until(900U);
    assert(compare().phase==CONTROL_HYBRID_OBD);
    assert(compare().obd_confirm_count==1U);
}

static void run_case(const char *name)
{
    boot();
    if (!strcmp(name,"early")) {early();return;}
    if (!strcmp(name,"rejection"))
    {
        early();until(520U+HYBRID_FX_ON_CONFIRM_MS+HYBRID_OBD_CONFIRM_MS-1U);assert(compare().wing_hybrid==WING_AIR_BRAKE);
        until(520U+HYBRID_FX_ON_CONFIRM_MS+HYBRID_OBD_CONFIRM_MS);assert(compare().wing_hybrid==WING_NORMAL);
        until(2000U);assert(compare().fx_early_count==1U && compare().fx_reject_count==1U);
        longitudinal=0;until(2120U);assert(compare().fx_armed);
        longitudinal=-.2f;until(2240U);assert(compare().fx_early_count==2U);return;
    }
    if (!strcmp(name,"pulse") || !strcmp(name,"short_series"))
    {
        longitudinal=-.4f;until(!strcmp(name,"pulse") ? 520U : 560U);
        longitudinal=0;until(1000U);assert(compare().fx_early_count==0U);return;
    }
    if (!strcmp(name,"duplicate"))
    {
        longitudinal=-.2f;until(520U);frozen=1U;until(620U);
        assert(compare().fx_early_count==0U);return;
    }
    if (!strcmp(name,"interval"))
    {
        fx_period=40U;uint32_t entry=520U+HYBRID_FX_ON_CONFIRM_MS;
    longitudinal=-.2f;until(entry-1U);assert(compare().fx_early_count==0U);
        until(600U);assert(compare().fx_early_count==1U);return;
    }
    if (!strcmp(name,"gap"))
    {
        longitudinal=-.2f;until(560U);sequence+=2U;until(1000U);
        assert(compare().fx_early_count==0U);return;
    }
    if (!strcmp(name,"obd_only") || !strcmp(name,"missing_fx"))
    {
        fx_enabled=0U;slowing=1U;until(1000U);
        assert(compare().phase==CONTROL_HYBRID_OBD && compare().fx_early_count==0U);return;
    }
    if (!strcmp(name,"fx_wait_loss"))
    {
        early();fx_enabled=0U;until(701U);
        assert(compare().wing_hybrid==WING_NORMAL && compare().fx_reject_count==1U);return;
    }
    if (!strcmp(name,"imu_wait_loss"))
    {
        early();raw_enabled=0U;until(701U);assert(compare().wing_hybrid==WING_NORMAL);return;
    }
    if (!strcmp(name,"speed"))
    {
        early();speed=29U;until(650U);assert(compare().wing_hybrid==WING_NORMAL);return;
    }
    if (!strcmp(name,"accel_veto"))
    {
        longitudinal=.2f;until(600U);accelerating=1U;until(1000U);
        longitudinal=-.3f;until(1300U);accelerating=0U;until(2000U);
        assert(compare().fx_early_count==0U && !compare().fx_armed);return;
    }
    if (!strcmp(name,"wait_nan") || !strcmp(name,"wait_inf"))
    {
        early();longitudinal=!strcmp(name,"wait_nan") ? NAN : INFINITY;
        until(620U);assert(compare().wing_hybrid==WING_NORMAL && compare().fx_reject_count==1U);return;
    }
    confirmed();
    if (!strcmp(name,"takeover"))
    {
        longitudinal=0;until(1500U);assert(compare().phase==CONTROL_HYBRID_OBD);return;
    }
    if (!strcmp(name,"fx_loss_confirmed"))
    {
        fx_enabled=0U;until(1100U);assert(compare().phase==CONTROL_HYBRID_OBD);return;
    }
    if (!strcmp(name,"nan") || !strcmp(name,"inf") || !strcmp(name,"ninf"))
    {
        longitudinal=!strcmp(name,"nan") ? NAN : !strcmp(name,"inf") ? INFINITY : -INFINITY;
        until(1100U);assert(compare().phase==CONTROL_HYBRID_OBD && compare().faults_fx);return;
    }
    if (!strcmp(name,"raw_corner"))
    {
        raw_lateral=16384;until(1100U);
        assert(compare().corner_raw && compare().wing_hybrid==WING_NORMAL && compare().wing_legacy==WING_NORMAL);return;
    }
    if (!strcmp(name,"fx_lateral"))
    {
        lateral=1.0f;until(1100U);
        assert(!compare().corner_raw && compare().phase==CONTROL_HYBRID_OBD);return;
    }
    if (!strcmp(name,"obd_loss"))
    {
        obd_enabled=0U;until(1401U);assert(compare().wing_hybrid==WING_NORMAL);return;
    }
    if (!strcmp(name,"imu_loss"))
    {
        raw_enabled=0U;until(1001U);assert(compare().wing_hybrid==WING_NORMAL);return;
    }
    if (!strcmp(name,"end") || !strcmp(name,"partial") || !strcmp(name,"logger") || !strcmp(name,"tail") || !strcmp(name,"off_duplicate"))
    {
        until(1500U);slowing=0U;
        if (strcmp(name,"tail")) longitudinal=0;
        if (!strcmp(name,"off_duplicate"))
        {
            until(1750U);assert(compare().off_pending);obd_enabled=0U;
            until(1950U);assert(compare().phase==CONTROL_HYBRID_OBD);return;
        }
        if (!strcmp(name,"partial"))
        {
            until(1700U);slowing=1U;until(1900U);
            assert(compare().phase==CONTROL_HYBRID_OBD);return;
        }
        until(1900U);assert(compare().wing_hybrid==WING_NORMAL);
        until(2600U);assert(compare().wing_hybrid==WING_NORMAL);return;
    }
    assert(!"unknown scenario");
}

int main(int argc,char **argv)
{
    assert(argc==2);
    const char *name=argv[1];
    if (!strcmp(name,"logger")) host_uart_mirror=1U;
    if (!strcmp(name,"sequence_wrap")) {sequence=UINT32_MAX-30U;name="rejection";}
    if (!strcmp(name,"wrap")) {origin=UINT32_MAX-550U;name="rejection";}
    if (!strcmp(name,"long_run"))
    {
        Servo_SetWingState(WING_NORMAL);tick();
        while(t<600000U)
        {
            t++;
            uint32_t phase=t%10000U;
            longitudinal=phase>=1000U && phase<3000U ? -.2f :
                         phase>=4000U && phase<6000U ? .2f : 0;
            speed=phase<1000U ? 60U : phase<3000U ? (uint8_t)(60U-(phase-1000U)*7U/1000U) :
                  phase<4000U ? 46U : phase<6000U ? (uint8_t)(46U+(phase-4000U)*7U/1000U) : 60U;
            raw_lateral=phase>=6000U && phase<8000U ? 8000 : 0;
            tick();
            if (phase==3500U || phase==6500U) assert(compare().wing_hybrid==WING_NORMAL);
        }
        assert(compare().fx_early_count==60U && compare().fx_reject_count==0U);
    }
#ifdef HOST_COMPARE_MAX
    else if (!strcmp(name,"logger_maximum"))
    {
        boot();host_uart_mirror=1U;maximum_diag=1U;
        host_tick=UINT32_MAX;injected.timestamp_ms=host_tick;injected.sample_id=UINT32_MAX;
        DiagLog_Task();maximum_diag=0U;
    }
#endif
    else if (!strcmp(name,"mode"))
    {
        boot();longitudinal=-.2f;until(650U);
#if APP_WING_CONTROL_MODE == WING_CONTROL_LEGACY_ONLY
        assert(compare().wing_hybrid==WING_NORMAL);
#else
        assert(compare().wing_hybrid==WING_AIR_BRAKE);
#endif
    }
    else run_case(name);
    printf("PASS: %s\n",argv[1]);
    Host_UART_Drain();return 0;
}
