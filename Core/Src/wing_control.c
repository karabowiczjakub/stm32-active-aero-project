/*
 * wing_control.c
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */


#include "wing_control.h"
#include "app_config.h"
#include <string.h>

#include "main.h"
#include "app_types.h"
#include "servo.h"
#include "imu.h"
#include "obd.h"
#include "debug_utils.h"

#include <stdio.h>



/* =========================
 * ALGORYTM
 * =========================
 */

#define OPEN_ON_SPEED_KMH             55U
#define OPEN_OFF_SPEED_KMH            45U

#define ACCEL_THRESHOLD_X10           25

#define BRAKE_ON_THRESHOLD_X10        55
#define BRAKE_OFF_THRESHOLD_X10       30

#define OPEN_DECEL_BLOCK_X10          20

#define AIR_BRAKE_MIN_SPEED_KMH       30U

#define OPEN_CONFIRM_MS               1000U
#define POST_BRAKE_NORMAL_MS          1000U

#define SPEED_FILTER_WINDOW_MS        300U
#define SPEED_FILTER_TOLERANCE_MS      30U
#define SPEED_FILTER_MIN_SPAN_MS      250U
#define SPEED_HISTORY_SIZE              8U
typedef enum
{
    MOTION_ACCELERATION = 0,

    MOTION_COASTING,

    MOTION_BREAKING

} MotionState_t;






static MotionState_t motion_state =
    MOTION_COASTING;


typedef struct
{
    WingState_t wing_state;
    WingState_t reported_state;
    uint8_t post_brake_hold_active, open_confirm_active;
    uint32_t post_brake_start_tick, open_confirm_start_tick;
} WingControlState_t;

typedef struct
{
    uint32_t start_tick;
    uint8_t active;
} ControlConfirmation_t;

static WingControlState_t legacy_state, hybrid_state;
static ControlConfirmation_t hybrid_on, hybrid_rearm, hybrid_off;
static uint8_t hybrid_brake_detected, hybrid_fault_flags, hybrid_fx_faults, hybrid_fx_armed;
static Control_HybridPhase_t hybrid_phase;
static uint32_t hybrid_wait_tick, hybrid_last_sample_id, hybrid_last_sample_tick, hybrid_last_obd_count;
static uint32_t hybrid_early_count, hybrid_confirm_count, hybrid_reject_count;

static void Control_ApplyOutput(void);
static void Control_BlockHybrid(const char *reason);
static void Control_ResetHybrid(const char *reason);
static void Control_CommitState(WingControlState_t *state, const char *path, const char *reason);
static void Control_UpdateState(WingControlState_t *state, uint8_t speed_kmh,
                                uint32_t now, uint8_t brake_detected, uint8_t open_allowed_by_decel);

static uint8_t control_fault_flags = 0U;
static uint8_t diag_rate_valid = 0U;
static int32_t diag_rate_x10 = 0;
static int32_t diag_accel_mg = 0;
static uint32_t diag_filter_span_ms = 0U;
static uint8_t diag_filter_samples = 0U;
static uint8_t diag_brake_detected = 0U;


typedef struct
{
    uint8_t speed_kmh;
    uint32_t tick_ms;

} SpeedSample_t;


static SpeedSample_t speed_history[SPEED_HISTORY_SIZE];

static uint8_t speed_history_head = 0U;
static uint8_t speed_history_count = 0U;




static void SpeedHistory_Add(
    uint8_t speed_kmh,
    uint32_t tick_ms)
{
    speed_history[speed_history_head].speed_kmh =
        speed_kmh;

    speed_history[speed_history_head].tick_ms =
        tick_ms;

    speed_history_head =
        (uint8_t)(
            (speed_history_head + 1U) %
            SPEED_HISTORY_SIZE
        );

    if (speed_history_count < SPEED_HISTORY_SIZE)
    {
        speed_history_count++;
    }
}

static uint8_t SpeedHistory_CalculateRateX10(
    int32_t *rate_x10,
    uint32_t *span_ms,
    uint8_t *used_samples)
{
    if (speed_history_count < 2U)
    {
        return 0U;
    }

    uint8_t newest_index =
        (uint8_t)(
            (speed_history_head +
             SPEED_HISTORY_SIZE - 1U) %
            SPEED_HISTORY_SIZE
        );

    uint32_t newest_tick =
        speed_history[newest_index].tick_ms;


    uint8_t first_pos = 0xFFU;

    for (uint8_t pos = 0U;
         pos < speed_history_count;
         pos++)
    {
        uint8_t index =
            (uint8_t)(
                (speed_history_head +
                 SPEED_HISTORY_SIZE -
                 speed_history_count +
                 pos) %
                SPEED_HISTORY_SIZE
            );

        uint32_t age_ms =
            newest_tick -
            speed_history[index].tick_ms;

        if (age_ms <=
            (SPEED_FILTER_WINDOW_MS +
             SPEED_FILTER_TOLERANCE_MS))
        {
            first_pos = pos;
            break;
        }
    }

    if (first_pos == 0xFFU)
    {
        return 0U;
    }

    uint8_t oldest_index =
        (uint8_t)(
            (speed_history_head +
             SPEED_HISTORY_SIZE -
             speed_history_count +
             first_pos) %
            SPEED_HISTORY_SIZE
        );

    uint32_t oldest_tick =
        speed_history[oldest_index].tick_ms;

    uint32_t actual_span_ms =
        newest_tick - oldest_tick;

    if (actual_span_ms < SPEED_FILTER_MIN_SPAN_MS)
    {
        return 0U;
    }

    /*
     * Regresja liniowa w postaci
     *
     * v(t) = a*t + b
     *
     * Interesuje nas wspolczynnik a,
     * czyli sredni trend predkosci.
     */
    int64_t sum_t = 0;
    int64_t sum_v = 0;
    int64_t sum_tv = 0;
    int64_t sum_t2 = 0;

    uint8_t n = 0U;

    for (uint8_t pos = first_pos;
         pos < speed_history_count;
         pos++)
    {
        uint8_t index =
            (uint8_t)(
                (speed_history_head +
                 SPEED_HISTORY_SIZE -
                 speed_history_count +
                 pos) %
                SPEED_HISTORY_SIZE
            );

        int64_t t =
            (int64_t)(
                speed_history[index].tick_ms -
                oldest_tick
            );

        int64_t v =
            (int64_t)
                speed_history[index].speed_kmh;

        sum_t += t;
        sum_v += v;
        sum_tv += t * v;
        sum_t2 += t * t;

        n++;
    }

    if (n < 4U)
    {
        return 0U;
    }

    int64_t numerator =
        ((int64_t)n * sum_tv) -
        (sum_t * sum_v);

    int64_t denominator =
        ((int64_t)n * sum_t2) -
        (sum_t * sum_t);

    if (denominator == 0)
    {
        return 0U;
    }


    *rate_x10 =
        (int32_t)(
            (numerator * 10000LL) /
            denominator
        );

    *span_ms = actual_span_ms;
    *used_samples = n;

    return 1U;
}




static int32_t RateX10_To_mG(
    int32_t rate_x10
)
{
    return (rate_x10 * 1000L) / 353L;
}




void Control_GetDiagSnapshot(Control_Diag_t *out)
{
    if (out == NULL) return;
    uint32_t now = HAL_GetTick();
    out->wing_state = legacy_state.wing_state;
    out->motion_state = (uint8_t)motion_state;
    out->brake_detected = diag_brake_detected;
    out->rate_valid = diag_rate_valid;
    out->failsafe_flags = control_fault_flags;
    out->rate_x10 = diag_rate_x10;
    out->accel_mg = diag_accel_mg;
    out->filter_span_ms = diag_filter_span_ms;
    out->filter_samples = diag_filter_samples;
    out->open_confirm_active = legacy_state.open_confirm_active;
    out->open_remaining_ms = legacy_state.open_confirm_active ?
        ((now - legacy_state.open_confirm_start_tick) >= OPEN_CONFIRM_MS ? 0U :
         OPEN_CONFIRM_MS - (now - legacy_state.open_confirm_start_tick)) : 0U;
    out->post_brake_hold_active = legacy_state.post_brake_hold_active;
    out->post_remaining_ms = legacy_state.post_brake_hold_active ?
        ((now - legacy_state.post_brake_start_tick) >= POST_BRAKE_NORMAL_MS ? 0U :
         POST_BRAKE_NORMAL_MS - (now - legacy_state.post_brake_start_tick)) : 0U;
#if !APP_LAP_SIM_ENABLED && APP_WING_CONTROL_MODE == WING_CONTROL_HYBRID_ACTIVE
    out->wing_state = hybrid_state.wing_state;
    out->brake_detected = hybrid_brake_detected;
    out->motion_state = hybrid_brake_detected ? MOTION_BREAKING :
        diag_rate_x10 >= ACCEL_THRESHOLD_X10 ? MOTION_ACCELERATION : MOTION_COASTING;
    out->failsafe_flags = hybrid_fault_flags;
    out->open_confirm_active = hybrid_state.open_confirm_active;
    out->open_remaining_ms = hybrid_state.open_confirm_active &&
        (uint32_t)(now - hybrid_state.open_confirm_start_tick) < OPEN_CONFIRM_MS ?
        OPEN_CONFIRM_MS - (now - hybrid_state.open_confirm_start_tick) : 0U;
    out->post_brake_hold_active = hybrid_state.post_brake_hold_active;
    out->post_remaining_ms = hybrid_state.post_brake_hold_active &&
        (uint32_t)(now - hybrid_state.post_brake_start_tick) < POST_BRAKE_NORMAL_MS ?
        POST_BRAKE_NORMAL_MS - (now - hybrid_state.post_brake_start_tick) : 0U;
#endif

}

void Control_ForceNormalForCorner(void)
{
    legacy_state.wing_state = WING_NORMAL;

    legacy_state.open_confirm_active = 0U;
    legacy_state.post_brake_hold_active = 0U;

    Control_CommitState(&legacy_state, "LEGACY", "corner_raw");
#if !APP_LAP_SIM_ENABLED
    Control_BlockHybrid("corner_raw");
#endif
    Control_ApplyOutput();
}

/*
 * Logika stanow:
 *
 * 1. Mocne hamowanie -> AIR_BRAKE
 * 2. Koniec hamowania -> od razu NORMAL
 * 3. Po AIR_BRAKE blokujemy OPEN przez kilka probek
 * 4. OPEN wlaczamy dopiero po kilku probkach bez zwalniania
 */


void Control_Update_From_Speed(uint8_t speed_kmh)
{
    Control_Update_From_SpeedAt(speed_kmh, HAL_GetTick());
}

static void Control_ResetWithReason(const char *reason)
{
    memset(speed_history, 0, sizeof(speed_history));
    speed_history_head = 0U;
    speed_history_count = 0U;
    motion_state = MOTION_COASTING;
    legacy_state.wing_state = WING_NORMAL;
    legacy_state.post_brake_hold_active = 0U;
    legacy_state.post_brake_start_tick = 0U;
    legacy_state.open_confirm_active = 0U;
    legacy_state.open_confirm_start_tick = 0U;
    diag_rate_valid = 0U;
    diag_rate_x10 = 0;
    diag_accel_mg = 0;
    diag_filter_span_ms = 0U;
    diag_filter_samples = 0U;
    diag_brake_detected = 0U;
    Control_CommitState(&legacy_state, "LEGACY", reason);
    Control_ResetHybrid(reason);
    Control_ApplyOutput();
}

void Control_Reset(void)
{
    Control_ResetWithReason("reset");
}

#if !APP_LAP_SIM_ENABLED
static void Control_SetFaults(uint8_t faults)
{
    if (faults != control_fault_flags)
    {
        control_fault_flags = faults;
        printf("FAILSAFE flags=%u\r\n", (unsigned)faults);
    }
}
#endif

void Control_CheckDataFreshness(void)
{
#if !APP_LAP_SIM_ENABLED
    OBD_Diag_t obd;
    IMU_Diag_t imu;
    OBD_GetDiagSnapshot(&obd);
    IMU_GetDiagSnapshot(&imu);
    uint8_t faults = 0U;
    const uint8_t source_mask = CONTROL_FAULT_OBD_MISSING | CONTROL_FAULT_OBD_STALE |
                                CONTROL_FAULT_IMU_MISSING | CONTROL_FAULT_IMU_STALE;

    if (!obd.speed_valid) faults |= CONTROL_FAULT_OBD_MISSING;
    else if (obd.speed_age_ms > APP_OBD_SPEED_MAX_AGE_MS) faults |= CONTROL_FAULT_OBD_STALE;
    if (!imu.valid) faults |= CONTROL_FAULT_IMU_MISSING;
    else if (imu.age_ms > APP_IMU_MAX_AGE_MS) faults |= CONTROL_FAULT_IMU_STALE;

    if (faults != 0U && (control_fault_flags & source_mask) == 0U)
    {
        Control_ResetWithReason(faults & (CONTROL_FAULT_OBD_MISSING | CONTROL_FAULT_OBD_STALE) ?
                                "obd_data" : "imu_data");
    }
    if (!diag_rate_valid) faults |= CONTROL_FAULT_RATE;
    Control_SetFaults(faults);
#endif
}

void Control_Update_From_SpeedAt(uint8_t speed_kmh, uint32_t now)
{
#if !APP_LAP_SIM_ENABLED
    Control_CheckDataFreshness();
    if (control_fault_flags & (CONTROL_FAULT_OBD_MISSING | CONTROL_FAULT_OBD_STALE |
                              CONTROL_FAULT_IMU_MISSING | CONTROL_FAULT_IMU_STALE))
    {
        return;
    }
#endif


    SpeedHistory_Add(
        speed_kmh,
        now
    );


    int32_t rate_x10 = 0;
    uint32_t filter_span_ms = 0U;
    uint8_t filter_samples = 0U;



    uint8_t rate_valid =
        SpeedHistory_CalculateRateX10(
            &rate_x10,
            &filter_span_ms,
            &filter_samples
        );


    diag_rate_valid = rate_valid;
    diag_rate_x10 = rate_x10;
    diag_accel_mg = rate_valid ? RateX10_To_mG(rate_x10) : 0;
    diag_filter_span_ms = filter_span_ms;
    diag_filter_samples = filter_samples;
    diag_brake_detected = 0U;
#if !APP_LAP_SIM_ENABLED
    if (rate_valid) Control_SetFaults(0U);
#endif


    /*
     * ==========================================
     * ZAKRET - NAJWYZSZY PRIORYTET
     * ==========================================
     */
    if (IMU_IsCornerDetected())
    {
        motion_state = MOTION_COASTING;

        legacy_state.wing_state = WING_NORMAL;

        legacy_state.open_confirm_active = 0U;
        legacy_state.post_brake_hold_active = 0U;

        Control_CommitState(&legacy_state, "LEGACY", "corner_raw");
        Control_ApplyOutput();

        return;
    }



    if (rate_valid == 0U)
    {
        motion_state = MOTION_COASTING;
        legacy_state.wing_state = WING_NORMAL;
        legacy_state.open_confirm_active = 0U;
        legacy_state.post_brake_hold_active = 0U;
        Control_CommitState(&legacy_state, "LEGACY", "rate_invalid");
        Control_ApplyOutput();
#if !APP_LAP_SIM_ENABLED
        Control_SetFaults(CONTROL_FAULT_RATE);
#endif
        return;
    }





    uint8_t brake_detected = 0U;
    uint8_t open_allowed_by_decel = 0U;


    /*
     * ==========================================
     * HISTERеза AIR_BRAKE
     * ==========================================
     *
     * Wejscie:
     * rate <= -5.5 km/h/s
     *
     * Wyjscie:
     * dopiero gdy rate > -3.0 km/h/s
     *
     * Jesli juz jestesmy AIR_BRAKE,
     * stosujemy lagodniejszy prog OFF.
     */
    if (speed_kmh >= AIR_BRAKE_MIN_SPEED_KMH)
    {
        if (legacy_state.wing_state == WING_AIR_BRAKE)
        {
            if (rate_x10 <=
                -BRAKE_OFF_THRESHOLD_X10)
            {
                brake_detected = 1U;
            }
        }
        else
        {
            if (rate_x10 <=
                -BRAKE_ON_THRESHOLD_X10)
            {
                brake_detected = 1U;
            }
        }
    }


    /*
     * Warunek tylko do WEJSCIA
     * NORMAL -> OPEN.
     */
    diag_brake_detected = brake_detected;

    if (rate_x10 >= -OPEN_DECEL_BLOCK_X10)
    {
        open_allowed_by_decel = 1U;
    }



    if (rate_x10 >= ACCEL_THRESHOLD_X10)
    {
        motion_state =
            MOTION_ACCELERATION;
    }
    else if (brake_detected)
    {
        motion_state =
            MOTION_BREAKING;
    }
    else
    {
        motion_state =
            MOTION_COASTING;
    }


    /*
     * ==========================================
     * MASZYNA STANOW
     * ==========================================
     */
    WingState_t previous_state = legacy_state.wing_state;
    Control_UpdateState(&legacy_state, speed_kmh, now, brake_detected, open_allowed_by_decel);
    Control_CommitState(&legacy_state, "LEGACY", brake_detected ? "brake_on" :
                        previous_state == WING_AIR_BRAKE ? "brake_off" :
                        legacy_state.wing_state == WING_OPEN ? "open_confirm" : "speed");

    Control_ApplyOutput();


#if 0
    printf(
        "SPEED=%u km/h"
        " | filter=%lu ms n=%u"
        " | rateFilt=",

        (unsigned int)speed_kmh,
        (unsigned long)filter_span_ms,
        (unsigned int)filter_samples
    );


    Print_Signed_X10(
        rate_x10
    );


    printf(
        " km/h/s | aFilt="
    );


    Print_Signed_mG_As_G(
        accel_mg
    );


    printf(" g");

    /* Wypisuje corner, lateral, ax, ay, az */
    IMU_PrintCornerInline();

    printf(
        " | brake=%u"
        " | openConfirm=%u"
        " | postBrake=%u"
        " | Acceleration=%u"
        " Coasting=%u"
        " Breaking=%u"
        " | WING=%s\r\n",

        (unsigned int)brake_detected,
        (unsigned int)legacy_state.open_confirm_active,
        (unsigned int)legacy_state.post_brake_hold_active,

        (motion_state == MOTION_ACCELERATION) ? 1U : 0U,
        (motion_state == MOTION_COASTING) ? 1U : 0U,
        (motion_state == MOTION_BREAKING) ? 1U : 0U,

        WingState_ToString(legacy_state.wing_state)
    );
#endif
}




static void Control_UpdateState(WingControlState_t *state, uint8_t speed_kmh,
                                uint32_t now, uint8_t brake_detected, uint8_t open_allowed_by_decel)
{
    if (brake_detected)
    {
        state->wing_state =
            WING_AIR_BRAKE;

        /*
         * Hold NORMAL zacznie sie dopiero
         * po wyjsciu z AIR_BRAKE.
         */
        state->post_brake_hold_active = 0U;

        state->open_confirm_active = 0U;
    }


    /*
     * AIR_BRAKE wlasnie sie skonczyl.
     *
     * Zawsze:
     *
     * AIR_BRAKE -> NORMAL
     */
    else if (state->wing_state == WING_AIR_BRAKE)
    {
        state->wing_state =
            WING_NORMAL;

        state->post_brake_hold_active = 1U;
        state->post_brake_start_tick = now;

        state->open_confirm_active = 0U;
    }


    /*
     * Po AIR_BRAKE chwilowo blokujemy OPEN.
     */
    else if (state->post_brake_hold_active)
    {
        if ((uint32_t)(
                now -
                state->post_brake_start_tick
            ) < POST_BRAKE_NORMAL_MS)
        {
            state->wing_state =
                WING_NORMAL;

            state->open_confirm_active = 0U;
        }
        else
        {
            /*
             * Czas blokady zakonczony.
             *
             * W tej probce nadal zostajemy
             * NORMAL. Od kolejnej probki
             * moze rozpoczac sie potwierdzanie OPEN.
             */
            state->post_brake_hold_active = 0U;

            state->wing_state =
                WING_NORMAL;

            state->open_confirm_active = 0U;
        }
    }


    /*
     * Jestesmy juz OPEN.
     */
    else if (state->wing_state == WING_OPEN)
    {
        /*
         * Przy spadku ponizej dolnego
         * progu predkosci wracamy NORMAL.
         */
        if (speed_kmh <= OPEN_OFF_SPEED_KMH)
        {
            state->wing_state =
                WING_NORMAL;

            state->open_confirm_active = 0U;
        }
        else
        {
            /*
             * COASTING / lekkie zwalnianie
             * pozostawia nas w OPEN.
             *
             * Mocne hamowanie zostalo
             * przechwycone wyzej.
             */
            state->wing_state =
                WING_OPEN;
        }
    }


    /*
     * Jestesmy NORMAL.
     */
    else
    {
        if (speed_kmh <= OPEN_OFF_SPEED_KMH)
        {
            state->wing_state =
                WING_NORMAL;

            state->open_confirm_active = 0U;
        }


        /*
         * OPEN mozemy wlaczyc dopiero
         * powyzej gornego progu.
         */
        else if (speed_kmh >= OPEN_ON_SPEED_KMH)
        {
            if (open_allowed_by_decel)
            {
                /*
                 * Pierwsza poprawna probka
                 * rozpoczyna odliczanie.
                 */
                if (state->open_confirm_active == 0U)
                {
                    state->open_confirm_active = 1U;
                    state->open_confirm_start_tick = now;
                }


                /*
                 * Warunek musi pozostac
                 * spelniony przez 300 ms.
                 */
                if ((uint32_t)(
                        now -
                        state->open_confirm_start_tick
                    ) >= OPEN_CONFIRM_MS)
                {
                    state->wing_state =
                        WING_OPEN;

                    state->open_confirm_active = 0U;
                }
                else
                {
                    state->wing_state =
                        WING_NORMAL;
                }
            }
            else
            {
                /*
                 * Trend przestal pozwalac
                 * na OPEN.
                 *
                 * Potwierdzanie zacznie sie
                 * od nowa.
                 */
                state->open_confirm_active = 0U;

                state->wing_state =
                    WING_NORMAL;
            }
        }



        else
        {
            state->wing_state =
                WING_NORMAL;

            state->open_confirm_active = 0U;
        }
    }



}

const char *Control_ModeToString(void)
{
#if APP_LAP_SIM_ENABLED
    return "SIM_LEGACY";
#elif APP_WING_CONTROL_MODE == WING_CONTROL_LEGACY_ONLY
    return "LEGACY_ONLY";
#elif APP_WING_CONTROL_MODE == WING_CONTROL_HYBRID_ACTIVE
    return "HYBRID_ACTIVE";
#else
    return "LEGACY_SHADOW_HYBRID";
#endif
}

static void Control_ApplyOutput(void)
{
#if !APP_LAP_SIM_ENABLED && APP_WING_CONTROL_MODE == WING_CONTROL_HYBRID_ACTIVE
    Servo_UpdateWingStateIfChanged(hybrid_state.wing_state);
#else
    Servo_UpdateWingStateIfChanged(legacy_state.wing_state);
#endif
}

static void Control_CommitState(WingControlState_t *state, const char *path, const char *reason)
{
    (void)path;
    (void)reason;
    if (state->reported_state == state->wing_state) return;
#if !APP_LAP_SIM_ENABLED
    printf("CTRL_EVT t_ms=%lu path=%s from=%s to=%s reason=%s\r\n",
           (unsigned long)HAL_GetTick(), path, WingState_ToString(state->reported_state),
           WingState_ToString(state->wing_state), reason);
#endif
    state->reported_state = state->wing_state;
}

static void Control_HybridEvent(const char *reason)
{
#if !APP_LAP_SIM_ENABLED
    printf("CTRL_HYB t_ms=%lu phase=%u reason=%s\r\n",
           (unsigned long)HAL_GetTick(), (unsigned)hybrid_phase, reason);
#else
    (void)reason;
#endif
}

static void Control_BlockHybrid(const char *reason)
{
    WingState_t reported = hybrid_state.reported_state;
    hybrid_state = (WingControlState_t){.reported_state = reported};
    hybrid_brake_detected = 0U;
    hybrid_phase = CONTROL_HYBRID_IDLE;
    hybrid_fx_armed = 0U;
    hybrid_on.active = hybrid_rearm.active = hybrid_off.active = 0U;
    Control_CommitState(&hybrid_state, "HYBRID", reason);
}

static void Control_ResetHybrid(const char *reason)
{
    Control_BlockHybrid(reason);
    hybrid_last_sample_id = hybrid_last_sample_tick = hybrid_last_obd_count = 0U;
}

static uint8_t Control_Confirm(ControlConfirmation_t *condition, uint8_t satisfied,
                               uint32_t now, uint32_t duration)
{
    if (!satisfied)
    {
        condition->active = 0U;
        return 0U;
    }
    if (!condition->active)
    {
        condition->active = 1U;
        condition->start_tick = now;
    }
    return (uint32_t)(now - condition->start_tick) >= duration;
}

void Control_Task(void)
{
    if (APP_LAP_SIM_ENABLED || APP_WING_CONTROL_MODE == WING_CONTROL_LEGACY_ONLY) return;

    OBD_Diag_t obd;
    IMU_Diag_t imu;
    IMU_Acceleration_t acceleration;
    OBD_GetDiagSnapshot(&obd);
    IMU_GetDiagSnapshot(&imu);
    IMU_GetAccelerationFromSource(ACC_SOURCE_MOTIONFX, &acceleration);
    uint32_t now = HAL_GetTick();
    uint8_t faults = 0U;
    const char *reason = "rate_invalid";
    if (!diag_rate_valid) faults |= CONTROL_FAULT_RATE;
    if (!obd.speed_valid || obd.speed_age_ms > APP_OBD_SPEED_MAX_AGE_MS)
    {
        faults |= obd.speed_valid ? CONTROL_FAULT_OBD_STALE : CONTROL_FAULT_OBD_MISSING;
        reason = "obd_data";
    }
    if (!imu.valid || imu.age_ms > APP_IMU_MAX_AGE_MS)
    {
        faults |= imu.valid ? CONTROL_FAULT_IMU_STALE : CONTROL_FAULT_IMU_MISSING;
        reason = "imu_data";
    }
    if (faults)
    {
        if (!hybrid_fault_flags) Control_ResetHybrid(reason);
        hybrid_fault_flags = faults;
        Control_ApplyOutput();
        return;
    }
    hybrid_fault_flags = 0U;
    if (IMU_IsCornerDetected())
    {
        Control_BlockHybrid("corner_raw");
        Control_ApplyOutput();
        return;
    }

    uint8_t new_obd = obd.speed_rx_count != hybrid_last_obd_count;
    hybrid_last_obd_count = obd.speed_rx_count;
    uint32_t obd_tick = now - obd.speed_age_ms;
    uint8_t fx_faults = 0U;
    uint8_t new_fx = 0U;
    if (!acceleration.sample_id) fx_faults = CONTROL_FAULT_FX_MISSING;
    else if ((uint32_t)(now - acceleration.timestamp_ms) > APP_MOTIONFX_MAX_AGE_MS)
        fx_faults = CONTROL_FAULT_FX_STALE;
    else if (!acceleration.valid || acceleration.source != ACC_SOURCE_MOTIONFX)
        fx_faults = CONTROL_FAULT_FX_INVALID;
    else if (acceleration.sample_id != hybrid_last_sample_id)
    {
        uint32_t delta = acceleration.timestamp_ms - hybrid_last_sample_tick;
        uint32_t expected = hybrid_last_sample_id + 1U;
        if (!expected) expected = 1U;
        if (hybrid_last_sample_id && (!delta || delta > APP_MOTIONFX_MAX_AGE_MS ||
                                     acceleration.sample_id != expected))
            fx_faults = CONTROL_FAULT_FX_INVALID;
        hybrid_last_sample_id = acceleration.sample_id;
        hybrid_last_sample_tick = acceleration.timestamp_ms;
        new_fx = !fx_faults;
    }
    else if (acceleration.timestamp_ms != hybrid_last_sample_tick)
        fx_faults = CONTROL_FAULT_FX_INVALID;
    if (fx_faults != hybrid_fx_faults)
    {
        hybrid_fx_faults = fx_faults;
        Control_HybridEvent(fx_faults ? "fx_unavailable" : "fx_recovered");
    }
    if (fx_faults)
    {
        hybrid_fx_armed = 0U;
        hybrid_on.active = hybrid_rearm.active = 0U;
    }

    uint8_t fx_trigger = 0U;
    if (new_fx)
    {
        uint32_t tick = acceleration.timestamp_ms;
        if (hybrid_phase == CONTROL_HYBRID_IDLE)
        {
            if (Control_Confirm(&hybrid_rearm, acceleration.acc_longitudinal_mg >= HYBRID_FX_REARM_MG,
                                tick, HYBRID_FX_REARM_CONFIRM_MS))
                hybrid_fx_armed = 1U;
            uint8_t onset = acceleration.acc_longitudinal_mg <= HYBRID_FX_ON_MG;
            if (hybrid_fx_armed && onset && diag_rate_x10 >= ACCEL_THRESHOLD_X10)
            {
                hybrid_fx_armed = 0U;
                hybrid_on.active = 0U;
                Control_HybridEvent("fx_accel_veto");
            }
            fx_trigger = hybrid_fx_armed && Control_Confirm(&hybrid_on, onset,
                                                           tick, HYBRID_FX_ON_CONFIRM_MS);
        }
    }

    WingState_t previous = hybrid_state.wing_state;
    reason = "speed";
    if (obd.speed_kmh < AIR_BRAKE_MIN_SPEED_KMH)
    {
        hybrid_phase = CONTROL_HYBRID_IDLE;
        hybrid_fx_armed = 0U;
        hybrid_on.active = hybrid_rearm.active = hybrid_off.active = 0U;
        reason = "speed_gate";
    }
    else if (new_obd && diag_rate_x10 <= -BRAKE_ON_THRESHOLD_X10)
    {
        if (hybrid_phase == CONTROL_HYBRID_WAIT_OBD)
        {
            hybrid_confirm_count++;
            Control_HybridEvent("obd_takeover");
        }
        if (hybrid_phase == CONTROL_HYBRID_IDLE) reason = "obd_on";
        else reason = "obd_brake";
        hybrid_phase = CONTROL_HYBRID_OBD;
        hybrid_fx_armed = 0U;
        hybrid_on.active = hybrid_rearm.active = hybrid_off.active = 0U;
    }
    else if (hybrid_phase == CONTROL_HYBRID_OBD)
    {
        if (new_obd && Control_Confirm(&hybrid_off, diag_rate_x10 > -BRAKE_OFF_THRESHOLD_X10,
                                      obd_tick, HYBRID_OBD_OFF_CONFIRM_MS))
        {
            hybrid_phase = CONTROL_HYBRID_IDLE;
            hybrid_off.active = 0U;
            reason = "obd_end_confirm";
        }
    }
    else if (hybrid_phase == CONTROL_HYBRID_WAIT_OBD)
    {
        if (fx_faults || (uint32_t)(now - hybrid_wait_tick) >= HYBRID_OBD_CONFIRM_MS)
        {
            hybrid_phase = CONTROL_HYBRID_IDLE;
            hybrid_reject_count++;
            reason = fx_faults ? "fx_data" : "obd_not_confirmed";
            Control_HybridEvent(reason);
        }
    }
    else if (fx_trigger)
    {
        hybrid_phase = CONTROL_HYBRID_WAIT_OBD;
        hybrid_wait_tick = now;
        hybrid_early_count++;
        hybrid_fx_armed = 0U;
        hybrid_on.active = hybrid_rearm.active = 0U;
        reason = "fx_early";
    }
    hybrid_brake_detected = hybrid_phase != CONTROL_HYBRID_IDLE;
    if (new_obd || new_fx || previous == WING_AIR_BRAKE || hybrid_brake_detected)
    {
        Control_UpdateState(&hybrid_state, obd.speed_kmh, now, hybrid_brake_detected,
                            diag_rate_x10 >= -OPEN_DECEL_BLOCK_X10);
        Control_CommitState(&hybrid_state, "HYBRID", reason);
        Control_ApplyOutput();
    }
}

void Control_GetCompareSnapshot(Control_CompareDiag_t *out)
{
    if (out == NULL) return;
    uint32_t elapsed = HAL_GetTick() - hybrid_wait_tick;
    *out = (Control_CompareDiag_t){
        .wing_legacy = legacy_state.wing_state, .wing_hybrid = hybrid_state.wing_state,
        .brake_legacy = diag_brake_detected, .brake_hybrid = hybrid_brake_detected,
        .corner_raw = IMU_IsCornerDetected(),
        .faults_legacy = control_fault_flags, .faults_hybrid = hybrid_fault_flags,
        .faults_fx = hybrid_fx_faults,
        .hybrid_enabled = !APP_LAP_SIM_ENABLED && APP_WING_CONTROL_MODE != WING_CONTROL_LEGACY_ONLY,
        .fx_armed = hybrid_fx_armed, .on_pending = hybrid_on.active, .off_pending = hybrid_off.active,
        .phase = hybrid_phase,
        .wait_remaining_ms = hybrid_phase == CONTROL_HYBRID_WAIT_OBD && elapsed < HYBRID_OBD_CONFIRM_MS ?
                             HYBRID_OBD_CONFIRM_MS - elapsed : 0U,
        .fx_early_count = hybrid_early_count, .obd_confirm_count = hybrid_confirm_count,
        .fx_reject_count = hybrid_reject_count
    };
}
