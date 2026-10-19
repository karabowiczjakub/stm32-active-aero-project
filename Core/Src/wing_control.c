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


static WingState_t wing_state =
    WING_NORMAL;


static uint8_t post_brake_hold_active = 0U;
static uint32_t post_brake_start_tick = 0U;

static uint8_t open_confirm_active = 0U;
static uint32_t open_confirm_start_tick = 0U;


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
    out->wing_state = wing_state;
    out->motion_state = (uint8_t)motion_state;
    out->brake_detected = diag_brake_detected;
    out->rate_valid = diag_rate_valid;
    out->failsafe_flags = control_fault_flags;
    out->rate_x10 = diag_rate_x10;
    out->accel_mg = diag_accel_mg;
    out->filter_span_ms = diag_filter_span_ms;
    out->filter_samples = diag_filter_samples;
    out->open_confirm_active = open_confirm_active;
    out->open_remaining_ms = open_confirm_active ?
        ((now - open_confirm_start_tick) >= OPEN_CONFIRM_MS ? 0U :
         OPEN_CONFIRM_MS - (now - open_confirm_start_tick)) : 0U;
    out->post_brake_hold_active = post_brake_hold_active;
    out->post_remaining_ms = post_brake_hold_active ?
        ((now - post_brake_start_tick) >= POST_BRAKE_NORMAL_MS ? 0U :
         POST_BRAKE_NORMAL_MS - (now - post_brake_start_tick)) : 0U;
}

void Control_ForceNormalForCorner(void)
{
    wing_state = WING_NORMAL;

    open_confirm_active = 0U;
    post_brake_hold_active = 0U;

    Servo_UpdateWingStateIfChanged(
        wing_state
    );
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

void Control_Reset(void)
{
    memset(speed_history, 0, sizeof(speed_history));
    speed_history_head = 0U;
    speed_history_count = 0U;
    motion_state = MOTION_COASTING;
    wing_state = WING_NORMAL;
    post_brake_hold_active = 0U;
    post_brake_start_tick = 0U;
    open_confirm_active = 0U;
    open_confirm_start_tick = 0U;
    diag_rate_valid = 0U;
    diag_rate_x10 = 0;
    diag_accel_mg = 0;
    diag_filter_span_ms = 0U;
    diag_filter_samples = 0U;
    diag_brake_detected = 0U;
    Servo_UpdateWingStateIfChanged(WING_NORMAL);
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
        Control_Reset();
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

        wing_state = WING_NORMAL;

        open_confirm_active = 0U;
        post_brake_hold_active = 0U;

        Servo_UpdateWingStateIfChanged(
            wing_state
        );

        return;
    }



    if (rate_valid == 0U)
    {
        motion_state = MOTION_COASTING;
        wing_state = WING_NORMAL;
        open_confirm_active = 0U;
        post_brake_hold_active = 0U;
        Servo_UpdateWingStateIfChanged(WING_NORMAL);
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
        if (wing_state == WING_AIR_BRAKE)
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
    if (brake_detected)
    {
        wing_state =
            WING_AIR_BRAKE;

        /*
         * Hold NORMAL zacznie sie dopiero
         * po wyjsciu z AIR_BRAKE.
         */
        post_brake_hold_active = 0U;

        open_confirm_active = 0U;
    }


    /*
     * AIR_BRAKE wlasnie sie skonczyl.
     *
     * Zawsze:
     *
     * AIR_BRAKE -> NORMAL
     */
    else if (wing_state == WING_AIR_BRAKE)
    {
        wing_state =
            WING_NORMAL;

        post_brake_hold_active = 1U;
        post_brake_start_tick = now;

        open_confirm_active = 0U;
    }


    /*
     * Po AIR_BRAKE chwilowo blokujemy OPEN.
     */
    else if (post_brake_hold_active)
    {
        if ((uint32_t)(
                now -
                post_brake_start_tick
            ) < POST_BRAKE_NORMAL_MS)
        {
            wing_state =
                WING_NORMAL;

            open_confirm_active = 0U;
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
            post_brake_hold_active = 0U;

            wing_state =
                WING_NORMAL;

            open_confirm_active = 0U;
        }
    }


    /*
     * Jestesmy juz OPEN.
     */
    else if (wing_state == WING_OPEN)
    {
        /*
         * Przy spadku ponizej dolnego
         * progu predkosci wracamy NORMAL.
         */
        if (speed_kmh <= OPEN_OFF_SPEED_KMH)
        {
            wing_state =
                WING_NORMAL;

            open_confirm_active = 0U;
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
            wing_state =
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
            wing_state =
                WING_NORMAL;

            open_confirm_active = 0U;
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
                if (open_confirm_active == 0U)
                {
                    open_confirm_active = 1U;
                    open_confirm_start_tick = now;
                }


                /*
                 * Warunek musi pozostac
                 * spelniony przez 300 ms.
                 */
                if ((uint32_t)(
                        now -
                        open_confirm_start_tick
                    ) >= OPEN_CONFIRM_MS)
                {
                    wing_state =
                        WING_OPEN;

                    open_confirm_active = 0U;
                }
                else
                {
                    wing_state =
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
                open_confirm_active = 0U;

                wing_state =
                    WING_NORMAL;
            }
        }



        else
        {
            wing_state =
                WING_NORMAL;

            open_confirm_active = 0U;
        }
    }


    Servo_UpdateWingStateIfChanged(
        wing_state
    );


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
        (unsigned int)open_confirm_active,
        (unsigned int)post_brake_hold_active,

        (motion_state == MOTION_ACCELERATION) ? 1U : 0U,
        (motion_state == MOTION_COASTING) ? 1U : 0U,
        (motion_state == MOTION_BREAKING) ? 1U : 0U,

        WingState_ToString(wing_state)
    );
#endif
}



