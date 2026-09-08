/*
 * wing_control.c
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */


#include "wing_control.h"

#include "main.h"
#include "app_types.h"
#include "servo.h"
#include "imu.h"
#include "debug_utils.h"

#include <stdio.h>



/* =========================
 * ALGORYTM
 * =========================
 */

#define MIN_SPEED_SAMPLE_DT_MS        300U

#define OPEN_ON_SPEED_KMH             55U

#define OPEN_OFF_SPEED_KMH            45U


#define ACCEL_THRESHOLD_X10           25

#define BRAKE_ON_THRESHOLD_X10        55

#define OPEN_DECEL_BLOCK_X10          20


#define AIR_BRAKE_MIN_SPEED_KMH       40U


#define POST_BRAKE_NORMAL_SAMPLES     3U


#define OPEN_STABLE_SAMPLES           3U



typedef enum
{
    MOTION_ACCELERATION = 0,

    MOTION_COASTING,

    MOTION_BREAKING

} MotionState_t;



static uint8_t prev_speed_kmh = 0;

static uint32_t prev_speed_tick = 0;

static uint8_t speed_initialized = 0;


static MotionState_t motion_state =
    MOTION_COASTING;


static WingState_t wing_state =
    WING_NORMAL;


static uint8_t post_brake_normal_samples = 0;

static uint8_t open_stable_samples = 0;



static int32_t RateX10_To_mG(
    int32_t rate_x10
)
{
    return (rate_x10 * 1000L) / 353L;
}




void Control_ForceNormalForCorner(void)
{
    wing_state =
        WING_NORMAL;


    open_stable_samples = 0;


    post_brake_normal_samples = 0;


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


void Control_Update_From_Speed(
    uint8_t speed_kmh
)
{
    uint32_t now =
        HAL_GetTick();


    uint8_t corner_detected =
        IMU_IsCornerDetected();


    if (speed_initialized == 0)
    {
        speed_initialized = 1;


        prev_speed_kmh =
            speed_kmh;


        prev_speed_tick =
            now;


        motion_state =
            MOTION_COASTING;


        wing_state =
            WING_NORMAL;


        post_brake_normal_samples = 0;

        open_stable_samples = 0;


        Servo_UpdateWingStateIfChanged(
            wing_state
        );


        printf(
            "SPEED=%u km/h | Acceleration=0 Coasting=1 Breaking=0 | WING=%s\r\n",
            (unsigned int)speed_kmh,
            WingState_ToString(wing_state)
        );


        return;
    }



    uint32_t dt_ms =
        now - prev_speed_tick;


    if (dt_ms <
        MIN_SPEED_SAMPLE_DT_MS)
    {
        return;
    }



    int16_t delta_speed =
        (int16_t)speed_kmh -
        (int16_t)prev_speed_kmh;



    int32_t rate_x10 =
        (
            (int32_t)delta_speed *
            10000L
        ) /
        (int32_t)dt_ms;



    int32_t accel_mg =
        RateX10_To_mG(
            rate_x10
        );



    uint8_t brake_detected = 0;

    uint8_t open_allowed_by_decel = 0;



    if (rate_x10 <=
            -BRAKE_ON_THRESHOLD_X10 &&

        speed_kmh >=
            AIR_BRAKE_MIN_SPEED_KMH &&

        corner_detected == 0)
    {
        brake_detected = 1;
    }



    if (rate_x10 >=
        -OPEN_DECEL_BLOCK_X10)
    {
        open_allowed_by_decel = 1;
    }



    if (rate_x10 >=
        ACCEL_THRESHOLD_X10)
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



    if (corner_detected)
    {
        wing_state =
            WING_NORMAL;


        open_stable_samples = 0;


        post_brake_normal_samples = 0;
    }


    else if (brake_detected)
    {
        wing_state =
            WING_AIR_BRAKE;


        post_brake_normal_samples =
            POST_BRAKE_NORMAL_SAMPLES;


        open_stable_samples = 0;
    }


    else if (wing_state ==
             WING_AIR_BRAKE)
    {
        wing_state =
            WING_NORMAL;


        post_brake_normal_samples =
            POST_BRAKE_NORMAL_SAMPLES;


        open_stable_samples = 0;
    }


    else if (
        post_brake_normal_samples > 0
    )
    {
        wing_state =
            WING_NORMAL;


        post_brake_normal_samples--;


        open_stable_samples = 0;
    }


    else
    {
        if (speed_kmh <=
            OPEN_OFF_SPEED_KMH)
        {
            wing_state =
                WING_NORMAL;


            open_stable_samples = 0;
        }


        else if (
            speed_kmh >=
            OPEN_ON_SPEED_KMH
        )
        {
            if (open_allowed_by_decel)
            {
                if (open_stable_samples <
                    OPEN_STABLE_SAMPLES)
                {
                    open_stable_samples++;
                }
            }
            else
            {
                open_stable_samples = 0;


                wing_state =
                    WING_NORMAL;
            }


            if (open_stable_samples >=
                OPEN_STABLE_SAMPLES)
            {
                wing_state =
                    WING_OPEN;
            }
            else
            {
                wing_state =
                    WING_NORMAL;
            }
        }


        else
        {
            if (wing_state ==
                    WING_OPEN &&

                open_allowed_by_decel)
            {
                wing_state =
                    WING_OPEN;
            }
            else
            {
                wing_state =
                    WING_NORMAL;


                open_stable_samples = 0;
            }
        }
    }



    Servo_UpdateWingStateIfChanged(
        wing_state
    );



    printf(
        "SPEED=%u km/h | dt=%lu ms | dV=%d km/h | rate=",
        (unsigned int)speed_kmh,
        (unsigned long)dt_ms,
        (int)delta_speed
    );


    Print_Signed_X10(
        rate_x10
    );


    printf(
        " km/h/s | a="
    );


    Print_Signed_mG_As_G(
        accel_mg
    );


    printf(" g");


    IMU_PrintCornerInline();


    printf(
        " | openCnt=%u postBrake=%u | Acceleration=%u Coasting=%u Breaking=%u | WING=%s\r\n",

        (unsigned int)
            open_stable_samples,

        (unsigned int)
            post_brake_normal_samples,

        (motion_state ==
            MOTION_ACCELERATION)
            ? 1U : 0U,

        (motion_state ==
            MOTION_COASTING)
            ? 1U : 0U,

        (motion_state ==
            MOTION_BREAKING)
            ? 1U : 0U,

        WingState_ToString(
            wing_state
        )
    );



    prev_speed_kmh =
        speed_kmh;


    prev_speed_tick =
        now;
}
