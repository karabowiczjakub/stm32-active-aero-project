/*
 * servo.c
 *
 *  Created on: 7 wrz 2026
 *      Author: karab
 */


#include "servo.h"

#include "main.h"
#include "debug_utils.h"

#include <stdio.h>


/*
 * Uchwyt timera jest tworzony przez kod CubeMX w main.c.
 */
extern TIM_HandleTypeDef htim2;
/*
 * Znaczenie aerodynamiczne:
 *
 * OPEN       = maly kat / okolo 0 stopni / redukcja oporu
 * NORMAL     = lekki kat natarcia / generowanie docisku
 * AIR_BRAKE  = najwiekszy kat / maksymalny opor
 *
 * Uwaga: to sa wartosci robocze. Po montazu mechaniki dobierzemy je dokladnie.
 */

/* Zakres impulsu serwa */
#define SERVO_MIN_PULSE_US          1000U
#define SERVO_MAX_PULSE_US          2000U


/* Pozycje skrzydła */
#define SERVO_OPEN_PULSE_US         1500U
#define SERVO_NORMAL_PULSE_US       1700U
#define SERVO_AIR_BRAKE_PULSE_US    2000U


static uint16_t servo_current_pulse_us = SERVO_OPEN_PULSE_US;

static WingState_t last_servo_wing_state = WING_NORMAL;

static uint8_t servo_state_initialized = 0;



static void Servo_SetPulseUs(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US)
    {
        pulse_us = SERVO_MIN_PULSE_US;
    }

    if (pulse_us > SERVO_MAX_PULSE_US)
    {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    servo_current_pulse_us = pulse_us;

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_1,
        pulse_us
    );
}





void Servo_SetWingState(WingState_t state)
{
    switch (state)
    {
        case WING_OPEN:

            Servo_SetPulseUs(SERVO_OPEN_PULSE_US);
            break;


        case WING_NORMAL:

            Servo_SetPulseUs(SERVO_NORMAL_PULSE_US);
            break;


        case WING_AIR_BRAKE:

            Servo_SetPulseUs(SERVO_AIR_BRAKE_PULSE_US);
            break;


        default:

            Servo_SetPulseUs(SERVO_NORMAL_PULSE_US);
            break;
    }
}



void Servo_Init(void)
{
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * Bezpieczna pozycja początkowa.
     */
    Servo_SetWingState(WING_NORMAL);
}




void Servo_UpdateWingStateIfChanged(WingState_t new_state)
{
    if (servo_state_initialized == 0 ||
        new_state != last_servo_wing_state)
    {
        Servo_SetWingState(new_state);

        last_servo_wing_state = new_state;

        servo_state_initialized = 1;

        printf(
            "SERVO STATE = %s | pulse=%u us\r\n",
            WingState_ToString(new_state),
            (unsigned int)servo_current_pulse_us
        );
    }
}


uint16_t Servo_GetCurrentPulseUs(void)
{
    return servo_current_pulse_us;
}
