/*
 * motion_fx_app.h
 *
 *  Created on: 9 paź 2026
 *      Author: karab
 */

#ifndef MOTION_FX_APP_H_
#define MOTION_FX_APP_H_

#include <stdint.h>

/*
 * 0 - MotionFX jest wylaczony.
 * 1 - MotionFX pracuje diagnostycznie.
 */
#define MOTIONFX_DIAG_ENABLE 1

typedef struct
{
    float gravity_g[3];
    float linear_g[3];
    float gyro_dps[3];

    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float gyro_y_integral_deg; /* surowe calkowanie, nie orientacja */

    uint32_t timestamp_ms;
    uint8_t valid;

} MotionFxApp_Result_t;

uint8_t MotionFxApp_Init(void);

void MotionFxApp_Process(
    float ax_g,
    float ay_g,
    float az_g,
    float gx_dps,
    float gy_dps,
    float gz_dps,
    uint32_t timestamp_ms
);

uint8_t MotionFxApp_GetLatest(
    MotionFxApp_Result_t *result
);

void MotionFxApp_PrintDebug(void);

#endif
