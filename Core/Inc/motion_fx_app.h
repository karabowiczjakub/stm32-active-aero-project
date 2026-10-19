/*
 * motion_fx_app.h
 *
 *  Created on: 9 paź 2026
 *      Author: karab
 */

#ifndef MOTION_FX_APP_H_
#define MOTION_FX_APP_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>

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

static inline uint8_t MotionFxApp_IsLoggable(float value)
{
    float scaled = value * 1000.0f;
    return isfinite(value) && isfinite(scaled) &&
           (double)scaled >= (double)INT32_MIN && (double)scaled <= (double)INT32_MAX;
}

static inline uint8_t MotionFxApp_IsResultValid(const MotionFxApp_Result_t *result)
{
    if (result == NULL || !result->valid) return 0U;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (!MotionFxApp_IsLoggable(result->gravity_g[i]) ||
            !MotionFxApp_IsLoggable(result->linear_g[i]) ||
            !MotionFxApp_IsLoggable(result->gyro_dps[i])) return 0U;
    }
    return MotionFxApp_IsLoggable(result->yaw_deg) &&
           MotionFxApp_IsLoggable(result->pitch_deg) &&
           MotionFxApp_IsLoggable(result->roll_deg) &&
           MotionFxApp_IsLoggable(result->gyro_y_integral_deg);
}

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
