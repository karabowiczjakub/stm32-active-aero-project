/*
 * motion_fx_app.c
 *
 *  Created on: 9 paź 2026
 *      Author: karab
 */

#include "motion_fx_app.h"
#include "app_config.h"

#include <stdio.h>
#include <string.h>

#if MOTIONFX_DIAG_ENABLE

#include "main.h"
#include "motion_fx.h"

#define MOTIONFX_STATE_BYTES 4096U

static uint64_t mfx_state[
    MOTIONFX_STATE_BYTES / sizeof(uint64_t)
];

static MFX_input_t mfx_input;
static MFX_output_t mfx_output;

static MotionFxApp_Result_t latest_result;

static uint8_t mfx_initialized = 0U;
static uint8_t first_sample = 1U;
static uint32_t previous_tick_ms = 0U;
static float gyro_y_integral_deg = 0.0f;
static uint32_t mfx_sample_id = 0U;

static void MotionFxApp_MapVehicleAxes(MotionFxApp_Result_t *result)
{
    // robocze mapowanie, do sprawdzenia w pojezdzie
    result->acc_longitudinal_g = result->linear_g[1];
    result->acc_lateral_g = -result->linear_g[0];
    result->acc_vertical_g = result->linear_g[2];
}

static void MotionFxApp_FilterAcceleration(MotionFxApp_Result_t *result)
{
    result->acc_longitudinal_filtered_g = result->acc_longitudinal_g;
    result->acc_lateral_filtered_g = result->acc_lateral_g;
#if MOTIONFX_FILTER_ENABLE
    if (latest_result.valid &&
        (uint32_t)(result->timestamp_ms - latest_result.timestamp_ms) <= APP_MOTIONFX_MAX_AGE_MS)
    {
        result->acc_longitudinal_filtered_g = latest_result.acc_longitudinal_filtered_g +
            (result->acc_longitudinal_g - latest_result.acc_longitudinal_filtered_g) /
            (float)MOTIONFX_FILTER_DIVISOR;
        result->acc_lateral_filtered_g = latest_result.acc_lateral_filtered_g +
            (result->acc_lateral_g - latest_result.acc_lateral_filtered_g) /
            (float)MOTIONFX_FILTER_DIVISOR;
    }
#endif
}


uint8_t MotionFxApp_Init(void)
{

    mfx_initialized = 0U;
    memset(&latest_result, 0, sizeof(latest_result));
    __HAL_RCC_CRC_CLK_ENABLE();

    if (MotionFX_GetStateSize() > sizeof(mfx_state))
    {
        printf("MotionFX: insufficient state memory\r\n");
        return 0U;
    }

    memset(mfx_state, 0, sizeof(mfx_state));
    memset(&mfx_input, 0, sizeof(mfx_input));
    memset(&mfx_output, 0, sizeof(mfx_output));

    MFXState_t state = (MFXState_t)mfx_state;

    MotionFX_initialize(state);

    MFX_knobs_t knobs;
    MotionFX_getKnobs(state, &knobs);


    memcpy(knobs.acc_orientation, "nwu", 4);
    memcpy(knobs.gyro_orientation, "nwu", 4);

    knobs.output_type = MFX_ENGINE_OUTPUT_ENU;


    knobs.LMode = 1U;

    knobs.modx = 1U;

    MotionFX_setKnobs(state, &knobs);

    MotionFX_enable_9X(state, MFX_ENGINE_DISABLE);
    MotionFX_enable_6X(state, MFX_ENGINE_ENABLE);

    first_sample = 1U;
    gyro_y_integral_deg = 0.0f;
    mfx_initialized = 1U;

    printf("MotionFX 6X initialized\r\n");

    return 1U;
}


void MotionFxApp_Process(
    float ax_g,
    float ay_g,
    float az_g,
    float gx_dps,
    float gy_dps,
    float gz_dps,
    uint32_t timestamp_ms
)
{
    if (mfx_initialized == 0U)
    {
        return;
    }


    if (first_sample != 0U)
    {
        previous_tick_ms = timestamp_ms;
        first_sample = 0U;
        return;
    }

    uint32_t delta_ms =
        timestamp_ms - previous_tick_ms;

    previous_tick_ms = timestamp_ms;


    if ((delta_ms == 0U) || (delta_ms > 100U))
    {
        latest_result.valid = 0U;
        return;
    }

    float dt = (float)delta_ms / 1000.0f;

    if (!isfinite(ax_g) || !isfinite(ay_g) || !isfinite(az_g) ||
        !isfinite(gx_dps) || !isfinite(gy_dps) || !isfinite(gz_dps))
    {
        latest_result.valid = 0U;
        return;
    }
    float next_integral_deg = gyro_y_integral_deg + gy_dps * dt;

    mfx_input.acc[0] = ax_g;
    mfx_input.acc[1] = ay_g;
    mfx_input.acc[2] = az_g;

    mfx_input.gyro[0] = gx_dps;
    mfx_input.gyro[1] = gy_dps;
    mfx_input.gyro[2] = gz_dps;

    MFXState_t state = (MFXState_t)mfx_state;

    MotionFX_propagate(
        state,
        &mfx_output,
        &mfx_input,
        &dt
    );

    MotionFX_update(
        state,
        &mfx_output,
        &mfx_input,
        &dt,
        NULL
    );

    MotionFxApp_Result_t result = {0};
    for (uint8_t i = 0U; i < 3U; i++)
    {
        result.gravity_g[i] =
            mfx_output.gravity[i];

        result.linear_g[i] =
            mfx_output.linear_acceleration[i];

        result.gyro_dps[i] =
            mfx_input.gyro[i];
    }


    result.yaw_deg = mfx_output.rotation[0];
    result.pitch_deg = mfx_output.rotation[1];
    result.roll_deg = mfx_output.rotation[2];
    result.gyro_y_integral_deg = next_integral_deg;
    result.timestamp_ms = timestamp_ms;
    result.valid = 1U;
    MotionFxApp_MapVehicleAxes(&result);
    if (!MotionFxApp_IsResultValid(&result) ||
        (uint32_t)(HAL_GetTick() - timestamp_ms) > APP_MOTIONFX_MAX_AGE_MS)
    {
        latest_result.valid = 0U;
        return;
    }
    MotionFxApp_FilterAcceleration(&result);
    if (!MotionFxApp_IsResultValid(&result))
    {
        latest_result.valid = 0U;
        return;
    }
    if (++mfx_sample_id == 0U) mfx_sample_id = 1U;
    result.sample_id = mfx_sample_id;
    gyro_y_integral_deg = next_integral_deg;
    latest_result = result;
}


uint8_t MotionFxApp_GetLatest(
    MotionFxApp_Result_t *result
)
{
    if (result == NULL)
    {
        return 0U;
    }

    if (!MotionFxApp_IsResultValid(&latest_result) ||
        (uint32_t)(HAL_GetTick() - latest_result.timestamp_ms) > APP_MOTIONFX_MAX_AGE_MS)
    {
        latest_result.valid = 0U;
    }
    *result = latest_result;
    return latest_result.valid;
}


void MotionFxApp_PrintDebug(void)
{
    MotionFxApp_Result_t result;

    if (MotionFxApp_GetLatest(&result) == 0U)
    {
        return;
    }

    printf(
        "IMU_FX t=%lu "
        "pitch_mdeg=%ld "
        "lin_mg=%ld,%ld,%ld "
        "grav_mg=%ld,%ld,%ld "
        "gyro_mdps=%ld,%ld,%ld "
        "longRaw_mg=%ld latRaw_mg=%ld longFilt_mg=%ld latFilt_mg=%ld "
        "sample_ms=%lu seq=%lu valid=1 age_ms=%lu\r\n",

        (unsigned long)result.timestamp_ms,

        (long)(result.pitch_deg * 1000.0f),

        (long)(result.linear_g[0] * 1000.0f),
        (long)(result.linear_g[1] * 1000.0f),
        (long)(result.linear_g[2] * 1000.0f),

        (long)(result.gravity_g[0] * 1000.0f),
        (long)(result.gravity_g[1] * 1000.0f),
        (long)(result.gravity_g[2] * 1000.0f),

        (long)(result.gyro_dps[0] * 1000.0f),
        (long)(result.gyro_dps[1] * 1000.0f),
        (long)(result.gyro_dps[2] * 1000.0f),
        (long)(result.acc_longitudinal_g * 1000.0f),
        (long)(result.acc_lateral_g * 1000.0f),
        (long)(result.acc_longitudinal_filtered_g * 1000.0f),
        (long)(result.acc_lateral_filtered_g * 1000.0f),
        (unsigned long)result.timestamp_ms,
        (unsigned long)result.sample_id,
        (unsigned long)(HAL_GetTick() - result.timestamp_ms)
    );
}

#else



uint8_t MotionFxApp_Init(void)
{
    return 0U;
}

void MotionFxApp_Process(
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps,
    uint32_t timestamp_ms
)
{
    (void)ax_g;
    (void)ay_g;
    (void)az_g;
    (void)gx_dps;
    (void)gy_dps;
    (void)gz_dps;
    (void)timestamp_ms;
}

uint8_t MotionFxApp_GetLatest(
    MotionFxApp_Result_t *result
)
{
    if (result != NULL) memset(result, 0, sizeof(*result));
    return 0U;
}

void MotionFxApp_PrintDebug(void)
{
}

#endif
