#include "lap_sim.h"

#if APP_LAP_SIM_ENABLED
#include "lap_data.h"
#include "main.h"
#include "imu.h"
#include "wing_control.h"
#include <stdio.h>

#define LAP_SPEED_PERIOD_MS 50U

static LapSim_Diag_t sim;
static uint32_t start_tick;
static uint32_t next_lateral;
static uint32_t next_speed_ms;

static void LapSim_Finish(void)
{
    sim.state = LAP_SIM_FINISHED;
    Control_Reset();
    IMU_ResetCornerDetection();
}

void LapSim_Init(void)
{
    sim = (LapSim_Diag_t){.state = LAP_SIM_WAITING};
    next_lateral = 0U;
    next_speed_ms = 0U;
    Control_Reset();
    IMU_ResetCornerDetection();
    printf("SIM_WAITING USER_BUTTON\r\n");
}

void LapSim_Start(void)
{
    if (sim.state != LAP_SIM_WAITING) return;

    Control_Reset();
    IMU_ResetCornerDetection();
    sim = (LapSim_Diag_t){.state = LAP_SIM_RUNNING};
    next_lateral = 0U;
    next_speed_ms = 0U;
    start_tick = HAL_GetTick();
    printf("SIM_START\r\n");
    printf("SIM_HEADER,t_ms,speed_kmh,lat_g,long_g,rate_kmh_s,rate_valid,corner,wing,lag_ms,quality\r\n");
}

void LapSim_Task(void)
{
    if (sim.state != LAP_SIM_RUNNING) return;

    for (;;)
    {
        if (next_lateral == LAP_DATA_COUNT && next_speed_ms > LAP_DATA_END_MS)
        {
            sim.elapsed_ms = LAP_DATA_END_MS;
            LapSim_Finish();
            printf("SIM_END,%lu\r\n", (unsigned long)sim.elapsed_ms);
            return;
        }

        uint32_t lateral_ms = next_lateral < LAP_DATA_COUNT ?
            lap_data[next_lateral].t_ms : UINT32_MAX;
        uint8_t lateral_first = lateral_ms <= next_speed_ms;
        uint32_t due_ms = lateral_first ? lateral_ms : next_speed_ms;
        uint32_t elapsed_ms = HAL_GetTick() - start_tick;
        if (elapsed_ms < due_ms) return;

        sim.lateness_ms = elapsed_ms - due_ms;
        if (sim.lateness_ms > APP_LAP_SIM_MAX_LATENESS_MS)
        {
            sim.elapsed_ms = elapsed_ms;
            sim.timing_error = 1U;
            LapSim_Finish();
            printf("SIM_TIMING_ERROR,%lu,%lu\r\n",
                   (unsigned long)elapsed_ms, (unsigned long)sim.lateness_ms);
            return;
        }

        if (lateral_first)
        {
            const LapData_Sample_t *sample = &lap_data[next_lateral++];
            sim.lateral_mg = sample->lateral_mg;
            sim.longitudinal_mg = sample->longitudinal_mg;
            sim.lateral_sample_ms = sample->t_ms;
            sim.quality = (sim.quality & LAP_QUALITY_SPEED_INTERPOLATED) |
                         (sample->quality & LAP_QUALITY_G_EXTENDED_ROI);
            sim.lateral_samples++;
            IMU_ProcessLateralSample_mG(sample->lateral_mg);
            if (IMU_TakeCornerDetectedEvent()) Control_ForceNormalForCorner();
        }
        else
        {
            /* Lateral events win ties, so this sample is never in the future. */
            const LapData_Sample_t *sample = &lap_data[next_lateral - 1U];
            sim.speed_kmh = sample->speed_kmh;
            sim.speed_sample_ms = next_speed_ms;
            sim.quality = (sim.quality & LAP_QUALITY_G_EXTENDED_ROI) |
                         (sample->quality & LAP_QUALITY_SPEED_INTERPOLATED);
            sim.speed_samples++;
            Control_Update_From_SpeedAt(sample->speed_kmh, start_tick + next_speed_ms);
            next_speed_ms += LAP_SPEED_PERIOD_MS;
        }
    }
}

void LapSim_GetDiagSnapshot(LapSim_Diag_t *out)
{
    if (out == NULL) return;
    *out = sim;
    if (sim.state == LAP_SIM_RUNNING) out->elapsed_ms = HAL_GetTick() - start_tick;
}
#endif
