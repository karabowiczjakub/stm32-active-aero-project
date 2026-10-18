#ifndef LAP_SIM_H_
#define LAP_SIM_H_

#include "app_config.h"

#if APP_LAP_SIM_ENABLED
#include <stdint.h>

typedef enum
{
    LAP_SIM_WAITING = 0,
    LAP_SIM_RUNNING,
    LAP_SIM_FINISHED
} LapSim_State_t;

typedef struct
{
    LapSim_State_t state;
    uint32_t elapsed_ms;
    uint32_t lateral_sample_ms; /* Timestamp in the CSV. */
    uint32_t speed_sample_ms;   /* Control deadline; CSV value is 0 or 10 ms older. */
    uint32_t lateness_ms;       /* Delay of the last processed event, not sample age. */
    uint32_t lateral_samples;
    uint32_t speed_samples;
    int16_t lateral_mg;       /* Video dot right = positive, not sensor Y. */
    int16_t longitudinal_mg;  /* Video dot down = positive, diagnostic only. */
    uint8_t speed_kmh;        /* Last value actually submitted to control. */
    uint8_t quality;          /* LAP_QUALITY_* for the active input channels. */
    uint8_t timing_error;
} LapSim_Diag_t;

void LapSim_Init(void);
void LapSim_Start(void); /* Main loop only; accepted only in WAITING. */
void LapSim_Task(void);
void LapSim_GetDiagSnapshot(LapSim_Diag_t *out);

#endif
#endif
