/*
 * wing_control.h
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */

#ifndef WING_CONTROL_H_
#define WING_CONTROL_H_

#include <stdint.h>
#include "app_config.h"
#include "app_types.h"


void Control_Update_From_Speed(
    uint8_t speed_kmh
);



void Control_ForceNormalForCorner(void);
void Control_Update_From_SpeedAt(uint8_t speed_kmh, uint32_t tick_ms);
#if APP_LAP_SIM_ENABLED
void Control_Reset(void);
#endif


typedef struct
{
    WingState_t wing_state;
    uint8_t motion_state; /* 0=acceleration, 1=coasting, 2=braking */
    uint8_t brake_detected;
    uint8_t rate_valid;
    int32_t rate_x10; /* 0.1 km/h/s */
    int32_t accel_mg; /* przyspieszenie z regresji OBD */
    uint32_t filter_span_ms;
    uint8_t filter_samples;
    uint8_t open_confirm_active;
    uint32_t open_remaining_ms;
    uint8_t post_brake_hold_active;
    uint32_t post_remaining_ms;
} Control_Diag_t;

void Control_GetDiagSnapshot(Control_Diag_t *out);

#endif /* WING_CONTROL_H_ */
