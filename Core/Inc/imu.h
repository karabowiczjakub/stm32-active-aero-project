/*
 * imu.h
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include "app_config.h"

void IMU_ProcessLateralSample_mG(int32_t lateral_mg);
#if APP_LAP_SIM_ENABLED
void IMU_ResetCornerDetection(void);
#endif


	uint8_t LSM6DSO_Init(void);

	void IMU_Task(void);

	void IMU_PrintCornerInline(void);


	uint8_t IMU_IsCornerDetected(void);

	uint8_t IMU_TakeCornerDetectedEvent(void);

	void IMU_RequestSample(void);


	uint8_t IMU_GetAccelSnapshot(
	    int32_t *ax_mg,
	    int32_t *ay_mg,
	    int32_t *az_mg,
	    uint32_t *age_ms
	);


typedef struct
{
    uint8_t valid;
    uint32_t age_ms;
    int32_t ax_mg, ay_mg, az_mg;
    int32_t gx_mdps, gy_mdps, gz_mdps;
    int32_t lateral_raw_mg, lateral_filtered_mg;
    uint8_t corner;
    uint8_t corner_on_count, corner_off_count;
    uint32_t i2c_error_count;
    uint32_t dma_start_error_count;
} IMU_Diag_t;

void IMU_GetDiagSnapshot(IMU_Diag_t *out);

#endif /* IMU_H_ */
