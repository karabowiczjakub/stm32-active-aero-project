/*
 * imu.h
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>


	uint8_t LSM6DSO_Init(void);

	void IMU_Task(void);

	void IMU_PrintCornerInline(void);


	uint8_t IMU_IsCornerDetected(void);

	uint8_t IMU_TakeCornerDetectedEvent(void);


#endif /* IMU_H_ */
