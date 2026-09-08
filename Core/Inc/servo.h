/*
 * servo.h
 *
 *  Created on: 7 wrz 2026
 *      Author: karab
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

#include "app_types.h"


void Servo_Init(void);

void Servo_SetWingState(WingState_t state);

void Servo_UpdateWingStateIfChanged(WingState_t new_state);

uint16_t Servo_GetCurrentPulseUs(void);


#endif /* SERVO_H_ */
