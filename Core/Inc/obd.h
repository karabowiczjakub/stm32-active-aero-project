/*
 * obd.h
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */

#ifndef OBD_H_
#define OBD_H_

#include <stdint.h>


void CAN_Test_Init(void);

void CAN_Send_OBD_Request(
    uint8_t pid
);

void CAN_Poll_Rx(void);
void CAN_Process_Rx(void);

uint8_t CAN_Is_OBD_RequestPending(void);
void CAN_Check_OBD_RequestTimeout(void);

#endif /* OBD_H_ */

