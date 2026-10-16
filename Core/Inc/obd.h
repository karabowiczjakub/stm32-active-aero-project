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
    uint8_t pid);

void CAN_Poll_Rx(void);
void CAN_Process_Rx(void);

uint8_t CAN_Is_OBD_RequestPending(void);
void CAN_Check_OBD_RequestTimeout(void);


uint8_t OBD_GetSpeedSnapshot(
    uint8_t *speed_kmh,
    uint32_t *age_ms);


typedef struct
{
    uint8_t speed_valid;
    uint8_t speed_kmh;
    uint32_t speed_age_ms;
    uint8_t request_pending;
    uint32_t request_age_ms;
    uint32_t speed_rx_count;
    uint32_t request_queued_count;
    uint32_t tx_error_count;
    uint32_t response_timeout_count;
    uint32_t rx_overflow_count;
} OBD_Diag_t;

void OBD_GetDiagSnapshot(OBD_Diag_t *out);

#endif /* OBD_H_ */

