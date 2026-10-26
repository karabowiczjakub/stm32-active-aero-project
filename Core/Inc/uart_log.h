#ifndef UART_LOG_H_
#define UART_LOG_H_

#include <stdint.h>

#define UART_LOG_LINE_SIZE 544U
#define UART_LOG_QUEUE_SIZE 12U

typedef struct
{
    uint32_t dropped_count;
    uint32_t error_count;
    uint32_t busy_count;
    uint8_t queued;
    uint8_t active;
    uint8_t recovering;
} UARTLog_Diag_t;

void UARTLog_Init(void);
void UARTLog_Task(void);
void UARTLog_GetDiagSnapshot(UARTLog_Diag_t *out);

#endif
