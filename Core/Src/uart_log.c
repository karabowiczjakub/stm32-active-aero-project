#include "uart_log.h"
#include "main.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef hlpuart1;

static uint8_t lines[UART_LOG_QUEUE_SIZE][UART_LOG_LINE_SIZE];
static uint16_t lengths[UART_LOG_QUEUE_SIZE];
static uint8_t pending_line[UART_LOG_LINE_SIZE];
static uint16_t pending_length;
static uint8_t discard_line;
static uint8_t tail;
static volatile uint8_t head, queued, active, recovering, abort_pending;
static volatile uint32_t dropped_count, error_count, busy_count;

static uint8_t UARTLog_InterfaceReady(void)
{
    return hlpuart1.hdmatx != NULL &&
           hlpuart1.gState == HAL_UART_STATE_READY &&
           hlpuart1.Lock == HAL_UNLOCKED &&
           HAL_DMA_GetState(hlpuart1.hdmatx) == HAL_DMA_STATE_READY &&
           (hlpuart1.Instance->CR3 & USART_CR3_DMAT) == 0U &&
           (hlpuart1.Instance->CR1 & USART_CR1_TCIE) == 0U;
}

static void UARTLog_Pop(void)
{
    head = (uint8_t)((head + 1U) % UART_LOG_QUEUE_SIZE);
    queued--;
    active = 0U;
}

void UARTLog_Init(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

int _write(int file, char *data, int length)
{
    if (file != 1 && file != 2)
    {
        errno = EBADF;
        return -1;
    }
    if (length < 0 || (data == NULL && length != 0))
    {
        errno = EINVAL;
        return -1;
    }

    for (int i = 0; i < length; i++)
    {
        uint8_t ch = (uint8_t)data[i];
        if (discard_line)
        {
            if (ch == '\n') discard_line = 0U;
            continue;
        }
        if (pending_length == UART_LOG_LINE_SIZE)
        {
            dropped_count++;
            pending_length = 0U;
            discard_line = ch != '\n';
            continue;
        }
        pending_line[pending_length++] = ch;
        if (ch != '\n') continue;

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (queued == UART_LOG_QUEUE_SIZE)
        {
            dropped_count++;
        }
        else
        {
            memcpy(lines[tail], pending_line, pending_length);
            lengths[tail] = pending_length;
            tail = (uint8_t)((tail + 1U) % UART_LOG_QUEUE_SIZE);
            queued++;
        }
        __set_PRIMASK(primask);
        pending_length = 0U;
    }
    return length;
}

void UARTLog_Task(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (recovering)
    {
        if (hlpuart1.hdmatx != NULL && hlpuart1.gState == HAL_UART_STATE_READY &&
            (hlpuart1.Instance->CR3 & USART_CR3_DMAT) == 0U &&
            HAL_DMA_GetState(hlpuart1.hdmatx) == HAL_DMA_STATE_BUSY)
        {
            HAL_DMA_Abort_IT(hlpuart1.hdmatx);
        }
        if (!abort_pending)
        {
            abort_pending = 1U;
            if (HAL_UART_AbortTransmit_IT(&hlpuart1) != HAL_OK)
            {
                abort_pending = 0U;
            }
        }
        if (!UARTLog_InterfaceReady())
        {
            __set_PRIMASK(primask);
            return;
        }
        if (active)
        {
            UARTLog_Pop();
            dropped_count++;
        }
        recovering = 0U;
        abort_pending = 0U;
    }
    if (!active && queued && UARTLog_InterfaceReady())
    {
        active = 1U;
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&hlpuart1, lines[head], lengths[head]);
        if (status == HAL_BUSY)
        {
            active = 0U;
            busy_count++;
        }
        else if (status != HAL_OK)
        {
            recovering = 1U;
            error_count++;
        }
    }
    __set_PRIMASK(primask);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (huart == &hlpuart1 && active && !recovering && UARTLog_InterfaceReady())
    {
        UARTLog_Pop();
    }
    __set_PRIMASK(primask);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (huart == &hlpuart1 && huart->ErrorCode != HAL_UART_ERROR_NONE && !recovering)
    {
        recovering = 1U;
        error_count++;
    }
    __set_PRIMASK(primask);
}

void UARTLog_GetDiagSnapshot(UARTLog_Diag_t *out)
{
    if (out == NULL) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out->dropped_count = dropped_count;
    out->error_count = error_count;
    out->busy_count = busy_count;
    out->queued = queued;
    out->active = active;
    out->recovering = recovering;
    __set_PRIMASK(primask);
}
