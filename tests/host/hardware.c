#include "main.h"
#include "imu.h"
#include "motion_fx_app.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

uint32_t host_tick, host_pwm_writes, host_can_tx, host_fx_calls;
float host_fx_acc[3], host_fx_gyro[3];
uint32_t BspButtonState;
I2C_HandleTypeDef hi2c1 = {I2C1};
TIM_HandleTypeDef htim2 = {TIM2}, htim6 = {TIM6}, htim7 = {TIM7};
FDCAN_HandleTypeDef hfdcan1 = {FDCAN1};
static uint8_t *dma_target;
static uint16_t dma_reg, dma_size;
static uint8_t queued_speed, rx_pending;

uint32_t HAL_GetTick(void) { return host_tick; }
void Error_Handler(void) { abort(); }
void BSP_LED_Toggle(uint32_t led) { (void)led; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; return HAL_OK; }
void Host_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint16_t pulse)
{
    assert(timer->Instance == TIM2 && channel == TIM_CHANNEL_1);
    assert(pulse == 1100U || pulse == 1600U || pulse == 1900U);
    host_pwm_writes++;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t a, uint16_t reg,
    uint16_t mode, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)h; (void)a; (void)mode; (void)timeout;
    memset(data, 0, size);
    if (reg == 0x0FU) data[0] = 0x6CU;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t a, uint16_t reg,
    uint16_t mode, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)h; (void)a; (void)reg; (void)mode; (void)data; (void)size; (void)timeout;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *h, uint16_t a,
    uint16_t reg, uint16_t mode, uint8_t *data, uint16_t size)
{
    (void)h; (void)a; (void)mode;
    dma_target = data; dma_reg = reg; dma_size = size;
    return HAL_OK;
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *);
void Host_FeedIMU(int16_t lateral_raw)
{
    IMU_RequestSample();
    IMU_Task();
    assert(dma_reg == 0x1EU && dma_size == 1U);
    *dma_target = 3U;
    HAL_I2C_MemRxCpltCallback(&hi2c1);
    IMU_Task();
    assert(dma_reg == 0x22U && dma_size == 12U);
    int16_t values[6] = {100, -200, 300, 1000, lateral_raw, 16384};
    for (unsigned i = 0; i < 6; i++)
    {
        dma_target[2*i] = (uint8_t)values[i];
        dma_target[2*i+1] = (uint8_t)((uint16_t)values[i] >> 8);
    }
    HAL_I2C_MemRxCpltCallback(&hi2c1);
    IMU_Task();
}
void MotionFxApp_Process(float ax, float ay, float az, float gx, float gy,
    float gz, uint32_t timestamp)
{
    assert(timestamp == host_tick);
    host_fx_acc[0] = ax; host_fx_acc[1] = ay; host_fx_acc[2] = az;
    host_fx_gyro[0] = gx; host_fx_gyro[1] = gy; host_fx_gyro[2] = gz;
    host_fx_calls++;
}
uint8_t MotionFxApp_GetLatest(MotionFxApp_Result_t *out)
{ memset(out, 0, sizeof(*out)); return 0U; }

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h, FDCAN_FilterTypeDef *f)
{ (void)h; (void)f; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h,
    uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{ (void)h; (void)a; (void)b; (void)c; (void)d; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h)
{ (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b)
{ (void)h; (void)a; (void)b; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,
    FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    (void)h;
    assert(header->Identifier == 0x7DFU && data[2] == 0x0DU);
    host_can_tx++;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; (void)fifo; return rx_pending; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo;
    assert(rx_pending);
    *header = (FDCAN_RxHeaderTypeDef){.Identifier=0x7E8U,
        .IdType=FDCAN_STANDARD_ID, .DataLength=FDCAN_DLC_BYTES_8};
    memset(data, 0, 8);
    data[0] = 3U; data[1] = 0x41U; data[2] = 0x0DU; data[3] = queued_speed;
    rx_pending = 0U;
    return HAL_OK;
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *, uint32_t);
void Host_QueueSpeed(uint8_t speed)
{
    queued_speed = speed;
    rx_pending = 1U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}
