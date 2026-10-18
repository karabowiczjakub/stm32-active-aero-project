#ifndef HOST_MAIN_H_
#define HOST_MAIN_H_
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct { void *Instance; } I2C_HandleTypeDef;
typedef struct { void *Instance; } TIM_HandleTypeDef;
typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct {
    uint32_t Identifier, IdType, DataLength, TxFrameType, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef FDCAN_TxHeaderTypeDef FDCAN_RxHeaderTypeDef;
typedef struct {
    uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2;
} FDCAN_FilterTypeDef;
typedef enum { BUTTON_USER = 0 } Button_TypeDef;

#define I2C1 ((void *)1)
#define TIM2 ((void *)2)
#define TIM6 ((void *)6)
#define TIM7 ((void *)7)
#define FDCAN1 ((void *)8)
#define I2C_MEMADD_SIZE_8BIT 1U
#define TIM_CHANNEL_1 1U
#define BUTTON_RELEASED 0U
#define BUTTON_PRESSED 1U
#define LED_GREEN 0U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_EXTENDED_ID 1U
#define FDCAN_FILTER_MASK 0U
#define FDCAN_FILTER_TO_RXFIFO0 0U
#define FDCAN_ACCEPT_IN_RX_FIFO0 0U
#define FDCAN_REJECT_REMOTE 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_DLC_BYTES_0 0U
#define FDCAN_DLC_BYTES_1 1U
#define FDCAN_DLC_BYTES_2 2U
#define FDCAN_DLC_BYTES_3 3U
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_DLC_BYTES_5 5U
#define FDCAN_DLC_BYTES_6 6U
#define FDCAN_DLC_BYTES_7 7U
#define FDCAN_DLC_BYTES_8 8U

extern uint32_t host_tick, host_pwm_writes, host_can_tx, host_fx_calls;
extern float host_fx_acc[3], host_fx_gyro[3];
extern uint32_t BspButtonState;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2, htim6, htim7;
extern FDCAN_HandleTypeDef hfdcan1;
uint32_t HAL_GetTick(void);
void Error_Handler(void);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
void Host_SetCompare(TIM_HandleTypeDef *, uint32_t, uint16_t);
#define __HAL_TIM_SET_COMPARE(timer, channel, pulse) Host_SetCompare(timer, channel, pulse)
#define __WFI() ((void)0)
void BSP_LED_Toggle(uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, FDCAN_TxHeaderTypeDef *, uint8_t *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
void BSP_PB_Callback(Button_TypeDef);
void Host_MainLoopOnce(void);
void Host_FeedIMU(int16_t lateral_raw);
void Host_QueueSpeed(uint8_t speed);
#endif
