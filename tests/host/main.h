#ifndef HOST_MAIN_H_
#define HOST_MAIN_H_
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef enum { HAL_I2C_STATE_READY, HAL_I2C_STATE_BUSY_RX, HAL_I2C_STATE_ABORT } HAL_I2C_StateTypeDef;
typedef enum { HAL_DMA_STATE_READY, HAL_DMA_STATE_BUSY } HAL_DMA_StateTypeDef;
typedef enum { HAL_UNLOCKED, HAL_LOCKED } HAL_LockTypeDef;
typedef enum { HAL_I2C_MODE_NONE, HAL_I2C_MODE_MEM } HAL_I2C_ModeTypeDef;
typedef struct { uint32_t CR1, ISR; } I2C_TypeDef;
typedef struct { HAL_DMA_StateTypeDef State; } DMA_HandleTypeDef;
typedef struct { uint32_t CR1, CR3; } USART_TypeDef;
typedef enum { HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_TX } HAL_UART_StateTypeDef;
typedef struct {
    USART_TypeDef *Instance;
    HAL_UART_StateTypeDef gState;
    HAL_LockTypeDef Lock;
    DMA_HandleTypeDef *hdmatx;
    uint32_t ErrorCode;
} UART_HandleTypeDef;
typedef struct {
    I2C_TypeDef *Instance;
    HAL_I2C_StateTypeDef State;
    HAL_I2C_ModeTypeDef Mode;
    HAL_LockTypeDef Lock;
    DMA_HandleTypeDef *hdmarx;
} I2C_HandleTypeDef;
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

extern I2C_TypeDef host_i2c;
#define I2C1 (&host_i2c)
#define I2C_FLAG_BUSY 1U
#define I2C_CR1_RXDMAEN 2U
#define RESET 0U
#define USART_CR3_DMAT 1U
#define USART_CR1_TCIE 2U
#define HAL_UART_ERROR_NONE 0U
#define HAL_UART_ERROR_DMA 16U
#define HAL_UART_ERROR_FE 4U
#define LPUART1 (&host_usart)
#define __HAL_I2C_GET_FLAG(handle, flag) ((handle)->Instance->ISR & (flag))
#define __HAL_RCC_CRC_CLK_ENABLE() ((void)0)
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
extern DMA_HandleTypeDef host_dma;
extern USART_TypeDef host_usart;
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef host_uart_dma;
extern uint32_t host_uart_starts, host_uart_aborts;
extern HAL_StatusTypeDef host_uart_start_status, host_uart_abort_status;
extern uint8_t host_uart_abort_complete, host_uart_mirror, host_uart_hold;
extern const uint8_t *host_uart_data;
extern uint16_t host_uart_size;
extern uint8_t host_uart_output[262144];
extern size_t host_uart_output_size;
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void Host_UART_DMAComplete(void);
void Host_UART_Complete(void);
void Host_UART_AbortComplete(void);
void Host_UART_Poll(void);
void Host_UART_Drain(void);
extern uint32_t host_dma_starts, host_abort_calls, host_dma_abort_calls, host_primask;
extern HAL_StatusTypeDef host_read_dma_status, host_abort_status, host_dma_abort_status;
extern void (*host_before_disable_hook)(void);
void Host_CompleteIMU(uint8_t status, int16_t lateral_raw);
void Host_SetI2CReady(void);
void Host_AbortComplete(void);
void Host_I2CError(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
HAL_I2C_StateTypeDef HAL_I2C_GetState(const I2C_HandleTypeDef *);
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *, uint16_t);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *);
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
void Host_Boot(void);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *);
void BSP_LED_On(uint32_t);
void Host_FeedIMU(int16_t lateral_raw);
void Host_QueueSpeed(uint8_t speed);
#endif
