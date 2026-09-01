/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t FDCAN_DLC_ToBytes(uint32_t dlc)
{
  switch (dlc)
  {
    case FDCAN_DLC_BYTES_0: return 0;
    case FDCAN_DLC_BYTES_1: return 1;
    case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3;
    case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5;
    case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7;
    case FDCAN_DLC_BYTES_8: return 8;
    default: return 0;
  }
}

static void CAN_Test_Init(void)
{
  FDCAN_FilterTypeDef sFilterConfig = {0};

  // Filtr dla standardowych ramek 11-bit
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x000;   // maska 0 = przyjmij wszystkie ID

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Filtr dla rozszerzonych ramek 29-bit, tylko jeśli w CubeMX dałeś Ext Filters Nbr = 1
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;   // maska 0 = przyjmij wszystkie ID

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  printf("FDCAN listening started\r\n");
}

static void CAN_Send_Test_Frame(void)
{
  static uint8_t counter = 0;

  FDCAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, counter++};

  txHeader.Identifier = 0x123;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
  {
    printf("FDCAN TX error\r\n");
  }
}


static void CAN_Send_OBD_RPM_Request(void)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  // OBD-II request: Mode 01, PID 0C = Engine RPM
  uint8_t txData[8] = {0x02, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00};

  txHeader.Identifier = 0x7DF;              // broadcast OBD-II request
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
  {
    printf("OBD RPM request TX error\r\n");
  }
  else
  {
    printf("OBD RPM request sent\r\n");
  }
}



static void CAN_Send_OBD_Speed_Request(void)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  // OBD-II request: Mode 01, PID 0D = Vehicle Speed
  uint8_t txData[8] = {0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00};

  txHeader.Identifier = 0x7DF;              // broadcast OBD-II request
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
  {
    printf("OBD speed request TX error\r\n");
  }
}


static void CAN_Send_OBD_Accelerator_Request(void)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  // OBD-II request: Mode 01, PID 5A = Relative Accelerator Pedal Position
  uint8_t txData[8] = {0x02, 0x01, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00};

  txHeader.Identifier = 0x7DF;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
  {
    printf("OBD accelerator request TX error\r\n");
  }
}




static void CAN_Send_OBD_Request(uint8_t pid)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  uint8_t txData[8] = {0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00};

  txHeader.Identifier = 0x7DF;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
  {
    printf("OBD request TX error, PID=0x%02X\r\n", pid);
  }
  else
  {
    printf("REQ PID=0x%02X\r\n", pid);
  }
}



//algorytm sterowania skrzydłem

#define SPEED_REQUEST_PERIOD_MS        500U
#define MIN_SPEED_SAMPLE_DT_MS         300U

/*
 * OPEN = redukcja oporu.
 * Wlaczamy dopiero powyzej 55 km/h,
 * wylaczamy dopiero ponizej 45 km/h.
 */
#define OPEN_ON_SPEED_KMH              55U
#define OPEN_OFF_SPEED_KMH             45U

/*
 * Jednostka: 0.1 km/h/s
 *
 * 25 = 2.5 km/h/s
 * 55 = 5.5 km/h/s
 * 20 = 2.0 km/h/s
 */
#define ACCEL_THRESHOLD_X10            25
#define BRAKE_ON_THRESHOLD_X10         55
#define OPEN_DECEL_BLOCK_X10           20

/*
 * AIR_BRAKE ponizej tej predkosci nie ma duzego sensu.
 */
#define AIR_BRAKE_MIN_SPEED_KMH        40U

/*
 * Po AIR_BRAKE wymuszamy NORMAL przez kilka probek.
 * Przy probkowaniu 500 ms:
 * 3 probki = okolo 1.5 s.
 */
#define POST_BRAKE_NORMAL_SAMPLES      3U

/*
 * OPEN dopiero po kilku kolejnych probkach bez wyraznego zwalniania.
 */
#define OPEN_STABLE_SAMPLES            3U

typedef enum
{
  MOTION_ACCELERATION = 0,
  MOTION_COASTING,
  MOTION_BREAKING
} MotionState_t;

typedef enum
{
  WING_NORMAL = 0,
  WING_OPEN,
  WING_AIR_BRAKE
} WingState_t;

static uint8_t prev_speed_kmh = 0;
static uint32_t prev_speed_tick = 0;
static uint8_t speed_initialized = 0;

static MotionState_t motion_state = MOTION_COASTING;
static WingState_t wing_state = WING_NORMAL;

static uint8_t post_brake_normal_samples = 0;
static uint8_t open_stable_samples = 0;


//funkcje pomocnicze


static const char* WingState_ToString(WingState_t state)
{
  switch (state)
  {
    case WING_NORMAL:    return "NORMAL";
    case WING_OPEN:      return "OPEN";
    case WING_AIR_BRAKE: return "AIR_BRAKE";
    default:             return "UNKNOWN";
  }
}

static void Print_Signed_X10(int32_t value_x10)
{
  if (value_x10 < 0)
  {
    printf("-");
    value_x10 = -value_x10;
  }

  printf("%ld.%ld",
         (long)(value_x10 / 10),
         (long)(value_x10 % 10));
}



static int32_t RateX10_To_mG(int32_t rate_x10)
{
  /*
   * rate_x10: jednostka 0.1 km/h/s
   *
   * 1 g = okolo 35.3 km/h/s
   *
   * wynik: milli-g
   * np. -110 oznacza -0.110 g
   */
  return (rate_x10 * 1000L) / 353L;
}

static void Print_Signed_mG_As_G(int32_t value_mg)
{
  if (value_mg < 0)
  {
    printf("-");
    value_mg = -value_mg;
  }

  printf("%ld.%03ld",
         (long)(value_mg / 1000),
         (long)(value_mg % 1000));
}



///
///
//
//SERWO
#define SERVO_MIN_PULSE_US        1000U
#define SERVO_MAX_PULSE_US        2000U

/*
 * Znaczenie aerodynamiczne:
 *
 * OPEN       = maly kat / okolo 0 stopni / redukcja oporu
 * NORMAL     = lekki kat natarcia / generowanie docisku
 * AIR_BRAKE  = najwiekszy kat / maksymalny opor
 *
 * Uwaga: to sa wartosci robocze. Po montazu mechaniki dobierzemy je dokladnie.
 */
#define SERVO_OPEN_PULSE_US       1500U
#define SERVO_NORMAL_PULSE_US     1700U
#define SERVO_AIR_BRAKE_PULSE_US  2000U

static uint16_t servo_current_pulse_us = SERVO_OPEN_PULSE_US;

static void Servo_SetPulseUs(uint16_t pulse_us)
{
  if (pulse_us < SERVO_MIN_PULSE_US)
  {
    pulse_us = SERVO_MIN_PULSE_US;
  }

  if (pulse_us > SERVO_MAX_PULSE_US)
  {
    pulse_us = SERVO_MAX_PULSE_US;
  }

  servo_current_pulse_us = pulse_us;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
}

static void Servo_SetWingState(WingState_t state)
{
  switch (state)
  {
    case WING_OPEN:
      Servo_SetPulseUs(SERVO_OPEN_PULSE_US);
      break;

    case WING_NORMAL:
      Servo_SetPulseUs(SERVO_NORMAL_PULSE_US);
      break;

    case WING_AIR_BRAKE:
      Servo_SetPulseUs(SERVO_AIR_BRAKE_PULSE_US);
      break;

    default:
      Servo_SetPulseUs(SERVO_NORMAL_PULSE_US);
      break;
  }
}

static WingState_t last_servo_wing_state = WING_NORMAL;
static uint8_t servo_state_initialized = 0;

static void Servo_UpdateWingStateIfChanged(WingState_t new_state)
{
  if (servo_state_initialized == 0 || new_state != last_servo_wing_state)
  {
    Servo_SetWingState(new_state);

    last_servo_wing_state = new_state;
    servo_state_initialized = 1;

    printf("SERVO STATE = %s | pulse=%u us\r\n",
           WingState_ToString(new_state),
           (unsigned int)servo_current_pulse_us);
  }
}


//
//
//
//Algorytm

static void Control_Update_From_Speed(uint8_t speed_kmh)
{
  uint32_t now = HAL_GetTick();

  if (speed_initialized == 0)
  {
    speed_initialized = 1;
    prev_speed_kmh = speed_kmh;
    prev_speed_tick = now;

    motion_state = MOTION_COASTING;
    wing_state = WING_NORMAL;

    post_brake_normal_samples = 0;
    open_stable_samples = 0;

    Servo_UpdateWingStateIfChanged(wing_state);

    printf("SPEED=%u km/h | Acceleration=0 Coasting=1 Breaking=0 | WING=%s\r\n",
           (unsigned int)speed_kmh,
           WingState_ToString(wing_state));

    return;
  }

  uint32_t dt_ms = now - prev_speed_tick;

  if (dt_ms < MIN_SPEED_SAMPLE_DT_MS)
  {
    return;
  }

  int16_t delta_speed = (int16_t)speed_kmh - (int16_t)prev_speed_kmh;

  /*
   * rate_x10 = zmiana predkosci w 0.1 km/h/s
   * np. 39 oznacza 3.9 km/h/s
   */
  int32_t rate_x10 = ((int32_t)delta_speed * 10000L) / (int32_t)dt_ms;
  int32_t accel_mg = RateX10_To_mG(rate_x10);

  uint8_t brake_detected = 0;
  uint8_t open_allowed_by_decel = 0;

  /*
   * AIR_BRAKE tylko przy mocniejszym hamowaniu
   * i przy sensownej predkosci.
   */
  if (rate_x10 <= -BRAKE_ON_THRESHOLD_X10 &&
      speed_kmh >= AIR_BRAKE_MIN_SPEED_KMH)
  {
    brake_detected = 1;
  }

  /*
   * OPEN moze sie wlaczyc tylko wtedy,
   * gdy nie zwalniamy wyraznie.
   *
   * Dopuszczamy male wahania, np. -1.9 km/h/s,
   * bo PID predkosci ma rozdzielczosc 1 km/h.
   */
  if (rate_x10 >= -OPEN_DECEL_BLOCK_X10)
  {
    open_allowed_by_decel = 1;
  }

  if (rate_x10 >= ACCEL_THRESHOLD_X10)
  {
    motion_state = MOTION_ACCELERATION;
  }
  else if (brake_detected)
  {
    motion_state = MOTION_BREAKING;
  }
  else
  {
    motion_state = MOTION_COASTING;
  }

  /*
   * Logika stanow:
   *
   * 1. Mocne hamowanie -> AIR_BRAKE
   * 2. Koniec hamowania -> od razu NORMAL
   * 3. Po AIR_BRAKE blokujemy OPEN przez kilka probek
   * 4. OPEN wlaczamy dopiero po kilku probkach bez zwalniania
   */

  if (brake_detected)
  {
    wing_state = WING_AIR_BRAKE;

    post_brake_normal_samples = POST_BRAKE_NORMAL_SAMPLES;
    open_stable_samples = 0;
  }
  else if (wing_state == WING_AIR_BRAKE)
  {
    /*
     * Tu NIE trzymamy AIR_BRAKE.
     * Skoro hamowanie juz nie jest wykryte, natychmiast wracamy do NORMAL.
     */
    wing_state = WING_NORMAL;

    post_brake_normal_samples = POST_BRAKE_NORMAL_SAMPLES;
    open_stable_samples = 0;
  }
  else if (post_brake_normal_samples > 0)
  {
    /*
     * Po AIR_BRAKE przez kilka probek wymuszamy NORMAL,
     * zeby nie bylo przejscia AIR_BRAKE -> OPEN.
     */
    wing_state = WING_NORMAL;
    post_brake_normal_samples--;

    open_stable_samples = 0;
  }
  else
  {
    if (speed_kmh <= OPEN_OFF_SPEED_KMH)
    {
      wing_state = WING_NORMAL;
      open_stable_samples = 0;
    }
    else if (speed_kmh >= OPEN_ON_SPEED_KMH)
    {
      if (open_allowed_by_decel)
      {
        if (open_stable_samples < OPEN_STABLE_SAMPLES)
        {
          open_stable_samples++;
        }
      }
      else
      {
        open_stable_samples = 0;
        wing_state = WING_NORMAL;
      }

      if (open_stable_samples >= OPEN_STABLE_SAMPLES)
      {
        wing_state = WING_OPEN;
      }
      else
      {
        wing_state = WING_NORMAL;
      }
    }
    else
    {
      /*
       * Strefa histerezy 45-55 km/h.
       * Jesli juz bylo OPEN i nie zwalniamy wyraznie, mozemy je utrzymac.
       * Jesli nie bylo OPEN, zostajemy w NORMAL.
       */
      if (wing_state == WING_OPEN && open_allowed_by_decel)
      {
        wing_state = WING_OPEN;
      }
      else
      {
        wing_state = WING_NORMAL;
        open_stable_samples = 0;
      }
    }
  }

  Servo_UpdateWingStateIfChanged(wing_state);

  printf("SPEED=%u km/h | dt=%lu ms | dV=%d km/h | rate=",
         (unsigned int)speed_kmh,
         (unsigned long)dt_ms,
         (int)delta_speed);

  Print_Signed_X10(rate_x10);

  printf(" km/h/s | a=");

  Print_Signed_mG_As_G(accel_mg);

  printf(" g | openCnt=%u postBrake=%u | Acceleration=%u Coasting=%u Breaking=%u | WING=%s\r\n",
         (unsigned int)open_stable_samples,
         (unsigned int)post_brake_normal_samples,
         (motion_state == MOTION_ACCELERATION) ? 1U : 0U,
         (motion_state == MOTION_COASTING) ? 1U : 0U,
         (motion_state == MOTION_BREAKING) ? 1U : 0U,
         WingState_ToString(wing_state));

  prev_speed_kmh = speed_kmh;
  prev_speed_tick = now;
}

//
//
// Wywołanie

static void CAN_Poll_Rx(void)
{
  FDCAN_RxHeaderTypeDef rxHeader = {0};
  uint8_t rxData[8] = {0};

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
      uint8_t len = FDCAN_DLC_ToBytes(rxHeader.DataLength);

      /*
       * Standard OBD-II positive response:
       * ID   = 0x7E8 ... 0x7EF
       * DATA = 03 41 0D A ...
       *
       * 0x41 = positive response to Mode 01
       * 0x0D = Vehicle Speed PID
       * A    = speed in km/h
       */
      if (rxHeader.IdType == FDCAN_STANDARD_ID &&
          rxHeader.Identifier >= 0x7E8 &&
          rxHeader.Identifier <= 0x7EF &&
          len >= 4 &&
          rxData[1] == 0x41 &&
          rxData[2] == 0x0D)
      {
        uint8_t speed_kmh = rxData[3];

        Control_Update_From_Speed(speed_kmh);
      }

      /*
       * Optional diagnostic: negative response
       * Example: 03 7F 01 12 ...
       */
      else if (rxHeader.IdType == FDCAN_STANDARD_ID &&
               rxHeader.Identifier >= 0x7E8 &&
               rxHeader.Identifier <= 0x7EF &&
               len >= 4 &&
               rxData[1] == 0x7F)
      {
        printf("NEGATIVE RESPONSE: service=0x%02X code=0x%02X\r\n",
               rxData[2],
               rxData[3]);
      }
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to send message over COM1 port ---- */
  printf("Welcome to STM32 world !\n\r");

  CAN_Test_Init();

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  Servo_SetWingState(WING_NORMAL);

  printf("Servo PWM ready: NORMAL\r\n");

  /* -- Sample board code to switch on led ---- */
  BSP_LED_On(LED_GREEN);

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    CAN_Poll_Rx();
    static uint32_t last_request = 0;

    if (HAL_GetTick() - last_request >= SPEED_REQUEST_PERIOD_MS)
    {
      last_request = HAL_GetTick();


      BSP_LED_Toggle(LED_GREEN);
      CAN_Send_OBD_Request(0x0D);

    }

    HAL_Delay(10);



/*
	  Servo_SetPulseUs(SERVO_OPEN_PULSE_US);
	  printf("SERVO TEST: 1500 us\r\n");
	  HAL_Delay(2000);

	  Servo_SetPulseUs(SERVO_NORMAL_PULSE_US);
	  printf("SERVO TEST: 1000 us\r\n");
	  HAL_Delay(2000);

	  Servo_SetPulseUs(SERVO_AIR_BRAKE_PULSE_US);
	  printf("SERVO TEST: 2000 us\r\n");
	  HAL_Delay(2000);

	  Servo_SetPulseUs(1500);
	  printf("SERVO TEST: 1500 us\r\n");
	  HAL_Delay(2000);
   */


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 20;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
