/*
 * obd.c
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */


#include "obd.h"

#include "main.h"
#include "wing_control.h"

#include <stdio.h>


extern FDCAN_HandleTypeDef hfdcan1;

#define CAN_RX_QUEUE_SIZE 8U
#define OBD_RESPONSE_TIMEOUT_MS 200U


typedef struct
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

} CAN_RxFrame_t;


static CAN_RxFrame_t can_rx_queue[CAN_RX_QUEUE_SIZE];

static volatile uint8_t can_rx_head = 0U;
static volatile uint8_t can_rx_tail = 0U;

static volatile uint32_t can_rx_overflow_count = 0U;

static volatile uint8_t obd_request_pending = 0U;
static uint32_t obd_request_sent_tick = 0U;

static uint8_t FDCAN_DLC_ToBytes(
    uint32_t dlc
)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0:
            return 0;

        case FDCAN_DLC_BYTES_1:
            return 1;

        case FDCAN_DLC_BYTES_2:
            return 2;

        case FDCAN_DLC_BYTES_3:
            return 3;

        case FDCAN_DLC_BYTES_4:
            return 4;

        case FDCAN_DLC_BYTES_5:
            return 5;

        case FDCAN_DLC_BYTES_6:
            return 6;

        case FDCAN_DLC_BYTES_7:
            return 7;

        case FDCAN_DLC_BYTES_8:
            return 8;

        default:
            return 0;
    }
}



void CAN_Test_Init(void)
{
    FDCAN_FilterTypeDef
        sFilterConfig = {0};


    sFilterConfig.IdType =
        FDCAN_STANDARD_ID;


    sFilterConfig.FilterIndex = 0;


    sFilterConfig.FilterType =
        FDCAN_FILTER_MASK;


    sFilterConfig.FilterConfig =
        FDCAN_FILTER_TO_RXFIFO0;


    sFilterConfig.FilterID1 =
        0x000;


    sFilterConfig.FilterID2 =
        0x000;


    if (HAL_FDCAN_ConfigFilter(
            &hfdcan1,
            &sFilterConfig
        ) != HAL_OK)
    {
        Error_Handler();
    }



    sFilterConfig.IdType =
        FDCAN_EXTENDED_ID;


    sFilterConfig.FilterIndex = 0;


    sFilterConfig.FilterType =
        FDCAN_FILTER_MASK;


    sFilterConfig.FilterConfig =
        FDCAN_FILTER_TO_RXFIFO0;


    sFilterConfig.FilterID1 =
        0x00000000;


    sFilterConfig.FilterID2 =
        0x00000000;


    if (HAL_FDCAN_ConfigFilter(
            &hfdcan1,
            &sFilterConfig
        ) != HAL_OK)
    {
        Error_Handler();
    }



    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,

            FDCAN_ACCEPT_IN_RX_FIFO0,

            FDCAN_ACCEPT_IN_RX_FIFO0,

            FDCAN_REJECT_REMOTE,

            FDCAN_REJECT_REMOTE
        ) != HAL_OK)
    {
        Error_Handler();
    }



    if (HAL_FDCAN_Start(
            &hfdcan1
        ) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(
            &hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0U) != HAL_OK)
    {
        Error_Handler();
    }


    printf(
        "FDCAN listening started\r\n"
    );
}



static void CAN_Send_Test_Frame(void)
{
    static uint8_t counter = 0;


    FDCAN_TxHeaderTypeDef
        txHeader = {0};


    uint8_t txData[8] =
    {
        0x11,
        0x22,
        0x33,
        0x44,
        0x55,
        0x66,
        0x77,
        counter++
    };


    txHeader.Identifier =
        0x123;


    txHeader.IdType =
        FDCAN_STANDARD_ID;


    txHeader.TxFrameType =
        FDCAN_DATA_FRAME;


    txHeader.DataLength =
        FDCAN_DLC_BYTES_8;


    txHeader.ErrorStateIndicator =
        FDCAN_ESI_ACTIVE;


    txHeader.BitRateSwitch =
        FDCAN_BRS_OFF;


    txHeader.FDFormat =
        FDCAN_CLASSIC_CAN;


    txHeader.TxEventFifoControl =
        FDCAN_NO_TX_EVENTS;


    txHeader.MessageMarker = 0;


    if (HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan1,
            &txHeader,
            txData
        ) != HAL_OK)
    {
        printf(
            "FDCAN TX error\r\n"
        );
    }
}



static void CAN_Send_OBD_RPM_Request(void)
{
    FDCAN_TxHeaderTypeDef
        txHeader = {0};


    uint8_t txData[8] =
    {
        0x02,
        0x01,
        0x0C,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };


    txHeader.Identifier = 0x7DF;

    txHeader.IdType =
        FDCAN_STANDARD_ID;

    txHeader.TxFrameType =
        FDCAN_DATA_FRAME;

    txHeader.DataLength =
        FDCAN_DLC_BYTES_8;

    txHeader.ErrorStateIndicator =
        FDCAN_ESI_ACTIVE;

    txHeader.BitRateSwitch =
        FDCAN_BRS_OFF;

    txHeader.FDFormat =
        FDCAN_CLASSIC_CAN;

    txHeader.TxEventFifoControl =
        FDCAN_NO_TX_EVENTS;

    txHeader.MessageMarker = 0;


    if (HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan1,
            &txHeader,
            txData
        ) != HAL_OK)
    {
        printf(
            "OBD RPM request TX error\r\n"
        );
    }
    else
    {
        printf(
            "OBD RPM request sent\r\n"
        );
    }
}



static void CAN_Send_OBD_Speed_Request(void)
{
    FDCAN_TxHeaderTypeDef
        txHeader = {0};


    uint8_t txData[8] =
    {
        0x02,
        0x01,
        0x0D,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };


    txHeader.Identifier = 0x7DF;

    txHeader.IdType =
        FDCAN_STANDARD_ID;

    txHeader.TxFrameType =
        FDCAN_DATA_FRAME;

    txHeader.DataLength =
        FDCAN_DLC_BYTES_8;

    txHeader.ErrorStateIndicator =
        FDCAN_ESI_ACTIVE;

    txHeader.BitRateSwitch =
        FDCAN_BRS_OFF;

    txHeader.FDFormat =
        FDCAN_CLASSIC_CAN;

    txHeader.TxEventFifoControl =
        FDCAN_NO_TX_EVENTS;

    txHeader.MessageMarker = 0;


    if (HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan1,
            &txHeader,
            txData
        ) != HAL_OK)
    {
        printf(
            "OBD speed request TX error\r\n"
        );
    }
}



static void CAN_Send_OBD_Accelerator_Request(void)
{
    FDCAN_TxHeaderTypeDef
        txHeader = {0};


    uint8_t txData[8] =
    {
        0x02,
        0x01,
        0x5A,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };


    txHeader.Identifier = 0x7DF;

    txHeader.IdType =
        FDCAN_STANDARD_ID;

    txHeader.TxFrameType =
        FDCAN_DATA_FRAME;

    txHeader.DataLength =
        FDCAN_DLC_BYTES_8;

    txHeader.ErrorStateIndicator =
        FDCAN_ESI_ACTIVE;

    txHeader.BitRateSwitch =
        FDCAN_BRS_OFF;

    txHeader.FDFormat =
        FDCAN_CLASSIC_CAN;

    txHeader.TxEventFifoControl =
        FDCAN_NO_TX_EVENTS;

    txHeader.MessageMarker = 0;


    if (HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan1,
            &txHeader,
            txData
        ) != HAL_OK)
    {
        printf(
            "OBD accelerator request TX error\r\n"
        );
    }
}



void CAN_Send_OBD_Request(
    uint8_t pid
)
{
    FDCAN_TxHeaderTypeDef
        txHeader = {0};


    uint8_t txData[8] =
    {
        0x02,
        0x01,
        pid,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };


    txHeader.Identifier = 0x7DF;

    txHeader.IdType =
        FDCAN_STANDARD_ID;

    txHeader.TxFrameType =
        FDCAN_DATA_FRAME;

    txHeader.DataLength =
        FDCAN_DLC_BYTES_8;

    txHeader.ErrorStateIndicator =
        FDCAN_ESI_ACTIVE;

    txHeader.BitRateSwitch =
        FDCAN_BRS_OFF;

    txHeader.FDFormat =
        FDCAN_CLASSIC_CAN;

    txHeader.TxEventFifoControl =
        FDCAN_NO_TX_EVENTS;

    txHeader.MessageMarker = 0;


    if (HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan1,
            &txHeader,
            txData
        ) != HAL_OK)
    {
        printf(
            "OBD request TX error, PID=0x%02X\r\n",
            pid
        );
    }
    else
    {
    	obd_request_pending = 1U;
    	    obd_request_sent_tick = HAL_GetTick();

    	    printf("REQ PID=0x%02X\r\n", pid);
    }
}



void CAN_Poll_Rx(void)
{
    FDCAN_RxHeaderTypeDef
        rxHeader = {0};


    uint8_t rxData[8] = {0};


    while (HAL_FDCAN_GetRxFifoFillLevel(
               &hfdcan1,
               FDCAN_RX_FIFO0
           ) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(
                &hfdcan1,
                FDCAN_RX_FIFO0,
                &rxHeader,
                rxData
            ) == HAL_OK)
        {
            uint8_t len =
                FDCAN_DLC_ToBytes(
                    rxHeader.DataLength
                );


            if (
                rxHeader.IdType ==
                    FDCAN_STANDARD_ID &&

                rxHeader.Identifier >=
                    0x7E8 &&

                rxHeader.Identifier <=
                    0x7EF &&

                len >= 4 &&

                rxData[1] == 0x41 &&

                rxData[2] == 0x0D
            )
            {
            	uint8_t speed_kmh = rxData[3];

            	obd_request_pending = 0U;

            	Control_Update_From_Speed(speed_kmh);
            }


            else if (
                rxHeader.IdType ==
                    FDCAN_STANDARD_ID &&

                rxHeader.Identifier >=
                    0x7E8 &&

                rxHeader.Identifier <=
                    0x7EF &&

                len >= 4 &&

                rxData[1] == 0x7F
            )
            {
                printf(
                    "NEGATIVE RESPONSE: service=0x%02X code=0x%02X\r\n",

                    rxData[2],

                    rxData[3]
                );
            }
        }
    }
}




void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1)
    {
        return;
    }

    if ((RxFifo0ITs &
         FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];


    if (HAL_FDCAN_GetRxMessage(
            hfdcan,
            FDCAN_RX_FIFO0,
            &rx_header,
            rx_data) != HAL_OK)
    {
        return;
    }


    uint8_t next_head =
        (uint8_t)(
            (can_rx_head + 1U) %
            CAN_RX_QUEUE_SIZE
        );


    /*
     * Jesli next_head == tail,
     * kolejka jest pelna.
     */
    if (next_head == can_rx_tail)
    {
        can_rx_overflow_count++;
        return;
    }


    can_rx_queue[can_rx_head].header =
        rx_header;


    for (uint8_t i = 0U; i < 8U; i++)
    {
        can_rx_queue[can_rx_head].data[i] =
            rx_data[i];
    }


    /*
     * head zmieniamy dopiero po
     * skopiowaniu calej ramki.
     */
    can_rx_head = next_head;
}


static void CAN_ProcessFrame(
    const FDCAN_RxHeaderTypeDef *rx_header,
    const uint8_t *rx_data)
{
    /*
     * Standardowa odpowiedz ECU:
     *
     * ID: 0x7E8 ... 0x7EF
     *
     * DATA:
     * [0] = 0x03
     * [1] = 0x41
     * [2] = 0x0D
     * [3] = SPEED
     */

    if (rx_header->IdType != FDCAN_STANDARD_ID)
    {
        return;
    }


    if (rx_header->Identifier < 0x7E8U ||
        rx_header->Identifier > 0x7EFU)
    {
        return;
    }


    if (rx_data[1] != 0x41U)
    {
        return;
    }


    if (rx_data[2] != 0x0DU)
    {
        return;
    }

    uint8_t speed_kmh = rx_data[3];

    obd_request_pending = 0U;

    Control_Update_From_Speed(
        speed_kmh
    );
}


void CAN_Process_Rx(void)
{
    while (can_rx_tail != can_rx_head)
    {
        CAN_RxFrame_t frame =
            can_rx_queue[can_rx_tail];


        can_rx_tail =
            (uint8_t)(
                (can_rx_tail + 1U) %
                CAN_RX_QUEUE_SIZE
            );


        CAN_ProcessFrame(
            &frame.header,
            frame.data
        );
    }
}


uint8_t CAN_Is_OBD_RequestPending(void)
{
    return obd_request_pending;
}


void CAN_Check_OBD_RequestTimeout(void)
{
    if (obd_request_pending == 0U)
    {
        return;
    }

    if ((HAL_GetTick() - obd_request_sent_tick) >=
        OBD_RESPONSE_TIMEOUT_MS)
    {
        obd_request_pending = 0U;
    }
}


