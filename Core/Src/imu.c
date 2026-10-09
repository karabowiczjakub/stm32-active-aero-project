/*
 * imu.c
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */


#include "imu.h"

#include "main.h"
#include "debug_utils.h"

#include <stdio.h>


extern I2C_HandleTypeDef hi2c1;


/* =========================
 * LSM6DSO
 * =========================
 */

#define LSM6DSO_ADDR_LOW          (0x6A << 1)
#define LSM6DSO_ADDR_HIGH         (0x6B << 1)

#define LSM6DSO_WHO_AM_I_REG      0x0F
#define LSM6DSO_WHO_AM_I_VALUE    0x6C

#define LSM6DSO_CTRL1_XL          0x10
#define LSM6DSO_CTRL2_G           0x11
#define LSM6DSO_CTRL3_C           0x12
#define LSM6DSO_CTRL9_XL          0x18

#define LSM6DSO_OUTX_L_A          0x28

#define LSM6DSO_STATUS_REG         0x1E
#define LSM6DSO_STATUS_XLDA        0x01U



static uint16_t lsm6dso_addr =
    LSM6DSO_ADDR_HIGH;

static uint8_t imu_dma_buffer[6];

static volatile uint8_t imu_dma_busy = 0U;
static volatile uint8_t imu_sample_ready = 0U;



static uint8_t imu_status_reg = 0U;

static volatile uint8_t imu_read_due = 0U;
static volatile uint8_t imu_accel_read_due = 0U;

typedef enum
{
    IMU_DMA_NONE = 0,
    IMU_DMA_STATUS,
    IMU_DMA_ACCEL

} ImuDmaTransfer_t;

static volatile ImuDmaTransfer_t imu_dma_transfer =
    IMU_DMA_NONE;


void IMU_RequestSample(void)
{
    imu_read_due = 1U;
}

static HAL_StatusTypeDef LSM6DSO_ReadReg(
    uint8_t reg,
    uint8_t *data
)
{
    return HAL_I2C_Mem_Read(
        &hi2c1,
        lsm6dso_addr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        1,
        100
    );
}



static HAL_StatusTypeDef LSM6DSO_WriteReg(
    uint8_t reg,
    uint8_t value
)
{
    return HAL_I2C_Mem_Write(
        &hi2c1,
        lsm6dso_addr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        100
    );
}



static HAL_StatusTypeDef LSM6DSO_ReadMulti(
    uint8_t start_reg,
    uint8_t *data,
    uint16_t len
)
{
    return HAL_I2C_Mem_Read(
        &hi2c1,
        lsm6dso_addr,
        start_reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        100
    );
}



static uint8_t LSM6DSO_StartAccelRead_DMA(void)
{
    if (imu_dma_busy != 0U)
    {
        return 0U;
    }

    imu_dma_busy = 1U;
    imu_dma_transfer = IMU_DMA_ACCEL;

    if (HAL_I2C_Mem_Read_DMA(
            &hi2c1,
            lsm6dso_addr,
            LSM6DSO_OUTX_L_A,
            I2C_MEMADD_SIZE_8BIT,
            imu_dma_buffer,
            6U) != HAL_OK)
    {
        imu_dma_busy = 0U;
        imu_dma_transfer = IMU_DMA_NONE;

        return 0U;
    }

    return 1U;
}




static int32_t LSM6DSO_AccelRaw_To_mG(
    int16_t raw
)
{
    return ((int32_t)raw * 61L) / 1000L;
}

static uint8_t LSM6DSO_StartStatusRead_DMA(void)
{
    if (imu_dma_busy != 0U)
    {
        return 0U;
    }

    imu_dma_busy = 1U;
    imu_dma_transfer = IMU_DMA_STATUS;

    if (HAL_I2C_Mem_Read_DMA(
            &hi2c1,
            lsm6dso_addr,
            LSM6DSO_STATUS_REG,
            I2C_MEMADD_SIZE_8BIT,
            &imu_status_reg,
            1U) != HAL_OK)
    {
        imu_dma_busy = 0U;
        imu_dma_transfer = IMU_DMA_NONE;

        return 0U;
    }

    return 1U;
}

uint8_t LSM6DSO_Init(void)
{
    uint8_t who = 0;

    uint8_t reg = 0;


    lsm6dso_addr =
        LSM6DSO_ADDR_LOW;


    if (HAL_I2C_Mem_Read(
            &hi2c1,
            lsm6dso_addr,
            LSM6DSO_WHO_AM_I_REG,
            I2C_MEMADD_SIZE_8BIT,
            &who,
            1,
            100
        ) != HAL_OK ||
        who != LSM6DSO_WHO_AM_I_VALUE)
    {
        lsm6dso_addr =
            LSM6DSO_ADDR_HIGH;


        if (HAL_I2C_Mem_Read(
                &hi2c1,
                lsm6dso_addr,
                LSM6DSO_WHO_AM_I_REG,
                I2C_MEMADD_SIZE_8BIT,
                &who,
                1,
                100
            ) != HAL_OK ||
            who != LSM6DSO_WHO_AM_I_VALUE)
        {
            printf(
                "LSM6DSO not found, WHO_AM_I=0x%02X\r\n",
                who
            );

            return 0;
        }
    }


    printf(
        "LSM6DSO found, addr=0x%02X, WHO_AM_I=0x%02X\r\n",
        (unsigned int)(lsm6dso_addr >> 1),
        (unsigned int)who
    );


    if (LSM6DSO_WriteReg(
            LSM6DSO_CTRL3_C,
            0x44
        ) != HAL_OK)
    {
        printf(
            "LSM6DSO CTRL3_C write error\r\n"
        );

        return 0;
    }


    if (LSM6DSO_ReadReg(
            LSM6DSO_CTRL9_XL,
            &reg
        ) == HAL_OK)
    {
        reg |= 0x02;


        LSM6DSO_WriteReg(
            LSM6DSO_CTRL9_XL,
            reg
        );
    }


    LSM6DSO_WriteReg(
        LSM6DSO_CTRL2_G,
        0x00
    );


    if (LSM6DSO_WriteReg(
            LSM6DSO_CTRL1_XL,
            0x40
        ) != HAL_OK)
    {
        printf(
            "LSM6DSO CTRL1_XL write error\r\n"
        );

        return 0;
    }


    printf(
        "LSM6DSO accel configured: +-2g, 104 Hz\r\n"
    );


    return 1;
}



static uint8_t LSM6DSO_ReadAccel_mG(
    int32_t *ax_mg,
    int32_t *ay_mg,
    int32_t *az_mg
)
{
    uint8_t data[6];


    if (LSM6DSO_ReadMulti(
            LSM6DSO_OUTX_L_A,
            data,
            6
        ) != HAL_OK)
    {
        return 0;
    }


    int16_t raw_x =
        (int16_t)(
            (uint16_t)data[1] << 8 |
            data[0]
        );


    int16_t raw_y =
        (int16_t)(
            (uint16_t)data[3] << 8 |
            data[2]
        );


    int16_t raw_z =
        (int16_t)(
            (uint16_t)data[5] << 8 |
            data[4]
        );


    *ax_mg =
        LSM6DSO_AccelRaw_To_mG(raw_x);


    *ay_mg =
        LSM6DSO_AccelRaw_To_mG(raw_y);


    *az_mg =
        LSM6DSO_AccelRaw_To_mG(raw_z);


    return 1;
}



static void LSM6DSO_PrintAccelInline(void)
{
    int32_t ax_mg = 0;
    int32_t ay_mg = 0;
    int32_t az_mg = 0;


    if (LSM6DSO_ReadAccel_mG(
            &ax_mg,
            &ay_mg,
            &az_mg
        ))
    {
        printf(" | ax=");

        Print_Signed_mG_As_G(ax_mg);

        printf("g ay=");

        Print_Signed_mG_As_G(ay_mg);

        printf("g az=");

        Print_Signed_mG_As_G(az_mg);

        printf("g");
    }
    else
    {
        printf(" | IMU_READ_ERR");
    }
}



/* =========================
 * DETEKCJA ZAKRĘTU
 * =========================
 */

typedef enum
{
    IMU_AXIS_X = 0,
    IMU_AXIS_Y,
    IMU_AXIS_Z

} ImuAxis_t;


#define CORNER_AXIS                    IMU_AXIS_Y

#define CORNER_ON_THRESHOLD_MG         200
#define CORNER_OFF_THRESHOLD_MG        120

#define CORNER_ON_SAMPLES              3U
#define CORNER_OFF_SAMPLES             5U

#define IMU_SAMPLE_PERIOD_MS           20U



static uint8_t corner_detected = 0;

static uint8_t corner_on_counter = 0;

static uint8_t corner_off_counter = 0;


static int32_t imu_ax_mg = 0;

static int32_t imu_ay_mg = 0;

static int32_t imu_az_mg = 0;


static int32_t imu_lateral_raw_mg = 0;

static int32_t imu_lateral_filtered_mg = 0;


static uint8_t imu_lateral_filter_initialized = 0;



static uint8_t corner_detected_event = 0;



static int32_t Abs32(int32_t value)
{
    if (value < 0)
    {
        return -value;
    }

    return value;
}



static int32_t IMU_SelectAxis_mG(
    int32_t ax_mg,
    int32_t ay_mg,
    int32_t az_mg,
    ImuAxis_t axis
)
{
    switch (axis)
    {
        case IMU_AXIS_X:
            return ax_mg;

        case IMU_AXIS_Y:
            return ay_mg;

        case IMU_AXIS_Z:
            return az_mg;

        default:
            return ay_mg;
    }
}



static void IMU_UpdateCornerDetection(void)
{

	int16_t raw_x =
	    (int16_t)(
	        ((uint16_t)imu_dma_buffer[1] << 8) |
	        imu_dma_buffer[0]
	    );

	int16_t raw_y =
	    (int16_t)(
	        ((uint16_t)imu_dma_buffer[3] << 8) |
	        imu_dma_buffer[2]
	    );

	int16_t raw_z =
	    (int16_t)(
	        ((uint16_t)imu_dma_buffer[5] << 8) |
	        imu_dma_buffer[4]
	    );


	int32_t ax =
	    LSM6DSO_AccelRaw_To_mG(raw_x);

	int32_t ay =
	    LSM6DSO_AccelRaw_To_mG(raw_y);

	int32_t az =
	    LSM6DSO_AccelRaw_To_mG(raw_z);


	imu_ax_mg = ax;
	imu_ay_mg = ay;
	imu_az_mg = az;

    imu_lateral_raw_mg =
        IMU_SelectAxis_mG(
            ax,
            ay,
            az,
            CORNER_AXIS
        );


    if (imu_lateral_filter_initialized == 0)
    {
        imu_lateral_filtered_mg =
            imu_lateral_raw_mg;


        imu_lateral_filter_initialized = 1;
    }
    else
    {
        imu_lateral_filtered_mg +=
            (
                imu_lateral_raw_mg -
                imu_lateral_filtered_mg
            ) / 4;
    }


    int32_t lateral_abs_mg =
        Abs32(
            imu_lateral_filtered_mg
        );


    if (corner_detected == 0)
    {
        if (lateral_abs_mg >=
            CORNER_ON_THRESHOLD_MG)
        {
            if (corner_on_counter <
                CORNER_ON_SAMPLES)
            {
                corner_on_counter++;
            }
        }
        else
        {
            corner_on_counter = 0;
        }


        if (corner_on_counter >=
            CORNER_ON_SAMPLES)
        {
            corner_detected = 1;

            corner_off_counter = 0;


            /*
             * main() po IMU_Task()
             * natychmiast obsłuży to zdarzenie.
             */
            corner_detected_event = 1;


            printf(
                "CORNER DETECTED | lateral="
            );


            Print_Signed_mG_As_G(
                imu_lateral_filtered_mg
            );


            printf(" g\r\n");
        }
    }
    else
    {
        if (lateral_abs_mg <=
            CORNER_OFF_THRESHOLD_MG)
        {
            if (corner_off_counter <
                CORNER_OFF_SAMPLES)
            {
                corner_off_counter++;
            }
        }
        else
        {
            corner_off_counter = 0;
        }


        if (corner_off_counter >=
            CORNER_OFF_SAMPLES)
        {
            corner_detected = 0;

            corner_on_counter = 0;


            printf(
                "CORNER CLEARED | lateral="
            );


            Print_Signed_mG_As_G(
                imu_lateral_filtered_mg
            );


            printf(" g\r\n");
        }
    }
}


void HAL_I2C_MemRxCpltCallback(
    I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }

    imu_dma_busy = 0U;


    if (imu_dma_transfer == IMU_DMA_STATUS)
    {
        imu_dma_transfer = IMU_DMA_NONE;

        /*
         * XLDA = 1:
         * akcelerometr ma nowa probke.
         */
        if ((imu_status_reg &
             LSM6DSO_STATUS_XLDA) != 0U)
        {
            imu_accel_read_due = 1U;
        }
    }

    else if (imu_dma_transfer == IMU_DMA_ACCEL)
    {
        imu_dma_transfer = IMU_DMA_NONE;


        imu_sample_ready = 1U;
    }
}


void HAL_I2C_ErrorCallback(
    I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        imu_dma_busy = 0U;
        imu_dma_transfer = IMU_DMA_NONE;

        imu_accel_read_due = 0U;
    }
}


void IMU_Task(void)
{

    if (imu_sample_ready != 0U)
    {
        imu_sample_ready = 0U;

        IMU_UpdateCornerDetection();
    }



    if (imu_dma_busy != 0U)
    {
        return;
    }



    if (imu_accel_read_due != 0U)
    {
        imu_accel_read_due = 0U;

        LSM6DSO_StartAccelRead_DMA();

        return;
    }


    if (imu_read_due != 0U)
    {
        imu_read_due = 0U;

        LSM6DSO_StartStatusRead_DMA();
    }
}

void IMU_PrintCornerInline(void)
{
    printf(
        " | corner=%u lateral=",
        (unsigned int)corner_detected
    );


    Print_Signed_mG_As_G(
        imu_lateral_filtered_mg
    );


    printf("g ax=");

    Print_Signed_mG_As_G(
        imu_ax_mg
    );


    printf("g ay=");

    Print_Signed_mG_As_G(
        imu_ay_mg
    );


    printf("g az=");

    Print_Signed_mG_As_G(
        imu_az_mg
    );


    printf("g");
}



uint8_t IMU_IsCornerDetected(void)
{
    return corner_detected;
}



uint8_t IMU_TakeCornerDetectedEvent(void)
{
    uint8_t event =
        corner_detected_event;


    corner_detected_event = 0;


    return event;
}
