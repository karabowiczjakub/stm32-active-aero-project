/*
 * imu.c
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */


#include "imu.h"
#include "app_config.h"

#include "main.h"
#include "debug_utils.h"

#include <stdio.h>
#include "motion_fx_app.h"

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

#define LSM6DSO_OUTX_L_G       0x22U
#define LSM6DSO_STATUS_GDA     0x02U

static uint16_t lsm6dso_addr =
    LSM6DSO_ADDR_HIGH;

static uint8_t imu_dma_buffer[12];

static volatile uint8_t imu_dma_busy = 0U;
static volatile uint8_t imu_dma_recovering = 0U;
static volatile uint32_t imu_dma_start_tick = 0U;
static uint32_t imu_dma_timeout_count = 0U;
static uint32_t imu_dma_abort_error_count = 0U;
static volatile uint8_t imu_sample_ready = 0U;



static uint8_t imu_status_reg = 0U;

static volatile uint8_t imu_read_due = 0U;
static volatile uint8_t imu_accel_read_due = 0U;

static int32_t imu_gx_mdps = 0;
static int32_t imu_gy_mdps = 0;
static int32_t imu_gz_mdps = 0;

static volatile uint32_t imu_sample_tick_ms = 0U;

static uint8_t imu_gyro_ready_for_sample = 0U;


static uint32_t imu_last_processed_tick = 0U;
static uint8_t imu_has_sample = 0U;
static uint32_t imu_sample_id = 0U;
static volatile uint32_t imu_i2c_error_count = 0U;
static uint32_t imu_dma_start_error_count = 0U;

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



static uint8_t LSM6DSO_InterfaceReady(void)
{
    return HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY &&
           hi2c1.Lock == HAL_UNLOCKED && hi2c1.hdmarx != NULL &&
           HAL_DMA_GetState(hi2c1.hdmarx) == HAL_DMA_STATE_READY &&
           __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET &&
           (hi2c1.Instance->CR1 & I2C_CR1_RXDMAEN) == 0U;
}

static uint8_t LSM6DSO_StartRead_DMA(ImuDmaTransfer_t transfer, uint8_t reg,
                                    uint8_t *data, uint16_t size)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (imu_dma_busy || imu_dma_recovering || !LSM6DSO_InterfaceReady())
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    imu_dma_busy = 1U;
    imu_dma_transfer = transfer;
    imu_dma_start_tick = HAL_GetTick();
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1, lsm6dso_addr, reg,
                                                  I2C_MEMADD_SIZE_8BIT, data, size);
    if (status != HAL_OK)
    {
        imu_dma_busy = 0U;
        imu_dma_transfer = IMU_DMA_NONE;
        imu_dma_start_error_count++;
    }
    __set_PRIMASK(primask);
    return status == HAL_OK;
}

static uint8_t LSM6DSO_StartAccelRead_DMA(void)
{
    return LSM6DSO_StartRead_DMA(IMU_DMA_ACCEL, LSM6DSO_OUTX_L_G, imu_dma_buffer, 12U);
}




static int32_t LSM6DSO_AccelRaw_To_mG(
    int16_t raw
)
{
    return ((int32_t)raw * 61L) / 1000L;
}

static uint8_t LSM6DSO_StartStatusRead_DMA(void)
{
    return LSM6DSO_StartRead_DMA(IMU_DMA_STATUS, LSM6DSO_STATUS_REG, &imu_status_reg, 1U);
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

    if (LSM6DSO_WriteReg(
            LSM6DSO_CTRL2_G,
            0x40
        ) != HAL_OK)
    {
        printf("LSM6DSO gyro configuration error\r\n");
        return 0U;
    }


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



/*
 * DETEKCJA ZAKRĘTU

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



#if !APP_LAP_SIM_ENABLED
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



#endif

void IMU_ResetCornerDetection(void)
{
    corner_detected = 0U;
    corner_on_counter = 0U;
    corner_off_counter = 0U;
    corner_detected_event = 0U;
    imu_lateral_raw_mg = 0;
    imu_lateral_filtered_mg = 0;
    imu_lateral_filter_initialized = 0U;
}

static void IMU_UpdateCornerDetection(void)
{

	/* Zyroskop */
	int16_t raw_gx =
	    (int16_t)(((uint16_t)imu_dma_buffer[1] << 8) |
	              imu_dma_buffer[0]);

	int16_t raw_gy =
	    (int16_t)(((uint16_t)imu_dma_buffer[3] << 8) |
	              imu_dma_buffer[2]);

	int16_t raw_gz =
	    (int16_t)(((uint16_t)imu_dma_buffer[5] << 8) |
	              imu_dma_buffer[4]);

	/* Akcelerometr */
	int16_t raw_x = (int16_t)(((uint16_t)imu_dma_buffer[7] << 8) |
	              imu_dma_buffer[6]);

	int16_t raw_y =
	    (int16_t)(((uint16_t)imu_dma_buffer[9] << 8) |
	              imu_dma_buffer[8]);

	int16_t raw_z =
	    (int16_t)(((uint16_t)imu_dma_buffer[11] << 8) |
	              imu_dma_buffer[10]);


	/* Zakres gyro +-250 dps: 8.75 mdps/LSB */
	imu_gx_mdps = ((int32_t)raw_gx * 875L) / 100L;
	imu_gy_mdps = ((int32_t)raw_gy * 875L) / 100L;
	imu_gz_mdps = ((int32_t)raw_gz * 875L) / 100L;

	int32_t ax =
	    LSM6DSO_AccelRaw_To_mG(raw_x);

	int32_t ay =
	    LSM6DSO_AccelRaw_To_mG(raw_y);

	int32_t az =
	    LSM6DSO_AccelRaw_To_mG(raw_z);


	imu_ax_mg = ax;
	imu_ay_mg = ay;
	imu_az_mg = az;

#if !APP_LAP_SIM_ENABLED
    IMU_ProcessLateralSample_mG(IMU_SelectAxis_mG(ax, ay, az, CORNER_AXIS));
#endif
}

void IMU_ProcessLateralSample_mG(int32_t lateral_mg)
{
    imu_lateral_raw_mg = lateral_mg;

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


            corner_detected_event = 1;


#if !APP_LAP_SIM_ENABLED
            printf(
                "CORNER DETECTED | lateral="
            );


            Print_Signed_mG_As_G(
                imu_lateral_filtered_mg
            );


            printf(" g\r\n");
#endif
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


#if !APP_LAP_SIM_ENABLED
            printf(
                "CORNER CLEARED | lateral="
            );


            Print_Signed_mG_As_G(
                imu_lateral_filtered_mg
            );


            printf(" g\r\n");
#endif
        }
    }
}


void HAL_I2C_MemRxCpltCallback(
    I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1 || !imu_dma_busy || imu_dma_recovering ||
        !LSM6DSO_InterfaceReady() ||
        (uint32_t)(HAL_GetTick() - imu_dma_start_tick) >= APP_IMU_DMA_TIMEOUT_MS)
    {
        return;
    }

    imu_dma_busy = 0U;


    if (imu_dma_transfer == IMU_DMA_STATUS)
    {
        imu_dma_transfer = IMU_DMA_NONE;
        imu_gyro_ready_for_sample =
            ((imu_status_reg & LSM6DSO_STATUS_GDA) != 0U)
            ? 1U : 0U;

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
        imu_sample_tick_ms = HAL_GetTick();
    }
}


void HAL_I2C_ErrorCallback(
    I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        imu_i2c_error_count++;
        if (!imu_dma_recovering) imu_dma_start_tick = HAL_GetTick() - APP_IMU_DMA_TIMEOUT_MS;
        imu_dma_recovering = 1U;
        imu_dma_busy = 1U;
        imu_dma_transfer = IMU_DMA_NONE;
        imu_accel_read_due = 0U;
        imu_sample_ready = 0U;
        imu_gyro_ready_for_sample = 0U;
    }
}


void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1 && imu_dma_recovering)
    {
        imu_dma_transfer = IMU_DMA_NONE;
        imu_accel_read_due = 0U;
        imu_sample_ready = 0U;
        imu_gyro_ready_for_sample = 0U;
    }
}

static void IMU_CheckDmaTimeout(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = HAL_GetTick();
    if (imu_dma_busy && !imu_dma_recovering &&
        (uint32_t)(now - imu_dma_start_tick) >= APP_IMU_DMA_TIMEOUT_MS)
    {
        imu_dma_timeout_count++;
        imu_dma_recovering = 1U;
        imu_dma_transfer = IMU_DMA_NONE;
        imu_accel_read_due = 0U;
        imu_sample_ready = 0U;
        imu_gyro_ready_for_sample = 0U;
    }
    if (imu_dma_recovering)
    {
        if (LSM6DSO_InterfaceReady())
        {
            imu_dma_recovering = 0U;
            imu_dma_busy = 0U;
            imu_read_due = 1U;
        }
        else if ((uint32_t)(now - imu_dma_start_tick) >= APP_IMU_DMA_TIMEOUT_MS &&
                 HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_ABORT)
        {
            imu_dma_start_tick = now;
            HAL_StatusTypeDef status;
            if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY &&
                hi2c1.hdmarx != NULL && HAL_DMA_GetState(hi2c1.hdmarx) == HAL_DMA_STATE_BUSY)
            {
                status = HAL_DMA_Abort_IT(hi2c1.hdmarx);
            }
            else
            {
                status = HAL_I2C_Master_Abort_IT(&hi2c1, lsm6dso_addr);
            }
            if (status != HAL_OK) imu_dma_abort_error_count++;
        }
    }
    __set_PRIMASK(primask);
}

void IMU_Task(void)
{
    IMU_CheckDmaTimeout();
    if (imu_sample_ready &&
        (uint32_t)(HAL_GetTick() - imu_sample_tick_ms) > APP_IMU_MAX_AGE_MS)
    {
        imu_sample_ready = 0U;
        imu_read_due = 1U;
    }

	if (imu_sample_ready != 0U)
	{
	    imu_sample_ready = 0U;

#if !APP_LAP_SIM_ENABLED
        if (imu_has_sample &&
            (uint32_t)(imu_sample_tick_ms - imu_last_processed_tick) > APP_IMU_MAX_AGE_MS)
        {
            IMU_ResetCornerDetection();
        }
#endif
	    IMU_UpdateCornerDetection();

	    imu_last_processed_tick = imu_sample_tick_ms;
	    imu_has_sample = 1U;
        if (++imu_sample_id == 0U) imu_sample_id = 1U;


	    if (imu_gyro_ready_for_sample != 0U)
	    {
	        MotionFxApp_Process(
	            (float)imu_ax_mg / 1000.0f,
	            (float)imu_ay_mg / 1000.0f,
	            (float)imu_az_mg / 1000.0f,

	            (float)imu_gx_mdps / 1000.0f,
	            (float)imu_gy_mdps / 1000.0f,
	            (float)imu_gz_mdps / 1000.0f,

	            imu_sample_tick_ms
	        );
	    }
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



void IMU_GetDiagSnapshot(IMU_Diag_t *out)
{
    if (out == NULL) return;
    out->valid = imu_has_sample;
    out->age_ms = imu_has_sample ?
        (HAL_GetTick() - imu_last_processed_tick) : 0xFFFFFFFFU;
    out->ax_mg = imu_ax_mg;
    out->ay_mg = imu_ay_mg;
    out->az_mg = imu_az_mg;
    out->gx_mdps = imu_gx_mdps;
    out->gy_mdps = imu_gy_mdps;
    out->gz_mdps = imu_gz_mdps;
    out->lateral_raw_mg = imu_lateral_raw_mg;
    out->lateral_filtered_mg = imu_lateral_filtered_mg;
    out->corner = corner_detected;
    out->corner_on_count = corner_on_counter;
    out->corner_off_count = corner_off_counter;
    out->i2c_error_count = imu_i2c_error_count;
    out->dma_start_error_count = imu_dma_start_error_count;
    out->dma_timeout_count = imu_dma_timeout_count;
    out->dma_abort_error_count = imu_dma_abort_error_count;
    out->dma_recovering = imu_dma_recovering;
}

uint8_t IMU_GetAccelSnapshot(
    int32_t *ax_mg,
    int32_t *ay_mg,
    int32_t *az_mg,
    uint32_t *age_ms
)
{
    if (imu_has_sample == 0U)
    {
        return 0U;
    }

    *ax_mg = imu_ax_mg;
    *ay_mg = imu_ay_mg;
    *az_mg = imu_az_mg;

    *age_ms = HAL_GetTick() - imu_last_processed_tick;

    return 1U;
}

uint8_t IMU_GetAccelerationFromSource(uint8_t source, IMU_Acceleration_t *out)
{
    if (out == NULL) return 0U;
    *out = (IMU_Acceleration_t){.source = source, .age_ms = UINT32_MAX};
    if (source == ACC_SOURCE_RAW_IMU)
    {
        out->timestamp_ms = imu_last_processed_tick;
        out->sample_id = imu_sample_id;
        if (imu_has_sample)
        {
            out->age_ms = HAL_GetTick() - imu_last_processed_tick;
            out->valid = out->age_ms <= APP_IMU_MAX_AGE_MS;
        }
        if (out->valid)
        {
            out->acc_longitudinal_mg = imu_ax_mg;
            out->acc_lateral_mg = imu_ay_mg;
            out->acc_vertical_mg = imu_az_mg;
        }
    }
    else if (source == ACC_SOURCE_MOTIONFX)
    {
        MotionFxApp_Result_t fx = {0};
        uint8_t valid = MotionFxApp_GetLatest(&fx);
        out->timestamp_ms = fx.timestamp_ms;
        out->sample_id = fx.sample_id;
        if (fx.sample_id != 0U) out->age_ms = HAL_GetTick() - fx.timestamp_ms;
        if (valid && MotionFxApp_IsResultValid(&fx) && out->age_ms <= APP_MOTIONFX_MAX_AGE_MS)
        {
            out->acc_longitudinal_mg = (int32_t)(fx.acc_longitudinal_filtered_g * 1000.0f);
            out->acc_lateral_mg = (int32_t)(fx.acc_lateral_filtered_g * 1000.0f);
            out->acc_vertical_mg = (int32_t)(fx.acc_vertical_g * 1000.0f);
            out->valid = 1U;
        }
    }
    return out->valid;
}

uint8_t IMU_GetAccelerationSnapshot(IMU_Acceleration_t *out)
{
    return IMU_GetAccelerationFromSource(APP_ACC_SOURCE, out);
}
