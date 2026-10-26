#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#define ACC_SOURCE_RAW_IMU 0U
#define ACC_SOURCE_MOTIONFX 1U

#ifndef APP_ACC_SOURCE
#define APP_ACC_SOURCE ACC_SOURCE_RAW_IMU
#endif
#ifndef MOTIONFX_FILTER_ENABLE
#define MOTIONFX_FILTER_ENABLE 1
#endif
#ifndef MOTIONFX_FILTER_DIVISOR
#define MOTIONFX_FILTER_DIVISOR 4
#endif

#if APP_ACC_SOURCE != ACC_SOURCE_RAW_IMU && APP_ACC_SOURCE != ACC_SOURCE_MOTIONFX
#error "invalid acceleration source"
#endif
#if MOTIONFX_FILTER_ENABLE != 0 && MOTIONFX_FILTER_ENABLE != 1
#error "invalid motionfx filter enable"
#endif
#if MOTIONFX_FILTER_DIVISOR < 1
#error "invalid motionfx filter divisor"
#endif

#ifndef APP_LAP_SIM_ENABLED
#define APP_LAP_SIM_ENABLED 0
#endif

/* Less than one 20 ms sample: never replay several stale lateral decisions. */
#ifndef APP_LAP_SIM_MAX_LATENESS_MS
#define APP_LAP_SIM_MAX_LATENESS_MS 10U
#endif

#ifndef APP_OBD_SPEED_MAX_AGE_MS
#define APP_OBD_SPEED_MAX_AGE_MS 500U
#endif
#ifndef APP_IMU_MAX_AGE_MS
#define APP_IMU_MAX_AGE_MS 100U
#endif

#ifndef APP_IMU_DMA_TIMEOUT_MS
#define APP_IMU_DMA_TIMEOUT_MS 10U
#endif
#ifndef APP_MOTIONFX_MAX_AGE_MS
#define APP_MOTIONFX_MAX_AGE_MS 100U
#endif

#if APP_IMU_DMA_TIMEOUT_MS == 0 || APP_IMU_DMA_TIMEOUT_MS >= 0x80000000U
#error "invalid imu dma timeout"
#endif
#if APP_MOTIONFX_MAX_AGE_MS == 0 || APP_MOTIONFX_MAX_AGE_MS >= 0x80000000U
#error "invalid motionfx freshness limit"
#endif

#if APP_OBD_SPEED_MAX_AGE_MS == 0 || APP_OBD_SPEED_MAX_AGE_MS >= 0x80000000U
#error "invalid obd freshness limit"
#endif
#if APP_IMU_MAX_AGE_MS == 0 || APP_IMU_MAX_AGE_MS >= 0x80000000U
#error "invalid imu freshness limit"
#endif

#if APP_LAP_SIM_ENABLED != 0 && APP_LAP_SIM_ENABLED != 1
#error "APP_LAP_SIM_ENABLED must be 0 or 1"
#endif
#if APP_LAP_SIM_MAX_LATENESS_MS >= 20U
#error "Lap timing tolerance must be less than the 20 ms sample period"
#endif

#endif
