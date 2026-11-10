#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#define ACC_SOURCE_RAW_IMU 0U
#define ACC_SOURCE_MOTIONFX 1U

#define WING_CONTROL_LEGACY_ONLY 0U
#define WING_CONTROL_LEGACY_SHADOW_HYBRID 1U
#define WING_CONTROL_HYBRID_ACTIVE 2U
#define WING_CONTROL_LEGACY_SHADOW_FX WING_CONTROL_LEGACY_SHADOW_HYBRID
#define WING_CONTROL_FX_ACTIVE WING_CONTROL_HYBRID_ACTIVE

#ifndef APP_WING_CONTROL_MODE
#define APP_WING_CONTROL_MODE WING_CONTROL_HYBRID_ACTIVE
#endif
#ifndef MOTIONFX_ATIME
#define MOTIONFX_ATIME 6.0f
#endif
#ifndef HYBRID_FX_ON_MG
#define HYBRID_FX_ON_MG (-140)
#endif
#ifndef HYBRID_FX_REARM_MG
#define HYBRID_FX_REARM_MG (-70)
#endif
#ifndef HYBRID_FX_ON_CONFIRM_MS
#define HYBRID_FX_ON_CONFIRM_MS 60U
#endif
#ifndef HYBRID_FX_REARM_CONFIRM_MS
#define HYBRID_FX_REARM_CONFIRM_MS 80U
#endif
#ifndef HYBRID_OBD_CONFIRM_MS
#define HYBRID_OBD_CONFIRM_MS 400U
#endif
#ifndef HYBRID_OBD_OFF_CONFIRM_MS
#define HYBRID_OBD_OFF_CONFIRM_MS 100U
#endif

#if APP_WING_CONTROL_MODE > WING_CONTROL_HYBRID_ACTIVE
#error "invalid wing control mode"
#endif
#if HYBRID_FX_ON_MG >= HYBRID_FX_REARM_MG || HYBRID_FX_REARM_MG > 0
#error "invalid hybrid thresholds"
#endif
#if HYBRID_FX_ON_CONFIRM_MS == 0 || HYBRID_FX_REARM_CONFIRM_MS == 0 || HYBRID_OBD_CONFIRM_MS == 0
#error "hybrid confirmation must be positive"
#endif
#if HYBRID_FX_ON_CONFIRM_MS >= 0x80000000U || HYBRID_FX_REARM_CONFIRM_MS >= 0x80000000U || \
    HYBRID_OBD_CONFIRM_MS >= 0x80000000U || HYBRID_OBD_OFF_CONFIRM_MS >= 0x80000000U
#error "invalid hybrid confirmation time"
#endif

// zrodlo api diagnostycznego, niezalezne od trybu sterowania
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
