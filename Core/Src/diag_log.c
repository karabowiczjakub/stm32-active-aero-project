/*
 * diag_log.c - rozszerzona diagnostyka pojazdu/IMU/MotionFX.
 * Nie ingeruje w sterowanie skrzydlem i nie uruchamia odczytow czujnikow.
 * Wszystkie wartosci pochodza z ostatnich dostepnych probek.
 */
#include "diag_log.h"

#include "main.h"
#include "obd.h"
#include "imu.h"
#include "wing_control.h"
#include "servo.h"
#include "motion_fx_app.h"
#include "debug_utils.h"

#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "app_config.h"

#if APP_LAP_SIM_ENABLED
#include "lap_sim.h"

void DiagLog_Task(void)
{
    static uint32_t last_sim_log_ms;
    static LapSim_State_t last_sim_state = LAP_SIM_WAITING;
    static WingState_t last_sim_wing = WING_NORMAL;
    LapSim_Diag_t sim;
    Control_Diag_t ctrl;
    LapSim_GetDiagSnapshot(&sim);
    if (sim.state == LAP_SIM_WAITING) return;
    Control_GetDiagSnapshot(&ctrl);
    uint8_t changed = sim.state != last_sim_state || ctrl.wing_state != last_sim_wing;
    if (!changed && (sim.state == LAP_SIM_FINISHED ||
                     sim.elapsed_ms - last_sim_log_ms < 100U)) return;
    last_sim_log_ms = sim.elapsed_ms;
    last_sim_state = sim.state;
    last_sim_wing = ctrl.wing_state;

    printf("SIM,%lu,%u,", (unsigned long)sim.elapsed_ms, (unsigned)sim.speed_kmh);
    Print_Signed_mG_As_G(sim.lateral_mg);
    printf(",");
    Print_Signed_mG_As_G(sim.longitudinal_mg);
    printf(",");
    if (ctrl.rate_valid) Print_Signed_X10(ctrl.rate_x10);
    else printf("nan");
    printf(",%u,%u,%s,%lu,%u\r\n", (unsigned)ctrl.rate_valid,
           (unsigned)IMU_IsCornerDetected(), WingState_ToString(ctrl.wing_state),
           (unsigned long)sim.lateness_ms, (unsigned)sim.quality);
}
#else
#define DIAG_LOG_PERIOD_MS 100U
#define DIAG_LOG_LINE_SIZE 640U

static char diag_line[DIAG_LOG_LINE_SIZE];
static uint32_t diag_last_log_tick = 0U;
static uint8_t diag_first_log = 1U;

static void Diag_Append(char *buf, size_t cap, size_t *used,
                        const char *fmt, ...)
{
    if (*used >= cap - 1U) return;

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + *used, cap - *used, fmt, ap);
    va_end(ap);

    if (n < 0) return;
    if ((size_t)n >= cap - *used)
    {
        *used = cap - 1U;
        buf[*used] = '\0';
    }
    else
    {
        *used += (size_t)n;
    }
}


static void Diag_Format3(char *out, size_t out_len, int32_t milli)
{
    int64_t v = milli;
    uint64_t abs_v = (uint64_t)(v < 0 ? -v : v);
    (void)snprintf(out, out_len, "%s%lu.%03lu",
                   v < 0 ? "-" : "+",
                   (unsigned long)(abs_v / 1000U),
                   (unsigned long)(abs_v % 1000U));
}

static void Diag_AddAge(char *buf, size_t cap, size_t *used,
                        const char *name, uint32_t age_ms, uint8_t known)
{
    if (known) Diag_Append(buf, cap, used, " %s=%lu", name, (unsigned long)age_ms);
    else Diag_Append(buf, cap, used, " %s=-1", name);
}

static void Diag_Add3(char *buf, size_t cap, size_t *used,
                      const char *name, int32_t milli)
{
    char v[24];
    Diag_Format3(v, sizeof(v), milli);
    Diag_Append(buf, cap, used, " %s=%s", name, v);
}

static void Diag_AddVec3(char *buf, size_t cap, size_t *used,
                         const char *name, int32_t x, int32_t y, int32_t z)
{
    char sx[24], sy[24], sz[24];
    Diag_Format3(sx, sizeof(sx), x);
    Diag_Format3(sy, sizeof(sy), y);
    Diag_Format3(sz, sizeof(sz), z);
    Diag_Append(buf, cap, used, " %s=%s,%s,%s", name, sx, sy, sz);
}

void DiagLog_Task(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - diag_last_log_tick) < DIAG_LOG_PERIOD_MS)
    {
        return;
    }
    diag_last_log_tick = now;

    if (diag_first_log)
    {
        diag_first_log = 0U;
        printf("LOG_INFO version=2 period_ms=%u units: speed=km/h; accel=g; *_mg=mg; gyro=deg/s; angles=deg; time=ms; fx_vectors=device; fx_map=Y,-X,Z provisional=1; control_acc=RAW\r\n",
               (unsigned)DIAG_LOG_PERIOD_MS);
    }

    OBD_Diag_t obd = {0};
    IMU_Diag_t imu = {0};
    Control_Diag_t ctrl = {0};
    MotionFxApp_Result_t fx = {0};
    IMU_Acceleration_t acc, raw_acc;

    OBD_GetDiagSnapshot(&obd);
    IMU_GetDiagSnapshot(&imu);
    Control_GetDiagSnapshot(&ctrl);
    IMU_GetAccelerationSnapshot(&acc);
    IMU_GetAccelerationFromSource(ACC_SOURCE_RAW_IMU, &raw_acc);
    uint8_t fx_valid = MotionFxApp_GetLatest(&fx);
    uint32_t fx_sample_ms = fx.timestamp_ms;
    uint32_t fx_sample_id = fx.sample_id;
    uint32_t fx_age_ms = now - fx.timestamp_ms;
    if (!fx_valid || !MotionFxApp_IsResultValid(&fx) ||
        (uint32_t)(now - fx.timestamp_ms) > APP_MOTIONFX_MAX_AGE_MS)
    {
        fx_valid = 0U;
        memset(&fx, 0, sizeof(fx));
    }

    size_t n = 0U;
    /* VEH: informacje zwiazane z CAN, filtrem OBD i decyzja skrzydla. */
    Diag_Append(diag_line, sizeof(diag_line), &n,
                "VEH t_ms=%lu speed_kmh=%u obd_valid=%u obd_age_ms=%ld"
                " req_pending=%u req_age_ms=%lu rx=%lu req=%lu txerr=%lu"
                " timeout=%lu rxovf=%lu",
                (unsigned long)now,
                (unsigned)obd.speed_kmh,
                (unsigned)obd.speed_valid,
                (long)(obd.speed_valid ? (int32_t)obd.speed_age_ms : -1),
                (unsigned)obd.request_pending,
                (unsigned long)obd.request_age_ms,
                (unsigned long)obd.speed_rx_count,
                (unsigned long)obd.request_queued_count,
                (unsigned long)obd.tx_error_count,
                (unsigned long)obd.response_timeout_count,
                (unsigned long)obd.rx_overflow_count);
    Diag_Add3(diag_line, sizeof(diag_line), &n,
              "rate_kmh_s", ctrl.rate_x10 * 100);
    Diag_Add3(diag_line, sizeof(diag_line), &n,
              "aOBD_g", ctrl.accel_mg);
    Diag_Append(diag_line, sizeof(diag_line), &n,
                " filter_ok=%u filter_ms=%lu n=%u motion=%u"
                " brake=%u corner=%u wing=%s pwm_us=%u"
                " open=%u open_left_ms=%lu post=%u post_left_ms=%lu",
                (unsigned)ctrl.rate_valid,
                (unsigned long)ctrl.filter_span_ms,
                (unsigned)ctrl.filter_samples,
                (unsigned)ctrl.motion_state,
                (unsigned)ctrl.brake_detected,
                (unsigned)imu.corner,
                WingState_ToString(ctrl.wing_state),
                (unsigned)Servo_GetCurrentPulseUs(),
                (unsigned)ctrl.open_confirm_active,
                (unsigned long)ctrl.open_remaining_ms,
                (unsigned)ctrl.post_brake_hold_active,
                (unsigned long)ctrl.post_remaining_ms);
    Diag_Append(diag_line, sizeof(diag_line), &n,
                " acc_src=%s acc_valid=%u",
                acc.source == ACC_SOURCE_RAW_IMU ? "RAW" : "MFX",
                (unsigned)acc.valid);
    Diag_AddAge(diag_line, sizeof(diag_line), &n, "acc_age_ms", acc.age_ms, acc.sample_id != 0U);
    Diag_Append(diag_line, sizeof(diag_line), &n,
                " acc_sample_ms=%lu acc_seq=%lu acc_long_mg=%ld acc_lat_mg=%ld",
                (unsigned long)acc.timestamp_ms, (unsigned long)acc.sample_id,
                (long)acc.acc_longitudinal_mg, (long)acc.acc_lateral_mg);
    printf("%s\r\n", diag_line);

    /* IMU: pomiary w fizycznych osiach czujnika (X,Y,Z). */
    n = 0U;
    Diag_Append(diag_line, sizeof(diag_line), &n,
                "IMU t_ms=%lu valid=%u age_ms=%ld",
                (unsigned long)now, (unsigned)imu.valid,
                (long)(imu.valid ? (int32_t)imu.age_ms : -1));
    Diag_AddVec3(diag_line, sizeof(diag_line), &n,
                 "rawXYZ_g", imu.ax_mg, imu.ay_mg, imu.az_mg);
    Diag_AddVec3(diag_line, sizeof(diag_line), &n,
                 "gyroXYZ_dps", imu.gx_mdps, imu.gy_mdps, imu.gz_mdps);
    Diag_Add3(diag_line, sizeof(diag_line), &n,
              "latRaw_g", imu.lateral_raw_mg);
    Diag_Add3(diag_line, sizeof(diag_line), &n,
              "latFilt_g", imu.lateral_filtered_mg);
    Diag_Append(diag_line, sizeof(diag_line), &n,
                " corner=%u on_n=%u off_n=%u i2c_err=%lu dma_err=%lu"
                " dma_timeout=%lu dma_abort_err=%lu dma_recovering=%u",
                (unsigned)imu.corner,
                (unsigned)imu.corner_on_count,
                (unsigned)imu.corner_off_count,
                (unsigned long)imu.i2c_error_count,
                (unsigned long)imu.dma_start_error_count,
                (unsigned long)imu.dma_timeout_count,
                (unsigned long)imu.dma_abort_error_count,
                (unsigned)imu.dma_recovering);
    Diag_Append(diag_line, sizeof(diag_line), &n,
                " sample_ms=%lu seq=%lu acc_valid=%u longRaw_mg=%ld latRaw_mg=%ld latFilt_mg=%ld",
                (unsigned long)raw_acc.timestamp_ms, (unsigned long)raw_acc.sample_id,
                (unsigned)raw_acc.valid, (long)imu.ax_mg, (long)imu.ay_mg,
                (long)imu.lateral_filtered_mg);
    printf("%s\r\n", diag_line);

    // wektory biblioteki przed mapowaniem na pojazd
    n = 0U;
    Diag_Append(diag_line, sizeof(diag_line), &n,
                "IMU_FX t_ms=%lu valid=%u",
                (unsigned long)now, (unsigned)fx_valid);
    Diag_AddAge(diag_line, sizeof(diag_line), &n, "age_ms", fx_age_ms, fx_sample_id != 0U);
    Diag_Add3(diag_line, sizeof(diag_line), &n,
              "yaw_deg", (int32_t)(fx.yaw_deg * 1000.0f));
    Diag_Add3(diag_line, sizeof(diag_line), &n,
              "pitch_deg", (int32_t)(fx.pitch_deg * 1000.0f));
    Diag_Add3(diag_line, sizeof(diag_line), &n,
              "roll_deg", (int32_t)(fx.roll_deg * 1000.0f));
    Diag_Add3(diag_line, sizeof(diag_line), &n,
              "gyroIntY_deg", (int32_t)(fx.gyro_y_integral_deg * 1000.0f));
    Diag_AddVec3(diag_line, sizeof(diag_line), &n,
                 "linFX_g", (int32_t)(fx.linear_g[0] * 1000.0f),
                 (int32_t)(fx.linear_g[1] * 1000.0f),
                 (int32_t)(fx.linear_g[2] * 1000.0f));
    Diag_AddVec3(diag_line, sizeof(diag_line), &n,
                 "gravFX_g", (int32_t)(fx.gravity_g[0] * 1000.0f),
                 (int32_t)(fx.gravity_g[1] * 1000.0f),
                 (int32_t)(fx.gravity_g[2] * 1000.0f));
    Diag_Append(diag_line, sizeof(diag_line), &n,
                " sample_ms=%lu seq=%lu src=MFX filter=%u divisor=%u"
                " longRaw_mg=%ld latRaw_mg=%ld longFilt_mg=%ld latFilt_mg=%ld vert_mg=%ld",
                (unsigned long)fx_sample_ms, (unsigned long)fx_sample_id,
                (unsigned)MOTIONFX_FILTER_ENABLE, (unsigned)MOTIONFX_FILTER_DIVISOR,
                (long)(fx.acc_longitudinal_g * 1000.0f),
                (long)(fx.acc_lateral_g * 1000.0f),
                (long)(fx.acc_longitudinal_filtered_g * 1000.0f),
                (long)(fx.acc_lateral_filtered_g * 1000.0f),
                (long)(fx.acc_vertical_g * 1000.0f));
    printf("%s\r\n", diag_line);
}

#endif /* APP_LAP_SIM_ENABLED */
