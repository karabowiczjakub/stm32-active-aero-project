#include "main.h"
#include "app_config.h"
#include "imu.h"
#include "obd.h"
#include "wing_control.h"
#include "servo.h"
#include "motion_fx_app.h"
#include "motion_fx.h"
#include "diag_log.h"
#include "lap_sim.h"
#include <assert.h>
#include <float.h>
#include <string.h>

extern MFX_output_t host_mfx_output;
extern uint32_t host_mfx_calls, host_mfx_initializations;
extern size_t host_mfx_size;
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *);

static uint8_t inject_logger;
static MotionFxApp_Result_t logger_result;
uint8_t __real_MotionFxApp_GetLatest(MotionFxApp_Result_t *);
uint8_t __wrap_MotionFxApp_GetLatest(MotionFxApp_Result_t *out)
{
    if (!inject_logger) return __real_MotionFxApp_GetLatest(out);
    *out=logger_result;
    return 1U;
}

static IMU_Diag_t imu(void)
{
    IMU_Diag_t out; IMU_GetDiagSnapshot(&out); return out;
}
static Control_Diag_t control(void)
{
    Control_Diag_t out; Control_GetDiagSnapshot(&out); return out;
}
static void start_stage(uint8_t data)
{
    IMU_RequestSample(); IMU_Task();
    assert(host_dma_starts==1U && hi2c1.State==HAL_I2C_STATE_BUSY_RX);
    if (data)
    {
        Host_CompleteIMU(3U,0); IMU_Task();
        assert(host_dma_starts==2U && !imu().valid);
    }
}
static void timeout_stage(uint8_t data)
{
    start_stage(data);
    host_tick+=APP_IMU_DMA_TIMEOUT_MS-1U; IMU_Task();
    assert(!imu().dma_timeout_count && !host_abort_calls);
    host_tick++; IMU_Task();
    assert(imu().dma_timeout_count==1U && imu().dma_recovering);
    assert(host_abort_calls==1U && hi2c1.State==HAL_I2C_STATE_ABORT);
    assert(!imu().valid);
}
static void complete_read(void)
{
    Host_CompleteIMU(3U,0); IMU_Task();
    Host_CompleteIMU(3U,0); IMU_Task();
    assert(imu().valid && imu().age_ms==0U && !imu().dma_recovering);
}
static void normal_read(void)
{
    Host_FeedIMU(16384);
    assert(imu().valid && imu().ay_mg==999 && !imu().dma_timeout_count);
    assert(host_dma_starts==2U && !host_abort_calls && !host_primask);
    for (unsigned k=0;k<10U;k++) IMU_Task();
    assert(host_dma_starts==2U);
}
static void error_callback(void)
{
    start_stage(1U); Host_I2CError();
    assert(imu().i2c_error_count==1U && !imu().valid && imu().dma_recovering);
    IMU_Task();
    assert(!imu().dma_recovering && host_dma_starts==3U);
    complete_read();
    assert(!imu().dma_timeout_count);
}
static void recovery(void)
{
    timeout_stage(1U);
    Host_AbortComplete();
    assert(imu().dma_recovering && !imu().valid);
    IMU_Task();
    assert(host_dma_starts==3U && !imu().dma_recovering);
    complete_read();
    assert(imu().dma_timeout_count==1U);
    host_tick+=APP_IMU_DMA_TIMEOUT_MS*2U; IMU_Task();
    assert(imu().dma_timeout_count==1U);
}
static void failed_recovery(void)
{
    host_abort_status=HAL_ERROR;
    start_stage(0U);
    host_tick+=APP_IMU_DMA_TIMEOUT_MS; IMU_Task();
    assert(imu().dma_timeout_count==1U && imu().dma_abort_error_count==1U);
    assert(imu().dma_recovering && host_dma_starts==1U);
    host_tick+=APP_IMU_DMA_TIMEOUT_MS-1U; IMU_Task();
    assert(host_abort_calls==1U);
    host_tick++; IMU_Task();
    assert(host_abort_calls==2U && imu().dma_abort_error_count==2U);
    host_abort_status=HAL_OK;
    host_tick+=APP_IMU_DMA_TIMEOUT_MS; IMU_Task();
    assert(hi2c1.State==HAL_I2C_STATE_ABORT && host_dma_starts==1U);
    Host_AbortComplete(); IMU_Task(); complete_read();
    assert(imu().dma_timeout_count==1U);
}
static void abort_never_completes(void)
{
    timeout_stage(0U);
    for (unsigned k=0;k<20U;k++)
    { host_tick+=20U; IMU_RequestSample(); IMU_Task(); }
    assert(host_abort_calls==1U && host_dma_starts==1U);
    assert(imu().dma_recovering && !imu().valid && imu().dma_timeout_count==1U);
}
static void late_callback(void)
{
    timeout_stage(1U);
    Host_CompleteIMU(3U,16384);
    assert(!imu().valid && imu().dma_recovering && !IMU_IsCornerDetected());
    IMU_Task();
    uint32_t starts=host_dma_starts;
    HAL_I2C_MemRxCpltCallback(&hi2c1);
    HAL_I2C_AbortCpltCallback(&hi2c1);
    IMU_Task();
    assert(host_dma_starts==starts && !imu().valid);
    complete_read();
    assert(imu().ay_mg==0 && imu().dma_timeout_count==1U);
}
static void late_before_timeout_poll(void)
{
    start_stage(1U);
    host_tick+=APP_IMU_DMA_TIMEOUT_MS;
    Host_CompleteIMU(3U,16384);
    assert(!imu().valid);
    IMU_Task();
    assert(imu().dma_timeout_count==1U && host_abort_calls==0U);
    assert(host_dma_starts==3U);
    complete_read();
}
static void orphan_dma(void)
{
    start_stage(0U);
    hi2c1.State=HAL_I2C_STATE_READY;
    hi2c1.Mode=HAL_I2C_MODE_NONE;
    host_i2c.CR1=0U; host_i2c.ISR=0U;
    HAL_I2C_ErrorCallback(&hi2c1);
    IMU_Task();
    assert(host_dma_abort_calls==1U && host_dma_starts==1U);
    IMU_Task(); complete_read();
    assert(!imu().dma_timeout_count);
}
static void hardware_not_ready(void)
{
    timeout_stage(0U);
    Host_AbortComplete();
    host_i2c.ISR=I2C_FLAG_BUSY;
    IMU_Task();
    assert(imu().dma_recovering && host_dma_starts==1U);
    host_i2c.ISR=0U; host_i2c.CR1=I2C_CR1_RXDMAEN;
    IMU_Task();
    assert(imu().dma_recovering && host_dma_starts==1U);
    host_i2c.CR1=0U; host_dma.State=HAL_DMA_STATE_BUSY;
    IMU_Task();
    assert(imu().dma_recovering && host_dma_starts==1U);
    Host_SetI2CReady(); IMU_Task(); complete_read();
}
static void callback_before_mask(void)
{
    Host_CompleteIMU(3U,0);
}
static void completion_race(void)
{
    start_stage(1U);
    host_tick+=APP_IMU_DMA_TIMEOUT_MS-1U;
    host_before_disable_hook=callback_before_mask;
    IMU_Task();
    assert(imu().valid && !imu().dma_timeout_count && !imu().dma_recovering);
    assert(!host_primask);
    host_primask=1U; IMU_Task(); assert(host_primask==1U);
    host_primask=0U;
}
static void stale_completed_sample(void)
{
    start_stage(1U);
    host_tick=1U; Host_CompleteIMU(3U,16384);
    host_tick=APP_IMU_MAX_AGE_MS+2U; IMU_Task();
    assert(!imu().valid && !imu().dma_timeout_count && !IMU_IsCornerDetected());
    assert(host_dma_starts==3U && imu().lateral_raw_mg==0);
    complete_read();
}

static void failsafe_after_stall(void)
{
    for (uint32_t t=0;t<=1250U;t+=50U)
    { host_tick=t; Host_FeedIMU(0); Host_QueueSpeed(60U); Host_MainLoopOnce(); }
    assert(control().wing_state==WING_OPEN);
    IMU_RequestSample(); IMU_Task();
    for (uint32_t t=1260U;t<=1400U;t+=10U)
    { host_tick=t; Host_QueueSpeed(60U); Host_MainLoopOnce(); }
    assert(imu().dma_timeout_count==1U && imu().dma_recovering);
    assert(control().wing_state==WING_NORMAL && Servo_GetCurrentPulseUs()==1600U);
    assert(control().failsafe_flags & CONTROL_FAULT_IMU_STALE);
}
static void good_fx(uint32_t t)
{
    host_tick=t;
    MotionFxApp_Process(0.0f,0.0f,1.0f,1.0f,2.0f,3.0f,t);
}
static MotionFxApp_Result_t fx(void)
{
    MotionFxApp_Result_t out; MotionFxApp_GetLatest(&out); return out;
}
static void init_fx(void)
{
    memset(&host_mfx_output,0,sizeof(host_mfx_output));
    host_mfx_output.gravity[2]=1.0f;
    host_mfx_output.linear_acceleration[0]=0.125f;
    host_mfx_output.rotation[0]=12.5f;
    assert(MotionFxApp_Init());
    good_fx(0U); assert(!fx().valid);
    good_fx(20U); assert(fx().valid && fx().timestamp_ms==20U);
}
static void valid_fx(void)
{
    init_fx(); MotionFxApp_Result_t out=fx();
    assert(out.yaw_deg==12.5f && out.linear_g[0]==0.125f && out.gravity_g[2]==1.0f);
    assert(out.gyro_dps[1]==2.0f && out.gyro_y_integral_deg==0.04f);
    assert(!MotionFxApp_GetLatest(NULL));
    assert(host_mfx_calls==1U && host_mfx_initializations==1U);
    host_tick=100U; good_fx(100U); DiagLog_Task(); MotionFxApp_PrintDebug();
}
static void bad_fx(float value)
{
    init_fx();
    float *fields[9]={&host_mfx_output.rotation[0],&host_mfx_output.rotation[1],&host_mfx_output.rotation[2],
        &host_mfx_output.gravity[0],&host_mfx_output.gravity[1],&host_mfx_output.gravity[2],
        &host_mfx_output.linear_acceleration[0],&host_mfx_output.linear_acceleration[1],&host_mfx_output.linear_acceleration[2]};
    uint32_t t=40U;
    for (unsigned i=0;i<9U;i++)
    {
        float previous=*fields[i]; *fields[i]=value;
        good_fx(t); assert(!fx().valid && fx().timestamp_ms==t-20U);
        DiagLog_Task(); MotionFxApp_PrintDebug();
        *fields[i]=previous;
        good_fx(t+20U); assert(fx().valid && fx().timestamp_ms==t+20U);
        t+=40U;
    }
    assert(host_mfx_initializations==1U);
}
static void stale_fx(void)
{
    init_fx(); host_tick=20U+APP_MOTIONFX_MAX_AGE_MS;
    assert(fx().valid);
    host_tick++; assert(!fx().valid && fx().timestamp_ms==20U);
    DiagLog_Task(); MotionFxApp_PrintDebug();
    good_fx(host_tick+20U); assert(!fx().valid && fx().timestamp_ms==20U);
    good_fx(host_tick+20U); assert(fx().valid && host_mfx_initializations==1U);
}
static void bad_inputs(void)
{
    init_fx();
    host_tick=40U; MotionFxApp_Process(NAN,0,1,0,0,0,40U);
    assert(!fx().valid && fx().timestamp_ms==20U && host_mfx_calls==1U);
    good_fx(60U); assert(fx().valid);
    host_tick=80U; MotionFxApp_Process(0,0,1,0,INFINITY,0,80U);
    assert(!fx().valid && fx().timestamp_ms==60U && host_mfx_calls==2U);
    good_fx(100U); assert(fx().valid && isfinite(fx().gyro_y_integral_deg));
    host_tick=120U; MotionFxApp_Process(0,0,1,FLT_MAX,FLT_MAX,-FLT_MAX,120U);
    assert(!fx().valid && fx().timestamp_ms==100U);
    good_fx(140U); assert(fx().valid && isfinite(fx().gyro_y_integral_deg));
}
static void invalid_dt(void)
{
    init_fx(); good_fx(20U); assert(!fx().valid && fx().timestamp_ms==20U);
    good_fx(40U); assert(fx().valid);
    good_fx(141U); assert(!fx().valid && fx().timestamp_ms==40U);
    good_fx(161U); assert(fx().valid);
}
static void fx_wrap(void)
{
    assert(MotionFxApp_Init());
    good_fx(UINT32_MAX-30U); good_fx(UINT32_MAX-10U);
    assert(fx().valid);
    host_tick=UINT32_MAX-10U+APP_MOTIONFX_MAX_AGE_MS;
    assert(fx().valid);
    host_tick++; assert(!fx().valid);
    good_fx(host_tick+20U); good_fx(host_tick+20U);
    assert(fx().valid);
}
static void init_failure(void)
{
    init_fx(); host_mfx_size=5000U;
    assert(!MotionFxApp_Init() && !fx().valid);
    good_fx(40U); assert(!fx().valid && host_mfx_calls==1U);
}
static void logger_defense(void)
{
    init_fx(); logger_result=fx(); inject_logger=1U;
    float *fields[13]={&logger_result.gravity_g[0],&logger_result.gravity_g[1],&logger_result.gravity_g[2],
        &logger_result.linear_g[0],&logger_result.linear_g[1],&logger_result.linear_g[2],
        &logger_result.gyro_dps[0],&logger_result.gyro_dps[1],&logger_result.gyro_dps[2],
        &logger_result.yaw_deg,&logger_result.pitch_deg,&logger_result.roll_deg,&logger_result.gyro_y_integral_deg};
    const float bad[]={NAN,INFINITY,-INFINITY,FLT_MAX,2147483.75f,-2147484.0f};
    for (unsigned i=0;i<13U;i++)
    {
        float previous=*fields[i];
        for (unsigned j=0;j<sizeof(bad)/sizeof(bad[0]);j++)
        {
            host_tick+=100U; logger_result.timestamp_ms=host_tick;
            *fields[i]=bad[j]; DiagLog_Task();
        }
        *fields[i]=previous;
    }
    assert(!MotionFxApp_IsLoggable(2147483.75f));
    assert(MotionFxApp_IsLoggable(nextafterf(2147483.75f,0.0f)));
    assert(MotionFxApp_IsLoggable(-2147483.75f));
    assert(!MotionFxApp_IsLoggable(nextafterf(-2147483.75f,-INFINITY)));
}
static void integral_range(void)
{
    init_fx();
    for (unsigned i=1;i<=21U;i++)
    {
        host_tick=20U+i*100U;
        MotionFxApp_Process(0,0,1,0,1000000.0f,0,host_tick);
        assert(fx().valid);
    }
    uint32_t last=fx().timestamp_ms;
    host_tick+=100U; MotionFxApp_Process(0,0,1,0,1000000.0f,0,host_tick);
    assert(!fx().valid && fx().timestamp_ms==last);
    good_fx(host_tick+20U);
    assert(fx().valid && host_mfx_initializations==1U);
}

static void fx_has_no_control(void)
{
    init_fx();
    for (uint32_t t=40U;t<=1400U;t+=50U)
    {
        host_tick=t;
        host_mfx_output.rotation[0]=NAN;
        Host_FeedIMU(0); Host_QueueSpeed(60U); Host_MainLoopOnce();
        assert(!fx().valid);
    }
    assert(control().wing_state==WING_OPEN && !control().failsafe_flags);
    assert(Servo_GetCurrentPulseUs()==1900U);
}
#if APP_LAP_SIM_ENABLED
static void sim_no_imu(uint8_t stalled)
{
    if (!stalled) host_read_dma_status=HAL_ERROR;
    LapSim_Init(); BSP_PB_Callback(BUTTON_USER);
    for (uint32_t t=0;t<=104500U;t++)
    {
        host_tick=t;
        if (t%20U==0U) HAL_TIM_PeriodElapsedCallback(&htim7);
        Host_MainLoopOnce();
    }
    LapSim_Diag_t out; LapSim_GetDiagSnapshot(&out);
    assert(out.state==LAP_SIM_FINISHED && !out.timing_error);
    assert(out.lateral_samples==5226U && out.speed_samples==2091U);
    assert(control().wing_state==WING_NORMAL && Servo_GetCurrentPulseUs()==1600U);
    assert(!control().failsafe_flags && !imu().valid && !host_can_tx);
    if (stalled) assert(imu().dma_timeout_count==1U && imu().dma_recovering);
    else assert(imu().dma_start_error_count>0U);
}
#endif
int main(int argc,char **argv)
{
    assert(argc==2); const char *name=argv[1]; Servo_SetWingState(WING_NORMAL);
    if (!strcmp(name,"normal_read")) normal_read();
    else if (!strcmp(name,"error_callback")) error_callback();
    else if (!strcmp(name,"status_timeout")) timeout_stage(0U);
    else if (!strcmp(name,"data_timeout")) timeout_stage(1U);
    else if (!strcmp(name,"recovery")) recovery();
    else if (!strcmp(name,"failed_recovery")) failed_recovery();
    else if (!strcmp(name,"abort_never_completes")) abort_never_completes();
    else if (!strcmp(name,"late_callback")) late_callback();
    else if (!strcmp(name,"late_before_poll")) late_before_timeout_poll();
    else if (!strcmp(name,"orphan_dma")) orphan_dma();
    else if (!strcmp(name,"hardware_not_ready")) hardware_not_ready();
    else if (!strcmp(name,"completion_race")) completion_race();
    else if (!strcmp(name,"stale_sample")) stale_completed_sample();
    else if (!strcmp(name,"logger_defense")) logger_defense();
    else if (!strcmp(name,"integral_range")) integral_range();
    else if (!strcmp(name,"failsafe_after_stall")) failsafe_after_stall();
    else if (!strcmp(name,"dma_wrap")) { host_tick=UINT32_MAX-5U; recovery(); }
    else if (!strcmp(name,"valid_fx")) valid_fx();
    else if (!strcmp(name,"nan_fx")) bad_fx(NAN);
    else if (!strcmp(name,"inf_fx")) bad_fx(INFINITY);
    else if (!strcmp(name,"negative_inf_fx")) bad_fx(-INFINITY);
    else if (!strcmp(name,"huge_fx")) bad_fx(FLT_MAX);
    else if (!strcmp(name,"stale_fx")) stale_fx();
    else if (!strcmp(name,"bad_inputs")) bad_inputs();
    else if (!strcmp(name,"invalid_dt")) invalid_dt();
    else if (!strcmp(name,"fx_wrap")) fx_wrap();
    else if (!strcmp(name,"init_failure")) init_failure();
    else if (!strcmp(name,"fx_has_no_control")) fx_has_no_control();
#if APP_LAP_SIM_ENABLED
    else if (!strcmp(name,"sim_disconnected")) sim_no_imu(0U);
    else if (!strcmp(name,"sim_stalled")) sim_no_imu(1U);
#endif
    else assert(!"unknown case");
    printf("PASS: %s\n",name);
    return 0;
}
