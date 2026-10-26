#include "main.h"
#include "uart_log.h"
#include "wing_control.h"
#include "imu.h"
#include "lap_sim.h"
#include "servo.h"

#include <assert.h>
#include <errno.h>
#include <string.h>

int _write(int, char *, int);

static UARTLog_Diag_t diag(void)
{
    UARTLog_Diag_t d;
    UARTLog_GetDiagSnapshot(&d);
    return d;
}

static void write_text(const char *text)
{
    assert(_write(1,(char *)text,(int)strlen(text))==(int)strlen(text));
}

static void output_is(const char *expected)
{
    assert(host_uart_output_size==strlen(expected));
    assert(memcmp(host_uart_output,expected,host_uart_output_size)==0);
}

static void error_now(void)
{
    hlpuart1.ErrorCode=HAL_UART_ERROR_DMA;
    HAL_UART_ErrorCallback(&hlpuart1);
}

static void fragments(void)
{
    write_text("SIM,0,"); UARTLog_Task();
    assert(!diag().queued && !host_uart_starts);
    write_text("60,+0.000"); write_text(",NORMAL\r");
    assert(!diag().queued);
    write_text("\n");
    assert(diag().queued==1U && !host_uart_starts);
    Host_UART_Drain();
    output_is("SIM,0,60,+0.000,NORMAL\r\n");
}

static void printf_fragments(void)
{
    printf("SIM,%u,",12U); printf("%d",60); printf(",NORMAL\r\n");
    Host_UART_Drain();
    output_is("SIM,12,60,NORMAL\r\n");
}

static void immutable(void)
{
    char line[]="VEH original\r\n";
    write_text(line); UARTLog_Task();
    memset(line,'X',sizeof(line)-1U);
    write_text("IMU next\r\n");
    Host_UART_Drain();
    output_is("VEH original\r\nIMU next\r\n");
}

static void sequential(void)
{
    write_text("first\nsecond\nthird\n"); UARTLog_Task();
    assert(host_uart_starts==1U && diag().queued==3U);
    for (unsigned i=0;i<20;i++) UARTLog_Task();
    assert(host_uart_starts==1U);
    Host_UART_Complete();
    assert(!diag().active && diag().queued==2U);
    Host_UART_Drain();
    assert(host_uart_starts==3U);
    output_is("first\nsecond\nthird\n");
}

static void overflow(void)
{
    for (unsigned i=0;i<UART_LOG_QUEUE_SIZE;i++) write_text("kept\n");
    UARTLog_Task();
    const uint8_t *active_data=host_uart_data;
    write_text("lost prefix");
    assert(!diag().dropped_count);
    write_text(" lost suffix\n");
    assert(diag().queued==UART_LOG_QUEUE_SIZE && diag().dropped_count==1U);
    assert(host_uart_data==active_data && memcmp(active_data,"kept\n",5)==0);
    Host_UART_Drain(); write_text("recovered\n"); Host_UART_Drain();
    assert(host_uart_output_size==UART_LOG_QUEUE_SIZE*5U+10U);
    assert(memcmp(host_uart_output+UART_LOG_QUEUE_SIZE*5U,"recovered\n",10)==0);
    assert(diag().dropped_count==1U);
}

static void oversized(void)
{
    char big[UART_LOG_LINE_SIZE+2U]; memset(big,'x',sizeof(big));
    assert(_write(1,big,sizeof(big))==(int)sizeof(big));
    assert(diag().dropped_count==1U && !diag().queued);
    write_text("discard rest\nokay\n"); Host_UART_Drain();
    output_is("okay\n");
}

static void exact_limit(void)
{
    char line[UART_LOG_LINE_SIZE]; memset(line,'x',sizeof(line)); line[sizeof(line)-1U]='\n';
    assert(_write(1,line,sizeof(line))==(int)sizeof(line));
    Host_UART_Drain(); assert(host_uart_output_size==sizeof(line));
    assert(memcmp(host_uart_output,line,sizeof(line))==0 && !diag().dropped_count);
}

static void ring_wrap(void)
{
    char expected[4096]; size_t used=0U;
    for (unsigned cycle=0;cycle<20U;cycle++)
    {
        for (unsigned i=0;i<UART_LOG_QUEUE_SIZE;i++)
        {
            char text[32];
            int length=snprintf(text,sizeof(text),"record %u/%u\r\n",cycle,i);
            memcpy(expected+used,text,(size_t)length); used+=(size_t)length;
            write_text(text);
        }
        Host_UART_Drain();
    }
    assert(host_uart_output_size==used && memcmp(host_uart_output,expected,used)==0);
    assert(!diag().dropped_count && host_uart_starts==20U*UART_LOG_QUEUE_SIZE);
}

static void busy(void)
{
    host_uart_start_status=HAL_BUSY;
    write_text("waiting\n"); UARTLog_Task();
    assert(diag().queued==1U && !diag().active && diag().busy_count==1U);
    assert(!host_uart_starts && !diag().dropped_count);
    host_uart_start_status=HAL_OK; Host_UART_Drain(); output_is("waiting\n");
}

static void hardware_busy(void)
{
    write_text("waiting\n");
    hlpuart1.gState=HAL_UART_STATE_BUSY_TX; UARTLog_Task();
    hlpuart1.gState=HAL_UART_STATE_READY;
    host_uart_dma.State=HAL_DMA_STATE_BUSY; UARTLog_Task();
    host_uart_dma.State=HAL_DMA_STATE_READY;
    host_usart.CR3=USART_CR3_DMAT; UARTLog_Task();
    host_usart.CR3=0U; hlpuart1.Lock=HAL_LOCKED; UARTLog_Task();
    assert(!host_uart_starts && diag().queued==1U);
    hlpuart1.Lock=HAL_UNLOCKED; Host_UART_Drain(); output_is("waiting\n");
}

static void dma_then_uart(void)
{
    write_text("first\nsecond\n"); UARTLog_Task();
    Host_UART_DMAComplete(); HAL_UART_TxCpltCallback(&hlpuart1); UARTLog_Task();
    assert(diag().queued==2U && diag().active && host_uart_starts==1U);
    Host_UART_Complete(); Host_UART_Drain(); output_is("first\nsecond\n");
}

static void late_callback(void)
{
    HAL_UART_TxCpltCallback(&hlpuart1);
    write_text("first\nsecond\n"); UARTLog_Task();
    Host_UART_Complete(); HAL_UART_TxCpltCallback(&hlpuart1);
    assert(diag().queued==1U && !diag().active);
    UARTLog_Task(); HAL_UART_TxCpltCallback(&hlpuart1);
    assert(diag().queued==1U && diag().active);
    hlpuart1.ErrorCode=HAL_UART_ERROR_NONE; HAL_UART_ErrorCallback(&hlpuart1);
    assert(!diag().recovering);
    Host_UART_Drain(); output_is("first\nsecond\n");
}

static void start_error(void)
{
    host_uart_start_status=HAL_ERROR; write_text("failed\ngood\n"); UARTLog_Task();
    assert(diag().recovering && diag().active && diag().error_count==1U);
    host_uart_start_status=HAL_OK; Host_UART_Drain(); output_is("good\n");
    assert(diag().dropped_count==1U && host_uart_aborts==1U);
}

static void callback_error(void)
{
    write_text("failed\ngood\n"); UARTLog_Task(); error_now(); error_now();
    HAL_UART_TxCpltCallback(&hlpuart1);
    assert(diag().error_count==1U && diag().queued==2U && !host_uart_aborts);
    Host_UART_Drain(); output_is("good\n"); assert(diag().dropped_count==1U);
}

static void abort_pending(void)
{
    host_uart_abort_complete=0U;
    write_text("failed\ngood\n"); UARTLog_Task(); error_now(); UARTLog_Task();
    for (unsigned i=0;i<100;i++) { UARTLog_Task(); HAL_UART_TxCpltCallback(&hlpuart1); }
    assert(host_uart_aborts==1U && host_uart_starts==1U && diag().queued==2U);
    assert(memcmp(host_uart_data,"failed\n",7)==0);
    Host_UART_AbortComplete(); UARTLog_Task(); HAL_UART_TxCpltCallback(&hlpuart1);
    assert(diag().active && diag().queued==1U && diag().dropped_count==1U);
    Host_UART_Drain(); output_is("good\n");
}

static void abort_failure(void)
{
    host_uart_abort_status=HAL_ERROR;
    write_text("failed\ngood\n"); UARTLog_Task(); error_now(); UARTLog_Task();
    assert(diag().active && diag().recovering && !diag().dropped_count);
    for (unsigned i=0;i<20;i++) UARTLog_Task();
    assert(host_uart_starts==1U && diag().queued==2U);
    host_uart_abort_status=HAL_OK; Host_UART_Drain(); output_is("good\n");
    assert(diag().dropped_count==1U);
}

static void orphan_dma(void)
{
    write_text("failed\ngood\n"); UARTLog_Task();
    hlpuart1.gState=HAL_UART_STATE_READY; host_usart.CR3=0U;
    host_dma_abort_status=HAL_ERROR; error_now(); UARTLog_Task();
    assert(diag().recovering && diag().queued==2U && !diag().dropped_count);
    assert(host_uart_starts==1U && host_dma_abort_calls==1U);
    host_dma_abort_status=HAL_OK; Host_UART_Drain(); output_is("good\n");
    assert(diag().dropped_count==1U && host_dma_abort_calls==2U);
}

static void idle_error(void)
{
    error_now(); write_text("good\n"); Host_UART_Drain();
    output_is("good\n"); assert(!diag().dropped_count && diag().error_count==1U);
}

static void primask(void)
{
    host_primask=1U;
    write_text("masked\n"); UARTLog_Task(); assert(host_primask==1U);
    UARTLog_GetDiagSnapshot(NULL); assert(host_primask==1U);
    Host_UART_Complete(); assert(host_primask==1U);
    assert(!diag().queued && host_primask==1U);
    host_primask=0U; output_is("masked\n");
}

static void completion_race(void)
{
    write_text("first\n"); UARTLog_Task();
    host_before_disable_hook=Host_UART_Complete;
    write_text("second\n");
    assert(!host_primask && diag().queued==1U && !diag().active);
    Host_UART_Drain(); output_is("first\nsecond\n");
}

static void wrong_uart(void)
{
    UART_HandleTypeDef other=hlpuart1;
    write_text("kept\n"); UARTLog_Task();
    other.ErrorCode=HAL_UART_ERROR_DMA;
    HAL_UART_TxCpltCallback(&other); HAL_UART_ErrorCallback(&other);
    assert(diag().active && !diag().recovering && !diag().error_count);
    Host_UART_Drain(); output_is("kept\n");
}

static void arguments(void)
{
    assert(_write(3,"x\n",2)==-1 && errno==EBADF);
    assert(_write(1,NULL,2)==-1 && errno==EINVAL);
    assert(_write(1,"",-1)==-1 && errno==EINVAL);
    assert(_write(1,NULL,0)==0);
    assert(_write(2,"stderr\n",7)==7); Host_UART_Drain(); output_is("stderr\n");
}

static void tick_wrap(void)
{
    host_tick=UINT32_MAX-1U; write_text("wrap\n"); UARTLog_Task();
    host_tick=1U; Host_UART_Poll();
    assert(!diag().queued); output_is("wrap\n");
}

static void integration(uint8_t stalled)
{
    host_uart_mirror=!stalled;
    host_uart_hold=stalled;
    Host_Boot();
    assert(!diag().dropped_count);
#if APP_LAP_SIM_ENABLED
    BspButtonState=BUTTON_PRESSED;
    host_read_dma_status=HAL_ERROR;
    uint32_t end=104500U;
#else
    uint32_t end=4000U;
#endif
    for (uint32_t t=0;t<=end;t++)
    {
        host_tick=t;
#if APP_LAP_SIM_ENABLED
        if (t%20U==0U) HAL_TIM_PeriodElapsedCallback(&htim7);
#else
        if (t%20U==0U) Host_FeedIMU(t>=3000U && t<3500U ? 16384 : 0);
        if (t%50U==0U)
        {
            uint8_t speed=t<2000U ? 80U : t<2500U ? (uint8_t)(80U-(t-2000U)/50U) : 60U;
            Host_QueueSpeed(speed); HAL_TIM_PeriodElapsedCallback(&htim6);
        }
#endif
        Host_MainLoopOnce();
    }
#if APP_LAP_SIM_ENABLED
    LapSim_Diag_t sim; LapSim_GetDiagSnapshot(&sim);
    assert(sim.state==LAP_SIM_FINISHED && !sim.timing_error && sim.lateral_samples==5226U);
    assert(!host_can_tx && Servo_GetCurrentPulseUs()==1600U);
#else
    Control_Diag_t ctrl; Control_GetDiagSnapshot(&ctrl);
    assert(ctrl.rate_valid && ctrl.wing_state==WING_NORMAL);
#endif
    if (!stalled)
    {
        Host_UART_Drain();
        assert(!diag().dropped_count && !diag().error_count);
#ifdef HOST_UART_CAPTURE
        assert(host_uart_starts>100U && host_uart_output_size>10000U);
#endif
    }
    else assert(diag().dropped_count>100U && diag().queued==UART_LOG_QUEUE_SIZE &&
                diag().active && host_uart_starts==1U);
}

int main(int argc, char **argv)
{
    assert(argc==2); UARTLog_Init();
    if (!strcmp(argv[1],"fragments")) fragments();
    else if (!strcmp(argv[1],"printf_fragments")) printf_fragments();
    else if (!strcmp(argv[1],"immutable")) immutable();
    else if (!strcmp(argv[1],"sequential")) sequential();
    else if (!strcmp(argv[1],"overflow")) overflow();
    else if (!strcmp(argv[1],"oversized")) oversized();
    else if (!strcmp(argv[1],"exact_limit")) exact_limit();
    else if (!strcmp(argv[1],"ring_wrap")) ring_wrap();
    else if (!strcmp(argv[1],"busy")) busy();
    else if (!strcmp(argv[1],"hardware_busy")) hardware_busy();
    else if (!strcmp(argv[1],"dma_then_uart")) dma_then_uart();
    else if (!strcmp(argv[1],"late_callback")) late_callback();
    else if (!strcmp(argv[1],"start_error")) start_error();
    else if (!strcmp(argv[1],"callback_error")) callback_error();
    else if (!strcmp(argv[1],"abort_pending")) abort_pending();
    else if (!strcmp(argv[1],"abort_failure")) abort_failure();
    else if (!strcmp(argv[1],"orphan_dma")) orphan_dma();
    else if (!strcmp(argv[1],"idle_error")) idle_error();
    else if (!strcmp(argv[1],"primask")) primask();
    else if (!strcmp(argv[1],"completion_race")) completion_race();
    else if (!strcmp(argv[1],"wrong_uart")) wrong_uart();
    else if (!strcmp(argv[1],"arguments")) arguments();
    else if (!strcmp(argv[1],"tick_wrap")) tick_wrap();
    else if (!strcmp(argv[1],"integration")) integration(0U);
    else if (!strcmp(argv[1],"integration_stall")) integration(1U);
    else assert(0);
    fputs("PASS: ",stdout); fputs(argv[1],stdout); fputc('\n',stdout);
    return 0;
}
