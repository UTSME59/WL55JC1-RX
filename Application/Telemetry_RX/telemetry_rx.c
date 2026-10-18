#include "telemetry_rx.h"

#include "radio_service.h"
#include "stm32_seq.h"
#include "utilities_def.h"

#define TASK_RADIO_RX (1UL << CFG_Seq_Task_RADIO_RX)

static void Telemetry_RX_Task(void)
{
    Radio_ProcessRx();
}


void Telemetry_RX_Init(void)
{
    UTIL_SEQ_RegTask(TASK_RADIO_RX, UTIL_SEQ_RFU, Telemetry_RX_Task);
}


void Telemetry_RX_RequestProcessing(void)
{
    UTIL_SEQ_SetTask(TASK_RADIO_RX, CFG_SEQ_Prio_0);
}
