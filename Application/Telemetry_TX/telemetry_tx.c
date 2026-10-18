/*
 * telemetry_tx.c
 *
 *  Created on: 15 Aug 2026
 *      Author: james
 */

#include "telemetry_tx.h"

#include "canbus.h"
#include "radio_service.h"
#include "radio_driver.h"

#include "spi.h"
#include "usart.h"

#include "stm32_seq.h"
#include "stm32_timer.h"
#include "stm32wlxx_hal.h"
#include "utilities_def.h"

#include "main.h"

#include <stdio.h>

#define TASK_LED_BLINK 			(1UL << CFG_SEQ_Task_LED_BLINK)
#define TASK_CAN_RX				(1UL << CFG_SEQ_Task_CAN_RX)

#define LED_PERIOD_MS        	5000U

static UTIL_TIMER_Object_t ledTimer;

static CAN_t CAN1;

static CAN_Config_t CAN1_Config =
{
	.hspi     = &hspi1,

	.cs_port  = CAN_CS_GPIO_Port,
	.cs_pin   = CAN_CS_Pin,

	.int_port = CAN_INT_GPIO_Port,
	.int_pin  = CAN_INT_Pin
};

static const CAN_Frame_t testFrame =
{
		.id = 0x123,
		.dlc = 8,
		.data = {
			0x11, 0x22, 0x33, 0x44,
			0x55, 0x66, 0x77, 0x88
		},
		.extended = false,
		.rtr = false
};

static void LED_Blink_Task(void);
static void LED_Timer_Callback(void *context);

static void CAN_RX_Task(void);
static void CAN_PrintFrame(const CAN_Frame_t *frame);

void Telemetry_TX_Init(void)
{
	//Not required due to initialisation in app_subghz_phy.c
	//UTIL_TIMER_Init();

	UTIL_SEQ_Init();

	UTIL_SEQ_RegTask(TASK_LED_BLINK, UTIL_SEQ_RFU, LED_Blink_Task);
	UTIL_SEQ_RegTask(TASK_CAN_RX, UTIL_SEQ_RFU, CAN_RX_Task);

	CAN_Create(&CAN1, &CAN1_Config);
	if (CAN1.Init(&CAN1) == CAN_OK) {
		const char message[] = "CAN INIT OK\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)message, sizeof(message)-1U, HAL_MAX_DELAY);
	}
	else
	{
		const char message[] = "CAN INIT FAILED\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)message, sizeof(message)-1U, HAL_MAX_DELAY);
	}

	UTIL_TIMER_Create(&ledTimer, LED_PERIOD_MS, UTIL_TIMER_ONESHOT, LED_Timer_Callback, NULL);
	UTIL_TIMER_Start(&ledTimer);

	if (Radio_SendCAN(&testFrame)) {
		printf("[RADIO] CAN frame transmission started\r\n");
	} else {
		printf("[RADIO] Failed to start tranmission\r\n");
	}
}

void Telemetry_TX_Run(void)
{
	UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == CAN_INT_Pin) {
		HAL_GPIO_TogglePin(INT_LED_GPIO_Port, INT_LED_Pin);
		UTIL_SEQ_SetTask(TASK_CAN_RX, 0);
	}
}

static void LED_Timer_Callback(void *context)
{
	(void)context;
	UTIL_SEQ_SetTask(TASK_LED_BLINK, 0);
}

static void LED_Blink_Task(void)
{
	HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
	//CAN_PrintDiagnostics(&CAN1);
	UTIL_TIMER_Start(&ledTimer);
}

static void CAN_RX_Task(void)
{
	CAN_Frame_t frame;

	while (CAN1.Read(&CAN1, &frame) == CAN_OK)
	{
		CAN_PrintFrame(&frame);
	}
}

static void CAN_PrintFrame(const CAN_Frame_t *frame)
{
	char buffer[128];

	int length = snprintf(buffer, sizeof(buffer), "ID: 0x%08lX DLC: %u DATA:", (unsigned long)frame->id, frame->dlc);

	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);

	for (uint8_t i = 0; i < frame->dlc; i++)
	{
		length = snprintf(buffer, sizeof(buffer), " %02X", frame->data[i]);

		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
	}

	const char newline[] = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *)newline, sizeof(newline) - 1U, HAL_MAX_DELAY);
}
