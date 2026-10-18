/******************************************************************************
 *
 *  				 _   _ _____ ____  __  __ _____
 * 					| | | |_   _/ ___||  \/  | ____|
 * 					| | | | | | \___ \| |\/| |  _|
 * 					| |_| | | |  ___) | |  | | |___
 *  				 \___/  |_| |____/|_|  |_|_____|
 *
 * @file    mcp2515.h
 * @brief   Public interface for the MCP2515 CAN controller driver.
 *
 * @details
 * This file defines the public data types and function prototypes used to
 * configure and communicate with the MCP2515 CAN controller.
 *
 * Application code should include this file when using the MCP2515 driver.
 *
 * @author  James Kumar
 * @date    16 Aug 2026
 *
 ******************************************************************************/
#ifndef MCP2515_H
#define MCP2515_H

#include "stm32wlxx_hal.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef 	  *cs_port;
	uint16_t 		   cs_pin;
} MCP2515_Config_t;

typedef struct
{
	uint32_t id;
	uint8_t  dlc;
	uint8_t  data[8];
	bool 	 extended;
	bool  	 rtr;
} MCP2515_Frame_t;

typedef enum
{
	MCP2515_OK = 0,
	MCP2515_NO_MESSAGE,
	MCP2515_ERROR
} MCP2515_Status_t;

MCP2515_Status_t MCP2515_Init(MCP2515_Config_t *dev);
MCP2515_Status_t MCP2515_ReadMessage(MCP2515_Config_t *dev, MCP2515_Frame_t *frame);
MCP2515_Status_t MCP2515_PrintDiagnostics(MCP2515_Config_t *dev);
MCP2515_Status_t MCP2515_ForceRX0Interrupt(MCP2515_Config_t *dev);
bool MCP2515_MessageAvailable(MCP2515_Config_t *dev);
uint8_t MCP2515_GetErrorFlags(MCP2515_Config_t *dev);

#endif /* MCP2515_H */
