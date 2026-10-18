/******************************************************************************
 *
 *  				 _   _ _____ ____  __  __ _____
 * 					| | | |_   _/ ___||  \/  | ____|
 * 					| | | | | | \___ \| |\/| |  _|
 * 					| |_| | | |  ___) | |  | | |___
 *  				 \___/  |_| |____/|_|  |_|_____|
 *
 * @file    canbus.h
 * @brief   Generic CAN interface definitions.
 *
 * @details
 * This file defines the generic CAN data types, hardware configuration and
 * interface used by the application.
 *
 * Each CAN_t instance represents one CAN bus and contains the hardware
 * configuration required by the underlying SPI CAN controller, together with
 * function pointers used to initialise the controller and read CAN frames.
 *
 * The application interacts with CAN through the CAN_t interface without
 * depending directly on a specific CAN controller implementation.
 *
 * @author  James Kumar
 * @date    16 Aug 2026
 *
 ******************************************************************************/
#ifndef CANBUS_H
#define CANBUS_H

#include "stm32wlxx_hal.h"

#include <stdint.h>
#include <stdbool.h>


/* Generic CAN Status */
typedef enum
{
	CAN_OK = 0,
	CAN_NO_MESSAGE,
	CAN_ERROR
} CAN_Status_t;


/* Generic CAN Frame */
typedef struct
{
	uint32_t id;
	uint8_t  dlc;
	uint8_t  data[8];
	bool     extended;
	bool     rtr;
} CAN_Frame_t;


/* Generic CAN Hardware Configuration */
typedef struct
{
	SPI_HandleTypeDef *hspi;

	GPIO_TypeDef      *cs_port;
	uint16_t           cs_pin;

	GPIO_TypeDef      *int_port;
	uint16_t           int_pin;
} CAN_Config_t;


/* Generic CAN Interface */
typedef struct CAN_s
{
	CAN_Status_t (*Init)(struct CAN_s *can);
	CAN_Status_t (*Read)(struct CAN_s *can, CAN_Frame_t *frame);
	CAN_Config_t config;

} CAN_t;

/* Public Functions */
void CAN_Create(CAN_t *can, const CAN_Config_t *config);
CAN_Status_t CAN_PrintDiagnostics(CAN_t *can);
CAN_Status_t CAN_TestInterrupt(CAN_t *can);
#endif /* CANBUS_H */
