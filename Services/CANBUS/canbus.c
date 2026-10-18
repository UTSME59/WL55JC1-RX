/******************************************************************************
 *
 *  				 _   _ _____ ____  __  __ _____
 * 					| | | |_   _/ ___||  \/  | ____|
 * 					| | | | | | \___ \| |\/| |  _|
 * 					| |_| | | |  ___) | |  | | |___
 *  				 \___/  |_| |____/|_|  |_|_____|
 *
 * @file    canbus.c
 * @brief   CAN interface implementation.
 *
 * @details
 * This file implements the CAN interface using the MCP2515 CAN controller
 * driver.
 *
 * Each CAN_t instance contains its own hardware configuration, allowing
 * multiple independent CAN buses to be created using separate MCP2515
 * controllers.
 *
 * The CAN layer translates between the generic CAN types used by the
 * application and the MCP2515-specific types used by the device driver.
 *
 * @author  James Kumar
 * @date    16 Aug 2026
 *
 ******************************************************************************/

#include "canbus.h"
#include "mcp2515.h"

#include <stddef.h>
#include <string.h>
#include <stdio.h>


/***********************************************************
 *
 * 					 FUNCTION PROTOTYPES
 *
 **********************************************************/

static CAN_Status_t CAN_MCP2515_Init(CAN_t *can);
static CAN_Status_t CAN_MCP2515_Read(CAN_t *can, CAN_Frame_t *frame);


/***********************************************************
 *
 * 					PUBLIC FUNCTIONS
 *
 **********************************************************/

void CAN_Create(CAN_t *can, const CAN_Config_t *config)
{
	if ((can == NULL) || (config == NULL))
	{
		return;
	}

	/*
	 * Link the generic CAN interface to the MCP2515
	 * controller implementation.
	 */
	can->Init = CAN_MCP2515_Init;
	can->Read = CAN_MCP2515_Read;

	/*
	 * Store the hardware configuration for this CAN instance.
	 */
	can->config = *config;
}

CAN_Status_t CAN_PrintDiagnostics(CAN_t *can)
{
	if (can == NULL)
	{
		return CAN_ERROR;
	}

	MCP2515_Config_t dev =
	{
			.hspi 	 = can->config.hspi,
			.cs_port = can->config.cs_port,
			.cs_pin  = can->config.cs_pin
	};

	if (MCP2515_PrintDiagnostics(&dev) != MCP2515_OK)
	{
		return CAN_ERROR;
	}

	return CAN_OK;
}

CAN_Status_t CAN_TestInterrupt(CAN_t *can)
{
    if (can == NULL)
    {
        return CAN_ERROR;
    }

    MCP2515_Config_t dev =
    {
        .hspi    = can->config.hspi,
        .cs_port = can->config.cs_port,
        .cs_pin  = can->config.cs_pin
    };

    if (MCP2515_ForceRX0Interrupt(&dev) != MCP2515_OK)
    {
        return CAN_ERROR;
    }

    return CAN_OK;
}

/***********************************************************
 *
 * 					PRIVATE FUNCTIONS
 *
 **********************************************************/

static CAN_Status_t CAN_MCP2515_Init(CAN_t *can)
{
    if (can == NULL)
    {
        printf("[CAN] Invalid CAN handle\r\n");
        return CAN_ERROR;
    }

    MCP2515_Config_t dev =
    {
        .hspi    = can->config.hspi,
        .cs_port = can->config.cs_port,
        .cs_pin  = can->config.cs_pin
    };

    printf("[CAN] Initialising MCP2515\r\n");

    if (MCP2515_Init(&dev) != MCP2515_OK)
    {
        printf("[CAN] MCP2515 init failed\r\n");
        return CAN_ERROR;
    }

    printf("[CAN] MCP2515 init OK\r\n");

    return CAN_OK;
}


static CAN_Status_t CAN_MCP2515_Read(CAN_t *can, CAN_Frame_t *frame)
{
	if ((can == NULL) || (frame == NULL))
	{
		return CAN_ERROR;
	}

	MCP2515_Config_t dev =
	{
		.hspi    = can->config.hspi,
		.cs_port = can->config.cs_port,
		.cs_pin  = can->config.cs_pin
	};

	MCP2515_Frame_t mcp_frame;

	MCP2515_Status_t status = MCP2515_ReadMessage(&dev, &mcp_frame);

	switch (status)
	{
		case MCP2515_OK:
			frame->id       = mcp_frame.id;
			frame->dlc      = mcp_frame.dlc;
			frame->extended = mcp_frame.extended;
			frame->rtr      = mcp_frame.rtr;

			memcpy(frame->data, mcp_frame.data, sizeof(frame->data));
			return CAN_OK;

		case MCP2515_NO_MESSAGE:
			//printf("[CAN] NO MESSAGE\r\n");
			return CAN_NO_MESSAGE;

		default:
			return CAN_ERROR;
	}
}
