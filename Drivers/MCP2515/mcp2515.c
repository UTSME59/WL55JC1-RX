/******************************************************************************
 *
 * 					 _   _ _____ ____  __  __ _____
 * 					| | | |_   _/ ___||  \/  | ____|
 * 					| | | | | | \___ \| |\/| |  _|
 * 					| |_| | | |  ___) | |  | | |___
 *  			 	 \___/  |_| |____/|_|  |_|_____|
 *
 * @file    mcp2515.c
 * @brief   MCP2515 CAN controller driver implementation.
 *
 * @details
 * This file contains the low-level driver implementation for the MCP2515
 * stand-alone CAN controller.
 *
 * The driver provides functions for:
 *  - Initialising the MCP2515
 *  - Configuring CAN operating modes
 *  - Reading and writing MCP2515 registers over SPI
 *  - Receiving CAN frames from RXB0 and RXB1
 *  - Reading device error flags
 *  - Managing receive interrupt flags
 *
 * The driver is independent of application-specific logic. Hardware resources
 * such as the SPI peripheral and chip-select GPIO are supplied through an
 * MCP2515_Config_t instance.
 *
 * @author  James Kumar
 * @date    16 Aug 2026
 *
 * @note
 * This driver uses the STM32 HAL SPI and GPIO interfaces.
 *
 ******************************************************************************/
#include "mcp2515.h"
#include "mcp2515_regs.h"

#include <string.h>
#include <stdio.h>

#define MCP2515_SPI_TIMEOUT_MS      100U
#define MCP2515_MODE_TIMEOUT_MS     10U

#define MCP2515_DEBUG 				0

/***********************************************************
 *
 * 					 	DEBUG PRINT
 *
 **********************************************************/
/*
 * Normal driver status messages.
 * These are always printed.
 */
#define MCP2515_PRINT(fmt, ...) \
    printf("[MCP2515] " fmt "\r\n", ##__VA_ARGS__)

/*
 * Detailed debugging information.
 * Only printed when MCP2515_DEBUG is enabled.
 */
#if MCP2515_DEBUG
#define MCP2515_DEBUG_PRINT(fmt, ...) \
    printf("[MCP2515 DEBUG] " fmt "\r\n", ##__VA_ARGS__)
#else
#define MCP2515_DEBUG_PRINT(fmt, ...)
#endif

/***********************************************************
 *
 * 					 FUNCTION PROTOTYPES
 *
 **********************************************************/
static void MCP2515_Select(MCP2515_Config_t *dev);
static void MCP2515_Deselect(MCP2515_Config_t *dev);
static MCP2515_Status_t MCP2515_WriteRegister(MCP2515_Config_t *dev, uint8_t address, uint8_t data);
static MCP2515_Status_t MCP2515_ReadRegister(MCP2515_Config_t *dev, uint8_t address, uint8_t *buffer);
static MCP2515_Status_t MCP2515_ReadRegisters(MCP2515_Config_t *dev, uint8_t address, uint8_t *buffer, uint8_t length);
static MCP2515_Status_t MCP2515_BitModify(MCP2515_Config_t *dev, uint8_t address, uint8_t mask, uint8_t data);
static MCP2515_Status_t MCP2515_Reset(MCP2515_Config_t *dev);
static MCP2515_Status_t MCP2515_SetMode(MCP2515_Config_t *dev, uint8_t mode);
static MCP2515_Status_t MCP2515_ParseFrame(uint8_t *buffer, MCP2515_Frame_t *frame);

/***********************************************************
 *
 * 					PUBLIC FUNCTIONS
 *
 **********************************************************/
MCP2515_Status_t MCP2515_Init(MCP2515_Config_t *dev)
{
	if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL) || (dev->cs_pin == 0))
	{
		MCP2515_DEBUG_PRINT("Invalid configuration");
		return MCP2515_ERROR;
	}

	MCP2515_PRINT("Beginning initialisation.");

	/* CS is active low.
	 * Ensure device begins de-selected.
	 */
	MCP2515_Deselect(dev);

	MCP2515_PRINT("Sending reset command.");

	/* Reset MCP2515.
	 *
	 * Reset automatically places device in Configuration mode.
	 */
	if (MCP2515_Reset(dev) != MCP2515_OK)
	{
		MCP2515_PRINT("Reset failed");
		return MCP2515_ERROR;
	}

	MCP2515_PRINT("Reset command sent.");
	MCP2515_PRINT("Entering configuration mode.");

	/* Request and verify Configuration mode.
	 *
	 * This is redundant as the device automatically enters Configuration mode following a reset.
	 * However, it is included for redundancy and to provide another check the device is responding.
	 */
	if (MCP2515_SetMode(dev, CANCTRL_MODE_CONFIG) != MCP2515_OK)
	{
		MCP2515_PRINT("Failed to enter configuration mode");
		return MCP2515_ERROR;
	}

	MCP2515_PRINT("Configuration mode entered.");

	/*
	 * CAN bit timing
	 *
	 * MCP2515 oscillator: 16 MHz
	 * CAN bitrate:        1 Mbit/s
	 *
	 * BRP       = 0
	 * SJW       = 1 TQ
	 * PropSeg   = 1 TQ
	 * PhaseSeg1 = 4 TQ
	 * PhaseSeg2 = 2 TQ
	 *
	 * Total = 8 TQ / bit
	 *
	 * TQ = 2 * (BRP + 1) / Fosc
	 *    = 2 / 16 MHz
	 *    = 125 ns
	 *
	 * Bit time = 8 * 125 ns
	 *          = 1 us
	 *
	 * Bitrate = 1 / 1 us
	 *         = 1 Mbit/s
	 *
	 * Sample point = 6 / 8
	 *              = 75%
	 */
	if (MCP2515_WriteRegister(dev, MCP2515_CNF1, 0x00) != MCP2515_OK)
	{
		MCP2515_DEBUG_PRINT("CNF1 write failed");
	    return MCP2515_ERROR;
	}

	if (MCP2515_WriteRegister(dev, MCP2515_CNF2, 0x98) != MCP2515_OK)
	{
		MCP2515_DEBUG_PRINT("CNF2 write failed");
	    return MCP2515_ERROR;
	}


	if (MCP2515_WriteRegister(dev, MCP2515_CNF3, 0x01) != MCP2515_OK)
	{
		MCP2515_DEBUG_PRINT("CNF3 write failed");
	    return MCP2515_ERROR;
	}

	if (MCP2515_DEBUG)
	{
		uint8_t CNF1, CNF2, CNF3;
		if (MCP2515_ReadRegister(dev, MCP2515_CNF1, &CNF1) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}
		if (MCP2515_ReadRegister(dev, MCP2515_CNF2, &CNF2) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}
		if (MCP2515_ReadRegister(dev, MCP2515_CNF3, &CNF3) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}
		MCP2515_DEBUG_PRINT("CONFIG: CNF1=0x%02X CNF2=0x%02X CNF3=0x%02X", CNF1, CNF2, CNF3);
	}

	MCP2515_PRINT("CAN bit timing configured.");

	/*
	 * RXM controls received message filtering.
	 *
	 * RXM  = 0b11: masks/filters disabled
	 * BUKT = 0b1 : RXB0 message will roll over and be written to RXB1 if RXB0 is full
	 *
	 * IT IS STRONGLY RECOMMENDED TO INCLUDE FILTERING TO AVOID THE LIMITED BUFFERS OVERFLOWING
	 */
	if (MCP2515_BitModify(dev, MCP2515_RXB0CTRL, RXBCTRL_RXM_MASK | RXBCTRL_BUKT_MASK, RXBCTRL_RXM_ALL | RXBCTRL_BUKT_EN) != MCP2515_OK)
	{
		MCP2515_DEBUG_PRINT("RXB0CTRL configuration failed");
		return MCP2515_ERROR;
	}
	if (MCP2515_BitModify(dev, MCP2515_RXB1CTRL, RXBCTRL_RXM_MASK, RXBCTRL_RXM_ALL) != MCP2515_OK)
	{
		MCP2515_DEBUG_PRINT("RXB1CTRL configuration failed");
		return MCP2515_ERROR;
	}

	if (MCP2515_DEBUG)
	{
		uint8_t RXB0CTRL, RXB1CTRL;
		if (MCP2515_ReadRegister(dev, MCP2515_RXB0CTRL, &RXB0CTRL) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}
		if (MCP2515_ReadRegister(dev, MCP2515_RXB1CTRL, &RXB1CTRL) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}
		MCP2515_DEBUG_PRINT("CONFIG: RXB0CTRL=0x%02X RXB1CTRL=0x%02X", RXB0CTRL, RXB1CTRL);
	}

	/*
	 * Clear pending interrupt flags
	 *
	 * Interrupt flags must be cleared by the MCU to reset the interrupt condition
	 */
	if (MCP2515_WriteRegister(dev, MCP2515_CANINTF, 0x00) != MCP2515_OK)
	{
		MCP2515_DEBUG_PRINT("CANINTF clear failed");
		return MCP2515_ERROR;
	}

	if (MCP2515_DEBUG)
	{
		uint8_t CANINTF;
		if (MCP2515_ReadRegister(dev, MCP2515_CANINTF, &CANINTF) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}
		MCP2515_DEBUG_PRINT("CONFIG: CANINTF=0x%02X", CANINTF);
	}

	/*
	 * Enable receive interrupts for both RX buffers.
	 *
	 * INT becomes active-low when either:
	 *
	 * RX0IF = 1
	 * RX1IF = 1
	 */
	if (MCP2515_WriteRegister(dev, MCP2515_CANINTE, CANINTE_RX0IE | CANINTE_RX1IE) != MCP2515_OK)
	{
		MCP2515_DEBUG_PRINT("CANINTE configuration failed");
		return MCP2515_ERROR;
	}

	if (MCP2515_DEBUG)
	{
		uint8_t CANINTE;
		if (MCP2515_ReadRegister(dev, MCP2515_CANINTE, &CANINTE) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}
		MCP2515_DEBUG_PRINT("CONFIG: CANINTE=0x%02X", CANINTE);
	}

	/*
	 * Enter Normal mode.
	 *
	 * The MCP2515 will now receive CAN traffic and participate normally on the CAN bus.
	 */
	if (MCP2515_SetMode(dev, CANCTRL_MODE_NORMAL) != MCP2515_OK)
	{
		MCP2515_DEBUG_PRINT("Failed to enter normal mode");
		return MCP2515_ERROR;
	}

	MCP2515_PRINT("Initialisation complete.");
	return MCP2515_OK;
}

MCP2515_Status_t MCP2515_ReadMessage(MCP2515_Config_t *dev, MCP2515_Frame_t *frame)
{
	if ((dev == NULL) || (frame == NULL))
	{
		return MCP2515_ERROR;
	}

	uint8_t flags;
	if (MCP2515_ReadRegister(dev, MCP2515_CANINTF, &flags) != MCP2515_OK) {
		return MCP2515_ERROR;
	}

	uint8_t buffer[13];

	/*
	 *  Give RX buffer 0 priority
	 */
	if ((flags & CANINTF_RX0IF) != 0U)
	{
		if (MCP2515_ReadRegisters(dev, MCP2515_RXB0SIDH, buffer, sizeof(buffer)) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}

		if (MCP2515_ParseFrame(buffer, frame) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}

		/* Clear RX0IF to allow for another interrupt */
		if (MCP2515_BitModify(dev, MCP2515_CANINTF, CANINTF_RX0IF, 0x00U) != MCP2515_OK) {
			return MCP2515_ERROR;
		}

		return MCP2515_OK;
	}

	/*
	 *  Check RX buffer 1 afterwards
	 */
	if ((flags & CANINTF_RX1IF) != 0U)
	{
		if (MCP2515_ReadRegisters(dev, MCP2515_RXB1SIDH, buffer, sizeof(buffer)) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}

		if (MCP2515_ParseFrame(buffer, frame) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}

		/* Clear RX1IF to allow for another interrupt */
		if (MCP2515_BitModify(dev, MCP2515_CANINTF, CANINTF_RX1IF, 0x00U) != MCP2515_OK)
		{
			return MCP2515_ERROR;
		}

		return MCP2515_OK;
	}

	return MCP2515_NO_MESSAGE;
}

MCP2515_Status_t MCP2515_PrintDiagnostics(MCP2515_Config_t *dev)
{
    uint8_t canstat;
    uint8_t canintf;
    uint8_t eflg;
    uint8_t tec;
    uint8_t rec;

    if (MCP2515_ReadRegister(dev, MCP2515_CANSTAT, &canstat) != MCP2515_OK)
    {
        return MCP2515_ERROR;
    }

    if (MCP2515_ReadRegister(dev, MCP2515_CANINTF, &canintf) != MCP2515_OK)
    {
        return MCP2515_ERROR;
    }

    if (MCP2515_ReadRegister(dev, MCP2515_EFLG, &eflg) != MCP2515_OK)
    {
        return MCP2515_ERROR;
    }

    if (MCP2515_ReadRegister(dev, MCP2515_TEC, &tec) != MCP2515_OK)
    {
        return MCP2515_ERROR;
    }

    if (MCP2515_ReadRegister(dev, MCP2515_REC, &rec) != MCP2515_OK)
    {
        return MCP2515_ERROR;
    }

    printf("CANSTAT = 0x%02X\r\n", canstat);
    printf("CANINTF = 0x%02X\r\n", canintf);
    printf("EFLG    = 0x%02X\r\n", eflg);
    printf("TEC     = 0x%02X\r\n", tec);
    printf("REC     = 0x%02X\r\n", rec);

    return MCP2515_OK;
}

MCP2515_Status_t MCP2515_ForceRX0Interrupt(MCP2515_Config_t *dev)
{
    if (dev == NULL)
    {
        return MCP2515_ERROR;
    }

    /*
     * Manually set RX0IF.
     * If RX0IE is enabled, this should pull INT low.
     */
    return MCP2515_BitModify(dev, MCP2515_CANINTF, CANINTF_RX0IF, CANINTF_RX0IF);
}

bool MCP2515_MessageAvailable(MCP2515_Config_t *dev)
{
	if (dev == NULL)
	{
		return false;
	}

	uint8_t flags;
	if (MCP2515_ReadRegister(dev, MCP2515_CANINTF, &flags) != MCP2515_OK)
	{
		return false;
	}

	return (flags & (CANINTF_RX0IF | CANINTF_RX1IF)) != 0U;
}

uint8_t MCP2515_GetErrorFlags(MCP2515_Config_t *dev)
{
	if (dev == NULL)
	{
		return 0U;
	}

	uint8_t flags = 0U;
	if (MCP2515_ReadRegister(dev, MCP2515_EFLG, &flags) != MCP2515_OK)
	{
		return 0U;
	}

	return flags;
}

/***********************************************************
 *
 * 					PRIVATE FUNCTIONS
 *
 **********************************************************/
static void MCP2515_Select(MCP2515_Config_t *dev)
{
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void MCP2515_Deselect(MCP2515_Config_t *dev)
{
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static MCP2515_Status_t MCP2515_WriteRegister(MCP2515_Config_t *dev, uint8_t address, uint8_t data)
{
	uint8_t tx[3] = {MCP2515_CMD_WRITE, address, data};

	MCP2515_Select(dev);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, tx, sizeof(tx), MCP2515_SPI_TIMEOUT_MS);
	MCP2515_Deselect(dev);

	if (status != HAL_OK)
	{
		return MCP2515_ERROR;
	}

	return MCP2515_OK;
}

static MCP2515_Status_t MCP2515_ReadRegister(MCP2515_Config_t *dev, uint8_t address, uint8_t *buffer)
{
	if (buffer == NULL)
	{
		return MCP2515_ERROR;
	}

	uint8_t tx[3] = {MCP2515_CMD_READ, address, 0x00U};
	uint8_t rx[3] = {0};

	MCP2515_Select(dev);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, sizeof(tx), MCP2515_SPI_TIMEOUT_MS);
	MCP2515_Deselect(dev);

	if (status != HAL_OK) {
		return MCP2515_ERROR;
	}

	/*
	 * rx[0] is received while sending READ command.
	 * rx[1] is received while sending register address.
	 * rx[2] contains the actual register value.
	*/
	*buffer = rx[2];
	MCP2515_DEBUG_PRINT("READ register 0x%02X -> RX: %02X %02X %02X", address, rx[0], rx[1], rx[2]);

	return MCP2515_OK;
}

static MCP2515_Status_t MCP2515_ReadRegisters(MCP2515_Config_t *dev, uint8_t address, uint8_t *buffer, uint8_t length)
{
	if ((buffer == NULL) || (length == 0) || (length > 13))
	{
		return MCP2515_ERROR;
	}

	/*
	* A received CAN frame occupies 13 consecutive bytes:
	*
	* SIDH
	* SIDL
	* EID8
	* EID0
	* DLC
	* DATA0 ... DATA7
	*/
	uint8_t dummy[13] = {0};
	uint8_t command[2] = {MCP2515_CMD_READ, address};

	MCP2515_Select(dev);
	if (HAL_SPI_Transmit(dev->hspi, command, sizeof(command), MCP2515_SPI_TIMEOUT_MS) != HAL_OK) {
		MCP2515_Deselect(dev);
		return MCP2515_ERROR;
	}
	if (HAL_SPI_TransmitReceive(dev->hspi, dummy, buffer, length, MCP2515_SPI_TIMEOUT_MS) != HAL_OK)
	{
		MCP2515_Deselect(dev);
		return MCP2515_ERROR;
	}
	MCP2515_Deselect(dev);

	return MCP2515_OK;
}

static MCP2515_Status_t MCP2515_BitModify(MCP2515_Config_t *dev, uint8_t address, uint8_t mask, uint8_t data)
{
	uint8_t tx[4] = {MCP2515_CMD_BITMODIFY, address, mask, data};

	MCP2515_Select(dev);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, tx, sizeof(tx), MCP2515_SPI_TIMEOUT_MS);
	MCP2515_Deselect(dev);

	if (status != HAL_OK)
	{
		return MCP2515_ERROR;
	}

	return MCP2515_OK;
}

static MCP2515_Status_t MCP2515_Reset(MCP2515_Config_t *dev)
{
	uint8_t command = MCP2515_CMD_RESET;

	MCP2515_Select(dev);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, &command, 1U, MCP2515_SPI_TIMEOUT_MS);
	MCP2515_Deselect(dev);

	if (status != HAL_OK)
	{
		return MCP2515_ERROR;
	}

	HAL_Delay(2);

	uint8_t canstat;

	if (MCP2515_ReadRegister(dev, MCP2515_CANSTAT, &canstat) != MCP2515_OK)
	{
		return MCP2515_ERROR;
	}
	MCP2515_DEBUG_PRINT("RESET: CANSTAT=0x%02X", canstat);
	return MCP2515_OK;
}

static MCP2515_Status_t MCP2515_SetMode(MCP2515_Config_t *dev, uint8_t mode)
{
	/*
	 * REQOP[2:0] occupies CANCTRL bits 7:5.
	 * Only modify those bits and leave the remainder of CANCTRL untouched.
	 */
	if (MCP2515_BitModify(dev, MCP2515_CANCTRL, CANCTRL_REQOP_MASK, mode) != MCP2515_OK)
	{
		MCP2515_DEBUG_PRINT("SetMode: BIT MODIFY failed");
		return MCP2515_ERROR;
	}

	uint32_t start = HAL_GetTick();

	/*
	 * CANSTAT.OPMOD should eventually match CANCTRL.REQOP
	 */
	while ((HAL_GetTick() - start) < MCP2515_MODE_TIMEOUT_MS)
	{
		uint8_t status;

	    if (MCP2515_ReadRegister(dev, MCP2515_CANSTAT, &status) != MCP2515_OK)
        {
	    	MCP2515_DEBUG_PRINT("SetMode: CANSTAT read failed");
	    	return MCP2515_ERROR;
        }
	    MCP2515_DEBUG_PRINT("SetMode: CANSTAT=0x%02X expected=0x%02X", status, mode);

	    if ((status & CANCTRL_REQOP_MASK) == mode)
	    {
	    	return MCP2515_OK;
	    }
	}

	MCP2515_DEBUG_PRINT("SetMode timeout waiting for mode 0x%02X", mode);
	return MCP2515_ERROR;
}

static MCP2515_Status_t MCP2515_ParseFrame(uint8_t *buffer, MCP2515_Frame_t *frame)
{
	if ((buffer == NULL) || (frame == NULL)) {
		return MCP2515_ERROR;
	}

	uint8_t sidh = buffer[0];
	uint8_t sidl = buffer[1];
	uint8_t eid8 = buffer[2];
	uint8_t eid0 = buffer[3];
	uint8_t dlc  = buffer[4];

	frame->extended = ((sidl & RXBnSIDL_IDE) != 0);
	if (frame->extended)
	{
		/* Reconstruct 29-bit extended identifier */
		frame->id = ((uint32_t)sidh << 21) 		    |
					((uint32_t)(sidl & 0xE0) << 13) |
					((uint32_t)(sidl & 0x03) << 16) |
					((uint32_t)eid8 << 8)			|
					((uint32_t)eid0);

		frame->rtr = ((dlc & DLC_RTR) != 0U);
	}
	else
	{
		/* Reconstruct 11-bit standard identifier */
		frame->id = ((uint32_t)sidh << 3)			|
					((uint32_t)sidl >> 5);

		frame->rtr = ((sidl & RXBnSIDL_SRR) != 0U);
	}

	frame->dlc = dlc & DLC_MASK;

	if (frame->dlc > 8) {
		frame->dlc = 8;
	}

	memset(frame->data, 0, sizeof(frame->data));
	if (!frame->rtr) {
		memcpy(frame->data, &buffer[5], frame->dlc);
	}

	return MCP2515_OK;
}
