/*
 * mcp2515_regs.h
 *
 *  Created on: 15 Aug 2026
 *      Author: james
 */

#ifndef MCP2515_REGS_H
#define MCP2515_REGS_H

/***********************************************************
 *
 * 					MCP2515 SPI COMMANDS
 *
 **********************************************************/
#define MCP2515_CMD_RESET 	 	0xC0U // Resets internal registers and sets configuration mode
#define MCP2515_CMD_WRITE 		0x02U // Writes data to the register beginning at the selected address
#define MCP2515_CMD_READ		0x03U // Reads data from the register beginning at selected address
#define MCP2515_CMD_BITMODIFY   0x05U // Allows the user to set or clear individual bits in a particular register

/***********************************************************
 *
 * 					MCP2515 REGISTERS
 *
 **********************************************************/

#define MCP2515_CANSTAT 			0x0EU // CAN Status Register
#define MCP2515_CANCTRL				0x0FU // CAN Control Register

#define MCP2515_CNF1				0x2AU // Configuration Register 1
#define MCP2515_CNF2				0x29U // Configuration Register 2
#define MCP2515_CNF3				0x28U // Configuration Register 3

#define MCP2515_CANINTE				0x2BU // CAN Interrupt Enable Register
#define MCP2515_CANINTF     		0x2CU // CAN Interrupt Flag Register
#define MCP2515_EFLG				0x2DU // Error Flag Register

#define MCP2515_RXB0CTRL			0x60U // Receive Buffer 0 Control Register
#define MCP2515_RXB0SIDH			0x61U // Receive Buffer 0 Starting Address

#define MCP2515_RXB1CTRL			0x70U // Receive Buffer 1 Control Register
#define MCP2515_RXB1SIDH			0x71U // Receive Buffer 1 Starting Address

#define MCP2515_TEC   				0x1CU // Transmit Error Counter Register
#define MCP2515_REC    				0x1DU // Receive Error Counter Register

/***********************************************************
 *
 * 			  MCP2515 REGISTER BIT DEFINITIONS
 *
 **********************************************************/
#define CANCTRL_REQOP_MASK			0xE0U // Request Operation Mode Bits

#define CANCTRL_MODE_NORMAL         0x00U
#define CANCTRL_MODE_SLEEP          0x20U
#define CANCTRL_MODE_LOOPBACK       0x40U
#define CANCTRL_MODE_LISTEN_ONLY    0x60U
#define CANCTRL_MODE_CONFIG         0x80U

#define RXBnSIDL_IDE		    	0x08U // Extended Identifier Flag Bit
#define RXBnSIDL_SRR				0x10U

#define RXBCTRL_RXM_MASK 			0x60U
#define RXBCTRL_RXM_ALL			    0x60U
#define RXBCTRL_BUKT_MASK			0x04U // Rollover Enable bit
#define RXBCTRL_BUKT_EN				0x04U // Enable rollover

#define DLC_RTR						0x40U // Remote Transmission Request Bit
#define DLC_MASK					0x0FU // Data Length Code Bits

#define CANINTE_RX0IE				0x01U // Receive Buffer 0 Full Interrupt Enable bit
#define CANINTE_RX1IE				0x02U // Receive Buffer 1 Full Interrupt Enable bit

#define CANINTF_RX0IF			  	0x01U
#define CANINTF_RX1IF				0x02U

#endif /* MCP2515_REGS_H */
