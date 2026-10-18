#include "radio_service.h"

#include "radio.h"
#include "subghz_phy_app.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "stm32wlxx_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* 4-byte ID + DLC + Flags + 8-byte data */
#define RADIO_MAX_PACKET_SIZE 14U

#define TASK_RADIO_RX (1UL << CFG_SEQ_Task_RADIO_RX)

/* Packet storage */
static uint8_t rxBuffer[RADIO_MAX_PACKET_SIZE];
static uint8_t rxLength = 0;
static int16_t rxRssi = 0;
static int8_t rxCfoKhz = 0;

/* Events set by radio callbacks */
static volatile bool rxPending = false;
static volatile bool crcErrorPending = false;
static volatile bool timeoutPending = false;

static volatile uint32_t droppedPackets = 0;
static volatile uint32_t invalidPackets = 0;

static uint32_t receivedPackets = 0;

void Radio_ProcessRx(void);

void Radio_Init(void) {
	Radio.SetChannel(RF_FREQUENCY);

	Radio.SetRxConfig(
			MODEM_FSK,
			FSK_BANDWIDTH,				// RX bandwidth: 50 kHz
			FSK_DATARATE,				// Bitrate: 50 kb/s
			0,							// Coding rate (unused)
			83333,						// AFC bandwidth: 83.33 kHz
			FSK_PREAMBLE_LENGTH,		// 5 bytes
			0,							// Symbol timeout
			false, 						// Variable packet length
			0, 							// Fixed length unused
			true, 						// CRC enabled
			false,						// Frequency hopping disabled
			0,							// Hop period
			false, 						// Iq Inversion
			true						// Continuous RX
	);

	Radio.SetMaxPayloadLength(
			MODEM_FSK,
			RADIO_MAX_PACKET_SIZE
	);

	// Start continuous reception
	Radio.Rx(0);

	printf("[RADIO] RX: 920 MHz GFSK, 50kb/s, 50 kHz BW\r\n");
	printf("[RADIO] Listening for packets...\r\n");
}

void Radio_RxDone(uint8_t *payload,
				  uint16_t size,
				  int16_t rssi,
				  int8_t cfoKhz)
{
	if ((payload == NULL) ||
		(size < 6U) ||
		(size > RADIO_MAX_PACKET_SIZE))
	{
		invalidPackets++;
		Telemetry_RX_RequestProcessing();
		return;
	}

	// Previous packet has not been processed
	if (rxPending) {
		droppedPackets++;
		Telemetry_RX_RequestProcessing();
		return;
	}

	memcpy(rxBuffer, payload, size);

	// Save packet information
	rxLength = (uint8_t)size;
	rxRssi = rssi;
	rxCfoKhz = cfoKhz;

	// Indicate packet is ready
	rxPending = true;

	// Schedule RX processing task
	Telemetry_RX_RequestProcessing();
}

void Radio_RxTimeout(void) {
	timeoutPending = true;
	Telemetry_RX_RequestProcessing();
}

void Radio_RxError(void) {
	crcErrorPending = true;
	Telemetry_RX_RequestProcessing();
}


/*
 * Process received RF packets.
 * Executed by the STM32 sequencer.
 */
void Radio_ProcessRx(void)
{
    uint8_t packet[RADIO_MAX_PACKET_SIZE];
    uint8_t length = 0;
    int16_t rssi = 0;
    int8_t cfoKhz = 0;

    bool havePacket;
    bool crcError;
    bool timeout;

    uint32_t dropped;
    uint32_t invalid;

    /*
     * Copy shared data safely.
     * The radio callback may run asynchronously.
     */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    havePacket = rxPending;

    if (havePacket)
    {
        length = rxLength;

        memcpy(packet, rxBuffer, length);

        rssi = rxRssi;
        cfoKhz = rxCfoKhz;

        rxPending = false;
    }

    crcError = crcErrorPending;
    timeout = timeoutPending;

    dropped = droppedPackets;
    invalid = invalidPackets;

    crcErrorPending = false;
    timeoutPending = false;
    droppedPackets = 0;
    invalidPackets = 0;

    __set_PRIMASK(primask);

    // Report radio errors
    if (crcError)
    {
        printf("[RADIO] CRC Error\r\n");
    }

    if (timeout)
    {
        printf("[RADIO] RX Timeout\r\n");

        // Restart continuous reception
        Radio.Rx(0);
    }

    if (dropped > 0U)
    {
        printf("[RADIO] Dropped %lu packet(s)\r\n",
               (unsigned long)dropped);
    }

    if (invalid > 0U)
    {
        printf("[RADIO] Invalid RF packet(s): %lu\r\n",
               (unsigned long)invalid);
    }

    // No valid packet to process
    if (!havePacket)
    {
        return;
    }

    /*
     * Decode CAN frame from RF payload.
     *
     * Bytes 0-3: CAN ID (little-endian)
     * Byte 4:    DLC
     * Byte 5:    Flags
     * Bytes 6+: CAN data
     */

    uint32_t id =
        ((uint32_t)packet[0])       |
        ((uint32_t)packet[1] << 8)  |
        ((uint32_t)packet[2] << 16) |
        ((uint32_t)packet[3] << 24);

    uint8_t dlc = packet[4];
    uint8_t flags = packet[5];

    bool extended = (flags & 0x01U) != 0U;
    bool rtr = (flags & 0x02U) != 0U;

    // Validate decoded CAN frame
    if ((dlc > 8U) ||
        (length != (uint8_t)(6U + dlc)) ||
        ((flags & 0xFCU) != 0U) ||
        (extended && id > 0x1FFFFFFFUL) ||
        (!extended && id > 0x7FFUL))
    {
        printf("[RADIO] Invalid CAN frame\r\n");
        return;
    }

    receivedPackets++;

    // Print RF diagnostic information
    printf(
        "[RADIO] RX #%lu | %u bytes | RSSI: %d dBm | CFO: %+d kHz\r\n",
        (unsigned long)receivedPackets,
        (unsigned)length,
        (int)rssi,
        (int)cfoKhz
    );

    // Print CAN frame header
    printf(
        "[CAN] ID: 0x%lX | DLC: %u | %s%s | DATA:",
        (unsigned long)id,
        (unsigned)dlc,
        extended ? "EXT" : "STD",
        rtr ? " RTR" : ""
    );

    // Print CAN data
    if (!rtr)
    {
        for (uint8_t i = 0; i < dlc; i++)
        {
            printf(" %02X", (unsigned)packet[6U + i]);
        }
    }

    printf("\r\n");
}
