#include "radio_service.h"

#include "radio.h"
#include "subghz_phy_app.h"


#include "radio_driver.h"
volatile uint16_t radioIrqStatus = 0;

#include <stdio.h>

#define RADIO_TX_TIMEOUT_MS 3000U

/* 4-byte ID + DLC + Flags + 8-byte data */
#define CAN_RF_MAX_SIZE 14U

static uint8_t txBuffer[CAN_RF_MAX_SIZE];

static volatile bool radioBusy = false;

void Radio_Init(void) {
	Radio.SetChannel(RF_FREQUENCY);

	Radio.SetTxConfig(
			MODEM_FSK, 					// GFSK
			TX_OUTPUT_POWER, 			// 14 dBm
			FSK_FDEV, 					// 25 kHz deviation
			FSK_BANDWIDTH,				// 50 kHz bandwidth
			FSK_DATARATE,  				// 50 kb/s
			0,							// Coding rate (unused)
			FSK_PREAMBLE_LENGTH,		// 5 bytes
			FSK_FIX_LENGTH_PAYLOAD_ON,  // Variable length
			true, 						// CRC enabled
			false, 						// Frequency hopping off
			0, 							// Hop period (unused)
			false, 						// IQ inversion (unused)
			RADIO_TX_TIMEOUT_MS
	);

	Radio.SetMaxPayloadLength(
			MODEM_FSK,
			CAN_RF_MAX_SIZE
	);

	printf("[RADIO] GFSK configured at 920 MHz, 50 kb/s\r\n");
}

bool Radio_SendCAN(const CAN_Frame_t *frame) {
	if ((frame == NULL) || radioBusy) {
		return false;
	}

	if (frame->dlc > 8U) {
		return false;
	}

	txBuffer[0] = (uint8_t)(frame->id);
	txBuffer[1] = (uint8_t)(frame->id >> 8);
	txBuffer[2] = (uint8_t)(frame->id >> 16);
	txBuffer[3] = (uint8_t)(frame->id >> 24);
	txBuffer[4] = frame->dlc;
	txBuffer[5] =
		(frame->extended ? 0x01U : 0U) |
		(frame->rtr		 ? 0x02U : 0U);

	for (uint8_t i = 0; i < frame->dlc; i++) {
		txBuffer[6U + i] = frame->data[i];
	}

	uint8_t length = 6U + frame->dlc;

	radioBusy = true;

	if (Radio.Send(txBuffer, length) != RADIO_STATUS_OK) {
		radioBusy = false;
		return false;
	}

	return true;
}

void Radio_TxDone(void) {
	radioBusy = false;
}

void Radio_TxTimeout(void) {
	radioBusy = false;
}

bool Radio_IsBusy(void) {
	return radioBusy;
}
