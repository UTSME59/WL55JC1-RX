#ifndef RADIO_SERVICE_H
#define RADIO_SERVICE_H

#include "telemetry_rx.h"

#include <stdint.h>

void Radio_Init(void);
void Radio_RxDone(uint8_t *payload,
				  uint16_t size,
				  int16_t rssi,
				  int8_t cfoKhz);
void Radio_RxTimeout(void);
void Radio_RxError(void);
void Radio_ProcessRx(void);

#endif
