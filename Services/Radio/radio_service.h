#ifndef RADIO_SERVICE_H
#define RADIO_SERVICE_H

#include "canbus.h"
#include <stdint.h>
#include <stdbool.h>

void Radio_Init(void);
bool Radio_SendCAN(const CAN_Frame_t *frame);
void Radio_TxDone(void);
void Radio_TxTimeout(void);
bool Radio_IsBusy(void);

#endif
