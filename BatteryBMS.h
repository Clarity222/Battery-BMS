#ifndef BATTERY_BMS_H
#define BATTERY_BMS_H

#include <Arduino.h>
#include <due_can.h>
#include "crc8_sae_j1850_zero.h"

struct E2ECounter {
    uint8_t count = 0;
    uint8_t increment();
};

extern E2ECounter sftyCounter;
extern E2ECounter ctlCounter;
extern E2ECounter vehCounter;

extern CAN_FRAME sftyFrame;
extern CAN_FRAME ctlFrame;
extern CAN_FRAME vehFrame;

void prepareSfty();
void prepareCtl(uint8_t batSt, uint8_t isInMeasDisaCmd);
void prepareVeh();

#endif