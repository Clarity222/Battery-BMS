#include "BatteryBMS.h"

E2ECounter sftyCounter;
E2ECounter ctlCounter;
E2ECounter vehCounter;

CAN_FRAME sftyFrame;
CAN_FRAME ctlFrame;
CAN_FRAME vehFrame;

uint8_t E2ECounter::increment() {
    count = (count + 1) > 0x0E ? 0 : count + 1;
    return count;
}

void prepareSfty() {
    uint8_t p[8] = {0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // 0xF5 = reserved bits=1 + Crash=1 + SftyOK=1
    p[1] = (0x8 << 4) | sftyCounter.increment();
    uint8_t crcData[9] = {0xBF, 0x08, 0};
    memcpy(&crcData[2], &p[1], 7);
    p[0] = CRC8_SAE_J1850_ZERO::calculate(crcData, 9);
    sftyFrame.id = 0x8BF; sftyFrame.extended = true; sftyFrame.length = 8;
    memcpy(sftyFrame.data.uint8, p, 8);
}

void prepareCtl(uint8_t hvReq, uint8_t isInMeasDisaCmd) {
   
    uint8_t p[8] = {0, 0, hvReq, 0xFC | isInMeasDisaCmd, 0xFF, 0xFF, 0xFF, 0xFF};
    p[1] = (0x6 << 4) | ctlCounter.increment();
    uint8_t crcData[9] = {0x0F, 0x06, 0};
    memcpy(&crcData[2], &p[1], 7);
    p[0] = CRC8_SAE_J1850_ZERO::calculate(crcData, 9);
    ctlFrame.id = 0x60F; ctlFrame.extended = true; ctlFrame.length = 8;
    memcpy(ctlFrame.data.uint8, p, 8);
}

void prepareVeh() {
    uint8_t p[8] = {0, 0, 0x50, 0xF8, 0x00, 0xF8, 0xFF, 0xFF};
    p[1] = (0x5 << 4) | vehCounter.increment();
    uint8_t crcData[9] = {0xB0, 0x05, 0};
    memcpy(&crcData[2], &p[1], 7);
    p[0] = CRC8_SAE_J1850_ZERO::calculate(crcData, 9);
    vehFrame.id = 0x5B0; vehFrame.extended = true; vehFrame.length = 8;
    memcpy(vehFrame.data.uint8, p, 8);
}