    #ifndef CRC8_SAE_J1850_ZERO_H
#define CRC8_SAE_J1850_ZERO_H

#include <Arduino.h>

class CRC8_SAE_J1850_ZERO
{
public:
    static uint8_t calculate(const uint8_t *data, uint8_t len);

private:
    static const uint8_t crc8_table[256];
};

class Counter
{
public:
    Counter();
    uint8_t increment();

private:
    uint8_t count;
};

#endif