#include <crc.h>

uint8_t crc8(uint8_t *data,uint8_t dataLen) {
        uint8_t crc = 0xff;
        for (uint8_t j = 0; j < dataLen; j++)
        {
            crc ^= data[j];
            for (uint8_t i = 0; i < 8; i++) {
                crc = (uint8_t) ((crc>>1) ^ ((0-(crc&1))&0xf5));
            }
        }
        return crc;
}
