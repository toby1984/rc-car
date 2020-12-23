#include "crc.h"

uint8_t crc8(uint8_t *data,uint8_t dataLen) {
    uint8_t shiftReg = 0;
    for ( uint8_t idx = 0 ; idx < dataLen; idx++ ) {
        uint8_t currentByte = *data++;
        for ( uint8_t bit = 7 ; bit >= 0 ; bit--, currentByte <<= 1 ) {

            uint8_t bit0 = (shiftReg & 0x01) != 0 ? 1:0;
            uint8_t bit1 = (shiftReg & 0x02) != 0 ? 1:0;
            uint8_t bit2 = (shiftReg & 0x04) != 0 ? 1:0;
            uint8_t bit7 = (shiftReg & 0x80) != 0 ? 1:0;

            shiftReg <<= 1;
            if ( (currentByte & 0x80) != 0 ) {
                shiftReg |= 1;
            }

            // bit1 = bit0 ^ bit7
            if ( bit0 ^ bit7 ) {
                shiftReg |= 0x02;
            } else {
                shiftReg &= ~0x02;
            }
            // bit2 = bit1 ^ bit7
            if ( bit1 ^ bit7 ) {
                shiftReg |= 0x04;
            } else {
                shiftReg &= ~0x04;
            }
            // bit3 = bit2 ^ bit7
            if ( bit2 ^ bit7 ) {
                shiftReg |= 0x08;
            } else {
                shiftReg &= ~0x08;
            }
        }
    }
    return shiftReg;
}