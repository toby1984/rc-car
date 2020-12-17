#include <stdint.h>
#include "radio_sender.h"

void radio_sender_init(void) {

}

inline void radioOn() {
    RADIO_OUT_REG |= _BV(RADIO_PIN);    
}

inline void radioOff() {
    RADIO_OUT_REG &= ~_BV(RADIO_PIN);          
}

void radio_send(uint8_t *data,uint8_t dataLen) {

    // send start bit
    radioOn();
    delayLong();
    radioOff();
    delayShort();

    // send data
    for (uint8_t i = 0 ; i< dataLen ; i++ ) {
        uint8_t value = data[i];            
        for ( uint8_t mask = 1<<7 ; mask != 0 ; mask >>>=1 ) {
            if ( (value & mask) != 0 ) {
                radioOff();
                delayShort();
                radioOn();
                delayShort();
            } else {
                radioOn();
                delayShort();
                radioOff();
                delayShort();
            }
        }
    }
    radioOff();
}