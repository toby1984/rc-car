#include <stdint.h>
#include "radio_common.h"
#include "radio_sender.h"

void radio_sender_init(void) {
}

inline void radio_on() {
    RADIO_OUT_REG |= _BV(RADIO_PIN);    
}

inline void radio_off() {
    RADIO_OUT_REG &= ~_BV(RADIO_PIN);          
}

void radio_send(uint8_t *data,uint8_t dataLen) {

    // send start bit
    radio_on();
    delay_long();
    radio_off();
    delay_short();

    // send data
    for (uint8_t i = 0 ; i< dataLen ; i++ ) {
        uint8_t value = data[i];            
        for ( uint8_t mask = 1<<7 ; mask != 0 ; mask >>>=1 ) {
            if ( (value & mask) != 0 ) {
                radio_off();
                delay_short();
                radio_on();
                delay_short();
            } else {
                radio_on();
                delay_short();
                radio_off();
                delay_short();
            }
        }
    }
    radio_off();
}