#include <stdint.h>
#include "avr/io.h"
#include "radio_common.h"
#include "radio_sender.h"

void radio_sender_init(void) {
    RADIO_OUT_DDR_REG |= (1<<RADIO_OUT_PIN);
}

void radio_on() {
    RADIO_OUT_REG |= (1<<RADIO_OUT_PIN);    
}

void radio_off() {
    RADIO_OUT_REG &= ~(1<<RADIO_OUT_PIN);          
}

void radio_send(uint8_t *data,uint8_t dataLen) {

    // send training sequence
    for ( uint8_t i = 0 ; i < TRAINING_BITS ; i++ ) {
        radio_on();
        radio_delay_short();
        radio_off();
        radio_delay_short();        
    }

    // send start bit
    radio_on();
    radio_delay_long();
    radio_off();
    radio_delay_short();

    // send data
    for (uint8_t i = 0 ; i< dataLen ; i++ ) {
        uint8_t value = data[i];            
        for ( uint8_t mask = 1<<7 ; mask != 0 ; mask >>=1 ) {
            if ( (value & mask) != 0 ) {
                radio_off();
                radio_delay_short();
                radio_on();
                radio_delay_short();
            } else {
                radio_on();
                radio_delay_short();
                radio_off();
                radio_delay_short();
            }
        }
    }
    radio_off();
}