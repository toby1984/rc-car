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

    // receiver waits for a 
    // 2T low or hi pulse 
    // that signals the start of 
    // the first data bit. 
    //
    // 2T low pulse => '1'
    // 2T hi pulse => '0'
    if ( (data[0] & 0x80 ) != 0 ) {
        radio_on();
        radio_delay_short();
        radio_off();        
        radio_delay_short();        
    } else {
        radio_on();        
        radio_delay_short();  
    }

    // send data
    for (uint8_t i = 0 ; i< dataLen ; i++ ) {
        uint8_t value = data[i];            
        for ( uint8_t mask = 1<<7 ; mask != 0 ; mask >>=1 ) {
            if ( (value & mask) != 0 ) { // rising edge == 1
                radio_off();
                radio_delay_short();
                radio_on();
                radio_delay_short();
            } else {
                radio_on(); // falling edge == 0
                radio_delay_short();
                radio_off();
                radio_delay_short();
            }
        }
    }
    radio_off();
}
