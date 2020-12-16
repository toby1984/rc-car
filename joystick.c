#include "joystick.h"
#include <avr/io.h>
#include <stdlib.h>

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define CENTER_MIN 448
#define CENTER_MAX 505

#define X_MAX 1005
#define Y_MAX 1005

void joystick_init(void) {
	ADMUX = 1<<6; // internal voltage reference, MUX01	
	ADCSRA |= (1<<7)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // enable ADC
	DIDR0 = (1<<0)|(1<<1); // disable digital input buffers on ADC0 and ADC1	
}

void joystick_read(joystick_readings *result) 
{
    uint16_t tmp;

	ADMUX &= 0b11110000; // internal voltage reference, MUX01		
	sbi(ADCSRA,ADSC);
	while (bit_is_set(ADCSRA, ADSC)); // wait for conversion to finish

    tmp = ADC;
    if ( tmp <= CENTER_MIN ) {
      result->y = ( 100.0*(CENTER_MIN-tmp+1)) / CENTER_MIN;
    } else if ( tmp >= CENTER_MAX ) {
      result->y = (-100.0*(tmp-CENTER_MAX) ) / (Y_MAX - CENTER_MAX );
    } else {
      result->y	= 0;
    }

	ADMUX = (ADMUX & 0b11110000) | 1<<0; // internal voltage reference, MUX01
	sbi(ADCSRA,ADSC);
	while (bit_is_set(ADCSRA, ADSC)); // wait for conversion to finish

    tmp = ADC;
    if ( tmp <= CENTER_MIN ) {
      result->x = ( 100.0*(CENTER_MIN-tmp+1)) / CENTER_MIN;
    } else if ( tmp >= CENTER_MAX ) {
      result->x = (-100.0*(tmp-CENTER_MAX) ) / (X_MAX - CENTER_MAX );
    } else {
      result->x	= 0;
    }
}
