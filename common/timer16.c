#include "timer16.h"
#include "avr/interrupt.h"
#include "avr/io.h"

static void (*irq_func)(void);

void timer16_set_overflow(uint16_t value,void (*irq_callback)(void)) {

    cli();
    
    OCR1AH = ( value >> 8 ) & 0xff;    
    OCR1AL = ( value & 0xff );

    irq_func = irq_callback;
    
    if ( irq_callback ) {
        TIMSK1 |= (1<<OCIE1A);
    } else {
        TIMSK1 &= ~(1<<OCIE1A);
    }                
    sei();
}

uint8_t timer16_has_overflowed() {
    return TIFR1 & ( 1<<OCF1A );
}

ISR(TIMER1_COMPA_vect) {
    irq_func();
}

void timer16_stop() {
	TCCR1B &= ~( _BV(CS12) | _BV(CS11) | _BV(CS10) );
}

void timer16_start() {
    TIFR1 |= (1<<OCF1A); // clear overflow marker bit
	TCCR1B = (TCCR1B & ~( _BV(CS12) | _BV(CS11) | _BV(CS10) ) ) | PRESCALER_BITS;
}

void timer16_reset() {
    timer16_stop();
    TCNT1H = 0;
    TCNT1L = 0;    
    timer16_start();
}

uint16_t timer16_elapsed() {
	uint8_t low = TCNT1L;
	uint8_t hi = TCNT1H;

#ifndef TIMER_DIVISOR	
	return (hi<<8) | low;
#else
	return ( (hi<<8) | low ) / TIMER_DIVISOR;
#endif
} 
