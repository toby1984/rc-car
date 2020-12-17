#include "timer16.h"
#include "avr/io.h"

inline void timer16_stop() {
	TCCR1B &= ~( _BV(CS12) | _BV(CS11) | _BV(CS10) );
}

inline void timer16_start() {
	TCCR1B = (TCCR1B & ~( _BV(CS12) | _BV(CS11) | _BV(CS10) ) ) | PRESCALER_BITS;
}

inline void timer16_reset() {
    timerStop();
    TCNT1H = 0;
    TCNT1L = 0;    
    timerStart();
}

inline uint16_t timer16_elapsed() {
	uint8_t low = TCNT1L;
	uint8_t hi = TCNT1H;

#ifndef TIMER_DIVISOR	
	return (hi<<8) | low;
#else
	return ( (hi<<8) | low ) / TIMER_DIVISOR;
#endif
} 