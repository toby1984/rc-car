#ifndef TIMER16_H
#define TIMER16_H

#include <stdint.h>
#include <avr/io.h>

#define TIMER_PRESCALING_FACTOR ( F_CPU / 1000000 )

#if TIMER_PRESCALING_FACTOR == 1
#define PRESCALER_BITS _BV(CS10)
#elif TIMER_PRESCALING_FACTOR == 8
#define PRESCALER_BITS _BV(CS11)
#elif TIMER_PRESCALING_FACTOR == 16
#define PRESCALER_BITS _BV(CS11)
#define TIMER_DIVISOR 2
#else 
#error Unsupported prescaling factor TIMER_PRESCALING_FACTOR
#endif

void timer16_stop(void);
void timer16_start(void);
void timer16_reset(void);
uint16_t timer16_elapsed(void);

#endif