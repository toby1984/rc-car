#include "encoder.h"
#include <stdint.h>
#include "avr/interrupt.h"
#include "avr/io.h"
#include "uart.h"

#define DEBUG
/*
 * TIMER USAGE:
 *
 * 8-bit timer/counter0  (UNUSED)
 * 16-bit timer/counter1 (radio)
 * 8-bit timer/counter2  (UNUSED)
 * 16-bit timer/counter3 (read encoder pulse counters and invoke callback to adjust motor speed)
 * 16-bit timer/counter4 (counts encoder impulses left motor)
 * 16-bit timer/counter5 (counts encoder impulses right motor)
 */

static volatile encoder_handler encoder_handler_callback;

static uint8_t prescaler_bitmask;

inline static void enc_reset_counters() {
    TCNT4 = 0;
    TCNT5 = 0;
}

static void enc_stop() {
	TCCR3B &= ~( _BV(CS12) | _BV(CS11) | _BV(CS10) );
}

static void enc_start() {
    TIFR3 |= (1<<OCF1A); // clear overflow marker bit
	TCCR3B = (TCCR3B & ~( _BV(CS12) | _BV(CS11) | _BV(CS10) ) ) | prescaler_bitmask;

    // setup timers 4 & 5 as counters
    enc_reset_counters();

    TCCR4B = 0;
	TCCR4B = (TCCR4B & ~( _BV(CS42) | _BV(CS41) | _BV(CS40) ) ) | 6; // CSn2:0 = 6 -> Trigger on rising edge of Tn
	TCCR5B = 0;
	TCCR5B = (TCCR5B & ~( _BV(CS52) | _BV(CS51) | _BV(CS50) ) ) | 6; // CSn2:0 = 6 -> Trigger on rising edge of Tn
}

uint8_t enc_init(float samplingIntervalMillis, encoder_handler callback)
{
    // calculate 16-bit timer TOP value and PRESCALER
    // based on current CPU frequency and the desired sampling samplingIntervalMillis

    uint16_t available_opts[5] = {1,8,64,256,1024};
    uint8_t prescaler_idx = 0;
    uint32_t top = (samplingIntervalMillis/1000.0f) * ( (float) F_CPU / available_opts[ prescaler_idx ]);
    while ( top > 65535 && prescaler_idx < 4 ) {
        prescaler_idx++;
        top = (samplingIntervalMillis/1000.0f) * ( (float) F_CPU / available_opts[ prescaler_idx] );
    }

#ifdef DEBUG
 	uart_print("\r\nTOP: ");
  	uart_putdecimal( top );
    uart_print("\r\n");
 #endif

    if ( top > 65535 ) {
        return 1;
    }

#ifdef DEBUG
 	uart_print("\r\nPRESCALER 2: ");
 	uart_putdecimal(available_opts[prescaler_idx]);
 #endif

    prescaler_bitmask = 1 + prescaler_idx;

    cli();

    OCR3AH = ( top >> 8 ) & 0xff;
    OCR3AL = ( top & 0xff );

    encoder_handler_callback = callback;

    // enable IRQ on COMPARE A MATCH
    TIMSK3 |= (1<<OCIE3A);

    // enable interrupts
    enc_start();
    sei();

    return 0; // success
}

ISR(TIMER3_COMPA_vect)
{
    uint16_t left = TCNT4;
    uint16_t right = TCNT5;

    enc_reset_counters();
    encoder_handler_callback( left, right );
}

void enc_reset()
{
    enc_stop();
    enc_start();
}
