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

static void enc_reset_counters() {

     // To do a 16-bit write, the high byte must be written before the low byte.
    TCNT4H = 0;
    TCNT4L = 0;

    TCNT5H = 0;
    TCNT5L = 0;
}

static void enc_stop() {
	TCCR3B &= ~( _BV(CS12) | _BV(CS11) | _BV(CS10) );
}

static void enc_start() {
    TIFR3 |= (1<<OCF1A); // clear overflow marker bit
	TCCR3B = (TCCR3B & ~( _BV(CS12) | _BV(CS11) | _BV(CS10) ) ) | prescaler_bitmask;
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

    // setup timers 4 & 5 as counters
    enc_reset_counters();

	TCCR4B = (TCCR4B & ~( _BV(CS42) | _BV(CS41) | _BV(CS40) ) ) | 7; // CSn2:0 = 6 -> Trigger on rising edge of Tn
	TCCR5B = (TCCR5B & ~( _BV(CS52) | _BV(CS51) | _BV(CS50) ) ) | 7; // CSn2:0 = 6 -> Trigger on rising edge of Tn

    // enable interrupts
    enc_start();
    sei();

    return 0; // success
}

ISR(TIMER3_COMPA_vect)
{
    // For a 16-bit read, the low byte must be read before the high byte.
    uint8_t left_low = TCNT4L;
    uint8_t left_hi = TCNT4H;

    uint8_t right_low = TCNT4L;
    uint8_t right_hi = TCNT4H;

    enc_reset_counters();
    encoder_handler_callback(left_hi<<8 | left_low, right_hi<<8 | right_low);
}

void enc_reset()
{
    enc_stop();
     // To do a 16-bit write, the high byte must be written before the low byte.
    TCNT3H = 0;
    TCNT3L = 0;

    enc_reset_counters();
    enc_start();
}
