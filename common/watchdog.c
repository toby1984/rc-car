#include "watchdog.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include <avr/wdt.h>
#include <stdint.h>

static volatile uint8_t watchdog_irq_initial_count;
static volatile uint8_t watchdog_irq_count;
static volatile watchdog_irq_handler watchdog_irq;

void watchdog_start(watchdog_irq_handler handler,uint8_t timeoutInSeconds) {

    cli();

    watchdog_irq_initial_count = timeoutInSeconds;
    watchdog_irq_count = timeoutInSeconds;
    watchdog_irq = handler;

    watchdog_reset();

    // Start timed sequence
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Set prescaler to WDP[2]|WDP[1] => 128k cycles => 1.0 s
    // switch watchdog to interrupt mode
    // Start watchdog
    WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1);

    sei();
}

void watchdog_stop()
{
    cli();

    watchdog_reset();

    /* Clear WDRF in MCUSR */
    MCUSR &= ~(1<<WDRF);

    // Write logical one to WDCE and WDE */
    // Keep old prescaler setting to prevent unintentional time-out
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Turn off WDT
    WDTCSR = 0x00;

    sei();
}

void watchdog_reset() {
    wdt_reset();
}

// interrupt routine for receiving data via TWI
ISR(WDT_vect) {
    watchdog_irq_count--;
    if ( watchdog_irq_count == 0 ) {
        watchdog_irq_count = watchdog_irq_initial_count;
        watchdog_irq();
    }
}
