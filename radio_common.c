#include <stdint.h>
#include "radio_common.h"
#include "avr/delay.h"

void delay_short()
{
	_delay_us(SHORT_MICROS);
}

void delay_long()
{
    _delay_us(LONG_MICROS);
}