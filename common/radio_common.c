#include <stdint.h>
#include "radio_common.h"
#include "util/delay.h"

void radio_delay_short(void)
{
	_delay_us(SHORT_MICROS);
}

void radio_delay_long(void)
{
    _delay_us(LONG_MICROS);
}
