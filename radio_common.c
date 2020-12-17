#include <stdint.h>
#include "radio.h"
#include "avr/delay.h"

void delayShort()
{
	_delay(SHORT_MS);
}

void delayLong()
{
    _delay(LONG_MS);
}