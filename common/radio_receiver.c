#include <avr/io.h>
#include <stdint.h>
#include "radio_common.h"
#include "radio_receiver.h"
#include "timer16.h"

void radio_receiver_init(void) {
	// switch radio pin to input
	RADIO_IN_DDR_REG &= ~_BV(RADIO_IN_PIN);
}    

inline uint8_t radio_read()
{
    return ( RADIO_IN_REG & _BV(RADIO_IN_PIN) ) != 0;
}    

uint16_t radio_wait_for_edge() {
    timer16_reset();
    uint8_t current = radio_read();
    // TODO: Bail out if waiting too long
    while ( radio_read() == current );
    return timer16_elapsed();
}    

int8_t radio_receive(uint8_t *buffer, uint8_t msgSize)
{
    uint8_t currentByte = 0;
    uint8_t currentByteIdx = 0;
    uint8_t currentBitMask = 0x80;

    // wait for start bit
    uint16_t elapsed;
    while (1)
    {
        while ( radio_read() != 0) ; // wait until signal goes low
        while ( radio_read() == 0) ; // wait while signal IS low
        timer16_reset();
        while ( radio_read() != 0 ) ;  // wait while signal is high
        // signal is low again
        elapsed = timer16_elapsed();
        if (fuzzyEquals(elapsed, LONG_LOW, LONG_HI))
        {
            break;
        }
    }

    // now wait for 1st bit to arrive
    uint16_t tsElapsed = radio_wait_for_edge();

    uint8_t currentBit;
    if ( fuzzyEquals(tsElapsed, SHORT_LOW, SHORT_HI) )
    {
        currentBit = 0;
    }
    else if ( fuzzyEquals(tsElapsed, LONG_LOW, LONG_HI) )
    {
        currentBit = 1;
    }
    else
    {
        return -1;
    }

    if (currentBit)
    {
        currentByte |= currentBitMask;
    }
    currentBitMask >>= 1;

    while (1)
    {
        uint16_t tsElapsed2 = radio_wait_for_edge();
        uint16_t delta = tsElapsed2;
        if (fuzzyEquals(delta, SHORT_LOW, SHORT_HI))
        {
            uint16_t tsElapsed3 = radio_wait_for_edge();
            if ( ! fuzzyEquals(tsElapsed3, SHORT_LOW, SHORT_HI))
            {
                return -2;
            }
            if (currentBit)
            {
                currentByte |= currentBitMask;
            }
        }
        else if (fuzzyEquals(delta, LONG_LOW, LONG_HI))
        {
            currentBit = !currentBit;
            if (currentBit)
            {
                currentByte |= currentBitMask;
            }
        }
        else
        {
            return -3;
        }
        currentBitMask >>= 1;
        if (currentBitMask == 0)
        {
            currentBitMask = 0x80;
            buffer[currentByteIdx++] = currentByte;
            if (currentByteIdx == msgSize)
            {
                return currentByteIdx;
            }
            currentByte = 0;
        }
    }
}