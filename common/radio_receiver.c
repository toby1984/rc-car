#include <avr/io.h>
#include <stdint.h>
#include "radio_common.h"
#include "radio_receiver.h"
#include "timer16.h"
#include "uart.h"

void radio_receiver_init(void) {
	// switch radio pin to input
	RADIO_IN_DDR_REG &= ~_BV(RADIO_IN_PIN);
}    

inline uint8_t radio_read()
{
    return ( RADIO_IN_REG & _BV(RADIO_IN_PIN) ) != 0;
}    

uint16_t radio_wait_for_edge() {
    uint8_t current = radio_read();    
    timer16_reset();    
    // TODO: Bail out if waiting too long
    while ( radio_read() == current );
    return timer16_elapsed();
}    

int8_t radio_receive(uint8_t *buffer, uint8_t msgSize)
{
    uint8_t currentByte = 0;
    uint8_t currentByteIdx = 0;
    uint8_t currentBitMask = 0x80;
    uint8_t currentBit = 0;

    // wait for start bit
    uint16_t elapsed;
    while (1)
    {
        elapsed = radio_wait_for_edge();
        if (fuzzyEquals(elapsed, LONG_LOW, LONG_HI))
        {
            if ( radio_read() ) {
                currentBit = 1;
                currentByte |= currentBitMask;
            }
            break;
        }
    }

    currentBitMask >>= 1;

    // radio_wait_for_edge();

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