#include <avr/io.h>
#include <stdint.h>
#include "radio_receiver.h"

void radio_receiver_init(void) {

/*
#define RADIO_IN_PIN 7
#define RADIO_IN_DDR_REG DDRD
#define RADIO_IN_REG PIND
*/
	// switch radio pin to input
	RADIO_IN_DDR_REG &= ~_BV(RADIO_IN_PIN);
}    

inline void timerStop() {
	TCCR1B &= ~( _BV(CS12) | _BV(CS11) | _BV(CS10) );
}

inline void timerStart() {
	TCCR1B = (TCCR1B & ~( _BV(CS12) | _BV(CS11) | _BV(CS10) ) ) | PRESCALER_BITS;
}

inline void timerReset() {
    timerStop();
    TCNT1H = 0;
    TCNT1L = 0;    
    timerStart();
}

inline uint16_t elapsedTime() {
	uint8_t low = TCNT1L;
	uint8_t hi = TCNT1H;
    return (hi<<8) | low;
} 

inline uint8_t readRadio()
{
    return ( RADIO_IN_REG & _BV(RADIO_IN_PIN) ) != 0;
}    

uint16_t waitForEdge() {
    timerReset();
    uint8_t current = readRadio();
    // TODO: Bail out if waiting too long
    while ( readRadio() == current );
    return elapsedTime();
}    

uint8_t receive(uint8_t *buffer, uint8_t msgSize)
{
    uint8_t currentByte = 0;
    uint8_t currentByteIdx = 0;
    uint8_t currentBitMask = 0x80;

    // wait for start bit
    uint16_t elapsed;
    while (true)
    {
        while ( readRadio() != 0) ; // wait until signal goes low
        while ( readRadio() == 0) ; // wait while signal IS low
        timerReset();
        while ( readRadio() != 0 ) ;  // wait while signal is high
        // signal is low again
        elapsed = elapsedTime();
        if (fuzzyEquals(elapsed, LONG_LOW, LONG_HI))
        {
            break;
        }
    }

    // now wait for 1st bit to arrive
    uint16_t tsElapsed = waitForEdge();

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
        return 0xff;
    }

    if (currentBit)
    {
        currentByte |= currentBitMask;
    }
    currentBitMask >>>= 1;

    while (true)
    {
        uint16_t tsElapsed2 = waitForEdge();
        final uint16_t delta = tsElapsed2;
        if (fuzzyEquals(delta, SHORT_LOW, SHORT_HI))
        {
            final uint16_t tsElapsed3 = waitForEdge();
            if ( ! fuzzyEquals(tsElapsed3, SHORT_LOW, SHORT_HI))
            {
                return 0xfe;
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
            return 0xfd;
        }
        currentBitMask >>>= 1;
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