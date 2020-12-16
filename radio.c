#include <stdint.h>
#include "radio.h"

#define fuzzyEquals(actual,low,high) ( actual >= low && actual <= high )

static char[25] recvBuffer;

    private static long timerStart;
    
    inline void timerReset() {
        timerStart = System.currentTimeMillis();
    }

    inline uint16_t elapsedTime() {
        return System.currentTimeMillis() - timerStart;
    } 

    void radio_init(void) {
        RADIO_OUT_REG &= ~_BV(RADIO_PIN);        
        RADIO_DDR_REG |= _BV(RADIO_PIN);
    }

    inline uint8_t readRadio()
    {
        return ( RADIO_PIN_REG & _BV(RADIO_PIN) ) != 0;
    }

    inline void radioOn() {
        RADIO_OUT_REG |= _BV(RADIO_PIN);    
    }

    inline void radioOff() {
        RADIO_OUT_REG &= ~_BV(RADIO_PIN);          
    }

    uint16_t waitForEdge() {
        timerReset();
        uint8_t current = readRadio();
        // TODO: Bail out if waiting too long
        while ( readRadio() == current );
        return elapsedTime();
    }

    void delayShort()
    {
        try { Thread.sleep(Manchester.SHORT_MS); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    void delayLong()
    {
        try { Thread.sleep(Manchester.LONG_MS); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    void send(char *data,uint8_t dataLen) {

        // send start bit
        radioOn();
        delayLong();
        radioOff();
        delayShort();

        // send data
        for (uint8_t i = 0 ; i< dataLen ; i++ ) {
            uint8_t value = data[i];            
            for ( uint8_t mask = 1<<7 ; mask != 0 ; mask >>>=1 ) {
                if ( (value & mask) != 0 ) {
                    radioOff();
                    delayShort();
                    radioOn();
                    delayShort();
                } else {
                    radioOn();
                    delayShort();
                    radioOff();
                    delayShort();
                }
            }
        }
        radioOff();
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