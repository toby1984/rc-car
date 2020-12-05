// #define __AVR_ATmega88PB__
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"

// #define DEBUG
// #define DEBUG_IR
#define DEBUG_PIN _BV(5) // PB5
#define MOTOR_LEFT _BV(4) // PB4
#define MOTOR_RIGHT _BV(3) // PB3
#define IR_IN 6 // PD6

#define IR_KEY_FORWARD 0x00ffa05f
#define IR_KEY_BACKWARD 0x00ff40bf
#define IR_KEY_LEFT 0x00ff50af
#define IR_KEY_RIGHT 0x00ff7887
#define IR_KEY_STOP 0x00ff02fd

#define CYCLES_PER_MS (F_CPU/1000.0)
#define HALF_MS 0.5*CYCLES_PER_MS

enum direction {
	FORWARD,BACKWARD,STOP
};

#define MOTOR_LOOP_COUNT 40

enum direction leftDirection;
enum direction rightDirection;

// current loop iteration count (counts down to zero and then reset to LOOP_COUNT)
uint8_t currentLoopCount;

// length of high pulse in terms of loop iterations (left motor)
uint8_t leftCycles;
// length of high pulse in terms of loop iterations (right motor)
uint8_t rightCycles;

void motor_init() {

	leftDirection = STOP;
	rightDirection = STOP;

	currentLoopCount = 0;
	leftCycles = 0;
	rightCycles = 0;
}

void motor_loop() {

	uint8_t initial = PORTB;
	uint8_t flags = initial;   

	cli();
	enum direction tmpLeftDir = leftDirection;
	enum direction tmpRightDir = rightDirection;
	sei();

	if ( currentLoopCount <= 0 ) 
	{
		currentLoopCount = MOTOR_LOOP_COUNT;    

		switch(tmpLeftDir) {
			case FORWARD:
			flags |= MOTOR_LEFT;       
              	leftCycles = 2; // 1ms high pulse
              	break; 
              	case STOP:
              	flags &= ~MOTOR_LEFT;
              	leftCycles = 0; 
              	break;
              	case BACKWARD:
              	flags |= MOTOR_LEFT;       
         		leftCycles = 4; // 2 ms high pulse
         		break; 
         	}
         	switch(tmpRightDir) {
         		case FORWARD:
         		flags |= MOTOR_RIGHT;  
         		rightCycles = 4; 
         		break; 
         		case STOP:
         		flags &= ~MOTOR_RIGHT;
         		rightCycles = 0; 
         		break;
         		case BACKWARD:
         		flags |= MOTOR_RIGHT;        
         		rightCycles = 2; 
         		break; 
         	}      
         }
         if ( leftCycles > 0 ) {
         	leftCycles--;
         	flags |= MOTOR_LEFT;
         } else {
         	flags &= ~MOTOR_LEFT;
         }
         if ( rightCycles > 0 ) {
         	rightCycles--;
         	flags |= MOTOR_RIGHT;
         } else {
         	flags &= ~MOTOR_RIGHT;
         }   

         if ( flags != initial ) {
         	PORTB = (PORTB & ~(MOTOR_LEFT|MOTOR_RIGHT)) | flags;
         }

   _delay_loop_2(HALF_MS/4); // divide cycle count by four as _delay_loop2() takes 4 cycles per iteration

   currentLoopCount--;
}

void change_direction(char newDir) {

	switch( newDir ) {
		case 'x':    
			leftDirection = STOP;
			rightDirection = STOP;       
			break;
		case 'w':    
			if ( leftDirection == BACKWARD && rightDirection == BACKWARD ) {
				leftDirection = STOP;
				rightDirection = STOP;     			
			} else {
				leftDirection = FORWARD;
				rightDirection = FORWARD;  
			}    
			break;
		case 's':
			if ( leftDirection == FORWARD && rightDirection == FORWARD) {
				leftDirection = STOP;
				rightDirection = STOP;     			
			} else {
				leftDirection = BACKWARD;
				rightDirection = BACKWARD;  
			} 		  
			break;      
		case 'a':
			leftDirection = BACKWARD;
			rightDirection = FORWARD;
			break;
		case 'd':
			leftDirection = FORWARD;
			rightDirection = BACKWARD;    
		break;
	}
}

void ir_init() {
	cli();
  // let INT1 be triggered by falling edge,
  // which marks the beginning of an IR transmission
	EICRA = (EICRA & ~(_BV(ISC11) | _BV(ISC10))) | _BV(ISC11);
	EIMSK |= _BV(INT1);
	sei();
}

#define MAX_IR_BITS 48
uint32_t lastResult;
uint16_t ir_rec_high[MAX_IR_BITS];
uint16_t ir_rec_low[MAX_IR_BITS];

// invoked at start of IR transmission (PD6 falling edge)
ISR(INT1_vect) {
	uint16_t counter;
	uint8_t bitCount = 0;
	while(1) { 
    // Wait for PD6 to become HIGH again    
    // NOTE: Signal from IR sensor is inverted to LOW input 
    //       is logical HIGH
		counter = 65535;
		while ( !(PIND & _BV(IR_IN)) && --counter > 0) {    
		}
		if ( counter == 0 ) {
			break;
		}      
		ir_rec_high[bitCount] = 65535-counter;
    // pin is high, start counting while waiting for pin to go low again
		counter = 65535;
		while ( PIND & _BV(IR_IN) && --counter > 0) {    
		}
		if ( counter == 0 ) {
			break;
		}    
		ir_rec_low[bitCount++] = 65535-counter;
		if ( bitCount == MAX_IR_BITS ) {
			break;
		}
	}

	if ( bitCount > 0 ) {

#ifdef DEBUG_IR
		uart_print("\r\n-------------\r\n");
		for ( uint8_t i = 0 ; i < bitCount ; i++ ) {
			uart_print("bit ");
			uart_putdecimal( i );
			uart_print(": ");        
			uart_putdecimal( ir_rec_high[i] );
			uart_print(" HIGH, ");  
			uart_putdecimal( ir_rec_low[i] );
			uart_print(" LOW\r\n");
		}
#endif

    	// decode
   		// first HIGH is 9 ms,
   		// following LOW is 4.5ms
   		// Logical '0' – a 562.5µs pulse burst followed by a 562.5µs space, with a total transmit time of 1.125ms
   		// Logical '1' – a 562.5µs pulse burst followed by a 1.6875ms space, with a total transmit time of 2.25ms

   		// first burst is 9 ms 
		uint16_t unit = ir_rec_high[0];	

		uint16_t pauseLowerBound = (uint16_t) ((unit/2)*0.95);
		uint16_t pauseUpperBound = (uint16_t) ((unit/2)*1.05);

        // A “0” is represented by a pulse distance of 1.125 ms 
		uint16_t zeroPulse = (uint16_t) ((1.125*unit)/9.0);
		uint16_t zeroLowerBound = (uint16_t) (zeroPulse*0.95);
		uint16_t zeroUpperBound = (uint16_t) (zeroPulse*1.05);	        
        // A “1” is represented by a pulse distance of 2.25 ms		
		uint16_t onePulse = (uint16_t) ((2.25*unit)/9.0);
		uint16_t oneLowerBound = (uint16_t) (onePulse*0.95);
		uint16_t oneUpperBound = (uint16_t) (onePulse*1.05);

		if ( ir_rec_low[0] < pauseLowerBound || ir_rec_low[0] > pauseUpperBound ) {
#ifdef DEBUG_IR
			uart_print("ERROR: Preable not recognized\r\n");
#endif 
		} else {
			uint32_t result = 0;

			if ( bitCount == 2 ) {
#ifdef DEBUG_IR
					uart_print("repetition\r\n");
#endif
			} else if ( bitCount == 35 ) {
				for ( uint8_t i = 1 ; i < 33 ; i++ ) 
				{
					uint16_t sum = ir_rec_low[i] + ir_rec_high[i];				

					result <<= 1;				
					if ( sum >= zeroLowerBound && sum <= zeroUpperBound ) {
                  		// Logical '0'				  				
					} else if ( sum >= oneLowerBound && sum <= oneUpperBound ) {
                    	// Logical '1'
						result |= 1;
					} else {
#ifdef DEBUG_IR
							uart_print("ERROR: Failed to decode bit ");
							uart_putdecimal( i );
							uart_print("\r\n");
#endif
						break;					
					}
				}
				lastResult = result;
#ifdef DEBUG_IR
				uart_print("result: ");
				uart_puthex( result );
				uart_print("\r\n");		
#endif					

				if ( result == IR_KEY_FORWARD ) {
#ifdef DEBUG_IR
					uart_print("forward\r\n");
#endif
					change_direction('w');
				} else if ( result == IR_KEY_BACKWARD ) {
#ifdef DEBUG_IR
					uart_print("backward\r\n");
#endif
					change_direction('s');
				} else if ( result == IR_KEY_LEFT ) {
#ifdef DEBUG_IR
                    uart_print("left\r\n");						
#endif
					change_direction('a');
				} else if ( result == IR_KEY_RIGHT ) {
#ifdef DEBUG_IR
					uart_print("right\r\n");	
#endif
					change_direction('d');
				} else if ( result == IR_KEY_STOP ) {
#ifdef DEBUG_IR
                    uart_print("stop\r\n");	
#endif
					change_direction('x');
				} else {
#ifdef DEBUG_IR
					uart_print("Unrecognized key ");
					uart_puthex( result );
					uart_print("\r\n");
#endif
				}				
			} else {
#ifdef DEBUG_IR
					uart_print("wrong bit count\r\n");	
#endif
			}
		}
	}
}

void start_pwm() {

    TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0;
    TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS11; // prescaler = 1/8

	ICR1 = 19999; // 50 hz at prescaler = F_CPU/8

	OCR1A = ICR1 - 2000; //18000
	OCR1B = ICR1 - 1000; //18000	
	
	DDRB |= 1<<1 | 1<<2; // PB1 (OCR1A) and PB2 (OC1B)
}

void main() {

	DDRB = MOTOR_LEFT | MOTOR_RIGHT | DEBUG_PIN;
	DDRD = 1<<5;

	start_pwm();

#if defined(DEBUG) || defined(DEBUG_IR)
	uart_init();
#endif	
	motor_init();
	ir_init();

	leftDirection = STOP;
	rightDirection = STOP;

#ifdef DEBUG
	uart_print("online");
#endif	

	while( 1 ) {
		motor_loop();
		// if ( bit_is_set(UCSR0A, RXC0 ) ) {
//			change_direction( UDR0 );
		// }
	}
}