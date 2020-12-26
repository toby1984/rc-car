// #define __AVR_ATmega88PB__
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "radio_receiver.h"
#include "uart.h"
#include "crc.h"

#define DEBUG
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

#define RIGHT_MOTOR_SPD_ADJUST_FWD 0.87
#define RIGHT_MOTOR_SPD_ADJUST_BWD 0.75

enum direction {
	FORWARD,BACKWARD,STOP
};

volatile uint8_t is_moving;

enum direction leftDirection;
enum direction rightDirection;

// duty cycle in milliseconds, left motor
uint16_t dutyLeftMillis;
// duty cycle in milliseconds, right motor
uint16_t dutyRightMillis;

#define ICR1_VALUE 19999 // 50 hz at prescaler = F_CPU/8

void motor_speed_changed();

void start_pwm() {

    TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0 | 1<<COM1B1 | 1<<COM1B0;
    TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS11; // prescaler = 1/8

	ICR1 = ICR1_VALUE; // 50 hz at prescaler = F_CPU/8

	OCR1A = ICR1_VALUE - dutyLeftMillis; 
	OCR1B = ICR1_VALUE - dutyRightMillis;

	DDRB |= 1<<1 | 1<<2; // PB1 (OCR1A) and PB2 (OCR1B)
}

void motor_init() {

    is_moving = 0;

	leftDirection = STOP;
	rightDirection = STOP;

	dutyLeftMillis = 0;
	dutyRightMillis = 0;

	start_pwm();
}

void change_direction(char newDir) {

    enum direction oldLeft = leftDirection;
    enum direction oldRight = rightDirection;

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

	if ( oldLeft != leftDirection || oldRight != rightDirection ) {
		is_moving = leftDirection != STOP || rightDirection != STOP;
		switch(leftDirection) {
			case FORWARD:
				dutyLeftMillis = 1000; // 1ms high pulse
	          	break; 
	      	case STOP:
	      		dutyLeftMillis = 0;
	          	break;
	      	case BACKWARD:
	      	    dutyLeftMillis = 2000; // 2 ms high pulse
	     		break; 
	     }
	 	switch(rightDirection) {
	 		case FORWARD:
	 			dutyRightMillis = 2000*RIGHT_MOTOR_SPD_ADJUST_FWD;
	     		break; 
	 		case STOP:
	     		dutyRightMillis = 0;
	     		break;
	 		case BACKWARD:
	     		dutyRightMillis = 1000*RIGHT_MOTOR_SPD_ADJUST_BWD;
	     		break; 
	 	}
	 	motor_speed_changed();
 	}
}

void motor_speed_changed() {

#ifdef DEBUG
	uart_print("\r\nDuty cycles: left = ");
	uart_putdecimal(dutyLeftMillis);
	uart_print(" , right = ");
	uart_putdecimal(dutyRightMillis);	
#endif
	OCR1A = ICR1_VALUE - dutyLeftMillis; //18000
	OCR1B = ICR1_VALUE - dutyRightMillis; //18000	
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

void main() {

	uint8_t msg[3];

 	DDRB |= (MOTOR_LEFT | MOTOR_RIGHT | DEBUG_PIN);
 	DDRD |= 1<<5;

 	DDRC |= (1<<2) | (1<<3);

  #if defined(DEBUG) || defined(DEBUG_IR)
      uart_init();
  #endif	

 	motor_init();
 	radio_receiver_init();
 	// ir_init();

 #ifdef DEBUG
 	uart_print("online");
 #endif	

 	while (1) {
 		int8_t received = radio_receive(&msg[0],3);
 		if ( received == 3 && crc8(&msg[0],3) == 0 ) {
 			uart_print("\r\nreceived : ");
 			uint32_t value = (uint32_t) msg[0] << 16 | (uint32_t) msg[1] << 8 | (uint32_t) msg[2];
 			uart_puthex( value );
 		}
 	}

 	// while( 1 ) {	
 	// 	if ( is_moving ) {
 	// 		PORTC = (PORTC & ~_BV(2)) | _BV(3);
 	// 		_delay_ms(250);
  //            PORTC = (PORTC & ~_BV(3)) | _BV(2);			
  //            _delay_ms(250);
 	// 	} else {
 	// 		PORTC = 0;
 	// 		while ( ! is_moving ) {				
 	// 		}
 	// 	}
 	// }
}
