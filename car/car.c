// #define __AVR_ATmega88PB__
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "radio_receiver.h"
#include "uart.h"
#include "crc.h"

#define DEBUG

#define DEBUG_PIN _BV(5) // PB5

#define MOTOR_LEFT_DIR _BV(2)
#define MOTOR_LEFT_DIR_DDR DDRD
#define MOTOR_LEFT_DIR_REG PORTD

#define MOTOR_RIGHT_DIR _BV(3)
#define MOTOR_RIGHT_DIR_DDR DDRD
#define MOTOR_RIGHT_DIR_REG PORTD

// maximum speed difference (in percent) between both
// motors while turning and moving forwards / backwards at the same time
// this ultimately defines how fast you're able to turn when not stationary
#define MAX_MOTOR_SPEED_DIFF 0.2

enum direction {
	FORWARD,BACKWARD,STOP
};

enum direction leftDir;
enum direction rightDir;

void motor_init() {

    MOTOR_LEFT_DIR_DDR |= MOTOR_LEFT_DIR;
    MOTOR_RIGHT_DIR_DDR |= MOTOR_RIGHT_DIR;
        
    // enable output pins for 
    // left motor (OC0A = PD6 , OC0B = PD5
	DDRD |= (1<<6 | 1<<5); 	

    // enable output pins for 
    // right motor (OC2A = PB3 , OC2B = PD3)
    DDRB |= (1<<3);
    DDRD |= (1<<3);

	leftDir = STOP;
	rightDir = STOP;
}

void motor_left_stop() {

    TCCR0B &= ~(1<<CS02|1<<CS01|1<<CS00);
    TCCR0A &= ~(1<<COM0A1 | 1<<COM0A0 | 1<<COM0B1 | 1<<COM0B0 );  
  
	PORTD &= ~(1<<5|1<<6);
    
	leftDir = STOP;
}

void motor_right_stop() {

    TCCR2B &= ~(1<<CS22|1<<CS21|1<<CS20);
    TCCR2A &= ~(1<<COM2A1 | 1<<COM2A0 | 1<<COM2B1 | 1<<COM2B0 );  
  
    PORTB &= ~(1<<3);
    PORTD &= ~(1<<3);
    
	rightDir = STOP;
}

void motor_left_start(enum direction newDir, uint8_t newDuty ) {

    // left motor (OC0A = PD6 , OC0B = PD5
	uint8_t pwm = (0xff * newDuty) / 100.0;

#ifdef DEBUG
	uart_print("\r\nSpeed %: left = ");
	uart_putdecimal(newDuty);
	uart_print(" (");	
	uart_putdecimal(pwm);
	uart_print(")");		
#endif	
  
    if ( newDir == FORWARD ) { 
        // 0C0A = pwm , OC0B = 1
		OCR0A = pwm;					
		PORTD |=  (1<<5);

        TCCR0A &= ~(1<<COM0A1 | 1<<COM0A0 | 1<<COM0B1 | 1<<COM0B0 );  
        TCCR0A |=  (1<<WGM00  | 1<<COM0A1 | 1<<COM0A0 );        		
    } else { 
    	// 0C0A (PD6) = 1 , OC0B (PD5) = pwm
		OCR0B = pwm;
		PORTD |=  (1<<6);						
	
        TCCR0A &= ~(1<<COM0A1 | 1<<COM0A0 | 1<<COM0B1 | 1<<COM0B0 );  
        TCCR0A |=  (1<<WGM00  | 1<<COM0B1 | 1<<COM0B0 );  		
    }

    TCCR0B |= 1<<CS00; // CPU_FREQ / 1	    
      
	leftDir = newDir;
}

void motor_right_start(enum direction newDir, uint8_t newDuty ) {

    // right motor (OC2A = PB3 , OC2B = PD3)
	uint8_t pwm = (0xff * newDuty) / 100.0;

#ifdef DEBUG
	uart_print("\r\nSpeed %: right = ");
	uart_putdecimal(newDuty);
	uart_print(" (");	
	uart_putdecimal(pwm);
	uart_print(")");		
#endif	
  
    if ( newDir == FORWARD ) { 
        // 0C2A (PB3) = pwm , OC2B (PD3) = 1
		OCR2A = pwm;					
		PORTD |=  (1<<3);

        TCCR2A &= ~(1<<COM2A1 | 1<<COM2A0 | 1<<COM2B1 | 1<<COM2B0 );  
        TCCR2A |=  (1<<WGM20  | 1<<COM2A1 | 1<<COM2A0 );        		
    } else { 
    	// 0C2A (PB3) = 1 , OC2B (PD3) = pwm
		OCR2B = pwm;
		PORTB |=  (1<<3);						
	
        TCCR2A &= ~(1<<COM2A1 | 1<<COM2A0 | 1<<COM2B1 | 1<<COM2B0 );  
        TCCR2A |=  (1<<WGM20  | 1<<COM2B1 | 1<<COM2B0 );  		
    }

    TCCR2B |= 1<<CS20; // CPU_FREQ / 1	    
      
	rightDir = newDir;
}

void motor_change(enum direction newLeftDir,enum direction newRightDir, uint8_t newDutyLeft, uint8_t newDutyRight) {

    // TODO: Maybe stop both 8-bit timers if any of the outputs needs to change and 
    //       start them at the same time ?
	if ( newLeftDir == STOP ) {
		motor_left_stop();
	} else {
		motor_left_start(newLeftDir, newDutyLeft);
	}
	if ( newRightDir == STOP ) {
		motor_right_stop();
	} else {
		motor_right_start(newRightDir, newDutyRight);
	}	
}

uint8_t msg_size_calculator(uint8_t first_byte) {
	// TODO: Implement this based on message type, see protocol.h
	return 3;
}

void main() {

	uint8_t msg[3];

 	DDRC |= (1<<2) | (1<<3);

  #if defined(DEBUG) || defined(DEBUG_IR)
      uart_init();
  #endif	

 	motor_init();
 	radio_receiver_init();

 #ifdef DEBUG
 	uart_print("online");
 #endif	

 	while (1) {
 		int8_t received = radio_receive(&msg[0],msg_size_calculator);
 		if ( received == 3 && crc8(&msg[0],3) == 0 ) 
 		{
 			uart_print("\r\nreceived : ");
 			uint32_t value = (uint32_t) msg[0] << 16 | (uint32_t) msg[1] << 8 | (uint32_t) msg[2];
 			uart_puthex( value );

 			int8_t xDir = msg[0];
 			int8_t yDir = msg[1];
 			uart_print("(");
 			uart_putsdecimal(xDir);
 			uart_print("/");
 			uart_putsdecimal(yDir);
 			uart_print(")");

 			// TODO: Fix sender to not send values > 100
 			if ( xDir < -100 ) {
 				xDir = -100;
 			}
 			if ( yDir < -100 ) {
 				yDir = -100;
 			}

 			if ( yDir == 0 ) {
 				if ( xDir == 0 ) {
 					// full stop
 					motor_change(STOP,STOP, 0, 0);
 				} else if ( xDir < 0 ) {
 					// turn left in-place (left motor backwards, right motor forwards)
 					motor_change(BACKWARD,FORWARD, -xDir, -xDir );
 				} else {
 					// turn right in-place (left motor forwards, right motor backwards)
 					motor_change(FORWARD,BACKWARD, xDir, xDir ); 					
 				}
 			} else if ( xDir == 0 ) {
 				// yDir != 0 , xDir == 0 
 				if ( yDir > 0 ) {
 					// forwards		
					motor_change(FORWARD,FORWARD, yDir, yDir ); 								
 				} else {
 					// backwards
					motor_change(BACKWARD,BACKWARD, -yDir, -yDir );  					
 				}
 			} else {
 				// yDir != 0 , xDir != 0 
 				enum direction dir = yDir > 0 ? FORWARD : BACKWARD;
 				if ( xDir < 0 ) {
 					// turn left while driving forward -> left motor must run SLOWER than right motor
 					float delta = 1.0 - MAX_MOTOR_SPEED_DIFF*(-xDir/100.0);
					motor_change(dir, dir, yDir*delta , yDir ); 
 				}  else {
 					// turn right while driving forward -> right motor must run SLOWER than left motor
					float delta = 1.0 - MAX_MOTOR_SPEED_DIFF*(xDir/100.0); 						
					motor_change(dir, dir, yDir, yDir*delta ); 						
 				}																
 			}
 		}
 	}
}
