#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "radio_receiver.h"
#include "uart.h"
#include "crc.h"
#include "watchdog.h"
#include "pid.h"
#include "encoder.h"

#define WATCHDOG_TIMEOUT_SECONDS 30
#define WATCHDOG_DISABLED
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
#define MAX_MOTOR_SPEED_DIFF 0.5

// MCU will only be powered as long as
// POWER_UP_PIN is HIGH
#define POWER_UP_PIN 6
#define POWER_UP_DDR DDRL
#define POWER_UP_PORT PORTL

enum direction {
	FORWARD,BACKWARD,STOP
};

enum direction leftDir;
enum direction rightDir;

pid_data left_motor_pid;
pid_data right_motor_pid;

void mcu_power_on() {
      POWER_UP_DDR |= (1<<POWER_UP_PIN);
      POWER_UP_PORT |= (1<<POWER_UP_PIN);
}

void mcu_power_off() {
      POWER_UP_DDR |= (1<<POWER_UP_PIN);
      POWER_UP_PORT &= ~(1<<POWER_UP_PIN);
}

void motor_init() {

    // initialize PID controllers
    pid_init(&left_motor_pid,0,0,0);
    pid_init(&right_motor_pid,0,0,0);

    // setup two timers for counting motor encoder
    // events.
    // 8 bit counters are enough since the motors & voltages I'm using
    // yields at most ~1300 ticks per second and
    // I'm going to update the PID controllers with the current tick rate
    // every 25 ms so I expected at most 1300 / (1000ms / 25ms) => ~33 ticks per sample

    MOTOR_LEFT_DIR_DDR |= MOTOR_LEFT_DIR;
    MOTOR_RIGHT_DIR_DDR |= MOTOR_RIGHT_DIR;
        
    // enable output pins for left motor
    // Atmega2560: (OC0A = PB7, OC0B = PG5)

	DDRB |= (1<<7);
	DDRG |= (1<<5);

    // enable output pins for right motor
    // Atmega2560: (OC2A = PB4 , OC2B = PH6)
    DDRB |= (1<<4);
    DDRH |= (1<<6);

	leftDir = STOP;
	rightDir = STOP;
}

void motor_left_stop() {

#ifdef DEBUG
	uart_print("\r\nMotor left: STOP");
#endif
    TCCR0B &= ~(1<<CS02|1<<CS01|1<<CS00);
    TCCR0A &= ~(1<<COM0A1 | 1<<COM0A0 | 1<<COM0B1 | 1<<COM0B0 );  
  
	PORTB &= ~(1<<7);
	PORTG &= ~(1<<5);
    
	leftDir = STOP;
}

void motor_right_stop() {

#ifdef DEBUG
	uart_print("\r\nMotor right: STOP");
#endif
    TCCR2B &= ~(1<<CS22|1<<CS21|1<<CS20);
    TCCR2A &= ~(1<<COM2A1 | 1<<COM2A0 | 1<<COM2B1 | 1<<COM2B0 );  
  
    PORTB &= ~(1<<4);
    PORTH &= ~(1<<6);
    
	rightDir = STOP;
}

void motor_left_start(enum direction newDir, uint8_t newDuty ) {

    // left motor (OC0A = PD6 , OC0B = PD5
	uint8_t pwm = (0xff * newDuty) / 100.0;

#ifdef DEBUG
	uart_print("\r\nMotor left: direction=");
	uart_putdecimal(newDir);
    uart_print(" , duty ");
	uart_putdecimal(newDuty);
	uart_print(" (pwm");
	uart_putdecimal(pwm);
	uart_print(")");		
#endif	
  
    if ( newDir == FORWARD ) { 
        // 0C0A = pwm , OC0B = 1
		OCR0A = pwm;					
		PORTG |=  (1<<5);

        TCCR0A &= ~(1<<COM0A1 | 1<<COM0A0 | 1<<COM0B1 | 1<<COM0B0 );  
        TCCR0A |=  (1<<WGM00  | 1<<COM0A1 | 1<<COM0A0 );        		
    } else { 
    	// 0C0A = 1 , OC0B = pwm
		OCR0B = pwm;
		PORTB |=  (1<<7);
	
        TCCR0A &= ~(1<<COM0A1 | 1<<COM0A0 | 1<<COM0B1 | 1<<COM0B0 );  
        TCCR0A |=  (1<<WGM00  | 1<<COM0B1 | 1<<COM0B0 );  		
    }

    TCCR0B |= 1<<CS00; // CPU_FREQ / 1	    
      
	leftDir = newDir;
}

void motor_right_start(enum direction newDir, uint8_t newDuty ) {

    // right motor (OC2A , OC2B)
	uint8_t pwm = (0xff * newDuty) / 100.0;

#ifdef DEBUG
	uart_print("\r\nMotor right: direction=");
	uart_putdecimal(newDir);
    uart_print(" , duty ");
	uart_putdecimal(newDuty);
	uart_print(" (pwm");
	uart_putdecimal(pwm);
	uart_print(")");
#endif
  
    if ( newDir == FORWARD ) { 
        // 0C2A = pwm , OC2B = 1
		OCR2A = pwm;					
		PORTH |=  (1<<6);

        TCCR2A &= ~(1<<COM2A1 | 1<<COM2A0 | 1<<COM2B1 | 1<<COM2B0 );  
        TCCR2A |=  (1<<WGM20  | 1<<COM2A1 | 1<<COM2A0 );        		
    } else { 
    	// 0C2A = 1 , OC2B = pwm
		OCR2B = pwm;
		PORTB |=  (1<<4);
	
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

int8_t clamp(int8_t input) {
    return input < -100 ? -100 : input > 100 ? 100 : input;
}

void watchdog_irq() {
    mcu_power_off();
}

// careful, this is an interrupt handler...
void encoder_callback(uint16_t ticksLeftMotor, uint16_t ticksRightMotor) {
 #ifdef DEBUG
 	uart_print("\r\n*** Encoders left/right: ");
    uart_putdecimal( ticksLeftMotor );
 	uart_print("/ ");
    uart_putdecimal( ticksRightMotor );
 	uart_print("\r\n");
 #endif
}

void main() {

	uint8_t msg[3];

    // change pin to HIGH as the
    // very first action here so
    // MCU stays powered
    mcu_power_on();

 	DDRC |= (1<<2) | (1<<3); // HMMMM... what's this for ? delete ?

  #if defined(DEBUG) || defined(DEBUG_IR)
      uart_init();
  #endif	

 	motor_init();
 	radio_receiver_init();

    enc_init(1000, &encoder_callback);

 #ifdef DEBUG
 	uart_print("online 2");
 #endif	

#ifndef WATCHDOG_DISABLED
    watchdog_start( watchdog_irq, WATCHDOG_TIMEOUT_SECONDS );
#endif

 	while (1) {
        msg[0]=0;
        msg[1]=0;
 		int8_t received = radio_receive(&msg[0],msg_size_calculator);
 		if ( received == 3 && crc8(&msg[0],3) == 0 ) 
 		{
#ifdef DEBUG
 			// uint32_t value = (uint32_t) msg[0] << 16 | (uint32_t) msg[1] << 8 | (uint32_t) msg[2];
 			// uart_puthex( value );
#endif

 			int8_t xDir = msg[0];
 			int8_t yDir = msg[1];

#ifdef DEBUG
 			uart_print("\r\n(");
 			uart_putsdecimal(xDir);
 			uart_print("/");
 			uart_putsdecimal(yDir);
 			uart_print(")");
#endif

			xDir = clamp(xDir);
			yDir = clamp(yDir);

 			if ( yDir == 0 ) {
 				if ( xDir == 0 ) {
 					// full stop
 					motor_change(STOP,STOP, 0, 0);
 				} else if ( xDir < 0 ) {
 					// turn left in-place (left motor backwards, right motor forwards)
 					motor_change(BACKWARD,FORWARD, -xDir, -xDir );
#ifndef WATCHDOG_DISABLED
                    watchdog_reset();
#endif
 				} else {
 					// turn right in-place (left motor forwards, right motor backwards)
 					motor_change(FORWARD,BACKWARD, xDir, xDir );
#ifndef WATCHDOG_DISABLED
                    watchdog_reset();
#endif
 				}
 			} else if ( xDir == 0 ) {
 				// yDir != 0 , xDir == 0 
                watchdog_reset();
 				if ( yDir > 0 ) {
 					// forwards		
					motor_change(FORWARD,FORWARD, yDir, yDir );
 				} else {
 					// backwards
					motor_change(BACKWARD,BACKWARD, -yDir, -yDir );  					
 				}
 			} else {
 				// yDir != 0 , xDir != 0 
                watchdog_reset();
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
