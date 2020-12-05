// #define __AVR_ATmega88PB__
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define DEBUG_PIN _BV(5) // PB5
#define MOTOR_LEFT _BV(4) // PB4
#define MOTOR_RIGHT _BV(3) // PB3
#define IR_IN 6 // PD6

#define BAUD 19200

#include <util/setbaud.h>

void uart_init(void) {
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

#if USE_2X
	UCSR0A |= _BV(U2X0);
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void uart_putchar(char c) {
	UCSR0A &= _BV(TXC0);
	UDR0 = c;
  loop_until_bit_is_set(UCSR0A, TXC0); /* Wait until transmission ready. */
}

void uart_print(char *c) {

	while ( *c ) {
		uart_putchar(*c++);
	}
}


char uart_getchar(void) {
    loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
	return UDR0;
}

#define CYCLES_PER_MS (8000000/1000.0)
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

	if ( currentLoopCount <= 0 ) 
	{
		currentLoopCount = MOTOR_LOOP_COUNT;    

		switch(leftDirection) {
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
     switch(rightDirection) {
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
		leftDirection = FORWARD;
		rightDirection = FORWARD;      
		break;
		case 's':
		leftDirection = BACKWARD;
		rightDirection = BACKWARD;     
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

char buffer[10];

void uart_putdecimal(uint16_t value) {

	utoa(value & 0xffff,buffer,10);
	uart_print(buffer);
}

const char HEX[] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};

void uart_puthex(uint32_t value) {
  for (uint8_t i = 4 ; i > 0 ; i-- ) {
  	uint8_t tmp = (uint8_t) ((value & 0xff000000) >> 24);
    uart_putchar( HEX[ (tmp & 0xf0) >> 4 ] );
    uart_putchar( HEX[ (tmp & 0x0f)      ] );    
  	value <<= 8;
  }
}

void irq_setup() {
	cli();
  // let INT1 be triggered by falling edge,
  // which marks the beginning of an IR transmission
	EICRA = (EICRA & ~(_BV(ISC11) | _BV(ISC10))) | _BV(ISC11);
	EIMSK |= _BV(INT1);
	sei();
}

#define MAX_IR_BITS 48
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
    	// decode
   		// first HIGH is 9 ms,
   		// following LOW is 4.5ms
   		// Logical '0' – a 562.5µs pulse burst followed by a 562.5µs space, with a total transmit time of 1.125ms
   		// Logical '1' – a 562.5µs pulse burst followed by a 1.6875ms space, with a total transmit time of 2.25ms
		uint16_t unit = ir_rec_high[0] / 16;
		uint16_t unitLowerBound = (uint16_t) (unit*0.95);
		uint16_t unitUpperBound = (uint16_t) (unit*1.05);
		if ( ir_rec_low[0] < 8*unitLowerBound || ir_rec_low[0] > 8*unitUpperBound ) {
			uart_print("ERROR: Preable not recognized\r\n");
		} else {
			uint32_t result = 0;
			for ( uint8_t i = 1 ; i < bitCount ; i++ ) {
				if ( ir_rec_high[i] < unitLowerBound || ir_rec_high[i] > unitUpperBound ) {
					uart_print("ERROR: Failed to decode HIGH part of bit ");
					uart_putdecimal( i );
					uart_print("\r\n");
					break;
				}
				result <<= 1;
				if ( ir_rec_low[i] >= unitLowerBound && ir_rec_high[i] <= unitUpperBound ) {
                  // Logical '0'				  				
				} else if ( ir_rec_low[i] >= 3*unitLowerBound && ir_rec_high[i] <= 3*unitUpperBound ) {
                  // Logical '1'
				  result |= 1;
				} else {
		            uart_print("ERROR: Failed to decode LOW part of bit ");
					uart_putdecimal( i );
					uart_print("\r\n");
					break;					
				}
			}
			uart_print("result: ");
			uart_puthex( result );
			uart_print("\r\n");			
		}
	}
}

void main() {

	DDRB = MOTOR_LEFT | MOTOR_RIGHT | DEBUG_PIN;
	DDRD = 1<<5;

	uart_init();
	motor_init();
	irq_setup();

	leftDirection = STOP;
	rightDirection = STOP;

	uart_print("online");

	while( 1 ) {
		motor_loop();
		if ( bit_is_set(UCSR0A, RXC0 ) ) {
			change_direction( UDR0 );
		}
	}

  // while ( 1 ) {
  //   PORTD = 1<<5; // Turn on LED
  //   char c = uart_getchar();
  //   uart_putchar(c);
  //   forward(1.5);
  //   _delay_ms(1000);
  //   PORTD = ~(1<<5); // Turn off LED
  //   _delay_ms(1000);
  // }
}
