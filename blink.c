// #define __AVR_ATmega88PB__
#include <avr/io.h>
#include <util/delay.h>

#define DEBUG_PIN (1<<5) // PB5
#define MOTOR_LEFT (1<<4) // PB4
#define MOTOR_RIGHT (1<<3) // PB3
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

void main() {

  DDRB = MOTOR_LEFT | MOTOR_RIGHT | DEBUG_PIN;
  DDRD = 1<<5;

  uart_init();
  motor_init();

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
