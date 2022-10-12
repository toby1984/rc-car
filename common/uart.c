#include <stdlib.h>
#include <avr/io.h>
#include "uart.h"

#define XSTR(x) STR(x)
#define STR(x) #x

char buffer[10];

void uart_init(void) {

#pragma message "Register is" XSTR( UBRR0H )
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

void uart_putdecimal(uint16_t value) {

	utoa(value & 0xffff,buffer,10);
	uart_print(buffer);
}

void uart_putsdecimal(int8_t value) {

	if ( value & (1<<7) ) {
		uart_putchar('-');
		value = ~value + 1;		
	} 
    uart_putdecimal(value);	
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
