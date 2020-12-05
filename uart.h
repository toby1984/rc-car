#ifndef UART_H
#define UART_H

#define BAUD 19200
#include <util/setbaud.h>

void uart_init(void);
void uart_putchar(char c);
void uart_print(char *c);
char uart_getchar(void);
void uart_putdecimal(uint16_t value);
void uart_puthex(uint32_t value);

#endif