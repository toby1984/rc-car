#include "joystick.h"
#include <avr/io.h>
#include <stdlib.h>
#include "uart.h"

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define X_MIN 0.0
#define X_CENTER 498
#define X_MAX 1023

#define X_CENTER_MIN (X_CENTER*0.95)
#define X_CENTER_MAX (X_CENTER*1.05)

#define Y_MIN 0.0
#define Y_CENTER 495
#define Y_MAX 1023

#define Y_CENTER_MIN (Y_CENTER*0.95)
#define Y_CENTER_MAX (Y_CENTER*1.05)

// #define DEBUG_ADC

void joystick_init(void) {
  ADMUX = 1<<6; // internal voltage reference, MUX01	
  ADCSRA |= (1<<7)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // enable ADC
  DIDR0 = (1<<0)|(1<<1); // disable digital input buffers on ADC0 and ADC1	
}

void joystick_read(joystick_readings *result) 
{
  uint16_t tmp;

  // read ADC0
  ADMUX &= 0b11110000; // internal voltage reference, MUX00	
  sbi(ADCSRA,ADSC);
  while (bit_is_set(ADCSRA, ADSC)); // wait for conversion to finish

  tmp = ADC;
  #ifdef DEBUG_ADC
  uart_print("\r\nY = ");
  uart_putdecimal(tmp);
  #endif

  if ( tmp <= Y_CENTER_MIN ) {
    result->y = 100.0*(Y_CENTER_MIN - tmp) / (float) (Y_CENTER_MIN-Y_MIN);
  } else if ( tmp >= Y_CENTER_MAX ) {
    result->y = -100.0*(tmp-Y_CENTER_MAX) / (float) (Y_MAX-Y_CENTER_MAX);
  } else {
    result->y = 0;
  }

  // read ADC1
  ADMUX = (ADMUX & 0b11110000) | 1<<0; // internal voltage reference, MUX01
  sbi(ADCSRA,ADSC);
  while (bit_is_set(ADCSRA, ADSC)); // wait for conversion to finish

  tmp = ADC;
  #ifdef DEBUG_ADC
  uart_print("   |    X = ");
  uart_putdecimal(tmp);
  #endif

  if ( tmp <= X_CENTER_MIN ) {
    result->x =  100.0 * (X_CENTER_MIN-tmp) / (float) (X_CENTER_MIN-X_MIN);
  } else if ( tmp >= X_CENTER_MAX ) {
    result->x = -(100.0 * (tmp-X_CENTER_MAX) / (float) (X_MAX-X_CENTER_MAX));
  } else {
    result->x = 0;
  }

  #ifdef DEBUG_ADC
  uart_print("\r\n result Y = ");
  uart_putsdecimal(result->y);
  uart_print("     | result X = ");
  uart_putsdecimal(result->x);
  #endif

}
