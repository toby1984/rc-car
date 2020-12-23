#include "radio_receiver.h"
#include "radio_common.h"
#include "uart.h"

void main(void) {

  uint8_t buffer[MSG_LEN];
  radio_receiver_init();
  uart_init();

  while(1) {
		uint8_t received_bytes = radio_receive(&buffer[0],MSG_LEN);
		if ( ( received_bytes & 0x80 ) == 0 && received_bytes > 0 ) {
			for ( uint8_t i = 0 ; i < received_bytes ; i++ ) {
				uart_print("\r\nByte ");
				uart_putsdecimal( i );
				uart_print(" : ");
				uart_putsdecimal( buffer[i] );
			}
		}
	}
}
