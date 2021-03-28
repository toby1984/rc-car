#include "i2c.h"
#include "util/delay.h"
#include <stdint.h>

void main(void) {

    uart_init();
	i2c_init(0x12);

	uint8_t buffer[4];
	buffer[0]=0xde;
	buffer[1]=0xad;
	buffer[2]=0xbe;
	buffer[3]=0xef;

    uart_print("online v2\r\n");
	while (1) {
		uint8_t sent = i2c_send_noresponse(0xab,&buffer[0],4);
		uart_print("\r\nbytes send: ");
		uart_putdecimal(sent);
		_delay_ms(500);
	}
}