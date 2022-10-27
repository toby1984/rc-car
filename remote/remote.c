#include "joystick.h"
#include "crc.h"
#include "util/delay.h"
#include <avr/io.h>
#include "uart.h"
#include "spi.h"
#include "si4432.h"

void main(void) 
{
    joystick_readings reading;
    uint8_t msg[3];

    joystick_init();
    uart_init();

    uart_print("\r\nReady\r\n");
    spi_master_init();

   _delay_us(2000);

    spi_write_register(0x07, 0x80); // write to Operating & Function Control1 register

   _delay_us(2000);
//
//     uint8_t deviceType = si4432_get_device_type();
//     uint8_t deviceVersion = si4432_get_device_version();
//
//     uart_print("\r\nDevice type: ");
//     uart_putdecimal( deviceType );
//     uart_print("\r\nDevice Version: ");
//     uart_putdecimal( deviceVersion);

	while(1) {

        for (uint8_t w = 0 ; w <= 255; w++) {
            spi_write_register(0x01, w);
            _delay_us(500);
        }

//         deviceType = si4432_get_device_type();
//         deviceVersion = si4432_get_device_version();
/*
		joystick_read(&reading);

		msg[0] = reading.x;
		msg[1] = reading.y;
		msg[2] = crc8(&msg[0],2);

        uart_print("\r\n=========\r\n");
        uart_putsdecimal(msg[0]);
        uart_print("\r\n");
        uart_putsdecimal(msg[1]);
        uart_print("\r\n");
        uart_putdecimal(msg[2]);
        uart_print("\r\n");
		radio_send(&msg[0]
		,3);
		_delay_ms(250);	*/

	}
}
