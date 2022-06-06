#include "joystick.h"
#include "radio_sender.h"
#include "crc.h"
#include "util/delay.h"
#include <avr/io.h>
#include "uart.h"
// #include "i2c.h"

void main(void) 
{
    joystick_readings reading;
    uint8_t msg[3];

    joystick_init();
    uart_init();
    radio_sender_init();

    // TODO: Remove i2c test code once done !!
#warning "Remove I2C test loop when done !!!"

//    // LSB (bit 0) is used as a R/W indicator.
//    // bit 0 set   -> READ
//    // bit 0 clear -> WRITE
//    uint8_t myAddr = 0b10011100;
//    uint8_t dstAddr = 0b1010100;
//
//    i2c_init(myAddr); // 7-bit address
//
//    msg[0] = 0x01;
//    msg[1] = 0x02;
//    msg[2] = 0x03;
//
//    while ( 1 ) {
//        uart_print("\r\n=== sending msg\r\n");
//        i2c_send_noresponse(dstAddr,&msg[0],3);
//		_delay_ms(500);
//    }
//    // TODO: End i2c test code

	while(1)
    {
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
		radio_send(&msg[0],3);
		_delay_ms(250);		

	}
}
