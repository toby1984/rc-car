#include "joystick.h"
#include "radio_sender.h"
#include "crc.h"
#include "util/delay.h"

void main(void) {

    joystick_readings reading;
    uint8_t msg[3];

    joystick_init();
    radio_sender_init();

	while(1) {
		joystick_read(&reading);
		msg[0] = reading.x;
		msg[1] = reading.y;
		msg[2] = 0; // we're using CRC8, shift-in 8 zero bits at the end of the message
		msg[2] = crc8(&msg[0],3);

		radio_send(&msg[0],3);
		_delay_ms(250);
	}
}