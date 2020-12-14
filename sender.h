#ifndef SENDER_H
#define SENDER_H
#include <stdint.h>

typedef struct joystick_readings {
	int8_t x;
	int8_t y;
} joystick_readings;

void joystick_init(void);
void joystick_read(joystick_readings *result);

#endif