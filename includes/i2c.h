#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define I2C_BUFFER_SIZE 128

// initialize I2C subsystem
// param: myAddress - I2C address to listen on
void i2c_init(uint8_t myAddress);

// returns the number of bytes that
// got dropped from the internal receive buffer
// because i2c_receive() was not called often enough
uint8_t i2c_get_dropped_byte_cnt();

// send I2C request in master transmitter "fire-and-forget" mode, we don't expect to receive any data back
void i2c_send_noresponse(uint8_t destinationAddress, uint8_t *msg, uint8_t len);

// read data from internal buffer
// result: number of bytes read
uint8_t i2c_receive(uint8_t *buffer, uint8_t len);
#endif
