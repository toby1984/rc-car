#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define I2C_BUFFER_SIZE 128

void i2c_init(uint8_t myAddress);
uint8_t i2c_get_dropped_byte_cnt();
// send I2C request in master transmitter "fire-and-forget" mode, we don't expect to receive any data back
uint8_t i2c_send_noresponse(uint8_t destinationAddress, uint8_t *msg, uint8_t len);
uint8_t i2c_receive(uint8_t *buffer, uint8_t len);
#endif