#ifndef SPI_H
#define SPI_H
#include <stdint.h>

// general purpose SPI
void spi_master_init(void);
void spi_master_putc(char data);
void spi_master_send(char *data,uint8_t len);

uint8_t spi_master_read(char *buffer,uint8_t bufferSize);
void spi_write_register(uint8_t reg, uint8_t value);
uint8_t spi_read_register(uint8_t reg);

#endif
