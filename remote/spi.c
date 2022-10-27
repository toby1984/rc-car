#include "spi.h"
#include "uart.h"
#include <stdint.h>
#include "avr/io.h"

void spi_master_init(void)
{
    // MOSI = Master Out, Slave In
    // MISO = Master In, Slave Out

    // AVR328p:
    //
    // SS   - PB2
    // MOSI - PB3
    // MISO - PB4
    // SCK  - PB5

    // Set SS (PB2), MOSI (PB3) and SCK (PB5) output, all others input
    DDRB = (DDRB & ~(1<<4 )) | (1<<5) | (1<<3) | (1<<2);
    PORTB |= (1<<2); // set SS to high, line is active-low

    // Enable SPI, Master, set clock rate fck/128
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
}

void spi_master_putc(char data)
{
    // Start transmission
    SPDR = data;
    // Wait for transmission complete
    while (!(SPSR & (1<<SPIF))) ;
}

void spi_master_send(char *data,uint8_t len)
{
    for ( ; len > 0 ; len-- ) {
        spi_master_putc(*data++);
    }
}

void spi_slave_init(void)
{
    // Set MISO output, all others input
    DDRB = (DDRB & ~(1<<2|1<<3|1<<5) ) | (1<<4);

    // Enable SPI
    SPCR = (1<<SPE);
}

char spi_slave_receive(void)
{
    // Wait for reception complete
    while(!(SPSR & (1<<SPIF)));
    // Return Data Register
    return SPDR;
}

/*
The void SpiWriteRegister(U8 reg, U8 value) function writes a new value into a register on the radio. If only one
register of the radio is written, a 16-bit value must be sent to the radio (the 8-bit address of the register followed by
the new 8-bit value of the register). To write a register, the MSB of the address is set to 1 to indicate a device write.
void SpiWriteRegister (U8 reg, U8 value)
{
 // Send SPI data using double buffered write
//Select the radio by pulling the nSEL pin to low
 NSS = 0;

 //write the address of the register into the SPI buffer of the MCU
//(important to set the MSB bit)
SPI1DAT = (reg|0x80); //write data into the SPI register
//wait until the MCU finishes sending the byte
while( SPIF1 == 0);
SPIF1 = 0;
//write the new value of the radio register into the SPI buffer of the MCU
SPI1DAT = value; //write data into the SPI register
//wait until the MCU finishes sending the byte
while( SPIF1 == 0); //wait for sending the data
SPIF1 = 0;
 //Deselect the radio by pulling high the nSEL pin
NSS = 1;
}
 */
void spi_write_register(uint8_t reg, uint8_t value)
{
  spi_master_init();
  PORTB &= ~(1<<2);
  spi_master_putc( reg | (1<<7) ); // WRITE
  spi_master_putc( value );
  PORTB |= (1<<2);
}

/*
The U8 SpiReadRegister(U8 reg) function reads one register from the radio. When reading a single register of the
radio, a 16 bit value must be sent to the radio (the 8-bit address of the register followed by a dummy 8-bit value).
The radio provides the value of the register during the second byte of the SPI transaction. Note that it is crucial to
clear the MSB of the register address to indicate a read cycle.
U8 SpiReadRegister (U8 reg)
{
//Select the radio by pulling the nSEL pin to low
NSS = 0;
//Write the address of the register into the SPI buffer of the MCU
//(important to clear the MSB bit)
 SPI1DAT = reg; //write data into the SPI register
//Wait until the MCU finishes sending the byte
while( SPIF1 == 0);
SPIF1 = 0;
//Write a dummy data byte into the SPI buffer of the MCU. During sending
//this byte the MCU will read the value of the radio register and save it
//in its SPI buffer.
SPI1DAT = 0xFF; //write dummy data into the SPI register
//Wait until the MCU finishes sending the byte
while( SPIF1 == 0);
SPIF1 = 0;
//Deselect the radio by pulling high the nSEL pin
NSS = 1;
//Read the received radio register value and return with it
 return SPI1DAT;
}
 */
uint8_t spi_read_register(uint8_t reg)
{
  spi_master_init();
  PORTB &= ~(1<<2);
  spi_master_putc( reg & ~(1<<7) ); // READ
  spi_master_putc( 0 ); // dummy value so we're generating clock pulses for the client to write with
  uint8_t result = spi_slave_receive();
  PORTB |= (1<<2);
  return result;
}

