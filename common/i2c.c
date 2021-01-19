#include "i2c.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include "util/twi.h"

#define DEBUG_I2C

#ifdef DEBUG_I2C
#include "uart.h"
#endif

static volatile uint8_t data[I2C_BUFFER_SIZE];

static volatile uint8_t bytes_in_buffer;
static volatile uint8_t read_offset;
static volatile uint8_t write_offset;
static volatile uint8_t dropped_byte_cnt;

#define MT_START 0x08
#define MT_SLA_ACK 0x18

#define i2c_status() (TWSR & 0xF8)

static void i2c_become_slave_receiver() {
	uint8_t current = TWCR;	

/*
Become slave receiver.

To initiate the SR mode, the TWI (Slave) Address Register n (TWARn) and the TWI Control Register n
(TWCRn) must be initialized as follows:
1. The upper seven bits of TWARn are the address to which the 2-wire Serial Interface will respond when addressed by a Master (TWARn.TWA[6:0]).
2. If the LSB of TWARn is written to TWARn.TWGCI=1, the TWIn will respond to the general call address (0x00), otherwise it will ignore the general call address.
3. TWCRn must hold a value of the type TWCRn=0100010x - TWCRn.
  * TWEN must be written to '1' to enable the TWI. 
  * TWEA bit must be written to '1' to enable the acknowledgment of the device’s own slave address or the general call address. 
   * TWSTA and TWSTO must be written to zero.
*/	

	current &= (1<<TWSTA) | (1<<TWSTO);
	current |= (1<<TWIE) | (1<<TWEN) | (1<<TWEA);
	TWCR = current;
}

void i2c_init(uint8_t myAddress) {
	TWAR = myAddress & ~(1<<0); // clear LSB as setting it would indicate we want to respond to the "general call" (0x00) I2C address

	dropped_byte_cnt = 0;
	bytes_in_buffer = 0;
	read_offset = write_offset = 0;

    // setup 100 kHz SCL
#if F_CPU == 16000000
    TWSR = 0x00; // Prescaler 1x
    TWBR = 72; 
#elif F_CPU == 8000000
    TWSR = 0x00; // Prescaler 1x
    TWBR = 32;
#else
#error Unhandled CPU frequency for I2C
#endif    

    //enable TWI
    TWCR = (1<<TWEN);
	DDRC |= (1<<4) | (1<<5);

	i2c_become_slave_receiver();
}

static void i2c_disable_read_irq() {
	TWCR &= ~(1<<TWIE);
}

static uint8_t readByte() {
	if ( bytes_in_buffer > 0 ) {
		uint8_t result = data[read_offset];
		read_offset = (read_offset+1) % I2C_BUFFER_SIZE;	
		bytes_in_buffer--;
		return result;
	}
	return 0xff;
}

static void i2c_await() {
	while (!(TWCR & (1<<TWINT)));
}

uint8_t i2c_send_noresponse(uint8_t destinationAddress, uint8_t *msg, uint8_t len) {

#ifdef DEBUG_I2C		
		uart_print("\r\ni2c_send_noresponse(): Number of bytes to sent: ");
		uart_putdecimal(len);
#endif

	uint8_t bytes_transmitted = 0;

	i2c_disable_read_irq();

  	TWCR = (1<<TWINT)| (1<<TWSTA) | (1<<TWEN); // send START
  	
	i2c_await();
	
	if ( i2c_status() != TW_START) { // check for error
#ifdef DEBUG_I2C		
		uart_print("\r\ni2c_send_noresponse(): Failed to send start, error: ");
		uart_puthex(i2c_status());
#endif		
		goto error2;
	}

	// send address + W
	TWDR = destinationAddress | 1;
	TWCR = (1<<TWINT) | (1<<TWEN);

	i2c_await(); 
	
	if ( i2c_status() != TW_MT_SLA_ACK) { // check for errors
#ifdef DEBUG_I2C		
		uart_print("\r\ni2c_send_noresponse(): Failed to send SLA+W, error: ");
		uart_puthex(i2c_status());
#endif			
		goto error;
	}

	uint8_t remaining = len;
	for (  ; remaining > 0 ; remaining--) 
	{		
		TWDR = *msg++; // load data byte
		TWCR = (1<<TWINT) | (1<<TWEN);

		i2c_await();

		if ( i2c_status() != TW_MT_DATA_ACK) { // check for errors
#ifdef DEBUG_I2C		
		uart_print("\r\ni2c_send_noresponse(): Failed to send data, error: ");
		uart_puthex(i2c_status());
#endif				
			break;
		}
	}
	bytes_transmitted = len - remaining;

error:		
	TWCR = (1<<TWINT)| (1<<TWEN)|(1<<TWSTO); // transmit STOP
error2:	

	// TODO: If we actually expected data bytes as a reply from the slave we've addressed, we'd need to
	//       send a REPEAT START and switch to master receiver mode, see AVR documentation
	i2c_become_slave_receiver();
	return bytes_transmitted;
}

uint8_t i2c_receive(uint8_t *buffer, uint8_t len) 
{
	uint8_t oldSreg = SREG;
	cli();

	uint8_t result = bytes_in_buffer > len ? len : bytes_in_buffer;
	for ( uint8_t i = result ; i > 0 ; i-- ) {
		*buffer++ = readByte();
	}

	if ( ( oldSreg & (1<<7) ) != 0 ) { // only enable IRQs if we were the ones disabling them
		sei();
	}		
	return result;
}

static void writeByte(uint8_t value) 
{
	data[write_offset] = value;
	if ( read_offset == write_offset && bytes_in_buffer > 0 ) {
		dropped_byte_cnt++;
	} else {
		bytes_in_buffer++;
	}
	write_offset = (write_offset+1) % I2C_BUFFER_SIZE;	
}

// interrupt routine for receiving data via TWI
ISR(TWI_vect) {	
	writeByte(TWDR);
	TWCR |= (1<<TWINT) | (1<<TWEA); // receive next byte and send ACK				
}