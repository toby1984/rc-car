#include "i2c.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include "util/twi.h"

#define DEBUG_I2C

#ifdef DEBUG_I2C
#include "uart.h"
#endif

enum i2c_mode {
        I2C_MODE_IDLE=0,
        I2C_MODE_SEND,
        I2C_MODE_RECEIVE
};

enum i2c_outcome {
    I2C_SUCCESS = 0,
    I2C_PENDING,
    I2C_ERROR
};

enum i2c_state {
    I2C_SEND_RECV_ADR,
    I2C_SEND_FIRST_BYTE,
    I2C_SEND_NEXT_BYTE,
}

static volatile i2c_mode i2c_mode;
static volatile uint8_t *i2c_buffer;
static volatile uint8_t i2c_bufferSize;
static volatile uint8_t i2c_offsetInBuffer;
static volatile i2c_outcome i2c_status;
static volatile i2c_state i2c_current_state;

#define MT_START 0x08
#define MT_SLA_ACK 0x18

#define i2c_status() (TWSR & 0xF8)

void i2c_init(uint8_t myAddress)
{
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

    // set TWI ports to output
	DDRC |= (1<<4) | (1<<5);
	PORTC &= ~(1<<4|1<<5);

    i2c_mode = I2C_MODE_IDLE;
    i2c_bufferSize = 0;
    i2c_offsetInBuffer = 0;
    i2c_status = I2C_SUCCESS;
}

static void i2c_disable_irq() {
	TWCR &= ~(1<<TWIE);
}

static void i2c_enable_irq() {
	TWCR |= (1<<TWIE);
}

static void i2c_await()
{
#ifdef DEBUG_I2C
		uart_print("\r\ni2c_await(): Waiting...");
#endif
	while (!(TWCR & (1<<TWINT)));
}

void i2c_send_noresponse(uint8_t destinationAddress, uint8_t *msg, uint8_t len)
{

	uint8_t bytes_transmitted = 0;

#ifdef DEBUG_I2C
		uart_print("\r\ni2c_send_noresponse(): About to disable read IRQ ");
		uart_putdecimal(len);
#endif

	i2c_disable_irq();

    if ( i2c_mode != I2C_MODE_IDLE ) {
#ifdef DEBUG_I2C
		uart_print("\r\ni2c_send_noresponse(): I2C still busy.");
#endif
        return;
    }
    i2c_buffer = msg;
    i2c_bufferSize = len;

    i2c_status = I2C_PENDING;
    i2c_state = I2C_SEND_RECV_ADR;

    i2c_enable_irq();

  	TWCR = (1<<TWINT)| (1<<TWSTA) | (1<<TWEN); // send START, let IRQ handle response
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

#ifdef DEBUG_I2C
void printError(char *msg) {
        uart_print("\r\ni2c_error(): ");
        uart_print( msg );
        uart_print( " - errno: " );
        uart_puthex(i2c_status());
}
#endif

void operationFinished(uint8_t success)
{
    i2c_disable_irq();
    i2c_status = success ? I2C_SUCCESS : I2C_ERROR;
    i2c_mode = I2C_MODE_IDLE;
}

void send_next_byte(uint8_t checkForError)
{
    if ( checkForError )
    {
        if ( i2c_status() != TW_MT_DATA_ACK)
        {
#ifdef DEBUG_I2C
            printError("Failed to send data");
#endif
            operationFinished(0);
            return;
        }
    }

    if ( i2c_offsetInBuffer < i2c_bufferSize )
    {
        TWDR = i2c_buffer[ i2c_offsetInBuffer++ ];
        TWCR = (1<<TWINT) | (1<<TWEN);
        return;
    }

#ifdef DEBUG_I2C
    printError("Finished sending data successfully.");
#endif

    operationFinished(1);
    return;
}

// interrupt routine for receiving data via TWI
ISR(TWI_vect)
{
    switch( i2c_mode )
    {
        case I2C_MODE_SEND:

            switch( i2c_current_state )
            {
                /* START send, check outcome & send SLA+W */
                case I2C_SEND_RECV_ADR:
                    if ( i2c_status() != TW_START) {
#ifdef DEBUG_I2C
                        printError("Failed to send START");
#endif
                        operationFinished(0);
                        return;
                    }
                    i2c_current_state = I2C_SEND_FIRST_BYTE;

                    // send address + W
                    // LSB is R/W flag
                    // set -> READ
                    // cleared -> WRITE
                    TWDR = destinationAddress & ~(1<<0);
                    TWCR = (1<<TWINT) | (1<<TWEN);
                    break;

                /* Send very first byte */
                case I2C_SEND_FIRST_BYTE:

                    if ( i2c_status() != TW_MT_SLA_ACK) {
#ifdef DEBUG_I2C
                        printError("Failed to send SLA+W");
#endif
                        operationFinished(0);
                        return;
                    }

                    i2c_current_state = I2C_SEND_NEXT_BYTE;
                    send_next_byte(0); // do not check for error before sending
                    break;
                /* Send subsequent bytes */
                case I2C_SEND_NEXT_BYTE:
                    send_next_byte(1); // check for error before sending another byte
                    break;
            }
            break;
        case I2C_MODE_RECEIVE:
            if ( i2c_status() !=
            writeByte(TWDR);
            TWCR |= (1<<TWINT) | (1<<TWEA); // receive next byte and send ACK

            break;

        default:
#ifdef DEBUG_I2C
		uart_print("\r\ni2c_send_noresponse(): TWI interrupt while idle ?");
#endif
    }
}
