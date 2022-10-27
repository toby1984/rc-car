#include "si4432.h"
#include "spi.h"
#include <stdint.h>
#include "avr/io.h"

#define INT_DDR DDRD
#define INT_REG PIND
// PD2
#define INT_PIN (1<<2)

uint8_t si4432_get_device_type() {
  return spi_read_register(0);
}

uint8_t si4432_get_device_version() {
  return spi_read_register(1);
}

uint8_t si4432_get_errors() {
    return spi_read_register(2) & 0b11111000;
}

uint8_t si4432_get_device_state() {
    return spi_read_register(2) & 0b111;
}

inline uint8_t si4432_int_active() {
        return (INT_REG & INT_PIN) != 0;
}

void si4432_sw_reset()
{
    // SW reset
    spi_write_register(0x07, 0x80); // write to Operating & Function Control1 register

    // wait for radio to release interrupt pin
    while ( si4432_int_active() );
}

void si4432_init_transmit()
{
    si4432_sw_reset();

    // TODO: Check device revision as SI4432 Revision V2 needs some special treatment

    // read interrupt status registers to clear the interrupt flags and release NIRQ pin
    spi_read_register(0x03); // read Interrupt Status1 register
    spi_read_register(0x04); // read Interrupt Status2 register

    // *** set the physical parameters ***

    // set the center frequency to 915 MHz
    spi_write_register(0x75, 0x75); // write Frequency Band Select register
    spi_write_register(0x76, 0xBB); // write Nominal Carrier Frequency1 register
    spi_write_register(0x77, 0x80); // write Nominal Carrier Frequency0 register

    // set the desired TX data rate (9.6kbps)
    spi_write_register(0x6E, 0x4E); // write TXDataRate 1 register
    spi_write_register(0x6F, 0xA5); // write TXDataRate 0 register
    spi_write_register(0x70, 0x2C); // write Modulation Mode Control 1 register

    /* Set the desired TX deviation (+-45 kHz)

    NOTE: For OOK modulation, there is no need to configure the transmit deviation.

    Besides the TX Data Rate 0 and TX Data Rate 1 registers, an additional bit is used to define the data rate: “txdtrtscale”
    (bit5 in the Modulation Mode Control1 register). The “txdtrtscale” bit must be set to 1 if the desired data rate is below
    30 kbps.
    The MSB of the deviation setting can be found in the Modulation Mode Control register 2 (fd[8] – bit2).
    */
    spi_write_register(0x72, 0x48);

    // *** set the packet structure and the modulation type ***
    // set the preamble length to 10 bytes if the antenna diversity is used and set to 5bytes if not
    spi_write_register(0x34, 0x0a); // register contains preamble length in NIBBLES, so 0x0a * 0x04 => 40 bits = 5 bytes

    /* Disable the header bytes (not used in this example) and set the synchron word length to two bytes.
    Note: The MSB bit of the preamble length setting is located in this register (“prealen8”—bit0 in the Header Control2 register).
    // Disable header bytes; set variable packet length (the length of the payload is defined by the received packet length field of the packet); set the synch word to two bytes long
    */
    spi_write_register(0x33, 0x02);

    /* Set the synchron word pattern for 0x2DD4. At least two bytes of synchron word are recommended to increase the
    robustness of the communication. The synchron word pattern is a system-level design consideration. For
    interoperability with EZRadio devices, the synchron word must be set to 0x2DD4. The synchron word can also be
    used as a packet filter by using different values for different applications or node types. Different synchron word
    vales can be used by the receiver to filter and receive only communication from the desired node.
    */
    //Set the sync word pattern to 0x2DD4
    spi_write_register(0x36, 0x2D);
    spi_write_register(0x37, 0xD4);

    // Enable the TX packet handler and the CRC16 calculation:
    spi_write_register(0x30, 0x0D);

    // In the current example, the FIFO is the source of the modulation and the TX packet handler is used to form and
    // transmit the packet, the Modulation Mode Control2 register is set accordingly: enable FIFO mode and GFSK modulation.
    spi_write_register(0x71, 0x63); // write to Modulation Mode Control 2 register

    /*
3.1.5. Registers Configuration for Si4432 Revision V2

The Si4432 Revision V2 requires some registers to be programmed to values other than their default values. Note: These settings are not required for subsequent versions of the radio

//set VCO and PLL
spi_write_register(0x5A, 0x7F);  // write VCO Current Trimming register
spi_write_register(0x59, 0x40);  // write Divider Current Trimming register

     */

    /*
3.1.6. Registers Configuration for Si4431 Revision A0

The Si4431 revision A0 requires some registers to be programmed to values other than their default value. The
PLL and VCO must be programmed to the following values to get optimal current consumption.

//set VCO and PLL
spi_write_register(0x57, 0x01); // write Chargepump Test register
spi_write_register(0x59, 0x00); // write Divider Current Trimming register
spi_write_register(0x5A, 0x01); // write VCO Current Trimming register
     */
}

void si4432_send(char *data, uint8_t len) {

    // set the length of the payload to 8bytes
    spi_write_register(0x3E, len);

    // fill the payload into the transmit FIFO
    for ( ; len > 0 ; len-- ) {
        spi_write_register(0x7F, *data++);
    }

    // Disable all other interrupts and enable the packet sent interrupt only.
    // This will be used for indicating the successful packet transmission for the MCU
    spi_write_register(0x05, 0x04); // write Interrupt Enable 1 register
    spi_write_register(0x06, 0x00); // write Interrupt Enable 2 register

    // read interrupt status registers. It clear all pending interrupts and the nIRQ pin goes back to high.
    spi_read_register(0x03);
    spi_read_register(0x04);

    // enable transmitter. the radio forms the packet and send it automatically.
    spi_write_register(0x07, 0x09);

    // write 0x09 to the Operating Function Control 1 register
    // wait for the packet sent interrupt
    // The MCU just needs to wait for the 'ipksent' interrupt.
    while( ! si4432_int_active() );

    // read interrupt status registers to release the interrupt flags
    spi_read_register(0x03); // read Interrupt Status1 register
    spi_read_register(0x04); // read Interrupt Status2 register
}

void si4432_init_receive()
{
    si4432_sw_reset();

    // read interrupt status registers to clear the interrupt flags and release NIRQ pin
    spi_read_register(0x03); // read Interrupt Status1 register
    spi_read_register(0x04); // read Interrupt Status2 register

    // *** set the physical parameters ***

    // set the center frequency to 915 MHz
    spi_write_register(0x75, 0x75); //write 0x75 to the Frequency Band Select register
    spi_write_register(0x76, 0xBB); //write 0xBB to the Nominal Carrier Frequency1 register
    spi_write_register(0x77, 0x80); //write 0x80 to the Nominal Carrier Frequency0 register

    // Configure SI4432 modem parameters for receiving the desired GFSK modulated data.

    // sset the modem parameters according to the excel calculator (parameters: 9.6 kbps, deviation: 45 kHz, channel filter BW: 112.1 kHz*

    // TODO: Make sure we're talking to a Si4432 that is NOT Si4432 revV2 as those need different initialization.

    spi_write_register(0x1C, 0x05); // write IF Filter Bandwidth register
    spi_write_register(0x20, 0xA1); // write Clock Recovery Oversampling Ratio register
    spi_write_register(0x21, 0x20); // write Clock Recovery Offset 2 register
    spi_write_register(0x22, 0x4E); // write Clock Recovery Offset 1 register
    spi_write_register(0x23, 0xA5); // write Clock Recovery Offset 0 register
    spi_write_register(0x24, 0x00); // write Clock Recovery Timing Loop Gain 1 register
    spi_write_register(0x25, 0x13); // write Clock Recovery Timing Loop Gain 0 register
    spi_write_register(0x1D, 0x40); // write AFC Loop Gearshift Override register

/*
 - the Si4432 revV2 chip uses the Frequency Deviation register to define the maximum allowable frequency offset for
Auto-Frequency Calibration. The value of this register must be set as an AFC limit before going to receive mode; Change the value of this register according to frequency deviation before going to transmit mode.
- the Si4431 revision A0 and Si443x revision B1 chips have a separate register for this purpose: AFC Limiter.
*/
    spi_write_register(0x72, 0x1F); // write Frequency Deviation register

    // configures the receive packet handler for the proper packet configuration
    // Disable header bytes; set variable packet length (the length of the payload is defined by the received packet length field of the packet);
    spi_write_register(0x33, 0x02 ); // set the synch word to two bytes long

    // Disable the receive header filters
    spi_write_register(0x32, 0x00 ); //write Header Control1 register

    // set the sync word pattern to 0x2DD4
    spi_write_register(0x36, 0x2D);
    spi_write_register(0x37, 0xD4);
    // Enable the receive packet handler and CRC-16 (IBM) check
    spi_write_register(0x30, 0x85); // write Data Access Control register
    // Enable FIFO mode and GFSK modulation
    spi_write_register(0x71, 0x63); // write Modulation Mode Control 2 register
    // set preamble detection threshold to 20 bits  (5 nibbles, bits 7-3 store the value so actually need to write 5<<3 = 0x28)
    spi_write_register(0x35, 0x28); // write Preamble Detection Control register
}

uint8_t si4432_receive(char *buffer, uint8_t bufferLen)
{
    // enable receiver chain
    spi_write_register(0x07, 0x05); // write Operating Function Control 1 register
    // Enable two interrupts:
    // a) one which shows that a valid packet received: 'ipkval'
    // b) second shows if the packet received with incorrect CRC: 'icrcerror'
    spi_write_register(0x05, 0x03); // write Interrupt Enable 1 register
    spi_write_register(0x06, 0x00); // write Interrupt Enable 2 register

    // read interrupt status registers to release all pending interrupts
    spi_read_register(0x03);//read the Interrupt Status1 register
    spi_read_register(0x04);//read the Interrupt Status2 register

    while(1) // TODO: Add timeout ? Or, even better: Move all the code to an IRQ handler ?
    {
        // wait for the interrupt event
        if ( si4432_int_active() )
        {
            // read interrupt status registers
            uint8_t ItStatus1 = spi_read_register(0x03);// read Interrupt Status1 register
            spi_read_register(0x04);// read Interrupt Status2 register
            if( (ItStatus1 & 0x01) == 0x01 )
            {
                // CRC Error interrupt occured
                //disable the receiver chain
                spi_write_register(0x07, 0x01); // write Operating Function Control 1 register
                //reset the RX FIFO
                spi_write_register(0x08, 0x02);// write Operating Function Control 2 register
                spi_write_register(0x08, 0x00);// write Operating Function Control 2 register
                // enable the receiver chain again
                spi_write_register(0x07, 0x05);//write Operating Function Control 1 register
                continue;
            }

            /*packet received interrupt occurred*/
            if ( (ItStatus1 & 0x02) == 0x02 )
            {
                //disable the receiver chain
                spi_write_register(0x07, 0x01);//write 0x01 to the Operating Function Control 1 register
                //Read the length of the received payload
                uint8_t length = spi_read_register(0x4B); // read Received Packet Length register
                //check whether the received payload is not longer than the allocated buffer in the MCU
                uint8_t toCopy = length <= bufferLen ? length : bufferLen;

                for ( ; toCopy > 0 ; toCopy--) {
                    uint8_t value = spi_read_register(0x7F);// read the FIFO Access register
                    *buffer++ = value;
                }
                // reset the RX FIFO
                spi_write_register(0x08, 0x02); // write Operating Function Control 2 register
                spi_write_register(0x08, 0x00); // write Operating Function Control 2 register
                // enable the receiver chain again
                spi_write_register(0x07, 0x05);//write Operating Function Control 1 register
                return toCopy;
            }
        }
    }
}
