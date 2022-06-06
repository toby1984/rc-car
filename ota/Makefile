INCLUDE_DIRS = -I/home/tobi/apps/arduino_ide/./hardware/tools/avr/avr/include -I../includes
LIB_DIR = 

# See Application Note 3341 "Basic Bootloader for the AVR® MCU DA (AVR-DA) Family"
# 
# To change the application's starting address, use the following LINKER options:
#
# -Wl,--section-start=.text=0x200

OBJECTS = ota.o

COMPILE = avr-gcc

all: ota.hex

.c.o:
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -Wall -Os -Wno-main $(INCLUDE_DIRS) -c $< -o $@

ota.elf: $(OBJECTS)
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -ffunction-sections -Wl,--verbose,-gc -Wall -Os $(INCLUDE_DIRS) -o ota.elf $(OBJECTS) ../common/i2c.o ../common/uart.o

ota.hex: ota.elf
	rm -f ota.hex
	avr-objcopy -j .text -j .data -O ihex ota.elf ota.hex

clean:
	rm -f ota.hex ota.elf $(OBJECTS)

upload: ota.hex
	/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U flash:w:ota.hex

readfuses:
	/usr/bin/avrdude -F -V -c stk500v2 -P $(AVRDUDE_DEVICE) -b 115200 -p $(AVRDUDE_CHIP) -U lfuse:r:-:b
