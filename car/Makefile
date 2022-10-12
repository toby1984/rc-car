INCLUDE_DIRS = -I/home/tobi/apps/arduino_ide/./hardware/tools/avr/avr/include -I../includes
LIB_DIR = 

# See Application Note 3341 "Basic Bootloader for the AVR® MCU DA (AVR-DA) Family"
# 
# To change the application's starting address, use the following LINKER options:
#
# -Wl,--section-start=.text=0x200

OBJECTS = car.o

ifndef AVRCC_CHIP
  AVRCC_CHIP=atmega2560
endif
ifndef AVRDUDE_CHIP
  AVRDUDE_CHIP=ATmega2560
endif
ifndef CPU_FREQ
  CPU_FREQ=16000000
endif
ifndef AVRDUDE_DEVICE
  AVRDUDE_DEVICE=/dev/ttyACM0
endif

COMPILE = avr-gcc

all: car.hex

.c.o:
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -Wall -Os -Wno-main $(INCLUDE_DIRS) -c $< -o $@

car.elf: $(OBJECTS)
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -ffunction-sections -Wl,-gc -Wall -Os $(INCLUDE_DIRS) -o car.elf $(OBJECTS) ../common/uart.o ../common/timer16.o ../common/radio_receiver.o ../common/crc.o ../common/watchdog.o

car.hex: car.elf
	rm -f car.hex
	avr-objcopy -j .text -j .data -O ihex car.elf car.hex

clean:
	rm -f car.hex car.elf $(OBJECTS)

upload: car.hex
	/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U flash:w:car.hex

readfuses:
	/usr/bin/avrdude -F -V -c stk500v2 -P $(AVRDUDE_DEVICE) -b 115200 -p $(AVRDUDE_CHIP) -U lfuse:r:-:b
