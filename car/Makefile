INCLUDE_DIRS = -I/home/tobi/apps/arduino_ide/./hardware/tools/avr/avr/include -I../includes
LIB_DIR = 

# AVRCC_CHIP=atmega88
# AVRDUDE_CHIP = m88p
# CPU_FREQ=8000000

AVRCC_CHIP=atmega328p
AVRDUDE_CHIP = m328p
CPU_FREQ=16000000

AVRDUDE_DEVICE = /dev/ttyACM0

OBJECTS = car.o

COMPILE = avr-gcc

all: car.hex

.c.o:
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -Wall -Os -Wno-main $(INCLUDE_DIRS) -c $< -o $@

car.elf: $(OBJECTS)
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -ffunction-sections -Wl,-gc -Wall -Os $(INCLUDE_DIRS) -o car.elf $(OBJECTS) ../common/uart.o ../common/timer16.o ../common/radio_receiver.o ../common/crc.o

car.hex: car.elf
	rm -f car.hex
	avr-objcopy -j .text -j .data -O ihex car.elf car.hex

clean:
	rm -f car.hex car.elf $(OBJECTS)

upload: car.hex
	/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U flash:w:car.hex

readfuses:
	/usr/bin/avrdude -F -V -c stk500v2 -P $(AVRDUDE_DEVICE) -b 115200 -p $(AVRDUDE_CHIP) -U lfuse:r:-:b
