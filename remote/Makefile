INCLUDE_DIRS = -I/home/tobi/apps/arduino_ide/./hardware/tools/avr/avr/include -I../includes
LIB_DIR = 

# AVRCC_CHIP=atmega88
# AVRDUDE_CHIP = m88p
# CPU_FREQ=8000000

AVRCC_CHIP=atmega328p
AVRDUDE_CHIP = m328p
CPU_FREQ=16000000

AVRDUDE_DEVICE = /dev/ttyACM0

OBJECTS = joystick.o remote.o

COMPILE = avr-gcc

all: remote.hex

.c.o:
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -Wall -Os -Wno-main $(INCLUDE_DIRS) -c $< -o $@

remote.elf: $(OBJECTS)
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -ffunction-sections -Wl,-gc -Wall -Os -Wno-main $(INCLUDE_DIRS) -o remote.elf $(OBJECTS) ../common/radio_common.o ../common/crc.o ../common/radio_sender.o ../common/uart.o ../common/i2c.o

remote.hex: remote.elf
	rm -f remote.hex
	avr-objcopy -j .text -j .data -O ihex remote.elf remote.hex

clean:
	rm -f remote.hex remote.elf $(OBJECTS)

upload: remote.hex
	/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U flash:w:remote.hex
