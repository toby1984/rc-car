INCLUDE_DIRS = -I/home/tobi/apps/arduino_ide/./hardware/tools/avr/avr/include -I../includes
LIB_DIR = 

AVRCC_CHIP=atmega328p
AVRDUDE_CHIP = m328p
CPU_FREQ=16000000

AVRDUDE_DEVICE = /dev/ttyACM1

OBJECTS = joystick.o remote.o crc.o radio_common.o radio_sender.o uart.o spi.o si4432.o

COMPILE = avr-gcc

all: remote.hex

.c.o:
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -Wall -Os -Wno-main $(INCLUDE_DIRS) -c $< -o $@

remote.elf: $(OBJECTS)
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -ffunction-sections -Wl,-gc -Wall -Os -Wno-main $(INCLUDE_DIRS) -o remote.elf $(OBJECTS) 

remote.hex: remote.elf
	rm -f remote.hex
	avr-objcopy -j .text -j .data -O ihex remote.elf remote.hex

clean:
	rm -f remote.hex remote.elf $(OBJECTS)

upload: remote.hex
	/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U flash:w:remote.hex

readfuses:
	/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U lfuse:r:-:b

# write atmega328p fuses to use 8 Mhz internal oscillator
writefuses:
	/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U lfuse:w:0xff:m -U hfuse:w:0xda:m -U efuse:w:0xfd:m
