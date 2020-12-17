INCLUDE_DIR = /home/tobi/apps/arduino_ide/./hardware/tools/avr/avr/include
LIB_DIR = 

# AVRCC_CHIP=atmega88
# AVRDUDE_CHIP = m88p
# CPU_FREQ=8000000

AVRCC_CHIP=atmega328p
AVRDUDE_CHIP = m328p
CPU_FREQ=16000000

AVRDUDE_DEVICE = /dev/ttyACM1

OBJECTS = tvremote.o uart.o joystick.o crc.o radio_common.o radio_sender.o radio_receiver.o timer16.o

COMPILE = avr-gcc

all: main.hex

.c.o:
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -Wall -Os -Wno-main -I $(INCLUDE_DIR) -c $< -o $@

main.elf: $(OBJECTS)
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -ffunction-sections -Wl,-gc -Wall -Os -Wno-main -I $(INCLUDE_DIR) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

clean:
	rm -f main.hex main.elf $(OBJECTS)

upload: main.hex
	/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U flash:w:main.hex

readfuses:
	/usr/bin/avrdude -F -V -c stk500v2 -P $(AVRDUDE_DEVICE) -b 115200 -p $(AVRDUDE_CHIP) -U lfuse:r:-:b
