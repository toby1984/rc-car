INCLUDE_DIRS = -I/home/tobi/apps/arduino_ide/./hardware/tools/avr/avr/include -I../includes

# AVRCC_CHIP=atmega88
# AVRDUDE_CHIP = m88p
# CPU_FREQ=8000000

AVRCC_CHIP=atmega328p
AVRDUDE_CHIP = m328p
CPU_FREQ=16000000

AVRDUDE_DEVICE = /dev/ttyACM0

OBJECTS = test_receiver.o

COMPILE = avr-gcc

all: test_receiver.hex

.c.o:
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -Wall -Os -Wno-main $(INCLUDE_DIRS) -c $< -o $@

test_receiver.elf: $(OBJECTS)
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -ffunction-sections -Wl,-gc -Wall -Os $(INCLUDE_DIRS) -o test_receiver.elf $(OBJECTS) ../common/uart.o ../common/radio_receiver.o ../common/radio_common.o ../common/timer16.o

test_receiver.hex: test_receiver.elf
	rm -f test_receiver.hex
	avr-objcopy -j .text -j .data -O ihex test_receiver.elf test_receiver.hex

clean:
	rm -f test_receiver.hex test_receiver.elf $(OBJECTS)

upload: test_receiver.hex
	/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U flash:w:test_receiver.hex
