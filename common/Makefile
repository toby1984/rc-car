INCLUDE_DIRS = -I/home/tobi/apps/arduino_ide/hardware/tools/avr/avr/include -I../includes -I/home/tobi/apps/arduino_ide//hardware/tools/avr/avr/include/util
LIB_DIR = 

# OBJECTS = crc.o radio_common.o uart.o radio_sender.o timer16.o radio_receiver.o i2c.o watchdog.o
OBJECTS = crc.o radio_common.o uart.o radio_sender.o timer16.o radio_receiver.o watchdog.o

COMPILE = avr-gcc

all: main

.c.o:
	$(COMPILE) -DF_CPU=$(CPU_FREQ) -mmcu=$(AVRCC_CHIP) -Wall -Os -Wno-main $(INCLUDE_DIRS) -c $< -o $@

main: $(OBJECTS)

clean:
	rm -f $(OBJECTS)
