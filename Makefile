INCLUDE_DIR = /home/tobi/apps/arduino_ide/./hardware/tools/avr/avr/include
LIB_DIR = 

OBJECTS = tvremote.o uart.o

COMPILE = avr-gcc

all: main.hex

.c.o:
	$(COMPILE) -DF_CPU=8000000 -mmcu=atmega88  -Wall -Os -Wno-main -I $(INCLUDE_DIR) -c $< -o $@

main.elf: $(OBJECTS)
	$(COMPILE) -DF_CPU=8000000 -mmcu=atmega88  -ffunction-sections -Wl,-gc -Wall -Os -Wno-main -I $(INCLUDE_DIR) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

clean:
	rm -f main.hex main.elf $(OBJECTS)

upload: main.hex
	/usr/bin/avrdude -F -V -c stk500v2 -p m88p -P /dev/ttyACM0 -b 115200 -U flash:w:main.hex
