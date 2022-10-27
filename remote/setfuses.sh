#!/bin/bash

AVRDUDE_CHIP=m328p
AVRDUDE_DEVICE=/dev/ttyACM0
/usr/bin/avrdude -F -V -c stk500v2 -p $(AVRDUDE_CHIP) -P $(AVRDUDE_DEVICE) -b 115200 -U lfuse:w:0xe2:m -U hfuse:w:0xda:m -U efuse:w:0xfd:m
