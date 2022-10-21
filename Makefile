# AVRCC_CHIP=atmega88
# AVRDUDE_CHIP = m88p
# CPU_FREQ=8000000

export AVRCC_CHIP=atmega2560
export AVRDUDE_CHIP=ATmega2560
export CPU_FREQ=16000000

export AVRDUDE_DEVICE = /dev/ttyUSB0

SUBDIRS := common car remote test_receiver

TOPTARGETS := all clean

$(TOPTARGETS): $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@ $(MAKECMDGOALS)

.PHONY: $(TOPTARGETS) $(SUBDIRS)

