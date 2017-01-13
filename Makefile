# This Makefile is compatible with both BSD and GNU make

ASM?= avra
SHELL = /bin/bash

.SUFFIXES: .inc .hex

AUX_TARGETS = $(patsubst %.inc,%.hex,$(wildcard *_pr[0-9]*.inc diy*.inc))
CPU_INC = $(wildcard m*def.inc)
ALL_INC = $(sort $(wildcard *.inc))
FILTER_TARGETS = boot.inc $(AUX_TARGETS) $(CPU_INC) $(pathsubst %.hex,%.inc,$(AUX_TARGETS))
ALL_TARGETS = $(patsubst %.inc,%.hex,$(filter-out $(FILTER_TARGETS),$(ALL_INC)))

.PHONY: all report_all
all: report_all $(ALL_TARGETS)
report_all::
	@echo "Building: $(ALL_TARGETS)"

.PHONY: aux report_aux
aux: report_aux $(AUX_TARGETS)
report_aux::
	@echo "Building: $(AUX_TARGETS)"

$(ALL_TARGETS): tgy.asm boot.inc
$(AUX_TARGETS): tgy.asm boot.inc

.inc.hex:
	@if test ! -e $*.asm ; then (grep -q "^.include \"m.*def.inc\"" $*.inc || echo ".include \"m8def.inc\"" ; echo ".include \"$*.inc\"";echo ".include \"tgy.asm\"") > $*.asm ; touch $*.asm.created; fi
	@echo "$(ASM) -fI -o $@ -D $*_esc -e $*.eeprom -d $*.obj $*.asm"
	@set -o pipefail; $(ASM) -fI -o $@ -D $*_esc -e $*.eeprom -d $*.obj $*.asm 2>&1 | sed '/PRAGMA directives currently ignored/d'
	@test -e $*.asm.created && rm -f $*.asm $*.asm.created || true

.PHONY: test
test: all

.PHONY: clean
clean:
	-rm -f $(ALL_TARGETS) $(AUX_TARGETS) *.cof *.obj *.eep.hex *.eeprom

.PHONY: binary_zip
binary_zip: $(ALL_TARGETS)
	TARGET="tgy_`date '+%Y-%m-%d'`_`git rev-parse --verify --short HEAD`"; \
	mkdir "$$TARGET" && \
	cp $(ALL_TARGETS) "$$TARGET" && \
	git archive -9 --prefix="$$TARGET/" -o "$$TARGET".zip HEAD && \
	zip -9 "$$TARGET".zip "$$TARGET"/*.hex && ls -l "$$TARGET".zip; \
	rm -f "$$TARGET"/*.hex; \
	rmdir "$$TARGET"

program_tgy_%: %.hex
	avrdude -c stk500v2 -b 9600 -P /dev/ttyUSB0 -u -p m8 -U flash:w:$<:i

program_usbasp_%: %.hex
	avrdude -c usbasp -B.5 -p m8 -U flash:w:$<:i

program_avrisp2_%: %.hex
	avrdude -c avrisp2 -p m8 -U flash:w:$<:i

program_jtag3isp_%: %.hex
	avrdude -c jtag3isp -p m8 -U flash:w:$<:i

program_dragon_%: %.hex
	avrdude -c dragon_isp -p m8 -P usb -U flash:w:$<:i

program_dapa_%: %.hex
	avrdude -c dapa -p m8 -U flash:w:$<:i

program_uisp_%: %.hex
	uisp -dprog=dapa --erase --upload --verify -v if=$<

bootload_usbasp:
	avrdude -c usbasp -u -p m8 -U hfuse:w:`avrdude -c usbasp -u -p m8 -U hfuse:r:-:h | sed -n '/^0x/{s/.$$/a/;p}'`:m

read: read_tgy

read_tgy:
	avrdude -c stk500v2 -b 9600 -P /dev/ttyUSB0 -u -p m8 -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_usbasp:
	avrdude -c usbasp -u -p m8 -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_avrisp2:
	avrdude -c avrisp2 -p m8 -P usb -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_jtag3isp:
	avrdude -c jtag3isp -p m8 -P usb -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_dragon:
	avrdude -c dragon_isp -p m8 -P usb -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_dapa:
	avrdude -c dapa -p m8 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_uisp:
	uisp -dprog=dapa --download -v of=flash.hex
