build: tgy.hex

%.hex: tgy.asm $(wildcard *.inc)
	avra --define $(patsubst %.hex,%_esc,$@) $<
	$(if $(patsubst tgy.hex,,$@),mv -f tgy.hex $@)

ALL_TARGETS = afro.hex afro2.hex birdie70a.hex bs_nfet.hex bs.hex bs40a.hex bs50.hex dlu40a.hex hk200a.hex kda.hex rb50a.hex rb70a.hex rct50a.hex tp.hex tp_i2c.hex tp_nfet.hex tgy6a.hex tgy.hex

all: $(ALL_TARGETS)
all_targets: all

test: all

clean:
	rm -f $(ALL_TARGETS)

binary_zip: $(ALL_TARGETS)
	TARGET="tgy_`date '+%Y-%m-%d'`_`git rev-parse --verify --short HEAD`.zip"; \
	git archive -9 -o "$$TARGET" HEAD && \
	zip -9 "$$TARGET" $(ALL_TARGETS) && ls -l "$$TARGET"

program_tgy_%: %.hex
	avrdude -c stk500v2 -b 9600 -P /dev/ttyUSB0 -u -p m8 -D -U flash:w:$<:i

program_usbasp_%: %.hex
	avrdude -c usbasp -u -p m8 -U flash:w:$<:i

program_dragon_%: %.hex
	avrdude -c dragon_isp -p m8 -P usb -U flash:w:$<:i

program_dapa_%: %.hex
	avrdude -c dapa -u -p m8 -U flash:w:$<:i

program_uisp_%: %.hex
	uisp -dprog=dapa --erase --upload --verify -v if=$<

program_mkii_%: %.hex
	avrdude -c avrispmkII -P usb -u -p m8 -U flash:w:$<:i

bootload_usbasp:
	avrdude -c usbasp -u -p m8 -U hfuse:w:$(shell avrdude -c usbasp -u -p m8 -U hfuse:r:-:h | sed -n '/^0x/{s/.$$/a/;p}'):m

read: read_tgy

read_tgy:
	avrdude -c stk500v2 -b 9600 -P /dev/ttyUSB0 -u -p m8 -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_usbasp:
	avrdude -c usbasp -u -p m8 -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_dragon:
	avrdude -c dragon_isp -p m8 -P usb -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_dapa:
	avrdude -c dapa -p m8 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

read_uisp:
	uisp -dprog=dapa --download -v of=flash.hex

read_mkii:
	avrdude -c avrispmkII -P usb -u -p m8 -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i

