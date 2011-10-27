all: tgy.hex

%.hex: tgy.asm $(wildcard *.inc)
	avra --define $(patsubst %.hex,%_esc,$@) $<
	$(if $(patsubst tgy.hex,,$@),mv -f tgy.hex $@)

test: all_targets

all_targets: bs_nfet.hex bs.hex afro.hex tp.hex tp_nfet.hex tgy.hex

program_dragon_%: %.hex
	avrdude -c dragon_isp -p m8 -P usb -U flash:w:$<:i

program_dapa_%: %.hex
	avrdude -c dapa -p m8 -U flash:w:$<:i

program_uisp_%: %.hex
	uisp -dprog=dapa --erase --upload --verify -v if=$<

program: program_dragon_tgy

program_dapa: program_dapa_tgy

program_uisp: program_uisp_tgy

read:
	avrdude -c dragon_isp -p m8 -P usb -U flash:r:flash.hex:i

read_dapa:
	avrdude -c dapa -p m8 -U flash:r:flash.hex:i

read_uisp:
	uisp -dprog=dapa --download -v of=flash.hex

readeeprom:
	avrdude -c dragon_isp -p m8 -P usb -U eeprom:r:eeprom.hex:i

readeeprom_dapa:
	avrdude -c dapa -p m8 -U eeprom:r:eeprom.hex:i

readeeprom_uisp:
	uisp -dprog=dapa --download --segment=eeprom -v of=eeprom.hex

terminal_dapa:
	avrdude -c dapa -p m8 -t
