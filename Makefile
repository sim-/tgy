all: tgy.hex

%.hex: tgy.asm $(wildcard *.inc)
	avra --define $(patsubst %.hex,%_esc,$@) $<
	$(if $(patsubst tgy.hex,,$@),mv -f tgy.hex $@)

test: all_targets

all_targets: bs_nfet.hex bs.hex afro.hex tgy.hex

program_dragon_%: %.hex
	avrdude -c dragon_isp -p m8 -P usb -U flash:w:$<:i

program_dapa_%: %.hex
	avrdude -c dapa -p m8 -U flash:w:$<:i

program: program_dragon_tgy

program_dapa: program_dapa_tgy

read:
	avrdude -c dragon_isp -p m8 -P usb -U flash:r:flash.hex:i

readeeprom:
	avrdude -c dragon_isp -p m8 -P usb -U eeprom:r:eeprom.hex:i
