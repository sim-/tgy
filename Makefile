all: build

build: tgy.hex

tgy.hex: tgy.asm $(wildcard *.inc)
	rm -f $@
	avra $<

program: tgy.hex
	avrdude -c dragon_isp -p m8 -P usb -U flash:w:tgy.hex:i

read:
	avrdude -c dragon_isp -p m8 -P usb -U flash:r:orig.hex:i

readeeprom:
	avrdude -c dragon_isp -p m8 -P usb -U eeprom:r:orig.hex:i

program_dapa: tgy.hex
	uisp -dprog=dapa --erase --upload --verify -v if=tgy.hex
