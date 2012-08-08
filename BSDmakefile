# This Makefile is compatible with both BSD and GNU make

ASM = avra 

.SUFFIXES: .inc .hex

all: afro.hex afro2.hex birdie70a.hex bs_nfet.hex bs.hex bs40a.hex dlu40a.hex hk200a.hex kda.hex rb50a.hex rb70a.hex rct50a.hex tp.hex tp_i2c.hex tp_nfet.hex tgy6a.hex tgy.hex

.inc.hex:
	$(ASM) -o $@ -D $*_esc -e /dev/null -d $*.obj tgy.asm

clean:
	-rm -f *.hex
	-rm -f *.eeprom
	-rm -f *.obj
