;***********************************************
;* DYS all N-channel ESC, a mash-up of tgy and *
;* Mystery pin configurations                  *
;* 2013-07-16                                  *
;* Fuses should be lfuse=0x3f hfuse=0xca       *
;***********************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 0
.equ	USE_I2C		= 1
.equ	USE_UART	= 1
.equ	USE_ICP		= 1

.equ	DEAD_LOW_NS	= 300
.equ	DEAD_HIGH_NS	= 300
.equ	MOTOR_ADVANCE	= 15
.equ	CHECK_HARDWARE	= 0

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck)
;.equ			= 4	(miso)
;.equ			= 3	(mosi)
;.equ			= 2
.equ	CnFET		= 1	;o
.equ	rcp_in		= 0	;i r/c pulse input

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<CnFET)

.equ	CnFET_port	= PORTB

;*********************
; PORT C definitions *
;*********************
.equ	mux_b		= 7	; ADC7
.equ	mux_a		= 6	; ADC6
.equ	i2c_clk		= 5	; ADC5/SCL
.equ	i2c_data	= 4	; ADC4/SDA
;.equ			= 3
.equ	mux_voltage	= 2	; ADC2 voltage input (220k from Vbat, 47k to gnd)
.equ	mux_temperature = 1	; ADC1 temperature input (10k to 5V, NTC and 68k to gnd)
.equ	mux_c		= 0	; ADC0

.equ	O_POWER		= 220
.equ	O_GROUND	= 47

.equ	INIT_PC		= (1<<i2c_clk)+(1<<i2c_data)
.equ	DIR_PC		= 0

;*********************
; PORT D definitions *
;*********************
.equ	BnFET		= 7
;.equ	sense_star	= 6 (comparator AN0)
.equ	AnFET		= 5
.equ	ApFET		= 4
.equ	BpFET		= 3
.equ	CpFET		= 2
.equ	txd		= 1
.equ	rxd		= 0

.equ	INIT_PD		= (1<<ApFET)+(1<<BpFET)+(1<<CpFET)+(1<<txd)
.equ	DIR_PD		= (1<<AnFET)+(1<<BnFET)+(1<<ApFET)+(1<<BpFET)+(1<<CpFET)+(1<<txd)

.equ	BnFET_port	= PORTD
.equ	AnFET_port	= PORTD
.equ	ApFET_port	= PORTD
.equ	BpFET_port	= PORTD
.equ	CpFET_port	= PORTD

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO
