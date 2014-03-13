;*****************************************
;* AfroESC HV                            *
;* 2013-04-25                            *
;* Fuses should be lfuse=0x3f hfuse=0xca *
;*****************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 0
.equ	USE_I2C		= 1
.equ	USE_UART	= 1
.equ	USE_ICP		= 1

.equ	DEAD_LOW_NS	= 300
.equ	DEAD_HIGH_NS	= 900
.equ	MOTOR_ADVANCE	= 15
.equ	CHECK_HARDWARE	= 1

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck)
;.equ			= 4	(miso)
;.equ			= 3	(mosi)
.equ	BpFET		= 2	;o
.equ	CpFET		= 1	;o
.equ	rcp_in		= 0	;i r/c pulse input

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<BpFET)+(1<<CpFET)

.equ	BpFET_port	= PORTB
.equ	CpFET_port	= PORTB

;*********************
; PORT C definitions *
;*********************
.equ	mux_voltage	= 7	; ADC7 voltage input (18k from Vbat, 3.3k to gnd, 10.10V -> 1.565V at ADC7)
.equ	mux_temperature	= 6	; ADC6 temperature input (3.3k from +5V, 10k NTC to gnd)
.equ	i2c_clk		= 5	; ADC5/SCL
.equ	i2c_data	= 4	; ADC4/SDA
.equ	red_led		= 3	; o
.equ	green_led	= 2	; o
.equ	mux_b		= 1	; ADC1 phase input
.equ	mux_a		= 0	; ADC0 phase input

.equ	O_POWER		= 470
.equ	O_GROUND	= 47

.equ	INIT_PC		= (1<<i2c_clk)+(1<<i2c_data)
.equ	DIR_PC		= 0

.MACRO RED_on
	sbi	DDRC, red_led
.ENDMACRO
.MACRO RED_off
	cbi	DDRC, red_led
.ENDMACRO
.MACRO GRN_on
	sbi	DDRC, green_led
.ENDMACRO
.MACRO GRN_off
	cbi	DDRC, green_led
.ENDMACRO

;*********************
; PORT D definitions *
;*********************
;.equ	mux_c		= 7 (comparator AN1)
;.equ	sense_star	= 6 (comparator AN0)
.equ	CnFET		= 5
.equ	BnFET		= 4
.equ	AnFET		= 3
.equ	ApFET		= 2
.equ	txd		= 1
.equ	rxd		= 0

.equ	INIT_PD		= (1<<txd)
.equ	DIR_PD		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)+(1<<ApFET)+(1<<txd)

.equ	AnFET_port	= PORTD
.equ	BnFET_port	= PORTD
.equ	CnFET_port	= PORTD
.equ	ApFET_port	= PORTD
