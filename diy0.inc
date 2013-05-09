;***********************************************************
;* HobbyKing Open ESC (unreleased rev 0)                   *
;* 2012-09-29                                              *
;***********************************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 0
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 1

;*********************
; PORT D definitions *
;*********************
.equ	red_led		= 7
.equ	blue_led	= 5
.equ	ApFET		= 4
.equ	io1		= 3
.equ	io2		= 2
.equ	txd		= 1
.equ	rxd		= 0

.equ	INIT_PD		= (1<<ApFET)
.equ	DIR_PD		= (1<<ApFET)|(1<<io1)|(1<<io2)|(1<<red_led)

.equ	ApFET_port	= PORTD

.MACRO RED_on
	sbi	DDRD, red_led
.ENDMACRO
.MACRO RED_off
	cbi	DDRD, red_led
.ENDMACRO
.MACRO GRN_on
	sbi	DDRD, blue_led
.ENDMACRO
.MACRO GRN_off
	cbi	DDRD, blue_led
.ENDMACRO

;*********************
; PORT C definitions *
;*********************
.equ	mux_b		= 7	; ADC7 phase input
.equ	mux_a		= 6	; ADC6 phase input
.equ	BpFET		= 5	;o (ADC5/SCL)
.equ	mux_current	= 4	; ADC4/SDA current input
.equ	CpFET		= 3	;o
.equ	mux_voltage	= 2	; ADC2 voltage input
.equ	mux_temperature	= 1	; ADC1 temperature input
.equ	mux_c		= 0	; ADC0 phase input

.equ	O_POWER		= 220
.equ	O_GROUND	= 51

.equ	INIT_PC		= (1<<BpFET)|(1<<CpFET)
.equ	DIR_PC		= (1<<BpFET)|(1<<CpFET)

.equ	BpFET_port	= PORTC
.equ	CpFET_port	= PORTC

;*********************
; PORT B definitions *
;*********************
;.equ			= 5	(sck)
;.equ			= 4	(miso)
.equ	AnFET		= 3	;o (mosi)
.equ	BnFET		= 2	;o
.equ	CnFET		= 1	;o
.equ	rcp_in		= 0	;i r/c pulse input

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<AnFET)|(1<<BnFET)|(1<<CnFET)

.equ	AnFET_port	= PORTB
.equ	BnFET_port	= PORTB
.equ	CnFET_port	= PORTB
