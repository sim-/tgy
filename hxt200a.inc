;*******************************************
;* HexTronik HXT200A                       *
;* 2015-01-21                              *
;* Based on prototype schematic from Josef *
;*******************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0
.equ	USE_UART	= 1
.equ	USE_ICP		= 0

.equ	DEAD_LOW_NS	= 1400	; Uncalibrated
.equ	DEAD_HIGH_NS	= 1700	; Uncalibrated
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
.equ	red_led		= 2
;.equ			= 1
.equ	AnFET		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<AnFET)

.equ	AnFET_port	= PORTB

.MACRO RED_on
	sbi	DDRC, red_led
.ENDMACRO
.MACRO RED_off
	cbi	DDRC, red_led
.ENDMACRO

;*********************
; PORT C definitions *
;*********************
.equ	mux_b		= 7	; ADC7 phase input
.equ	mux_c		= 6	; ADC6 phase input
.equ	BpFET		= 5	; ADC5/SCL
;.equ			= 4	; ADC4/SDA
.equ	ApFET		= 3
.equ	mux_voltage	= 2	; ADC2 voltage input (470k from Vbat, 49.9k to gnd, 10.10V -> .969V at ADC2)
.equ	mux_temperature	= 1	; ADC1 temperature input (1.5k from +5V, 10k NTC to gnd)
.equ	mux_a		= 0	; ADC0 phase input

.equ	O_POWER		= 470
.equ	O_GROUND	= 47

.equ	INIT_PC		= 0
.equ	DIR_PC		= (1<<BpFET)+(1<<ApFET)

.equ	ApFET_port	= PORTC
.equ	BpFET_port	= PORTC

;*********************
; PORT D definitions *
;*********************
.equ	BnFET		= 7
;.equ	sense_star	= 6 (comparator AN0)
.equ	CnFET		= 5
.equ	CpFET		= 4
;.equ	BpFET		= 3
.equ	rcp_in		= 2
.equ	txd		= 1
.equ	rxd		= 0

.equ	INIT_PD		= (1<<txd)
.equ	DIR_PD		= (1<<BnFET)+(1<<CnFET)+(1<<CpFET)+(1<<txd)

.equ	BnFET_port	= PORTD
.equ	CnFET_port	= PORTD
.equ	CpFET_port	= PORTD
