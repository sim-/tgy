;***************************
;* Arctic Tiger ESC 30A    *
;* Project of Ben Neubauer *
;* 2013-07-09              *
;***************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 0
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 1

.equ	DEAD_LOW_NS	= 300
.equ	DEAD_HIGH_NS	= 300
.equ	MOTOR_ADVANCE	= 15
.equ	CHECK_HARDWARE	= 0

;*********************
; PORT D definitions *
;*********************
;.equ			= 7 (comparator AN1)
;.equ	sense_star	= 6 (comparator AN0)
.equ	CnFET 		= 5
;.equ			= 4
.equ	CpFET		= 3
;.equ			= 2
.equ	BpFET		= 1
.equ	AnFET		= 0

.equ	INIT_PD		= (1<<BpFET)+(1<<CpFET)
.equ	DIR_PD		= (1<<AnFET)+(1<<BpFET)+(1<<CnFET)+(1<<CpFET)

.equ	AnFET_port	= PORTD
.equ	BpFET_port	= PORTD
.equ	CnFET_port	= PORTD
.equ	CpFET_port	= PORTD

;*********************
; PORT C definitions *
;*********************

.equ	mux_c		= 7	; ADC7 phase input
.equ	mux_voltage	= 6	; ADC6 connected with 0ohm to ground
.equ	BnFET		= 5
.equ	ApFET		= 4
;.equ			= 3
;.equ			= 2
.equ	mux_a		= 1	; ADC1 phase input
.equ	mux_b		= 0	; ADC0 phase input

;.equ	O_POWER		= 220
;.equ	O_GROUND	= 51

.equ	INIT_PC		= (1<<ApFET)
.equ	DIR_PC		= (1<<BnFET)+(1<<ApFET)

.equ	BnFET_port	= PORTC
.equ	ApFET_port	= PORTC

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5 (sck stk200 interface)
;.equ			= 4 (miso stk200 interface)
;.equ			= 3 (mosi stk200 interface)
;.equ			= 2
;.equ			= 1
.equ	rcp_in		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= 0

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO
