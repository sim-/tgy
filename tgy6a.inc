;************************************************************************
;* Turnigy Plush 6A boards                                              *
;* Similar to normal "type 2" boards except with all sense lines on ADC *
;* pins (2, 4, 5) and possibly a different FET orderding. It seems the  *
;* stock firmware actually commutates in C, B, A order, so we flip B    *
;* and C everywhere here to spin in the same direction. This makes the  *
;* pin configuration match tgy.inc except for the C (actually "B")      *
;* phase sense on PC2 / ADC2 instead of PD7 / AIN1.                     *
;* Based on 6a ppm-r08 from Bernhard Konze:                             *
;* http://home.versanet.de/~b-konze/blc_6a/blc_6a.htm                   *
;************************************************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 0

;*********************
; PORT D definitions *
;*********************
;.equ	mux_c		= 7	;i <plus> comparator input (AIN0)
;.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	ApFET		= 5	;o
.equ	BpFET		= 4	;o
.equ	CpFET		= 3	;o
.equ	rcp_in		= 2	;i r/c pulse input

.equ	INIT_PD		= 0
.equ	DIR_PD		= (1<<ApFET)+(1<<BpFET)+(1<<CpFET)

.equ	ApFET_port	= PORTD
.equ	BpFET_port	= PORTD
.equ	CpFET_port	= PORTD

;*********************
; PORT C definitions *
;*********************
;.equ			= 7	; ADC7
;.equ			= 6	; ADC6
.equ	mux_a		= 5	; ADC5 phase input
.equ	mux_b		= 4	; ADC4 phase input
;.equ			= 3	; ADC3
.equ	mux_c		= 2	; ADC2 phase input
;.equ			= 1	; ADC1
.equ	mux_voltage	= 0	; ADC0 voltage input (4.7k from Vbat, 1k to gnd, 10.10V -> 1.772V at ADC0)

.equ	O_POWER		= 47
.equ	O_GROUND	= 10

.equ	INIT_PC		= 0
.equ	DIR_PC		= 0

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck stk200 interface)
;.equ			= 4	(miso stk200 interface)
;.equ			= 3	(mosi stk200 interface)
.equ	AnFET		= 2
.equ	BnFET		= 1
.equ	CnFET		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.equ	AnFET_port	= PORTB
.equ	BnFET_port	= PORTB
.equ	CnFET_port	= PORTB

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO
