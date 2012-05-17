;***********************************************************
;* Pulso Advance Plus DLU40A with all FETs on PORTD, same  *
;* Same pinout as rct50a.inc but inverted low-side FETs,   *
;* and inverted PWM input due to the opto-isolator         *
;* Pull-up on PWM input helps make input from opto square  *
;* Warning: Not sure of exact model, might be DLPU40A      *
;* Original fuses are lfuse:0x9f hfuse:0xc9                *
;***********************************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 2
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 0

;*********************
; PORT D definitions *
;*********************
.equ	ApFET		= 7	;o
.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	AnFET		= 5	;o
.equ	BnFET		= 4	;o
.equ	BpFET		= 3	;o
.equ	rcp_in		= 2	;i r/c pulse input, opto-coupled, needs pull-up
.equ	CpFET		= 1	;o
.equ	CnFET		= 0	;o

.equ	INIT_PD		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)+(1<<rcp_in)
.equ	DIR_PD		= (1<<ApFET)+(1<<BpFET)+(1<<CpFET)+(1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.equ	ApFET_port	= PORTD
.equ	BpFET_port	= PORTD
.equ	CpFET_port	= PORTD
.equ	AnFET_port	= PORTD
.equ	BnFET_port	= PORTD
.equ	CnFET_port	= PORTD

.MACRO rcp_int_enable
	ldi	@0, (1<<INT0)	; enable ext0int
	out	GICR, @0
.ENDMACRO
.MACRO rcp_int_disable
	out	GICR, ZH	; disable ext0int
.ENDMACRO
.MACRO rcp_int_rising_edge
	ldi	@0, (1<<ISC01)
	out	MCUCR, @0	; set next int0 to falling edge (inverted)
.ENDMACRO
.MACRO rcp_int_falling_edge
	ldi	@0, (1<<ISC01)+(1<<ISC00)
	out	MCUCR, @0	; set next int0 to rising edge (inverted)
.ENDMACRO

.MACRO ApFET_on
	sbi	PORTD, ApFET
.ENDMACRO
.MACRO ApFET_off
	cbi	PORTD, ApFET
.ENDMACRO
.MACRO BpFET_on
	sbi	PORTD, BpFET
.ENDMACRO
.MACRO BpFET_off
	cbi	PORTD, BpFET
.ENDMACRO
.MACRO CpFET_on
	sbi	PORTD, CpFET
.ENDMACRO
.MACRO CpFET_off
	cbi	PORTD, CpFET
.ENDMACRO
.MACRO AnFET_on
	cbi	PORTD, AnFET
.ENDMACRO
.MACRO AnFET_off
	sbi	PORTD, AnFET
.ENDMACRO
.MACRO BnFET_on
	cbi	PORTD, BnFET
.ENDMACRO
.MACRO BnFET_off
	sbi	PORTD, BnFET
.ENDMACRO
.MACRO CnFET_on
	cbi	PORTD, CnFET
.ENDMACRO
.MACRO CnFET_off
	sbi	PORTD, CnFET
.ENDMACRO

.MACRO ApFET_on_reg
	sbr	@0, 1<<ApFET
.ENDMACRO
.MACRO ApFET_off_reg
	cbr	@0, 1<<ApFET
.ENDMACRO
.MACRO BpFET_on_reg
	sbr	@0, 1<<BpFET
.ENDMACRO
.MACRO BpFET_off_reg
	cbr	@0, 1<<BpFET
.ENDMACRO
.MACRO CpFET_on_reg
	sbr	@0, 1<<CpFET
.ENDMACRO
.MACRO CpFET_off_reg
	cbr	@0, 1<<CpFET
.ENDMACRO
.MACRO AnFET_on_reg
	cbr	@0, 1<<AnFET
.ENDMACRO
.MACRO AnFET_off_reg
	sbr	@0, 1<<AnFET
.ENDMACRO
.MACRO BnFET_on_reg
	cbr	@0, 1<<BnFET
.ENDMACRO
.MACRO BnFET_off_reg
	sbr	@0, 1<<BnFET
.ENDMACRO
.MACRO CnFET_on_reg
	cbr	@0, 1<<CnFET
.ENDMACRO
.MACRO CnFET_off_reg
	sbr	@0, 1<<CnFET
.ENDMACRO

.MACRO nFET_brake
	in	@0, PORTD
	cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTD, @0
.ENDMACRO
.MACRO all_nFETs_off
	in	@0, PORTD
	sbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTD, @0
.ENDMACRO

.MACRO all_pFETs_off
	in	@0, PORTD
	cbr	@0, (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
	out	PORTD, @0
.ENDMACRO


;*********************
; PORT C definitions *
;*********************
.equ	mux_vbat	= 7	; ADC7 voltage input (47k from Vbat, 2.0k to gnd, 14.7V in -> .598V at ADC7)
;.equ			= 6	; ADC6
;.equ			= 5	; ADC5
.equ	mux_c		= 4	; ADC4 phase input
.equ	mux_b		= 3	; ADC3 phase input
.equ	mux_a		= 2	; ADC2 phase input
;.equ			= 1	; ADC1
;.equ			= 0	; ADC0

.equ	INIT_PC		= 0
.equ	DIR_PC		= 0

.MACRO comp_init
	in	@0, SFIOR
	sbr	@0, (1<<ACME)	; set Analog Comparator Multiplexer Enable
	out	SFIOR, @0
	cbi	ADCSRA, ADEN	; disable ADC
.ENDMACRO
.MACRO set_comp_phase_a
	ldi	@0, mux_a	; set comparator multiplexer to phase A
	out	ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_b
	ldi	@0, mux_b	; set comparator multiplexer to phase B
	out	ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_c
	ldi	@0, mux_c	; set comparator multiplexer to phase C
	out	ADMUX, @0
.ENDMACRO


;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck stk200 interface)
;.equ			= 4	(miso stk200 interface)
;.equ			= 3	(mosi stk200 interface)
;.equ			= 2
;.equ			= 1
;.equ			= 0

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
