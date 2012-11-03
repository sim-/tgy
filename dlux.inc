;********************************************
;* Turnigy Dlux ESC 20A                     *
;* Original fuses are lfuse:0xae hfuse:0xcf *
;* 2012-10-07                               *
;********************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 0

;*********************
; PORT D definitions *
;*********************
.equ	BnFET		= 7	;o
.equ	AnFET		= 5	;o
.equ	ApFET		= 4	;o
;.equ			= 3
.equ	rcp_in		= 2	;i r/c pulse input
;.equ			= 1
;.equ			= 0

.equ	INIT_PD		= (1<<ApFET)
.equ	DIR_PD		= (1<<BnFET)|(1<<AnFET)|(1<<ApFET)

.equ	BnFET_port	= PORTD
.equ	AnFET_port	= PORTD
.equ	ApFET_port	= PORTD

.MACRO rcp_int_enable
	ldi	@0, (1<<INT0)	; enable ext0int
	out	GICR, @0
.ENDMACRO
.MACRO rcp_int_disable
	out	GICR, ZH	; disable ext0int
.ENDMACRO
.MACRO rcp_int_rising_edge
	ldi	@0, (1<<ISC01)+(1<<ISC00)
	out	MCUCR, @0	; set next int0 to rising edge
.ENDMACRO
.MACRO rcp_int_falling_edge
	ldi	@0, (1<<ISC01)
	out	MCUCR, @0	; set next int0 to falling edge
.ENDMACRO

.MACRO BnFET_on
	sbi	BnFET_port, BnFET
.ENDMACRO
.MACRO BnFET_off
	cbi	BnFET_port, BnFET
.ENDMACRO
.MACRO AnFET_on
	sbi	AnFET_port, AnFET
.ENDMACRO
.MACRO AnFET_off
	cbi	AnFET_port, AnFET
.ENDMACRO

.MACRO BnFET_on_reg
	sbr	@0, 1<<BnFET
.ENDMACRO
.MACRO BnFET_off_reg
	cbr	@0, 1<<BnFET
.ENDMACRO
.MACRO AnFET_on_reg
	sbr	@0, 1<<AnFET
.ENDMACRO
.MACRO AnFET_off_reg
	cbr	@0, 1<<AnFET
.ENDMACRO

.MACRO ApFET_on
	cbi	ApFET_port, ApFET
.ENDMACRO
.MACRO ApFET_off
	sbi	ApFET_port, ApFET
.ENDMACRO

.MACRO ApFET_on_reg
	cbr	@0, 1<<ApFET
.ENDMACRO
.MACRO ApFET_off_reg
	sbr	@0, 1<<ApFET
.ENDMACRO

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO

;*********************
; PORT C definitions *
;*********************
.equ	mux_b		= 7	; ADC7 phase input
.equ	mux_a		= 6	; ADC6 phase input
.equ	BpFET		= 5	;o
;.equ			= 4	; ADC4
.equ	CpFET		= 3	;o
;.equ			= 2	; ADC2
.equ	mux_vbat	= 1	; ADC1 voltage input (220k from Vbat, 51k to gnd, 10.10V in -> 1.888V at ADC1)
.equ	mux_c		= 0	; ADC0 phase input

.equ	INIT_PC		= (1<<CpFET)|(1<<BpFET)
.equ	DIR_PC		= (1<<CpFET)|(1<<BpFET)

.equ	CpFET_port	= PORTC
.equ	BpFET_port	= PORTC

.MACRO CpFET_on
	cbi	CpFET_port, CpFET
.ENDMACRO
.MACRO CpFET_off
	sbi	CpFET_port, CpFET
.ENDMACRO
.MACRO BpFET_on
	cbi	BpFET_port, BpFET
.ENDMACRO
.MACRO BpFET_off
	sbi	BpFET_port, BpFET
.ENDMACRO

.MACRO CpFET_on_reg
	cbr	@0, 1<<CpFET
.ENDMACRO
.MACRO CpFET_off_reg
	sbr	@0, 1<<CpFET
.ENDMACRO
.MACRO BpFET_on_reg
	cbr	@0, 1<<BpFET
.ENDMACRO
.MACRO BpFET_off_reg
	sbr	@0, 1<<BpFET
.ENDMACRO

.MACRO comp_init
	in	@0, SFIOR
	sbr	@0, (1<<ACME)		; switch to comparator multiplexed
	out	SFIOR, @0
	cbi	ADCSRA, ADEN		; disable ADC
.ENDMACRO
.MACRO set_comp_phase_a
	ldi	@0, mux_a		; set comparator multiplexer to phase A
	out	ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_b
	ldi	@0, mux_b		; set comparator multiplexer to phase B
	out	ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_c
	ldi	@0, mux_c		; set comparator multiplexer to phase C
	out	ADMUX, @0
.ENDMACRO

;*********************
; PORT B definitions *
;*********************
;.equ			= 5	(sck)
;.equ			= 4	(miso)
;.equ			= 3	(mosi)
;.equ			= 2
;.equ			= 1
.equ	CnFET		= 0	;o

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<CnFET)

.equ	CnFET_port	= PORTB

.MACRO CnFET_on
	sbi	CnFET_port, CnFET
.ENDMACRO
.MACRO CnFET_off
	cbi	CnFET_port, CnFET
.ENDMACRO

.MACRO CnFET_on_reg
	sbr	@0, 1<<CnFET
.ENDMACRO
.MACRO CnFET_off_reg
	cbr	@0, 1<<CnFET
.ENDMACRO

.MACRO nFET_brake
	AnFET_on
	BnFET_on
	CnFET_on
.ENDMACRO
.MACRO all_nFETs_off
	AnFET_off
	BnFET_off
	CnFET_off
.ENDMACRO
.MACRO all_pFETs_off
	ApFET_off
	BpFET_off
	CpFET_off
.ENDMACRO
