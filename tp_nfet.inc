;***********************************************************
;* TowerPro 25A 2009+ "type 3" boards with inverted nFETs  *
;* Phase A and B sense on ADC5, ADC4; based on tp-18a as   *
;* modified by Eduardo Reginato Lavratti (agressiva)       *
;***********************************************************

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

.MACRO all_pFETs_off
	in	@0, PORTD
	cbr	@0, (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
	out	PORTD, @0
.ENDMACRO


;*********************
; PORT C definitions *
;*********************
;.equ			= 7	; ADC7
;.equ			= 6	; ADC6
.equ	mux_a		= 5	; ADC5 phase input
.equ	mux_b		= 4	; ADC4 phase input
;.equ			= 3	; ADC3
;.equ			= 2	; ADC2 (plush has something on here)
;.equ	temp_adc	= 1	; ADC1 temperature control input
;.equ	accu_adc	= 0	; ADC0 voltage control input

.equ	INIT_PC		= 0
.equ	DIR_PC		= 0

.MACRO comp_init
	in	@0, SFIOR
	sbr	@0, (1<<ACME)	; set Analog Comparator Multiplexer Enable
	out	SFIOR, @0
.ENDMACRO
.MACRO set_comp_phase_a
	ldi	@0, mux_a	; set comparator multiplexer to phase A
	out	ADMUX, @0
	cbi	ADCSRA, ADEN	; disable ADC
.ENDMACRO
.MACRO set_comp_phase_b
	ldi	@0, mux_b	; set comparator multiplexer to phase B
	out	ADMUX, @0
	cbi	ADCSRA, ADEN	; disable ADC
.ENDMACRO
.MACRO set_comp_phase_c	
	sbi	ADCSRA, ADEN	; enable ADC and comparator to phase C (AIN1)
.ENDMACRO


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

.equ	INIT_PB		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
.equ	DIR_PB		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.equ	AnFET_port	= PORTB
.equ	BnFET_port	= PORTB
.equ	CnFET_port	= PORTB

.MACRO AnFET_on
	cbi	PORTB, AnFET
.ENDMACRO
.MACRO AnFET_off
	sbi	PORTB, AnFET
.ENDMACRO
.MACRO BnFET_on
	cbi	PORTB, BnFET
.ENDMACRO
.MACRO BnFET_off
	sbi	PORTB, BnFET
.ENDMACRO
.MACRO CnFET_on
	cbi	PORTB, CnFET
.ENDMACRO
.MACRO CnFET_off
	sbi	PORTB, CnFET
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
	in	@0, PORTB
	cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTB, @0
.ENDMACRO
.MACRO all_nFETs_off
	in	@0, PORTB
	sbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTB, @0
.ENDMACRO

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO
