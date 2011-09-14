;***************************************************************
;* For HK Blue Series with n- and p-Channel FETs (old version) *
;* IMPORTANT: Set fuses to internal 8MHz clock		       *
;* 09/2011						       *
;* https://github.com/sim-/tgy				       *
;***************************************************************


;*********************
; PORT D definitions *
;*********************
;.equ			= 7
;.equ	c_comp		= 6
.equ	AnFET		= 5
.equ	ApFET		= 4
;.equ			= 3
.equ	rcp_in		= 2

.equ	INIT_PD		= 0
.equ	DIR_PD		= (1<<AnFET)+(1<<ApFET)
;.equ	BRAKE_PD	= (1<<AnFET) ; see macro nFET_brake

.equ	USE_ICP		= 0
.equ	T1ICP		= 0

.MACRO rjmp_ext_int0
	rjmp	ext_int0
.ENDMACRO
.MACRO rjmp_icp1_int
	nop
.ENDMACRO
.MACRO rcp_int_enable
	ldi @0, (1<<INT0) ; enable ext0int
	out	GICR, @0
.ENDMACRO
.MACRO rcp_int_rising_edge
	ldi	@0, (1<<ISC01)+(1<<ISC00)
	out	MCUCR, @0 ; set next int0 to rising edge
.ENDMACRO
.MACRO rcp_int_falling_edge
	ldi @0, (1<<ISC01)
	out MCUCR, @0 ; set next int0 to falling edge
.ENDMACRO

.MACRO AnFET_on
	sbi PORTD, AnFET
.ENDMACRO
.MACRO AnFET_off
	cbi PORTD, AnFET
.ENDMACRO
.MACRO ApFET_on
	sbi PORTD, ApFET
.ENDMACRO
.MACRO ApFET_off
	cbi PORTD, ApFET
.ENDMACRO


;*********************
; PORT C definitions *
;*********************
.equ	mux_b		= 7	; ADC7
.equ	mux_a		= 6	; ADC6
.equ	BpFET		= 5
.equ	BnFET		= 4
.equ	CpFET		= 3
;.equ	accu_adc	= 2	; ADC2 -> not used
;.equ	AnREF		= 1 ; ADC1 -> not used
.equ	mux_c		= 0 ; ADC0

.equ	INIT_PC		= 0
.equ	DIR_PC		= (1<<BnFET)+(1<<BpFET)+(1<<CpFET)
;.equ	BRAKE_PC	= (1<<BnFET) ; see macro nFET_brake

.MACRO BnFET_on
	sbi PORTC, BnFET
.ENDMACRO
.MACRO BnFET_off
	cbi PORTC, BnFET
.ENDMACRO
.MACRO BpFET_on
	sbi PORTC, BpFET
.ENDMACRO
.MACRO BpFET_off
	cbi PORTC, BpFET
.ENDMACRO
.MACRO CpFET_on
	sbi PORTC, CpFET
.ENDMACRO
.MACRO CpFET_off
	cbi PORTC, CpFET
.ENDMACRO

.MACRO comp_init
	in @0, SFIOR
	sbr @0, (1<<ACME) ; switch to comparator multiplexed
	out SFIOR, @0
	cbi ADCSRA, ADEN ; disable ADC
.ENDMACRO
.MACRO set_comp_phase_a
	ldi @0, mux_a ; set comparator multiplexer to phase A
	out ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_b
	ldi @0, mux_b ; set comparator multiplexer to phase B
	out ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_c
	ldi @0, mux_c ; set comparator multiplexer to phase C
	out ADMUX, @0
.ENDMACRO


;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5 (sck stk200 interface)
;.equ			= 4	(miso stk200 interface)
;.equ			= 3	(mosi stk200 interface)
;.equ			= 2
;.equ			= 1
.equ	CnFET		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<CnFET)
;.equ	BRAKE_PB	= (1<<CnFET) ; see macro nFET_brake

.MACRO CnFET_on
	sbi PORTB, CnFET
.ENDMACRO
.MACRO CnFET_off
	cbi PORTB, CnFET
.ENDMACRO


;********************
; Some other Macros *
;********************

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

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO
