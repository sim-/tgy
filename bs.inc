;***************************************************************
;* For HK Blue Series with n- and p-Channel FETs (old version) *
;* Fuses should be set to -U lfuse:w:0x3f:m -U hfuse:w:0xd7:m  *
;* or similar (even intrc will work now)                       *
;* 09/2011                                                     *
;* https://github.com/sim-/tgy                                 *
;***************************************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0	; We could, but FETs are on the I2C ports
.equ	USE_UART	= 0
.equ	USE_ICP		= 0

;*********************
; PORT D definitions *
;*********************
;.equ			= 7
;.equ			= 6
.equ	AnFET		= 5
.equ	ApFET		= 4
;.equ			= 3
.equ	rcp_in		= 2

.equ	INIT_PD		= 0
.equ	DIR_PD		= (1<<AnFET)+(1<<ApFET)

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
	ldi @0, (1<<ISC01)
	out MCUCR, @0		; set next int0 to falling edge
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

.MACRO AnFET_on_reg
	sbr @0, 1<<AnFET
.ENDMACRO
.MACRO AnFET_off_reg
	cbr @0, 1<<AnFET
.ENDMACRO
.MACRO ApFET_on_reg
	sbr @0, 1<<ApFET
.ENDMACRO
.MACRO ApFET_off_reg
	cbr @0, 1<<ApFET
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
;.equ	AnREF		= 1	; ADC1 -> not used
.equ	mux_c		= 0	; ADC0

.equ	INIT_PC		= 0
.equ	DIR_PC		= (1<<BnFET)+(1<<BpFET)+(1<<CpFET)

.equ	BpFET_port	= PORTC
.equ	BnFET_port	= PORTC
.equ	CpFET_port	= PORTC

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

.MACRO BnFET_on_reg
	sbr @0, 1<<BnFET
.ENDMACRO
.MACRO BnFET_off_reg
	cbr @0, 1<<BnFET
.ENDMACRO
.MACRO BpFET_on_reg
	sbr @0, 1<<BpFET
.ENDMACRO
.MACRO BpFET_off_reg
	cbr @0, 1<<BpFET
.ENDMACRO
.MACRO CpFET_on_reg
	sbr @0, 1<<CpFET
.ENDMACRO
.MACRO CpFET_off_reg
	cbr @0, 1<<CpFET
.ENDMACRO

.MACRO comp_init
	in @0, SFIOR
	sbr @0, 1<<ACME		; switch to comparator multiplexed
	out SFIOR, @0
	cbi ADCSRA, ADEN	; disable ADC
.ENDMACRO
.MACRO set_comp_phase_a
	ldi @0, mux_a		; set comparator multiplexer to phase A
	out ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_b
	ldi @0, mux_b		; set comparator multiplexer to phase B
	out ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_c
	ldi @0, mux_c		; set comparator multiplexer to phase C
	out ADMUX, @0
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
.equ	CnFET		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<CnFET)

.equ	CnFET_port	= PORTB

.MACRO CnFET_on
	sbi PORTB, CnFET
.ENDMACRO
.MACRO CnFET_off
	cbi PORTB, CnFET
.ENDMACRO

.MACRO CnFET_on_reg
	sbr @0, 1<<CnFET
.ENDMACRO
.MACRO CnFET_off_reg
	cbr @0, 1<<CnFET
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
