;*********************************************************** 
;* tp-18a                                                  *
;* Mai.2007
;* http://home.versanet.de/~b-konze/                       *
;*********************************************************** 

;*********************
; PORT D definitions *
;*********************  
;.equ    mux_c   	= 7     ;i <plus> comparator input (AIN0)
;.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	ApFET		= 5	;o
.equ	BpFET		= 4	;o
.equ	CpFET		= 3	;o
.equ	rcp_in		= 2	;i r/c pulse input

.equ	INIT_PD		= 0
.equ	DIR_PD		= (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
.equ	BRAKE_PD	= 0

.equ	USE_ICP		= 0
.equ	T1ICP		= 0

.MACRO rjmp_ext_int0
	rjmp	ext_int0
.ENDMACRO
.MACRO rjmp_icp1_int
	nop
.ENDMACRO
.MACRO rcp_int_enable
	ldi     @0, (1<<INT0)	; enable ext0int
	out	GICR, @0
.ENDMACRO
.MACRO rcp_int_rising_edge
	ldi	@0, (1<<ISC01)+(1<<ISC00)
	out	MCUCR, @0	; set next int0 to rising edge 
.ENDMACRO
.MACRO rcp_int_falling_edge
	ldi     @0, (1<<ISC01)
	out     MCUCR, @0	; set next int0 to falling edge
.ENDMACRO

.MACRO ApFET_on
	sbi	PORTD,5
.ENDMACRO
.MACRO ApFET_off
	cbi	PORTD,5
.ENDMACRO
.MACRO BpFET_on
	sbi	PORTD,4
.ENDMACRO
.MACRO BpFET_off
	cbi	PORTD,4
.ENDMACRO
.MACRO CpFET_on
	sbi	PORTD,3
.ENDMACRO
.MACRO CpFET_off
	cbi	PORTD,3
.ENDMACRO
.MACRO all_pFETs_off
	in	@0, PORTD
	cbr	@0, (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
	out	PORTD, @0
.ENDMACRO


;*********************
; PORT C definitions *
;*********************
;.equ    	  	= 7	; ADC7
;.equ    	  	= 6	; ADC6
.equ	mux_a		= 5	; ADC5 phase input
.equ	mux_b		= 4	; ADC4 phase input
;.equ    	  	= 3	; ADC3
;.equ    	  	= 2	; ADC2
.equ 	temp_adc	= 1 	; ADC1 temperature control input 
.equ 	accu_adc	= 0 	; ADC0 voltage control input 

.equ	INIT_PC		= 0
.equ	DIR_PC		= 0
.equ	BRAKE_PC	= 0

.MACRO set_comp_phase_a
	ldi	@0, mux_a		; set comparator multiplexer to phase A
	out	ADMUX, @0
	cbi	ADCSRA, ADEN		; disable ADC
	in	@0, SFIOR
	sbr	@0, (1<<ACME)		; switch to comparator multiplexed
	out     SFIOR, @0
.ENDMACRO

.MACRO set_comp_phase_b
	ldi	@0, mux_b		; set comparator multiplexer to phase B
	out	ADMUX, @0
	cbi	ADCSRA, ADEN		; disable ADC
	in	@0, SFIOR
	sbr	@0, (1<<ACME)		; switch to comparator multiplexed
	out	SFIOR, @0
.ENDMACRO

.MACRO set_comp_phase_c
	in	@0, SFIOR
	cbr	@0, (1<<ACME)		; set to AN1 (phase C)
	out	SFIOR, @0
	sbi	ADCSRA, ADEN		; enable ADC
.ENDMACRO


;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5    	(sck stk200 interface) 
;.equ			= 4	(miso stk200 interface)
;.equ			= 3	(mosi stk200 interface) 
.equ	AnFET		= 2
.equ	BnFET		= 1
.equ	CnFET		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
.equ	BRAKE_PB	= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.MACRO AnFET_on
	sbi	PORTB,2
.ENDMACRO
.MACRO AnFET_off
	cbi	PORTB,2
.ENDMACRO
.MACRO BnFET_on
	sbi	PORTB,1
.ENDMACRO
.MACRO BnFET_off
	cbi	PORTB,1
.ENDMACRO
.MACRO CnFET_on
	sbi	PORTB,0
.ENDMACRO
.MACRO CnFET_off
	cbi	PORTB,0
.ENDMACRO
.MACRO nFET_brake
	in	@0, PORTB
	sbr	@0, BRAKE_PB
	out	PORTB, @0
.ENDMACRO
.MACRO all_nFETs_off
	in	@0, PORTB
	cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
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
