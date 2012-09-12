;***********************************************************
;* TowerPro 17A / 25A / HK-18A "type 1" boards             *
;* with all FETs on PORTD                                  *
;* based on bl-17a.inc from Bernhard's 17a410_ppm_r06.zip  *
;* http://home.versanet.de/~b-konze/                       *
;* http://home.versanet.de/~b-konze/blc_18a/blc_18a.htm    *
;***********************************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 0

; Some TowerPro type 1 clones have a P-channel FET gate noise
; problem that causes self-destruction if the P-ch body diode
; is conducting when the PWM chops on. This happens less often
; at 8kHz, so, uhh...fixed!
.equ    MIN_DUTY        = 58 * 40 / 16
.equ    POWER_RANGE     = 800 * 40 / 16 + MIN_DUTY

;*********************
; PORT D definitions *
;*********************
.equ	BpFET		= 7	;o
.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	ApFET		= 5	;o
.equ	CpFET		= 4	;o
.equ	CnFET		= 3	;o
.equ	rcp_in		= 2	;i r/c pulse input
.equ	BnFET		= 1	;o
.equ	AnFET		= 0	;o

.equ	INIT_PD		= 0
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
.MACRO AnFET_on
	sbi	PORTD, AnFET
.ENDMACRO
.MACRO AnFET_off
	cbi	PORTD, AnFET
.ENDMACRO
.MACRO BnFET_on
	sbi	PORTD, BnFET
.ENDMACRO
.MACRO BnFET_off
	cbi	PORTD, BnFET
.ENDMACRO
.MACRO CnFET_on
	sbi	PORTD, CnFET
.ENDMACRO
.MACRO CnFET_off
	cbi	PORTD, CnFET
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
	sbr	@0, 1<<AnFET
.ENDMACRO
.MACRO AnFET_off_reg
	cbr	@0, 1<<AnFET
.ENDMACRO
.MACRO BnFET_on_reg
	sbr	@0, 1<<BnFET
.ENDMACRO
.MACRO BnFET_off_reg
	cbr	@0, 1<<BnFET
.ENDMACRO
.MACRO CnFET_on_reg
	sbr	@0, 1<<CnFET
.ENDMACRO
.MACRO CnFET_off_reg
	cbr	@0, 1<<CnFET
.ENDMACRO

.MACRO all_pFETs_off
	in	@0, PORTD
	cbr	@0, (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
	out	PORTD, @0
.ENDMACRO

.MACRO nFET_brake
	in	@0, PORTD
	sbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTD, @0
.ENDMACRO
.MACRO all_nFETs_off
	in	@0, PORTD
	cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTD, @0
.ENDMACRO


;*********************
; PORT C definitions *
;*********************
;.equ			= 0	; ADC0
;.equ			= 1	; ADC1
.equ	mux_a		= 2	; ADC2 phase input
.equ	mux_b		= 3	; ADC3 phase input
.equ	mux_c		= 4	; ADC4 phase input
;.equ			= 5	; ADC5
;.equ			= 6	; ADC6
.equ	ACCU_MUX	= 7	; ADC7 voltage control input

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
;.equ			= 0	; connected with pb4 ???

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
