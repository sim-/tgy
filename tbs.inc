;*****************************************
;* TBS ESC                               *
;* 2013-03-27                            *
;* Fuses should be lfuse=0x3f hfuse=0xca *
;*****************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 0
.equ	USE_I2C		= 0
.equ	USE_UART	= 1
.equ	USE_ICP		= 1

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck)
;.equ			= 4	(miso)
;.equ			= 3	(mosi)
;.equ			= 2
;.equ			= 1
.equ	rcp_in		= 0	;i r/c pulse input

.equ	INIT_PB		= 0
.equ	DIR_PB		= 0

.MACRO rcp_int_enable
	in	@0, TIMSK
	sbr	@0, (1<<TICIE1)	; enable icp1_int
	out	TIMSK, @0
.ENDMACRO
.MACRO rcp_int_disable
	in	@0, TIMSK
	cbr	@0, (1<<TICIE1)	; disable icp1_int
	out	TIMSK, @0
.ENDMACRO
.MACRO rcp_int_rising_edge
	ldi	@0, T1CLK
	out	TCCR1B, @0
.ENDMACRO
.MACRO rcp_int_falling_edge
	ldi	@0, T1CLK & ~(1<<ICES1)
	out	TCCR1B, @0
.ENDMACRO

;*********************
; PORT C definitions *
;*********************
.equ	mux_b		= 7	; ADC7
.equ	mux_a		= 6	; ADC6
.equ	BpFET		= 5	; ADC5/SCL
.equ	AnFET		= 4	; ADC4/SDA
;.equ			= 3	; ADC3
.equ	mux_voltage	= 2	; ADC2 voltage input (18k from Vbat, 3.3k to gnd, 10.10V -> 1.565V at ADC7)
;.equ			= 1	; ADC1 phase input
.equ	mux_c		= 0	; ADC0 phase input

.equ	O_POWER		= 180
.equ	O_GROUND	= 33

.equ	INIT_PC		= (1<<BpFET)
.equ	DIR_PC		= (1<<AnFET)+(1<<BpFET)

.equ	BpFET_port	= PORTC
.equ	AnFET_port	= PORTC

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
; PORT D definitions *
;*********************
;.equ			= 7 (comparator AN1)
;.equ	sense_star	= 6 (comparator AN0)
.equ	ApFET		= 5
.equ	CnFET		= 4
.equ	CpFET		= 3
.equ	BnFET		= 2
.equ	txd		= 1
.equ	rxd		= 0

.equ	INIT_PD		= (1<<ApFET)+(1<<CpFET)+(1<<txd)
.equ	DIR_PD		= (1<<ApFET)+(1<<CnFET)+(1<<CpFET)+(1<<BnFET)+(1<<txd)

.equ	ApFET_port	= PORTD
.equ	CnFET_port	= PORTD
.equ	CpFET_port	= PORTD
.equ	BnFET_port	= PORTD

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO
