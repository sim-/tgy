;***********************************************************
;* HobbyKing Open ESC (unreleased rev 0)                   *
;* 2012-09-29                                              *
;***********************************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 0
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 1

;*********************
; PORT D definitions *
;*********************
.equ	red_led		= 7
.equ	blue_led	= 5
.equ	ApFET		= 4
.equ	io1		= 3
.equ	io2		= 2
.equ	txd		= 1
.equ	rxd		= 0

.equ	INIT_PD		= (1<<ApFET)
.equ	DIR_PD		= (1<<ApFET)|(1<<io1)|(1<<io2)|(1<<red_led)

.equ	ApFET_port	= PORTD

.MACRO RED_on
	sbi	DDRD, red_led
.ENDMACRO
.MACRO RED_off
	cbi	DDRD, red_led
.ENDMACRO
.MACRO GRN_on
	sbi	DDRD, blue_led
.ENDMACRO
.MACRO GRN_off
	cbi	DDRD, blue_led
.ENDMACRO

;*********************
; PORT C definitions *
;*********************
.equ	mux_b		= 7	; ADC7 phase input
.equ	mux_a		= 6	; ADC6 phase input
.equ	BpFET		= 5	;o (ADC5/SCL)
.equ	mux_current	= 4	; ADC4/SDA current input
.equ	CpFET		= 3	;o
.equ	mux_voltage	= 2	; ADC2 voltage input
.equ	mux_temperature	= 1	; ADC1 temperature input
.equ	mux_c		= 0	; ADC0 phase input

.equ	O_POWER		= 220
.equ	O_GROUND	= 51

.equ	INIT_PC		= (1<<BpFET)|(1<<CpFET)
.equ	DIR_PC		= (1<<BpFET)|(1<<CpFET)

.equ	BpFET_port	= PORTC
.equ	CpFET_port	= PORTC

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

.MACRO all_pFETs_off
	ApFET_off
	BpFET_off
	CpFET_off
.ENDMACRO

;*********************
; PORT B definitions *
;*********************
;.equ			= 5	(sck)
;.equ			= 4	(miso)
.equ	AnFET		= 3	;o (mosi)
.equ	BnFET		= 2	;o
.equ	CnFET		= 1	;o
.equ	rcp_in		= 0	;i r/c pulse input

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<AnFET)|(1<<BnFET)|(1<<CnFET)

.equ	AnFET_port	= PORTB
.equ	BnFET_port	= PORTB
.equ	CnFET_port	= PORTB

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
