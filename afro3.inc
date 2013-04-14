;*****************************************
;* AfroESC 3                             *
;* 2012-12-02                            *
;* Fuses should be lfuse=0x3f hfuse=0xca *
;*****************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 0
.equ	USE_I2C		= 1
.equ	USE_UART	= 1
.equ	USE_ICP		= 1

.equ	DEAD_LOW_NS	= 300
.equ	DEAD_HIGH_NS	= 300

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck)
;.equ			= 4	(miso)
;.equ			= 3	(mosi)
.equ	BpFET		= 2	;o
.equ	CpFET		= 1	;o
.equ	rcp_in		= 0	;i r/c pulse input

.equ	INIT_PB		= (1<<BpFET)+(1<<CpFET)
.equ	DIR_PB		= (1<<BpFET)+(1<<CpFET)

.equ	BpFET_port	= PORTB
.equ	CpFET_port	= PORTB

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

.MACRO BpFET_on
	cbi	BpFET_port, BpFET
.ENDMACRO
.MACRO BpFET_off
	sbi	BpFET_port, BpFET
.ENDMACRO
.MACRO CpFET_on
	cbi	CpFET_port, CpFET
.ENDMACRO
.MACRO CpFET_off
	sbi	CpFET_port, CpFET
.ENDMACRO

.MACRO BpFET_on_reg
	cbr	@0, 1<<BpFET
.ENDMACRO
.MACRO BpFET_off_reg
	sbr	@0, 1<<BpFET
.ENDMACRO
.MACRO CpFET_on_reg
	cbr	@0, 1<<CpFET
.ENDMACRO
.MACRO CpFET_off_reg
	sbr	@0, 1<<CpFET
.ENDMACRO

;*********************
; PORT C definitions *
;*********************
.equ	mux_voltage	= 7	; ADC7 voltage input (18k from Vbat, 3.3k to gnd, 10.10V -> 1.565V at ADC7)
.equ	mux_temperature	= 6	; ADC6 temperature input (3.3k from +5V, 10k NTC to gnd)
.equ	i2c_clk		= 5	; ADC5/SCL
.equ	i2c_data	= 4	; ADC4/SDA
.equ	red_led		= 3	; o
.equ	green_led	= 2	; o
.equ	mux_b		= 1	; ADC1 phase input
.equ	mux_a		= 0	; ADC0 phase input

.equ	O_POWER		= 180
.equ	O_GROUND	= 33

.equ	INIT_PC		= (1<<i2c_clk)+(1<<i2c_data)
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

.MACRO RED_on
	sbi	DDRC, red_led
.ENDMACRO
.MACRO RED_off
	cbi	DDRC, red_led
.ENDMACRO
.MACRO GRN_on
	sbi	DDRC, green_led
.ENDMACRO
.MACRO GRN_off
	cbi	DDRC, green_led
.ENDMACRO

;*********************
; PORT D definitions *
;*********************
;.equ	mux_c		= 7 (comparator AN1)
;.equ	sense_star	= 6 (comparator AN0)
.equ	CnFET		= 5
.equ	BnFET		= 4
.equ	AnFET		= 3
.equ	ApFET		= 2
.equ	txd		= 1
.equ	rxd		= 0

.equ	INIT_PD		= (1<<ApFET)+(1<<txd)
.equ	DIR_PD		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)+(1<<ApFET)+(1<<txd)

.equ	AnFET_port	= PORTD
.equ	BnFET_port	= PORTD
.equ	CnFET_port	= PORTD
.equ	ApFET_port	= PORTD

.MACRO AnFET_on
	sbi	AnFET_port, AnFET
.ENDMACRO
.MACRO AnFET_off
	cbi	AnFET_port, AnFET
.ENDMACRO
.MACRO BnFET_on
	sbi	BnFET_port, BnFET
.ENDMACRO
.MACRO BnFET_off
	cbi	BnFET_port, BnFET
.ENDMACRO
.MACRO CnFET_on
	sbi	CnFET_port, CnFET
.ENDMACRO
.MACRO CnFET_off
	cbi	CnFET_port, CnFET
.ENDMACRO
.MACRO ApFET_on
	cbi	ApFET_port, ApFET
.ENDMACRO
.MACRO ApFET_off
	sbi	ApFET_port, ApFET
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
.MACRO ApFET_on_reg
	cbr	@0, 1<<ApFET
.ENDMACRO
.MACRO ApFET_off_reg
	sbr	@0, 1<<ApFET
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
.MACRO all_pFETs_off
	ApFET_off
	BpFET_off
	CpFET_off
.ENDMACRO
