;*****************************************
;* MK BL-Ctrl V1.2                       *
;* 2012-12-02                            *
;*****************************************

; P and N are actually reversed here to do high side PWM to work around
; the sense divider being too weak for phase voltages higher than 10V.
; Also, the 100nf filtering capacitors shift the timing so much that
; there is no point for further waiting in software. We also need to add
; a delay before checking the comparator during starting to avoid being
; fooled by the same lag.

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 0
.equ	USE_I2C		= 1
.equ	USE_UART	= 1
.equ	USE_ICP		= 1
.equ	MK_ADDRESS_PADS	= 1

.equ	MOTOR_ADVANCE	= 30
.equ	START_DELAY_US	= 1000
.equ	HIGH_SIDE_PWM	= 1

;*********************
; PORT B definitions *
;*********************
.equ	adr2		= 7	;i address pad adr2 (2-3)
.equ	adr1		= 6	;i address pad adr1 (1-2)
;.equ			= 5	(sck)
;.equ			= 4	(miso)
.equ	AnFET		= 3	;o (mosi)
.equ	BnFET		= 2	;o
.equ	CnFET		= 1	;o
.equ	rcp_in		= 0	;i r/c pulse input

.equ	INIT_PB		= (1<<adr1)+(1<<adr2)
.equ	DIR_PB		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.equ	AnFET_port	= PORTB
.equ	BnFET_port	= PORTB
.equ	CnFET_port	= PORTB

.MACRO rcp_int_enable
	in	@0, TIMSK
	sbr	@0, (1<<TICIE1)	; enable icn1_int
	out	TIMSK, @0
.ENDMACRO
.MACRO rcp_int_disable
	in	@0, TIMSK
	cbr	@0, (1<<TICIE1)	; disable icn1_int
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

.MACRO AnFET_on
	sbi	PORTB, AnFET
.ENDMACRO
.MACRO AnFET_off
	cbi	PORTB, AnFET
.ENDMACRO
.MACRO BnFET_on
	sbi	PORTB, BnFET
.ENDMACRO
.MACRO BnFET_off
	cbi	PORTB, BnFET
.ENDMACRO
.MACRO CnFET_on
	sbi	PORTB, CnFET
.ENDMACRO
.MACRO CnFET_off
	cbi	PORTB, CnFET
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

.MACRO nFET_brake
	in	@0, PORTB
	sbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTB, @0
.ENDMACRO
.MACRO all_nFETs_off
	in	@0, PORTB
	cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTB, @0
.ENDMACRO

;*********************
; PORT C definitions *
;*********************
.equ	mux_voltage	= 7	; ADC7 voltage input
.equ	mux_current	= 6	; ADC6 current input
.equ	i2c_clk		= 5	; ADC5/SCL
.equ	i2c_data	= 4	; ADC4/SDA
.equ	red_led		= 3	; ADC3 temperature control input
.equ	mux_c		= 2	; ADC2 phase input
.equ	mux_b		= 1	; ADC1 phase input
.equ	mux_a		= 0	; ADC0 phase input

.equ	O_POWER		= 10
.equ	O_GROUND	= 1

.equ	INIT_PC		= (1<<i2c_clk)+(1<<i2c_data)
.equ	DIR_PC		= (1<<red_led)

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

.MACRO RED_on
	sbi	PORTC, red_led
.ENDMACRO
.MACRO RED_off
	cbi	PORTC, red_led
.ENDMACRO


;*********************
; PORT D definitions *
;*********************
.equ	green_led	= 7
;.equ	sense_star	= 6
.equ	CpFET		= 5
.equ	BpFET		= 4
.equ	ApFET		= 3
;.equ	int0		= 2
.equ	txd		= 1
.equ	rxd		= 0

.equ	INIT_PD		= (1<<txd)
.equ	DIR_PD		= (1<<ApFET)+(1<<BpFET)+(1<<CpFET)+(1<<green_led)+(1<<txd)

.equ	ApFET_port	= PORTD
.equ	BpFET_port	= PORTD
.equ	CpFET_port	= PORTD

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

.MACRO GRN_on
	cbi	PORTD, green_led
.ENDMACRO
.MACRO GRN_off
	sbi	PORTD, green_led
.ENDMACRO
