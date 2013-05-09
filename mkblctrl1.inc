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

.MACRO GRN_on
	cbi	PORTD, green_led
.ENDMACRO
.MACRO GRN_off
	sbi	PORTD, green_led
.ENDMACRO
