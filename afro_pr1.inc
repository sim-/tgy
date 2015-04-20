;*****************************************
;* AfroESC prototype rev1                *
;* 2015-02-02                            *
;* Fuses should be lfuse=0x3f hfuse=0xca *
;*****************************************

; We start with pull-up enabled on PWM and ENABLE to set diode emulation
; mode on the driver.

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 0
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 1

.equ	DEAD_LOW_NS	= 300
.equ	DEAD_HIGH_NS	= 900
.equ	MOTOR_ADVANCE	= 15
.equ	CHECK_HARDWARE	= 1
.equ	HIGH_SIDE_PWM	= 1

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck)
;.equ			= 4	(miso)
;.equ			= 3	(mosi)
.equ	PWM_B		= 2
.equ	PWM_C		= 1
.equ	rcp_in		= 0	;i r/c pulse input

.equ	INIT_PB		= (1<<PWM_B)+(1<<PWM_C)
.equ	DIR_PB		= 0

.equ	PWM_B_PORT	= PORTB
.equ	PWM_B_DDR	= DDRB
.equ	PWM_C_PORT	= PORTB
.equ	PWM_C_DDR	= DDRB

;*********************
; PORT C definitions *
;*********************
;.equ	mux_voltage	= 7	; ADC7 voltage input (18k from Vbat, 3.3k to gnd, 10.10V -> 1.565V at ADC7)
;.equ	mux_temperature	= 6	; ADC6 temperature input (3.3k from +5V, 10k NTC to gnd)
;.equ	i2c_clk		= 5	; ADC5/SCL
;.equ	i2c_data	= 4	; ADC4/SDA
.equ	green_led	= 3	; oish
.equ	red_led		= 2	; oish
.equ	mux_a		= 1	; ADC1 phase input
.equ	mux_b		= 0	; ADC0 phase input

.equ	O_POWER		= 330
.equ	O_GROUND	= 47

.equ	INIT_PC		= 0
.equ	DIR_PC		= (1<<green_led)+(1<<red_led)

.MACRO RED_on
	sbi	PORTC, red_led
.ENDMACRO
.MACRO RED_off
	cbi	PORTC, red_led
.ENDMACRO
.MACRO GRN_on
	sbi	PORTC, green_led
.ENDMACRO
.MACRO GRN_off
	cbi	PORTC, green_led
.ENDMACRO

;*********************
; PORT D definitions *
;*********************
;.equ	mux_c		= 7 (comparator AN1)
;.equ	sense_star	= 6 (comparator AN0)
;.equ			= 5
;.equ			= 4
.equ	ENABLE_ALL	= 3
.equ	VBAT_HIGH	= 2
.equ	txd		= 1
.equ	PWM_A		= 0

.equ	INIT_PD		= (1<<txd)+(1<<PWM_A)
.equ	DIR_PD		= (1<<txd)

.equ	ENABLE_ALL_PORT	= PORTD
.equ	VBAT_HIGH_PORT	= PORTD
.equ	PWM_A_PORT	= PORTD
.equ	PWM_A_DDR	= DDRD
