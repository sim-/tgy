;************************************
;* Maytech Eagle Mult-Series 50A HV *
;************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0	; We could, but FETs are on the I2C ports
.equ	USE_UART	= 0
.equ	USE_ICP		= 0
.equ	COMP_PWM	= 1

.equ	DEAD_LOW_NS	= 1000	; 20A and 30A boards clean at 300ns
.equ	DEAD_HIGH_NS	= 1000

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

;*********************
; PORT C definitions *
;*********************
.equ	mux_voltage		= 7	; ADC7
;.equ	       		= 6	; ADC6
.equ	BpFET		= 5
.equ	BnFET		= 4
.equ	CpFET		= 3
.equ	mux_a		= 2	; ADC2
.equ	mux_b		= 1	; ADC1
.equ	mux_c		= 0	; ADC0

.equ	O_POWER		= 220
.equ	O_GROUND	= 51

.equ	INIT_PC		= 0
.equ	DIR_PC		= (1<<BnFET)+(1<<BpFET)+(1<<CpFET)

.equ	BpFET_port	= PORTC
.equ	BnFET_port	= PORTC
.equ	CpFET_port	= PORTC

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck stk200 interface)
;.equ			= 4	(miso stk200 interface)
;.equ			= 3	(mosi stk200 interface)
.equ	GND_PIN		= 2	; Seems to be grounded on this board
;.equ			= 1
.equ	CnFET		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<CnFET)

.equ	CnFET_port	= PORTB

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO
