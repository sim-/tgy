;***********************************************************
;* XXD HW30A 30A Brushless Motor ESC                       *
;* with all nFETs                                          *
;* December.2018                                           *
;* https://www.banggood.com/Wholesale-XXD-HW30A-30A-Brushless-Motor-ESC-For-Airplane-Quadcopter-p-50621.html *
;* https://github.com/NicksonYap/XXD-HW30A-ESC-Schematic   *
;***********************************************************

.set	F_CPU		= 8000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 0

.equ    DEAD_LOW_NS     = 4000
.equ    DEAD_HIGH_NS    = 8000

;*********************
; PORT D definitions *
;*********************
.equ	mux_c		= 7	;i <plus> comparator input (AIN0)
.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	CpFET		= 5	;o
.equ	BpFET		= 4	;o
.equ	ApFET		= 3	;o
.equ	rcp_in		= 2	;i r/c pulse input

.equ	INIT_PD		= 0
.equ	DIR_PD		= (1<<ApFET)+(1<<BpFET)+(1<<CpFET)

.equ	ApFET_port	= PORTD
.equ	BpFET_port	= PORTD
.equ	CpFET_port	= PORTD

;*********************
; PORT C definitions *
;*********************
;.equ			= 7	; ADC7
;.equ			= 6	; ADC6
.equ	mux_a		= 5	; ADC5 phase input
.equ	mux_b		= 4	; ADC4 phase input
;.equ			= 3	; ADC3
;.equ			= 2	; ADC2 (plush has something on here)
;.equ	mux_temperature	= 1	; ADC1 temperature input
.equ	mux_voltage	= 0	; ADC0 voltage input (10k from Vbat, 1k to gnd, 10.10V -> .918V at ADC0)

.equ	O_POWER		= 10
.equ	O_GROUND	= 1

.equ	INIT_PC		= 0
.equ	DIR_PC		= 0

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck stk200 interface)
;.equ			= 4	(miso stk200 interface)
;.equ			= 3	(mosi stk200 interface)
.equ	CnFET		= 2
.equ	BnFET		= 1
.equ	AnFET		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.equ	AnFET_port	= PORTB
.equ	BnFET_port	= PORTB
.equ	CnFET_port	= PORTB
