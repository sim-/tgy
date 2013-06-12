;***********************************************************
;* TowerPro 18A / Turnigy Plush 18A/25A "type 2" boards    *
;* with pFETs on PORTD, nFETs on PORTB                     *
;* based on tp-18a from Bernhard's 18a_tpro_ppm_r05.zip    *
;* Mai.2007                                                *
;* http://home.versanet.de/~b-konze/                       *
;* http://home.versanet.de/~b-konze/blc_18a/blc_18a.htm    *
;***********************************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 0

.equ    DEAD_LOW_NS     = 450	; May work on some boards, but be careful!
.equ    DEAD_HIGH_NS    = 8000

;*********************
; PORT D definitions *
;*********************
;.equ	mux_c		= 7	;i <plus> comparator input (AIN0)
;.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	ApFET		= 5	;o
.equ	BpFET		= 4	;o
.equ	CpFET		= 3	;o
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
.equ	AnFET		= 2
.equ	BnFET		= 1
.equ	CnFET		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.equ	AnFET_port	= PORTB
.equ	BnFET_port	= PORTB
.equ	CnFET_port	= PORTB

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO
