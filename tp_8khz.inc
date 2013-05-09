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
.equ	mux_voltage	= 7	; ADC7 voltage input

.equ    O_POWER         = 470
.equ    O_GROUND        = 47

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
