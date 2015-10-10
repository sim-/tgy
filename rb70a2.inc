;********************************************
;* Red Brick 70A first seen September 2015  *
;* Blue circuit board labeled 4P0529A       *
;* If this doesn't work, try rb70a or rb50a *
;* Also new blue HK SS 190-200A boards      *
;********************************************

.equ    F_CPU           = 16000000
.equ    USE_INT0        = 1
.equ    USE_I2C         = 0
.equ    USE_UART        = 0
.equ    USE_ICP         = 0

;*********************
; PORT D definitions *
;*********************
.equ    c_comp          = 6     ;10 i common comparator input (AIN0)
.equ    AnFET           = 5     ;30 o
.equ    ApFET           = 4     ;31 o
.equ    rcp_in          = 2     ;32 i r/c pulse input

.equ    INIT_PD         = 0
.equ    DIR_PD          = (1<<ApFET)+(1<<AnFET)

.equ    ApFET_port      = PORTD
.equ    AnFET_port      = PORTD

;*********************
; PORT C definitions *
;*********************
.equ    mux_c           = 7     ; ADC7 voltage input (47k from Vbat, 2.0k to gnd, 10.10V in -> .402V at ADC7)
.equ    mux_a           = 6     ; ADC6
.equ    BpFET           = 5     ; ADC5
.equ    CnFET           = 4     ; ADC4
.equ    CpFET           = 3     ; ADC3
.equ    mux_voltage     = 2     ; ADC2
;.equ                   = 1     ; ADC1
.equ    mux_b           = 0     ; ADC0

.equ    BpFET_port      = PORTC
.equ    CpFET_port      = PORTC
.equ    CnFET_port      = PORTC

.equ    O_POWER         = 47
.equ    O_GROUND        = 2

.equ    INIT_PC         = 0
.equ    DIR_PC          = (1<<BpFET)+(1<<CpFET)+(1<<CnFET)

;*********************
; PORT B definitions *
;*********************
;.equ                   = 7
;.equ                   = 6
;.equ                   = 5     (sck stk200 interface)
;.equ                   = 4     (miso stk200 interface)
;.equ                   = 3     (mosi stk200 interface)
;.equ                   = 2
;.equ                   = 1
.equ   BnFET            = 0

.equ    BnFET_port      = PORTB

.equ    INIT_PB         = 0
.equ    DIR_PB          = (1<<BnFET)
