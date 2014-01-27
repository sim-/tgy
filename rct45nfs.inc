;***************************************************************
;* For RCTimer NFS 45A all n-Channel FETs 					   *
;* Original fuses are -U lfuse:w:0x2F:m -U hfuse:w:0xCA:m      *
;* By Nils Högberg								               *
;* 		                                                       *
;* https://github.com/sim-/tgy                                 *
;***************************************************************

.equ        F_CPU                = 16000000
.equ        USE_INT0        	 = 1
.equ        USE_I2C              = 0
.equ        USE_UART        	 = 0
.equ        USE_ICP              = 0
.equ		MOTOR_ADVANCE        = 18		; Degrees of timing advance (0 - 30, 30 meaning no delay)
.equ        TIMING_OFFSET        = 0
.equ		CHECK_HARDWARE       = 0

;*********************
; PORT D definitions *
;*********************
;.equ                            = 7
;.equ                            = 6        ; AIN0
.equ        ApFET                = 5
.equ        BpFET                = 4
.equ		CpFET                = 3
.equ        rcp_in               = 2        ; r/c pulse input
;.equ        txd                  = 1
;.equ        rxd                  = 0

.equ        INIT_PD              = 0
.equ        DIR_PD               = (1<<ApFET)+(1<<BpFET)+(1<<CpFET)

.equ        ApFET_port           = PORTD
.equ        BpFET_port           = PORTD
.equ        CpFET_port           = PORTD

;*********************
; PORT C definitions *
;*********************
;.equ                            = 7        ; ADC7
;.equ                            = 6        ; ADC6
.equ        mux_a                = 5        ; ADC5
.equ        mux_b                = 4        ; ADC4
.equ        mux_c                = 3        ; ADC3
.equ        mux_temperature      = 1        ; ADC1 temperature input (897 from +5V, ?k NTC to gnd at ADC1)
.equ        mux_voltage          = 0        ; ADC2 voltage input (220 from Vbat, 39 to gnd, ?V -> ?V at ADC2)



.equ        O_POWER              = 220
.equ        O_GROUND             = 39

.equ        INIT_PC              = 0
.equ        DIR_PC               = 0

;*********************
; PORT B definitions *
;*********************
;.equ                            = 7
;.equ                            = 6
;.equ                            = 5        ; SCK
;.equ                            = 4        ; MISO
;.equ                            = 3        ; MOSI
.equ        AnFET                = 2
.equ        BnFET                = 1
.equ        CnFET                = 0

.equ        INIT_PB              = 0
.equ        DIR_PB               = (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.equ        AnFET_port           = PORTB
.equ        BnFET_port           = PORTB
.equ        CnFET_port           = PORTB

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO