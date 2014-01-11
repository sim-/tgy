;***************************************************************
;* For RCTimer NFS 30A all n-Channel FETs 					   *
;* Fuses should be set to -U lfuse:w:0x2f:m -U hfuse:w:0xCA:m  *
;* By Nils Högberg								               *
;* 		                                                       *
;* https://github.com/sim-/tgy                                 *
;***************************************************************

.equ        F_CPU                = 16000000
.equ        USE_INT0        	 = 0          ; used by channel C
.equ        USE_I2C              = 0          ; We could, no FETs are on the I2C ports
.equ        USE_UART        	 = 0
.equ        USE_ICP              = 0

;.equ       MOTOR_ADVANCE        = 15
;.equ       CHECK_HARDWARE       = 1

;*********************
; PORT D definitions *
;*********************

.equ        ApFET                = 4
.equ        AnFET                = 5
.equ        BpFET                = 3
.equ        BnFET                = 7
.equ		CpFET                = 2

;.equ		c_comp	             = 6		;i common comparator input (AIN0)

.equ        INIT_PD              = (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
.equ        DIR_PD               = (1<<AnFET)+(1<<ApFET)+(1<<BnFET)+(1<<BpFET)+(1<<CpFET)

.equ        ApFET_port           = PORTD
.equ        AnFET_port           = PORTD
.equ        BpFET_port           = PORTD
.equ        BnFET_port           = PORTD
.equ        CpFET_port           = PORTD

;*********************
; PORT C definitions *
;*********************
;.equ        i2c_clk              = 5        ; ADC5/SCL
;.equ        i2c_data        	 = 4        ; ADC4/SDA

.equ        mux_a                = 6        ; ADC6
.equ        mux_b                = 7        ; ADC7
.equ        mux_c                = 0        ; ADC0

.equ        mux_voltage          = 2        ; ADC2 voltage input (? from Vbat, ? to gnd, ?V -> ?V at ADC2)
.equ        mux_temperature      = 1        ; ADC1 temperature input (?k from +5V, ?k NTC to gnd)


.equ        O_POWER              = 220
.equ        O_GROUND             = 51

.equ        INIT_PC              = 0 ;(1<<i2c_clk)+(1<<i2c_data)
.equ        DIR_PC               = 0

;*********************
; PORT B definitions *
;*********************
.equ        CnFET                = 1

.equ        rcp_in               = 0          ; r/c pulse input

.equ        INIT_PB              = 0
.equ        DIR_PB               = (1<<CnFET)

.equ        CnFET_port           = PORTB

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO