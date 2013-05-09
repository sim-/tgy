;***************************************************************
;* For HK Blue Series with n- and p-Channel FETs (old version) *
;* Fuses should be set to -U lfuse:w:0x3f:m -U hfuse:w:0xd7:m  *
;* or similar (even intrc will work now)                       *
;* 09/2011                                                     *
;* https://github.com/sim-/tgy                                 *
;***************************************************************

.equ	F_CPU		= 16000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0	; We could, but FETs are on the I2C ports
.equ	USE_UART	= 0
.equ	USE_ICP		= 0

.equ	DEAD_LOW_NS	= 1400	; Calibrated to F-40A green PCB -- bigger boards may vary!
.equ	DEAD_HIGH_NS	= 1700	; For P/N boards, this may need to be more like 30000ns!

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

.MACRO rcp_int_enable
	ldi	@0, (1<<INT0)	; enable ext0int
	out	GICR, @0
.ENDMACRO
.MACRO rcp_int_disable
	out	GICR, ZH	; disable ext0int
.ENDMACRO
.MACRO rcp_int_rising_edge
	ldi	@0, (1<<ISC01)+(1<<ISC00)
	out	MCUCR, @0	; set next int0 to rising edge
.ENDMACRO
.MACRO rcp_int_falling_edge
	ldi @0, (1<<ISC01)
	out MCUCR, @0		; set next int0 to falling edge
.ENDMACRO

;*********************
; PORT C definitions *
;*********************
.equ	mux_b		= 7	; ADC7
.equ	mux_a		= 6	; ADC6
.equ	BpFET		= 5
.equ	BnFET		= 4
.equ	CpFET		= 3
.equ	mux_voltage	= 2	; ADC2 voltage input (220k from Vbat, 51k to gnd, 10.10V -> 1.900V at ADC2)
.equ	mux_temperature = 1	; ADC1 temperature input (some boards) (10k NTC to 5V, 820 to gnd)
.equ	mux_c		= 0	; ADC0

.equ	O_POWER		= 220
.equ	O_GROUND	= 51

.equ	INIT_PC		= 0
.equ	DIR_PC		= (1<<BnFET)+(1<<BpFET)+(1<<CpFET)

.equ	BpFET_port	= PORTC
.equ	BnFET_port	= PORTC
.equ	CpFET_port	= PORTC

.MACRO comp_init
	in @0, SFIOR
	sbr @0, 1<<ACME		; switch to comparator multiplexed
	out SFIOR, @0
	cbi ADCSRA, ADEN	; disable ADC
.ENDMACRO
.MACRO set_comp_phase_a
	ldi @0, mux_a		; set comparator multiplexer to phase A
	out ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_b
	ldi @0, mux_b		; set comparator multiplexer to phase B
	out ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_c
	ldi @0, mux_c		; set comparator multiplexer to phase C
	out ADMUX, @0
.ENDMACRO

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
