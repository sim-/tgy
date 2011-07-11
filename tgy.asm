;**** **** **** **** ****
;
;Die Benutzung der Software ist mit folgenden Bedingungen verbunden:
;
;1. Da ich alles kostenlos zur Verfügung stelle, gebe ich keinerlei Garantie
;   und übernehme auch keinerlei Haftung für die Folgen der Benutzung.
;
;2. Die Software ist ausschließlich zur privaten Nutzung bestimmt. Ich
;   habe nicht geprüft, ob bei gewerblicher Nutzung irgendwelche Patentrechte
;   verletzt werden oder sonstige rechtliche Einschränkungen vorliegen.
;
;3. Jeder darf Änderungen vornehmen, z.B. um die Funktion seinen Bedürfnissen
;   anzupassen oder zu erweitern. Ich würde mich freuen, wenn ich weiterhin als
;   Co-Autor in den Unterlagen erscheine und mir ein Link zur entprechenden Seite
;   (falls vorhanden) mitgeteilt wird.
;
;4. Auch nach den Änderungen sollen die Software weiterhin frei sein, d.h. kostenlos bleiben.
;
;!! Wer mit den Nutzungbedingungen nicht einverstanden ist, darf die Software nicht nutzen !!
;
; tp-18a
; October 2004
; autor: Bernhard Konze
; email: bernhard.konze@versanet.de
;
; TGYP2010_416HzRCIntervalRate_PPM-Mod-only_NoCal
; For Turnigy Plush 30A or similar ESCs.
; Applying various small code changes documented in the RC Groups thread on
; converting TowerPro 25A ESCs. PPM - mod only - no code for TWI/I2C based implementation
; Corrected the minimum PPM pulse to 1040usec and decreased the min throttle value
; to 2 for larger range. Decreased RC Pulse Interval from ~58Hz to ~416Hz. No calibration
; code as didn't work
;
; Modified to not reboot if motor does not start, for full 8-bit PWM, and
; for better but not perfect starting with 2213N motors.
; -Simon, 2011-04-17
;
; WARNING: I have blown a FET on two Turnigy Plush 18A ESCs in
; upside-down crashes with this code. This didn't seem to happen with
; the original Plush code, but if I try to reproduce it, this code
; usually seems to behave better than the Plush code which seems to
; scream instead of stop in such cases. I'm not sure what the issue is.
; -Simon, 2011-07-04
;
; WARNING: Unlike the original Turnigy Plush code, this does not check
; the AVR temperature ADC or the battery voltage!
;
;**** **** **** **** ****
; Device
;
;**** **** **** **** ****
.include "m8def.inc"
;
; 8K Bytes of In-System Self-Programmable Flash
; 512 Bytes EEPROM
; 1K Byte Internal SRAM
;**** **** **** **** ****
;**** **** **** **** ****
; fuses must be set to internal calibrated oscillator = 8 mhz
;**** **** **** **** ****
;**** **** **** **** ****

.equ MOT_BRAKE    = 0

.equ RC_PULS 	  = 1

; 1040 here seems to low for output from multikopter board -sim
; 1110 here is right, but cold weather just trips it -sim
.equ	MIN_RC_PULS	= 1104	; µs (or lower) = NO_POWER

.include "tgy.inc"

;.equ	CHANGE_TIMEOUT	= 1
;.equ	CHANGE_TOT_LOW	= 2

.equ	POWER_RANGE	= 256			; full range of tcnt0 setting
; The following is Javierete's mod to ensure smoother start up with heavier motors.
; Note that in the I2C version original value is 8, changed to 2 !
.equ	MIN_DUTY	= 1			; no power
;.equ	NO_POWER	= 256-MIN_DUTY		; (POWER_OFF)
;.equ	MAX_POWER	= 256-POWER_RANGE	; (FULL_POWER)
.equ	NO_POWER	= 255
.equ	MAX_POWER	= 0

.equ	PWR_MAX_RPM1	= POWER_RANGE/4
.equ	PWR_MAX_RPM2	= POWER_RANGE/2

.equ	PWR_STARTUP	= 2 ;MIN_DUTY
.equ	PWR_MAX_STARTUP	= 8 ;MIN_DUTY+5

.equ	timeoutSTART	= 48000
.equ	timeoutMIN	= 36000

.equ	T1STOP	= 0x00
.equ	T1CK8	= 0x02

.equ	EXT0_DIS	= 0x00	; disable ext0int

.equ	EXT0_EN		= 0x40	; enable ext0int

;.equ	OCT1_RANGE1	= 16	; ( ~2400 RPM )
;.equ	OCT1_RANGE2	= 8	; ( ~4800 RPM )

.equ	PWR_RANGE_RUN	= 0x60	; ( ~1600 RPM )
.equ	PWR_RANGE1	= 0x40	; ( ~2400 RPM )
.equ	PWR_RANGE2	= 0x20	; ( ~4800 RPM )

.equ	ENOUGH_GOODIES	= 60

;**** **** **** **** ****
; Register Definitions
.def	i_sreg		 = r1	; status register save in interrupts
.def	tcnt0_power_on	 = r2	; timer0 counts nFETs are switched on
.def	tcnt0_change_tot = r3	; when zero, tcnt0_power_on is changed by one (inc or dec)
;.def	...	 	 = r4	; upper 8bit timer1 (software-) register
.def	uart_cnt	 = r5
.def	tcnt0_pwron_next = r6

.def	start_rcpuls_l	 = r7
.def	start_rcpuls_h	 = r8
;.def		 	 = r9
;.def			 = r10
.def	rcpuls_timeout	 = r11
.equ	RCP_TOT		 = 32	; Also affects time spent trying to start -Simon

.def	current_err	 = r12	; counts consecutive current errors
.equ	CURRENT_ERR_MAX  = 3	; performs a reset after MAX errors

.def	sys_control	 = r13
.def	t1_timeout	 = r14
.def	run_control	 = r15


.def	temp1	= r16			; main temporary
.def	temp2	= r17			; main temporary
.def	temp3	= r18			; main temporary
.def	temp4	= r19			; main temporary
.def	temp5	= r9
.def	temp6	= r10
.def	temp7	= r4

.def	i_temp1	= r20			; interrupt temporary
.def	i_temp2	= r21			; interrupt temporary
.def	i_temp3	= r22			; interrupt temporary

.def	flags0	= r23	; state flags
	.equ	OCT1_PENDING	= 0	; if set, output compare interrunpt is pending
	.equ	UB_LOW 		= 1	; set if accu voltage low
	.equ	I_pFET_HIGH	= 2	; set if over-current detect
	.equ	GET_STATE	= 3	; set if state is to be send
	.equ	C_FET		= 4	; if set, C-FET state is to be changed
	.equ	A_FET		= 5	; if set, A-FET state is to be changed
	     ; if neither 1 nor 2 is set, B-FET state is to be changed
	.equ	I_OFF_CYCLE	= 6	; if set, current off cycle is active
	.equ	T1OVFL_FLAG	= 7	; each timer1 overflow sets this flag - used for voltage + current watch

.def	flags1	= r24	; state flags
	.equ	POWER_OFF	= 0	; switch fets on disabled
	.equ	FULL_POWER	= 1	; 100% on - don't switch off, but do OFF_CYCLE working
	.equ	CALC_NEXT_OCT1	= 2	; calculate OCT1 offset, when wait_OCT1_before_switch is called
	.equ	RC_PULS_UPDATED	= 3	; new rc-puls value available
	.equ	EVAL_RC_PULS	= 4	; if set, new rc puls is evaluated, while waiting for OCT1
	.equ	EVAL_SYS_STATE	= 5	; if set, overcurrent and undervoltage are checked
	.equ	EVAL_RPM	= 6	; if set, next PWM on should look for current
	.equ	EVAL_UART	= 7	; if set, next PWM on should look for uart

.def	flags2	= r25
	.equ	RPM_RANGE1	= 0	; if set RPM is lower than 1831 RPM
	.equ	RPM_RANGE2	= 1	; if set RPM is between 1831 RPM and 3662 RPM
	.equ	SCAN_TIMEOUT	= 2	; if set a startup timeout occurred
	.equ	POFF_CYCLE	= 3	; if set one commutation cycle is performed without power
	.equ	COMP_SAVE	= 4	; if set ACO was high
	.equ	STARTUP		= 5	; if set startup-phase is active
	.equ	RC_INTERVAL_OK	= 6	;
	.equ	GP_FLAG		= 7	;

; here the XYZ registers are placed ( r26-r31)

; ZH = new_duty		; PWM destination


;**** **** **** **** ****
; RAM Definitions
.dseg					;EEPROM segment
.org SRAM_START

tcnt1_sav_l:	.byte	1	; actual timer1 value
tcnt1_sav_h:	.byte	1
last_tcnt1_l:	.byte	1	; last timer1 value
last_tcnt1_h:	.byte	1
timing_l:	.byte	1	; holds time of 4 commutations
timing_h:	.byte	1
timing_x:	.byte	1

timing_acc_l:	.byte	1	; holds the average time of 4 commutations
timing_acc_h:	.byte	1
timing_acc_x:	.byte	1

rpm_l:		.byte	1	; holds the average time of 4 commutations
rpm_h:		.byte	1
rpm_x:		.byte	1



wt_comp_scan_l:	.byte	1	; time from switch to comparator scan
wt_comp_scan_h:	.byte	1
com_timing_l:	.byte	1	; time from zero-crossing to switch of the appropriate FET
com_timing_h:	.byte	1
wt_OCT1_tot_l:	.byte	1	; OCT1 waiting time
wt_OCT1_tot_h:	.byte	1
zero_wt_l:	.byte	1
zero_wt_h:	.byte	1
last_com_l:	.byte	1
last_com_h:	.byte	1

stop_rcpuls_l:	.byte	1
stop_rcpuls_h:	.byte	1
new_rcpuls_l:	.byte	1
new_rcpuls_h:	.byte	1

duty_offset:	.byte	1
goodies:	.byte	1
comp_state:	.byte	1
gp_cnt:		.byte	1

uart_data:	.byte	100		; only for debug requirements


;**** **** **** **** ****
; ATmega8 interrupts

;.equ	INT0addr=$001	; External Interrupt0 Vector Address
;.equ	INT1addr=$002	; External Interrupt1 Vector Address
;.equ	OC2addr =$003	; Output Compare2 Interrupt Vector Address
;.equ	OVF2addr=$004	; Overflow2 Interrupt Vector Address
;.equ	ICP1addr=$005	; Input Capture1 Interrupt Vector Address
;.equ	OC1Aaddr=$006	; Output Compare1A Interrupt Vector Address
;.equ	OC1Baddr=$007	; Output Compare1B Interrupt Vector Address
;.equ	OVF1addr=$008	; Overflow1 Interrupt Vector Address
;.equ	OVF0addr=$009	; Overflow0 Interrupt Vector Address
;.equ	SPIaddr =$00a	; SPI Interrupt Vector Address
;.equ	URXCaddr=$00b	; USART Receive Complete Interrupt Vector Address
;.equ	UDREaddr=$00c	; USART Data Register Empty Interrupt Vector Address
;.equ	UTXCaddr=$00d	; USART Transmit Complete Interrupt Vector Address
;.equ	ADCCaddr=$00e	; ADC Interrupt Vector Address
;.equ	ERDYaddr=$00f	; EEPROM Interrupt Vector Address
;.equ	ACIaddr =$010	; Analog Comparator Interrupt Vector Address
;.equ	TWIaddr =$011	; Irq. vector address for Two-Wire Interface
;.equ	SPMaddr =$012	; SPM complete Interrupt Vector Address
;.equ	SPMRaddr =$012	; SPM complete Interrupt Vector Address
;-----bko-----------------------------------------------------------------

;**** **** **** **** ****
.cseg
.org 0
;**** **** **** **** ****

;-----bko-----------------------------------------------------------------
; reset and interrupt jump table
		rjmp	reset
		rjmp	ext_int0

		nop	; ext_int1
		nop	; t2oc_int
		;nop	; t2ovfl_int
		rjmp	t0ovfl_int
		nop	; icp1
		rjmp	t1oca_int
		nop	; t1ocb_int
		rjmp	t1ovfl_int
		;rjmp	t0ovfl_int
		nop	; t0ovfl_int
		nop	; spi_int
		nop	; urxc
		nop	; udre
		nop	; utxc

; not used	nop	; adc_int
; not used	nop	; eep_int
; not used	nop	; aci_int
; not used	nop	; wire2_int
; not used	nop	; spmc_int


version:	.db	0x0d, 0x0a
		.db	"TGYP2010_416HzRCIntervalRate_PPM-Mod-only_NoCal."
version_end:	.db	0x0d, 0x0a


;-----bko-----------------------------------------------------------------
; init after reset

reset:		ldi	temp1, high(RAMEND)	; stack = RAMEND
		out	SPH, temp1
		ldi	temp1, low(RAMEND)
		out 	SPL, temp1
; oscillator calibration byte is written into the uppermost position
; of the eeprom - by the script 1n1p.e2s an ponyprog
;CLEARBUFFER
;LOAD-PROG 1n1p.hex
;PAUSE "Connect and powerup the circuit, are you ready?"
;READ-CALIBRATION 0x21FF DATA 3     # <EEProm 8Mhz
;ERASE-ALL
;WRITE&VERIFY-ALL

	; portB - all FETs off
		ldi	temp1, INIT_PB		; PORTB initially holds 0x00
		out	PORTB, temp1
		ldi	temp1, DIR_PB
		out	DDRB, temp1

	; portC reads comparator inputs
		ldi	temp1, INIT_PC
		out	PORTC, temp1
		ldi	temp1, DIR_PC
		out	DDRC, temp1

	; portD reads rc-puls + AIN0 ( + RxD, TxD for debug )
		ldi	temp1, INIT_PD
		out	PORTD, temp1
		ldi	temp1, DIR_PD
		out	DDRD, temp1

	; timer0: PWM + beep control = 0x02 	; start timer0 with CK/8 (1µs/count)
		ldi	temp1, 0x01
		out	TCCR2, temp1

	; timer1: commutation control = 0x02	; start timer1 with CK/8 (1µs/count)
		ldi	temp1, T1CK8
		out	TCCR1B, temp1

	; reset state flags
		clr	flags0
		clr	flags1
		clr	flags2

	; clear RAM
		clr	XH
		ldi	XL, low (SRAM_START)
		clr	temp1
clear_ram:	st	X+, temp1
		cpi	XL, uart_data+1
		brlo	clear_ram

	; power off
		rcall	switch_power_off

	; reset rc puls timeout
		ldi	temp1, RCP_TOT
		mov	rcpuls_timeout, temp1

		rcall	wait260ms	; wait a while
;		rcall	wait260ms

		rcall	beep_f1
		rcall	wait30ms
		rcall	beep_f2
		rcall	wait30ms
		rcall	beep_f3
		rcall	wait30ms

control_start:	; init variables
;		ldi	temp1, CHANGE_TIMEOUT
;		mov	tcnt0_change_tot, temp1
		ldi	temp1, NO_POWER
		mov	tcnt0_power_on, temp1

		ldi	temp1, 0		; reset error counters
		mov	current_err,temp1
		mov	sys_control, temp1

	; init registers and interrupts
		ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE2)
		out	TIFR, temp1		; clear TOIE1,OCIE1A & TOIE0
		out	TIMSK, temp1		; enable TOIE1,OCIE1A & TOIE0 interrupts

		sei				; enable all interrupts

; init rc-puls
		ldi	temp1, (1<<ISC01)+(1<<ISC00)
		out	MCUCR, temp1		; set next int0 to rising edge
		ldi	temp1, EXT0_EN		; enable ext0int
		out	GIMSK, temp1
i_rc_puls1:	ldi	temp3, 10		; wait for this count of receiving power off
i_rc_puls2:	sbrs	flags1, RC_PULS_UPDATED
		rjmp	i_rc_puls2
		lds	temp1, new_rcpuls_l
		lds	temp2, new_rcpuls_h
		cbr	flags1, (1<<RC_PULS_UPDATED) ; rc impuls value is read out
		subi	temp1, low  (MIN_RC_PULS)	; power off received ?
		sbci	temp2, high (MIN_RC_PULS)
		brcc	i_rc_puls1		; no - reset counter
		dec	temp3			; yes - decrement counter
		brne	i_rc_puls2		; repeat until zero
		cli				; disable all interrupts
		rcall	beep_f4			; signal: rcpuls ready
		rcall	beep_f4
		rcall	beep_f4
		sei				; enable all interrupts


		ldi	temp1, 30
		sts	duty_offset, temp1

		rcall	set_all_timings

		rjmp	init_startup

;-----bko-----------------------------------------------------------------
; external interrupt0 = rc pulse input
; NOTE: This interrupt uses the 16-bit atomic timer read/write register
; by reading TCNT1L and TCNT1H, so this interrupt must be disabled before
; any other 16-bit timer options happen that might use the same register
; (see "Accessing 16-bit registers" in the Atmel documentation)
ext_int0:	in	i_sreg, SREG
		clr	i_temp1			; disable extint edge may be changed
		out	GIMSK, i_temp1

; evaluate edge of this interrupt
		in	i_temp1, MCUCR
		sbrs	i_temp1, ISC00
		rjmp	falling_edge		; bit is clear = falling edge

; should be rising edge - test rc impuls level state for possible jitter
		sbis	PIND, rcp_in
		rjmp	extint1_exit		; jump, if low state

; rc impuls is at high state
		ldi	i_temp1, (1<<ISC01)
		out	MCUCR, i_temp1		; set next int0 to falling edge

; get timer1 values
		in	i_temp1, TCNT1L
		in	i_temp2, TCNT1H
		mov	start_rcpuls_l, i_temp1
		mov	start_rcpuls_h, i_temp2
; test rcpulse interval
		cbr	flags2, (1<<RC_INTERVAL_OK) ; preset to not ok
		lds	i_temp3, stop_rcpuls_l
		sub	i_temp1, i_temp3
		lds	i_temp3, stop_rcpuls_h
		sbc	i_temp2, i_temp3
		cpi	i_temp1, low (25000)
		ldi	i_temp3, high(25000)	; test range high
		cpc	i_temp2, i_temp3
		brsh	extint1_fail		; through away
		cpi	i_temp1, low (5)	; 200 ok for 417Hz, 5 for 495Hz
		ldi	i_temp3, high(5)	; test range low
		cpc	i_temp2, i_temp3
		brlo	extint1_fail		; through away
		sbr	flags2, (1<<RC_INTERVAL_OK) ; set to rc impuls value is ok !
		rjmp	extint1_exit

extint1_fail:	tst	rcpuls_timeout
		breq	extint1_exit
		dec	rcpuls_timeout
		rjmp	extint1_exit

; rc impuls is at low state
falling_edge:	sbic	PIND, rcp_in		; test level of rc impuls
		rjmp	extint1_exit		; seems to be a spike

		ldi	i_temp1, (1<<ISC01)+(1<<ISC00)
		out	MCUCR, i_temp1		; set next int0 to rising edge
		sbrc	flags1, RC_PULS_UPDATED
		rjmp	extint1_exit

; get timer1 values
		in	i_temp1, TCNT1L
		in	i_temp2, TCNT1H
		sts	stop_rcpuls_l, i_temp1	; prepare next interval evaluation
		sts	stop_rcpuls_h, i_temp2

		sbrs	flags2, RC_INTERVAL_OK
		rjmp	extint1_exit
		cbr	flags2, (1<<RC_INTERVAL_OK) ; flag is evaluated

		sub	i_temp1, start_rcpuls_l
		sbc	i_temp2, start_rcpuls_h

	; save impuls length
		sts	new_rcpuls_l, i_temp1
		sts	new_rcpuls_h, i_temp2
		cpi	i_temp1, low (2200)
		ldi	i_temp3, high(2200)	; test range high
		cpc	i_temp2, i_temp3
		brsh	extint1_fail		; through away
		cpi	i_temp1, low (800)
		ldi	i_temp3, high(800)	; test range low
		cpc	i_temp2, i_temp3
		brlo	extint1_fail		; through away
		sbr	flags1, (1<<RC_PULS_UPDATED) ; set to rc impuls value is ok !
		mov	i_temp1, rcpuls_timeout
		cpi	i_temp1, RCP_TOT
		breq	extint1_exit
		inc	rcpuls_timeout

; enable int1 again -  also entry for spike detect
extint1_exit:	ldi	i_temp2, EXT0_EN
		out	GIMSK, i_temp2
		out	SREG, i_sreg
		reti

;-----bko-----------------------------------------------------------------
; output compare timer1 interrupt
t1oca_int:	in	i_sreg, SREG
		cbr	flags0, (1<<OCT1_PENDING) ; signal OCT1 passed
		out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; overflow timer1 / happens all 65536µs
t1ovfl_int:	in	i_sreg, SREG
		sbr	flags0, (1<<T1OVFL_FLAG)

		tst	t1_timeout
		breq	t1ovfl_10
		dec	t1_timeout
t1ovfl_10:

		tst	rcpuls_timeout
		breq	t1ovfl_99
		dec	rcpuls_timeout

t1ovfl_99:	out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; timer0 overflow interrupt
t0ovfl_int:	in	i_sreg, SREG
		sbrc	flags0, I_OFF_CYCLE
		rjmp	t0_on_cycle

t0_off_cycle:	sbr	flags2, (1<<COMP_SAVE)
		sbic	ACSR, ACO		; mirror inverted ACO to bit-var
		cbr	flags2, (1<<COMP_SAVE)

	; changes in PWM ?

		mov     tcnt0_power_on, tcnt0_pwron_next ; Just set it -Simon

;		mov	i_temp1, tcnt0_power_on
;		mov	i_temp2, tcnt0_pwron_next
;		cp	i_temp2, i_temp1
;		brsh	lower_pwm		; next power-on-time is lower or same
;higher_pwm:	dec	tcnt0_change_tot	; change-timeout passed ?
;		brne	nFET_off		; .. no
;		ldi	i_temp2, CHANGE_TIMEOUT	; .. yes - change-timeout for more power
;		mov	tcnt0_change_tot, i_temp2 ; reset change-timeout and decrement
;		dec	i_temp1			; <dec> increases power-on-time
;		rjmp	set_next_pwm
;
;lower_pwm:	breq	nFET_off		; pwm is unchanged
;		dec	tcnt0_change_tot	; change-timeout passed ?
;		brne	nFET_off		; .. no
;		ldi	i_temp2, CHANGE_TOT_LOW ; .. yes - change-timeout for lower power
;		mov	tcnt0_change_tot, i_temp2 ; reset change-timeout and increment
;		inc	i_temp1			; <inc> decreases power-on-time
;set_next_pwm:	mov	tcnt0_power_on, i_temp1

nFET_off:	sbr	flags0, (1<<I_OFF_CYCLE) ; PWM state = off cycle

	; switch appropriate nFET off
		sbrs	flags0, C_FET
		rjmp	test_AnFET

; C_FET is active
		sbrs	flags1, FULL_POWER
		CnFET_off		; Cn off
		rjmp	reload_t0_off_cycle

test_AnFET:	sbrs	flags0, A_FET
		rjmp	switch_BnFET

; A_FET is active
switch_AnFET:	sbrs	flags1, FULL_POWER
		AnFET_off		; An off
		rjmp	reload_t0_off_cycle

; B_FET is active
switch_BnFET:	sbrs	flags1, FULL_POWER
		BnFET_off		; Bn off

	; reload timer0 with the appropriate value
reload_t0_off_cycle:
;		ldi	i_temp1, POWER_RANGE
;		add	i_temp1, tcnt0_power_on
		mov	i_temp1, tcnt0_power_on
		com	i_temp1			; timer0 increments
		rjmp	t0_int_exit

; reload timer90 + switch appropriate nFET on
t0_on_cycle:
		cbr	flags0, (1<<I_OFF_CYCLE) ; PWM state = on cycle (no off cycle)
		mov	i_temp1, tcnt0_power_on

; switch appropriate nFET on
nFET_on:	sbrs	flags0, C_FET		; is Cn choppered ?
		rjmp	test_AnFET_on			; .. no - test An
		sbrs	flags1, POWER_OFF
		CnFET_on		; Cn on
		rjmp	eval_power_state
test_AnFET_on:	sbrs	flags0, A_FET		; is An choppered ?
		rjmp	sw_BnFET_on			; .. no - Bn has to be choppered
		sbrs	flags1, POWER_OFF
		AnFET_on		; An on
		rjmp	eval_power_state
sw_BnFET_on:	sbrs	flags1, POWER_OFF
		BnFET_on		; Bn on

	; evaluate power state
eval_power_state:
;		cpi	i_temp1, MAX_POWER+1
;		brsh	not_full_power
		tst	i_temp1
		brne	not_full_power
	; FULL POWER
		sbr	flags1, (1<<FULL_POWER)	; tcnt0_power_on = MAX_POWER means FULL_POWER
		cbr	flags1, (1<<POWER_OFF)
		rjmp	t0_int_exit
not_full_power:	cpi	i_temp1, NO_POWER
		brlo	neither_full_nor_off
	; POWER OFF
		cbr	flags1, (1<<FULL_POWER)	; tcnt0_power_on = NO_POWER means power off
		sbr	flags1, (1<<POWER_OFF)
		rjmp	t0_int_exit
neither_full_nor_off:
		cbr	flags1, (1<<FULL_POWER)	; tcnt0_power_on = MAX_POWER means FULL_POWER
		cbr	flags1, (1<<POWER_OFF)

t0_int_exit:	sbrc	flags2, POFF_CYCLE
		sbr	flags1, (1<<POWER_OFF)
		out	TCNT2, i_temp1		; reload t0
		out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; beeper: timer0 is set to 1µs/count
beep_f1:	ldi	temp4, 200
		ldi	temp2, 80
		rjmp	beep

beep_f2:	ldi	temp4, 180
		ldi	temp2, 100
		rjmp	beep

beep_f3:	ldi	temp4, 160
		ldi	temp2, 120
		rjmp	beep

beep_f4:	ldi	temp4, 140
		ldi	temp2, 140
		rjmp	beep

beep:		clr	temp1
		out	TCNT2, temp1
		BpFET_on		; BpFET on
		AnFET_on		; CnFET on
beep_BpCn10:	in	temp1, TCNT2
		cpi	temp1, 127		; 32µs on (was 32)
		brlo	beep_BpCn10
		BpFET_off		; BpFET off
		AnFET_off		; CnFET off
		ldi	temp3, 64		; 2040µs off (was 8)
beep_BpCn12:	clr	temp1
		out	TCNT2, temp1
beep_BpCn13:	in	temp1, TCNT2
		cp	temp1, temp4
		brlo	beep_BpCn13
		dec	temp3
		brne	beep_BpCn12
		dec	temp2
		brne	beep
		ret

wait30ms:	ldi	temp2, 15
beep_BpCn20:	ldi	temp3, 64	; was 8
beep_BpCn21:	clr	temp1
		out	TCNT2, temp1
		out	TIFR, temp1
beep_BpCn22:	in	temp1, TIFR
		sbrs	temp1, TOV2
		rjmp	beep_BpCn22
		dec	temp3
		brne	beep_BpCn21
		dec	temp2
		brne	beep_BpCn20
		ret

	; 128 periods = 261ms silence
wait260ms:	ldi	temp2, 128
beep2_BpCn20:	ldi	temp3, 64	; was 8
beep2_BpCn21:	clr	temp1
		out	TCNT2, temp1
beep2_BpCn22:	in	temp1, TCNT2
		cpi	temp1, 200
		brlo	beep2_BpCn22
		dec	temp3
		brne	beep2_BpCn21
		dec	temp2
		brne	beep2_BpCn20
		ret
;-----bko-----------------------------------------------------------------
tcnt1_to_temp:	ldi	temp4, EXT0_DIS		; disable ext0int
		out	GIMSK, temp4
		ldi	temp4, T1STOP		; stop timer1
		out	TCCR1B, temp4
		ldi	temp4, T1CK8		; preload temp with restart timer1
		in	temp1, TCNT1L		;  - the preload cycle is needed to complete stop operation
		in	temp2, TCNT1H
		out	TCCR1B, temp4
		ret				; !!! ext0int stays disabled - must be enabled again by caller
	; there seems to be only one TEMP register in the AVR
	; that is used for atomic 16-bit read/write timer operations
	; if the ext0int interrupt falls between readad LOW value while HIGH value is captured in TEMP and
	; read HIGH value, TEMP register is changed in ext_int0 routine,
	; so it must be disabled here (cli is also fine)
;-----bko-----------------------------------------------------------------
evaluate_rc_puls:
		cbr	flags1, (1<<EVAL_RC_PULS)
		sbrs	flags1, RC_PULS_UPDATED
		rjmp	eval_rc_p90
		lds	temp1, new_rcpuls_l
		lds	temp2, new_rcpuls_h
		cbr	flags1, (1<<RC_PULS_UPDATED) ; rc impuls value is read out
		subi	temp1, low  (MIN_RC_PULS)
		sbci	temp2, high (MIN_RC_PULS)
		brcc	eval_rc_p00
		clr	temp1
		clr	temp2
eval_rc_p00:
	; Limit the maximum PPM (800) here since it may wrap
	; when scaled to POWER_RANGE below
		cpi	temp2, high  (816)	; Should be 800
		brlo	eval_rc_p05
		brne	eval_rc_p09
		cpi	temp1, low   (816)	; but see below
		brsh	eval_rc_p09
eval_rc_p05:
;	; This used to shift 0-800 -> 0-200,
;	; but instead we now *8/25, which gives us 0-256.
;	; divide stolen from gcc.
;
;		lsl	temp1
;		rol	temp2
;		lsl	temp1
;		rol	temp2
;		lsl	temp1		; value low (0-6400)
;		rol	temp2		; value high
;		ldi	temp3, 17
;		mov	temp7, temp3	; divide iteration counter
;		ldi	temp3, 25	; r24	divisor low
;		clr	temp4		; r25	divisor high
;		sub	temp5, temp5	; r26
;		sub	temp6, temp6	; r27
;		; carry is cleared now
;		rjmp	__udivmodhi4_ep
;__udivmodhi4_loop:
;		adc	temp5, temp5	; r26
;		adc	temp6, temp6	; r27
;		cp	temp5, temp3	; r26, r22
;		cpc	temp6, temp4	; r27, r23
;		brcs	__udivmodhi4_ep
;		sub	temp5, temp3	; r26, r22
;		sbc	temp6, temp4	; r27, r23
;__udivmodhi4_ep:
;		adc	temp1, temp1	; r24
;		adc	temp2, temp2	; r25
;		dec	temp7
;		brne	__udivmodhi4_loop
;		com	temp1		; r24
;		com	temp2		; r25
;		movw	temp3, temp1	; r22, r24

	; Meh, scale 0-816 -> 0-255 with *5/16, since it seems to be
	; usable anyway, and this is much faster than dividing by 25
		mov	temp3, temp1
		mov	temp4, temp2
		lsl	temp1
		rol	temp2
		lsl	temp1
		rol	temp2
		add	temp1, temp3
		adc	temp2, temp4

		lsr	temp2
		ror	temp1
		lsr	temp2
		ror	temp1
		lsr	temp2
		ror	temp1
		lsr	temp2
		ror	temp1

		mov	temp3, temp1
;		subi	temp1, POWER_RANGE
;		brcs	eval_rc_p10
		rjmp	eval_rc_p10
eval_rc_p09:	ldi	temp3, POWER_RANGE - 1 ; - 1 to allow full 0-255
eval_rc_p10:	mov	ZH, temp3
eval_rc_p90:	ret
;-----bko-----------------------------------------------------------------
evaluate_uart:	cbr	flags1, (1<<EVAL_UART)
		ret
;-----bko-----------------------------------------------------------------
evaluate_sys_state:
		cbr	flags1, (1<<EVAL_SYS_STATE)
		sbrs	flags0, T1OVFL_FLAG
		rjmp	eval_sys_s99

	; do it not more often as every 65µs
		cbr	flags0, (1<<T1OVFL_FLAG)

	; control current
eval_sys_i:	sbrs	flags0, I_pFET_HIGH
		rjmp	eval_sys_i_ok
		cbr	flags0, (1<<I_pFET_HIGH)
		mov	i_temp1, current_err
		cpi	i_temp1, CURRENT_ERR_MAX
		brcc	panic_exit
		inc	current_err
		rjmp	eval_sys_ub

eval_sys_i_ok:	tst	current_err
		breq	eval_sys_ub
		dec	current_err

	; control voltage
eval_sys_ub:	sbrs	flags0, UB_LOW
		rjmp	eval_sys_ub_ok
		cbr	flags0, (1<<UB_LOW)
		mov	i_temp1, sys_control
		cpi	i_temp1, NO_POWER ; POWER_RANGE
		brcc	eval_sys_s99
		inc	sys_control
		rjmp	eval_sys_s99

eval_sys_ub_ok:	tst	sys_control
		breq	eval_sys_s99
		dec	sys_control

eval_sys_s99:	rcall	set_new_duty
		ret

panic_exit:	; !!!!!! OVERCURRENT !!!!!!!!
		cli
		rjmp	reset
;-----bko-----------------------------------------------------------------
evaluate_rpm:	cbr	flags1, (1<<EVAL_RPM)
		lds	temp3, rpm_x
		lds	temp2, rpm_h

		lds	temp1, rpm_l	; subtract 1/256
		sub	temp1, temp2
		sts	rpm_l, temp1
		lds	temp1, rpm_h
		sbc	temp1, temp3
		sts	rpm_h, temp1
		lds	temp1, rpm_x
		sbci	temp1, 0
		sts	rpm_x, temp1

		lds	temp3, timing_acc_x
		lds	temp2, timing_acc_h
		lds	temp1, timing_acc_l
		lsr	temp3		; make one complete commutation cycle
		ror	temp2
		ror	temp1
		lsr	temp3
		ror	temp2
		ror	temp1
	; temp3 is zero now - for sure !!
		sts	timing_acc_x, temp3
		sts	timing_acc_h, temp3
		sts	timing_acc_l, temp3
	; and add the result as 1/256
		lds	temp3, rpm_l
		add	temp3, temp1
		sts	rpm_l, temp3
		lds	temp3, rpm_h
		adc	temp3, temp2
		sts	rpm_h, temp3
		ldi	temp1, 0
		lds	temp3, rpm_x
		adc	temp3, temp1
		sts	rpm_x, temp3

		ret
;-----bko-----------------------------------------------------------------
set_all_timings:
		ldi	YL, low  (timeoutSTART)
		ldi	YH, high (timeoutSTART)
		sts	wt_OCT1_tot_l, YL
		sts	wt_OCT1_tot_h, YH
		ldi	temp3, 0xff
		ldi	temp4, 0x1f
		sts	wt_comp_scan_l, temp3
		sts	wt_comp_scan_h, temp4
		sts	com_timing_l, temp3
		sts	com_timing_h, temp4

set_timing_v:	ldi	ZL, 0x01
		sts	timing_x, ZL
		ldi	temp4, 0xff
		sts	timing_h, temp4
		ldi	temp3, 0xff
		sts	timing_l, temp3

		ret
;-----bko-----------------------------------------------------------------
update_timing:	rcall	tcnt1_to_temp
		sts	tcnt1_sav_l, temp1
		sts	tcnt1_sav_h, temp2
		add	temp1, YL
		adc	temp2, YH
		ldi	temp4, (1<<TOIE1)+(1<<TOIE2)
		out	TIMSK, temp4
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sbr	flags0, (1<<OCT1_PENDING)
		ldi	temp4, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE2) ; enable interrupt again
		out	TIMSK, temp4
		ldi	temp4, EXT0_EN		; ext0int enable
		out	GIMSK, temp4		; enable ext0int

	; calculate next waiting times - timing(-l-h-x) holds the time of 4 commutations
		lds	temp1, timing_l
		lds	temp2, timing_h
		lds	ZL, timing_x

		sts	zero_wt_l, temp1	; save for zero crossing timeout
		sts	zero_wt_h, temp2
		tst	ZL
		breq	update_t00
		ldi	temp4, 0xff
		sts	zero_wt_l, temp4	; save for zero crossing timeout
		sts	zero_wt_h, temp4
update_t00:
		lsr	ZL			; build a quarter
		ror	temp2
		ror	temp1

		lsr	ZL
		ror	temp2
		ror	temp1
		lds	temp3, timing_l		; .. and subtract from timing
		lds	temp4, timing_h
		lds	ZL, timing_x
		sub	temp3, temp1
		sbc	temp4, temp2
		sbci	ZL, 0

		lds	temp1, tcnt1_sav_l	; calculate this commutation time
		lds	temp2, tcnt1_sav_h
		lds	YL, last_tcnt1_l
		lds	YH, last_tcnt1_h
		sts	last_tcnt1_l, temp1
		sts	last_tcnt1_h, temp2
		sub	temp1, YL
		sbc	temp2, YH
		sts	last_com_l, temp1
		sts	last_com_h, temp2

		add	temp3, temp1		; .. and add to timing
		adc	temp4, temp2
		ldi	temp2, 0
		adc	ZL, temp2

	; limit RPM to 120.000
		tst	ZL
		brne	update_t90
		tst	temp4
		breq	update_t10
		cpi	temp4, 1
		brne	update_t90
		cpi	temp3, 0x4c		; 0x14c = 120.000 RPM
		brcc	update_t90
	; set RPM to 120.000
update_t10:	ldi	temp4, 0x01
		ldi	temp3, 0x4c
		tst	run_control
		brne	update_t90		; just active
		ldi	temp1, 0xff		; not active - reactivate
		mov	run_control, temp1

update_t90:	sts	timing_l, temp3
		sts	timing_h, temp4
		sts	timing_x, ZL
		cpi	ZL, 2		; limit range to 0x1ffff
		brcs	update_t99
		rcall	set_timing_v

update_t99:	lds	temp1, timing_acc_l
		add	temp1, temp3
		sts	timing_acc_l, temp1
		lds	temp1, timing_acc_h
		adc	temp1, temp4
		sts	timing_acc_h, temp1
		lds	temp1, timing_acc_x
		adc	temp1, ZL
		sts	timing_acc_x, temp1

		lsr	ZL			; a 16th is the next wait before scan
		ror	temp4
		ror	temp3
		lsr	ZL
		ror	temp4
		ror	temp3
		lsr	ZL
		ror	temp4
		ror	temp3
		lsr	ZL
		ror	temp4
		ror	temp3
		sts	wt_comp_scan_l, temp3
		sts	wt_comp_scan_h, temp4

	; use the same value for commutation timing (15°)
		sts	com_timing_l, temp3
		sts	com_timing_h, temp4

		ret
;-----bko-----------------------------------------------------------------
calc_next_timing:
		lds	YL, wt_comp_scan_l	; holds wait-before-scan value
		lds	YH, wt_comp_scan_h
		rcall	update_timing

		ret

wait_OCT1_tot:	sbrc	flags0, OCT1_PENDING
		rjmp	wait_OCT1_tot

set_OCT1_tot:
;		cbi	ADCSRA, ADEN		; switch to comparator multiplexed
;		in	temp1, SFIOR
;		ori	temp1, (1<<ACME)
;		out	SFIOR, temp1

		lds	YH, zero_wt_h
		lds	YL, zero_wt_l
		rcall	tcnt1_to_temp
		add	temp1, YL
		adc	temp2, YH
		ldi	temp4, (1<<TOIE1)+(1<<TOIE2)
		out	TIMSK, temp4
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sbr	flags0, (1<<OCT1_PENDING)
		ldi	temp4, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE2)
		out	TIMSK, temp4
		ldi	temp4, EXT0_EN		; ext0int enable
		out	GIMSK, temp4		; enable ext0int

		ret
;-----bko-----------------------------------------------------------------
wait_OCT1_before_switch:
		rcall	tcnt1_to_temp
		lds	YL, com_timing_l
		lds	YH, com_timing_h
		add	temp1, YL
		adc	temp2, YH
		ldi	temp3, (1<<TOIE1)+(1<<TOIE2)
		out	TIMSK, temp3
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sbr	flags0, (1<<OCT1_PENDING)
		ldi	temp3, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE2)
		out	TIMSK, temp3
		ldi	temp4, EXT0_EN		; ext0int enable
		out	GIMSK, temp4		; enable ext0int

	; don't waste time while waiting - do some controls, if indicated

		sbrc	flags1, EVAL_RC_PULS
		rcall	evaluate_rc_puls
		sbrc	flags1, EVAL_SYS_STATE
		rcall	evaluate_sys_state

		sbrc	flags1, EVAL_RPM
		rcall	evaluate_rpm

OCT1_wait:	sbrc	flags0, OCT1_PENDING
		rjmp	OCT1_wait
		ret
;-----bko-----------------------------------------------------------------
start_timeout:	lds	YL, wt_OCT1_tot_l
		lds	YH, wt_OCT1_tot_h
		rcall	update_timing

		in	temp1, TCNT1L
		andi	temp1, 0x0f
		sub	YH, temp1
		cpi	YH, high (timeoutMIN)
		brcc	set_tot2
		ldi	YH, high (timeoutSTART)
set_tot2:
		sts	wt_OCT1_tot_h, YH

		rcall	sync_with_poweron	; wait at least 100+ microseconds
		rcall	sync_with_poweron	; for demagnetisation - one sync may be added

		ret
;-----bko-----------------------------------------------------------------
set_new_duty:	mov	temp1, ZH
		sub	temp1, sys_control
		brcc	set_new_duty10
		ldi	temp1, MIN_DUTY-1
set_new_duty10:	lds	temp2, timing_x
		tst	temp2
		brne	set_new_duty12
		lds	temp2, timing_h	; get actual RPM reference high
		cpi	temp2, PWR_RANGE1	; lower range1 ?
		brcs	set_new_duty25		; on carry - test next range
set_new_duty12:	sbr	flags2, (1<<RPM_RANGE1)
		sbr	flags2, (1<<RPM_RANGE2)
		ldi	temp2, PWR_MAX_RPM1	; higher than range1 power max ?
		cp	temp1, temp2
		brcs	set_new_duty31		; on carry - not higher, no restriction
		mov	temp1, temp2		; low (range1) RPM - set PWR_MAX_RPM1
		rjmp	set_new_duty31
set_new_duty25:	cpi	temp2, PWR_RANGE2	; lower range2 ?
		brcs	set_new_duty30		; on carry - not lower, no restriction
		cbr	flags2, (1<<RPM_RANGE1)
		sbr	flags2, (1<<RPM_RANGE2)
		ldi	temp2, PWR_MAX_RPM2	; higher than range2 power max ?
		cp	temp1, temp2
		brcs	set_new_duty31		; on carry - not higher, no restriction
		mov	temp1, temp2		; low (range2) RPM - set PWR_MAX_RPM2
		rjmp	set_new_duty31
set_new_duty30:	cbr	flags2, (1<<RPM_RANGE1)+(1<<RPM_RANGE2)
set_new_duty31: sbrs	flags2, STARTUP		; Check for STARTUP phase
		rjmp	set_new_duty32
		ldi	temp3, PWR_MAX_STARTUP	; limit power in startup phase
		cp	temp1, temp3
		brcs	set_new_duty32		; on carry - not higher, test range 2
		mov	temp1, temp3		; set PWR_MAX_STARTUP limit
set_new_duty32: com	temp1			; down-count to up-count (T0)
		mov	tcnt0_pwron_next, temp1	; save in next
	; tcnt0_power_on is updated to tcnt0_pwron_next in acceptable steps
		ret
;-----bko-----------------------------------------------------------------
switch_power_off:
		ldi	ZH, MIN_DUTY-1		; ZH is new_duty
		ldi	temp1, NO_POWER		; lowest tcnt0_power_on value
		mov	tcnt0_power_on, temp1
		mov	tcnt0_pwron_next, temp1
		ldi	temp1, INIT_PB		; all off
		out	PORTB, temp1
		ldi	temp1, INIT_PD		; all off
		out	PORTD, temp1
;		ldi	temp1, CHANGE_TIMEOUT	; reset change-timeout
;		mov	tcnt0_change_tot, temp1
		sbr	flags1, (1<<POWER_OFF)	; disable power on
		cbr	flags2, (1<<POFF_CYCLE)
		sbr	flags2, (1<<STARTUP)
		ret				; motor is off
;-----bko-----------------------------------------------------------------
wait_if_spike:	ldi	temp1, 4
wait_if_spike2:	dec	temp1
		brne	wait_if_spike2
		ret
;-----bko-----------------------------------------------------------------
sync_with_poweron:
		sbrc	flags0, I_OFF_CYCLE	; first wait for power on
		rjmp	sync_with_poweron
wait_for_poweroff:
		sbrs	flags0, I_OFF_CYCLE	; now wait for power off
		rjmp	wait_for_poweroff

		ret
;-----bko-----------------------------------------------------------------
motor_brake:
.if MOT_BRAKE == 1
mot_brk10:
		ldi	temp1, INIT_PB		; all off
		in	temp1, tcnt1l
		sbrs	temp1, 6
		ldi	temp1, BRAKE_PB		; all N-FETs on
		out	PORTB, temp1
		rcall	evaluate_rc_puls
		cpi	ZH, MIN_DUTY+3		; avoid jitter detect
		brcs	mot_brk10
		ldi	temp1, INIT_PB		; all off
		out	PORTB, temp1
		ldi	temp1, INIT_PD		; all off
		out	PORTD, temp1
.endif	; MOT_BRAKE == 1
		ret

;-----bko-----------------------------------------------------------------
; **** startup loop ****
init_startup:	rcall	switch_power_off
wait_for_power_on:
		rcall	motor_brake
		rcall	evaluate_rc_puls

		cpi	ZH, MIN_DUTY
		brcs	wait_for_power_on

		ldi	temp1, PWR_STARTUP	; begin startup with low power
		com	temp1
		mov	tcnt0_pwron_next, temp1

		cbi	ADCSRA, ADEN		; switch to comparator multiplexed
		in	temp1, SFIOR
		sbr	temp1, (1<<ACME)
		out	SFIOR, temp1

		clr	temp4
		ldi	temp1, INIT_PB		; all off
		out	PORTB, temp1
		ldi	temp1, INIT_PD		; all off
		out	PORTD, temp1
		ldi	temp1, 27		; wait about 5mikosec
FETs_off_wt:	dec	temp1
		brne	FETs_off_wt

		rcall	com5com6
		rcall	com6com1

		cbr	flags2, (1<<SCAN_TIMEOUT)
		ldi	temp1, 0
		sts	goodies, temp1

		ldi	temp1, 40	; x 65msec
		mov	t1_timeout, temp1

		rcall	set_all_timings

		rcall	start_timeout

	; fall through start1

;-----bko-----------------------------------------------------------------
; **** start control loop ****

; state 1 = B(p-on) + C(n-choppered) - comparator A evaluated
; out_cA changes from low to high
start1:		sbrs	flags2, COMP_SAVE	; high ?
		rjmp	start1_2		; .. no - loop, while high

start1_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start1_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start1_9
start1_1:	rcall	sync_with_poweron

		sbrc	flags2, COMP_SAVE	; high ?
		rjmp	start1_0		; .. no - loop, while high

; do the special 120° switch
		ldi	temp1, 0
		sts	goodies, temp1
		rcall	com1com2
		rcall	com2com3
		rcall	com3com4

		rcall	evaluate_rc_puls

		rcall	evaluate_uart
		rcall	start_timeout
		rjmp	start4

start1_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start1_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start1_9
start1_3:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE	; high ?
		rjmp	start1_2		; .. no - loop, while low

start1_9:
		rcall	com1com2
		rcall	start_timeout

; state 2 = A(p-on) + C(n-choppered) - comparator B evaluated
; out_cB changes from high to low

start2:		sbrc	flags2, COMP_SAVE
		rjmp	start2_2

start2_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start2_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start2_9
start2_1:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start2_0
		rjmp	start2_9

start2_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start2_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start2_9
start2_3:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start2_2

start2_9:
		rcall	com2com3

		rcall	evaluate_rc_puls

		rcall	start_timeout

; state 3 = A(p-on) + B(n-choppered) - comparator C evaluated
; out_cC changes from low to high

start3:		sbrs	flags2, COMP_SAVE
		rjmp	start3_2

start3_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start3_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start3_9
start3_1:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start3_0
		rjmp	start3_9

start3_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start3_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start3_9
start3_3:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start3_2

start3_9:
		rcall	com3com4
		rcall	evaluate_uart
		rcall	start_timeout

; state 4 = C(p-on) + B(n-choppered) - comparator A evaluated
; out_cA changes from high to low

start4:		sbrc	flags2, COMP_SAVE
		rjmp	start4_2

start4_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start4_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start4_9
start4_1:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start4_0
		rjmp	start4_9

start4_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start4_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start4_9
start4_3:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start4_2

start4_9:
		rcall	com4com5
		rcall	start_timeout


; state 5 = C(p-on) + A(n-choppered) - comparator B evaluated
; out_cB changes from low to high


start5:		sbrs	flags2, COMP_SAVE
		rjmp	start5_2

start5_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start5_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start5_9
start5_1:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start5_0
		rjmp	start5_9

start5_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start5_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start5_9
start5_3:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start5_2

start5_9:
		rcall	com5com6
		rcall	evaluate_sys_state
		rcall	start_timeout

; state 6 = B(p-on) + A(n-choppered) - comparator C evaluated
; out_cC changes from high to low

start6:		sbrc	flags2, COMP_SAVE
		rjmp	start6_2

start6_0:	sbrc	flags0, OCT1_PENDING
		rjmp	start6_1
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start6_9
start6_1:	rcall	sync_with_poweron
		sbrs	flags2, COMP_SAVE
		rjmp	start6_0
		rjmp	start6_9

start6_2:	sbrc	flags0, OCT1_PENDING
		rjmp	start6_3
		sbr	flags2, (1<<SCAN_TIMEOUT)
		rjmp	start6_9
start6_3:	rcall	sync_with_poweron
		sbrc	flags2, COMP_SAVE
		rjmp	start6_2

start6_9:
		rcall	com6com1

		mov	temp1, tcnt0_power_on
		cpi	temp1, NO_POWER
		brne	s6_power_ok
		rjmp	init_startup

s6_power_ok:
		tst	rcpuls_timeout
		brne	s6_rcp_ok
		rjmp	restart_control

s6_rcp_ok:	tst	t1_timeout
		brne	s6_test_rpm
		rjmp	init_startup

s6_test_rpm:	lds	temp1, timing_x
		tst	temp1
		brne	s6_goodies
		lds	temp1, timing_h		; get actual RPM reference high
		cpi	temp1, PWR_RANGE_RUN
;		cpi	temp1, PWR_RANGE1
;		cpi	temp1, PWR_RANGE2
		brcs	s6_run1

s6_goodies:	lds	temp1, goodies
		sbrc	flags2, SCAN_TIMEOUT
		clr	temp1
		inc	temp1
		sts	goodies,  temp1
		cbr	flags2, (1<<SCAN_TIMEOUT)
		cpi	temp1, ENOUGH_GOODIES
		brcs	s6_start1

s6_run1:	ldi	temp1, 0xff
		mov	run_control, temp1

		rcall	calc_next_timing
		rcall	set_OCT1_tot

		cbr	flags2, (1<<STARTUP)
		cbr	flags2, (1<<GP_FLAG)	; OCT1-timeout
		rjmp	run1			; running state begins

s6_start1:	rcall	start_timeout		; need to be here for a correct temp1=comp_state
		rjmp	start1			; go back to state 1

;-----bko-----------------------------------------------------------------
; **** running control loop ****

; run 1 = B(p-on) + C(n-choppered) - comparator A evaluated
; out_cA changes from low to high

run1:		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		sbr	flags1, (1<<EVAL_RPM)
		rcall	wait_OCT1_before_switch
		rcall	com1com2
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 2 = A(p-on) + C(n-choppered) - comparator B evaluated
; out_cB changes from high to low

run2:		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		sbr	flags1, (1<<EVAL_RC_PULS)
		rcall	wait_OCT1_before_switch
		rcall	com2com3
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 3 = A(p-on) + B(n-choppered) - comparator C evaluated
; out_cC changes from low to high

run3:		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		sbr	flags1, (1<<EVAL_UART)
		rcall	wait_OCT1_before_switch
		rcall	com3com4
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 4 = C(p-on) + B(n-choppered) - comparator A evaluated
; out_cA changes from high to low
run4:		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		rcall	wait_OCT1_before_switch
		rcall	com4com5
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 5 = C(p-on) + A(n-choppered) - comparator B evaluated
; out_cB changes from low to high

run5:		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		sbr	flags1, (1<<EVAL_SYS_STATE)
		rcall	wait_OCT1_before_switch
		rcall	com5com6
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 6 = B(p-on) + A(n-choppered) - comparator C evaluated
; out_cC changes from high to low

run6:		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start
		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		rcall	wait_OCT1_before_switch
		rcall	com6com1
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

		tst	rcpuls_timeout
		breq	restart_control

		lds	temp1, timing_x
		tst	temp1
		breq	run6_2			; higher than 610 RPM if zero
		sbr	flags2, (1<<GP_FLAG)	; mark low RPM
run_to_start:	sbr	flags2, (1<<STARTUP)
		cbr	flags2, (1<<POFF_CYCLE)
		rjmp	wait_for_power_on

run6_2:		cbr	flags2, (1<<POFF_CYCLE)
		tst	run_control		; only once !
		breq	run6_9
		dec	run_control
		breq	run6_3			; poweroff if 0
		mov	temp1, run_control
		cpi	temp1, 1		; poweroff if 1
		breq	run6_3
		cpi	temp1, 2		; poweroff if 2
		brne	run6_9
run6_3:		sbr	flags2, (1<<POFF_CYCLE)

run6_9:

		rjmp	run1			; go back to run 1

restart_control:
		cli				; disable all interrupts
		rcall	switch_power_off
		; reset rc puls timeout
		ldi	temp1, RCP_TOT
		mov	rcpuls_timeout, temp1
		rcall	beep_f3
		rcall	beep_f2
		rcall	wait30ms
		sei
		rcall	set_all_timings
		rjmp	init_startup
		; Used to be: rjmp	reset

;-----bko-----------------------------------------------------------------
; *** scan comparator utilities ***
;
wait_for_low:	sbrs	flags0, OCT1_PENDING
		ret
		sbis	ACSR, ACO		; low ?
		rjmp	wait_for_low		; .. no - loop, while high
		rcall	wait_if_spike		; .. yes - look for a spike
		sbis	ACSR, ACO		; test again
		rjmp	wait_for_low		; .. is high again, was a spike
		ret

wait_for_high:	sbrs	flags0, OCT1_PENDING
		ret
		sbic	ACSR, ACO		; high ?
		rjmp	wait_for_high		; .. no - loop, while low
		rcall	wait_if_spike		; .. yes - look for a spike
		sbic	ACSR, ACO		; test again
		rjmp	wait_for_high		; .. is low again, was a spike
		ret
;-----bko-----------------------------------------------------------------
; *** commutation utilities ***
com1com2:	BpFET_off		; Bp off
		sbrs	flags1, POWER_OFF
		ApFET_on		; Ap on
		ldi	temp1, mux_b		; set comparator multiplexer to phase B
		out	ADMUX, temp1
		cbi	ADCSRA, ADEN		; disable ADC
		in	temp1, SFIOR
		sbr	temp1, (1<<ACME)	; switch to comparator multiplexed
		out	SFIOR, temp1
		ret

com2com3:	ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		cbr	flags0, (1<<A_FET)	; next nFET = BnFET
		cbr	flags0, (1<<C_FET)
		sbrc	flags1, FULL_POWER
		rjmp	c2_switch
		sbrc	flags0, I_OFF_CYCLE	; was power off ?
		rjmp	c2_done			; .. yes - futhermore work is done in timer0 interrupt
c2_switch:	CnFET_off		; Cn off
		sbrs	flags1, POWER_OFF
		BnFET_on		; Bn on
c2_done:	ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE2) ; let timer0 do his work again
		out	TIMSK, temp1
		in	temp1, SFIOR
		cbr	temp1, (1<<ACME)	; set to AN1
		out	SFIOR, temp1
		sbi	ADCSRA, ADEN		; enable ADC
		ret

com3com4:	ApFET_off		; Ap off
		sbrs	flags1, POWER_OFF
		CpFET_on		; Cp on
		ldi	temp1, mux_a		; set comparator multiplexer to phase A
		out	ADMUX, temp1
		cbi	ADCSRA, ADEN		; disable ADC
		in	temp1, SFIOR
		sbr	temp1, (1<<ACME)	; switch to comparator multiplexed
		out	SFIOR, temp1
		ret

com4com5:	ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		sbr	flags0, (1<<A_FET)	; next nFET = AnFET
		cbr	flags0, (1<<C_FET)
		sbrc	flags1, FULL_POWER
		rjmp	c4_switch
		sbrc	flags0, I_OFF_CYCLE	; was power off ?
		rjmp	c4_done			; .. yes - futhermore work is done in timer0 interrupt
c4_switch:	BnFET_off		; Bn off
		sbrs	flags1, POWER_OFF
		AnFET_on		; An on
c4_done:	ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE2) ; let timer0 do his work again
		out	TIMSK, temp1
		ldi	temp1, mux_b		; set comparator multiplexer to phase B
		out	ADMUX, temp1
		cbi	ADCSRA, ADEN		; disable ADC
		in	temp1, SFIOR
		sbr	temp1, (1<<ACME)	; switch to comparator multiplexed
		out	SFIOR, temp1
		ret

com5com6:	CpFET_off		; Cp off
		sbrs	flags1, POWER_OFF
		BpFET_on		; Bp on
		in	temp1, SFIOR
		cbr	temp1, (1<<ACME)	; set to AN1
		out	SFIOR, temp1
		sbi	ADCSRA, ADEN		; enable ADC
		ret

com6com1:	ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		cbr	flags0, (1<<A_FET)	; next nFET = CnFET
		sbr	flags0, (1<<C_FET)
		sbrc	flags1, FULL_POWER
		rjmp	c6_switch
		sbrc	flags0, I_OFF_CYCLE	; was power off ?
		rjmp	c6_done			; .. yes - futhermore work is done in timer0 interrupt
c6_switch:	AnFET_off		; An off
		sbrs	flags1, POWER_OFF
		CnFET_on		; Cn on
c6_done:	ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE2) ; let timer0 do his work again
		out	TIMSK, temp1
		ldi	temp1, mux_a		; set comparator multiplexer to phase A
		out	ADMUX, temp1
		cbi	ADCSRA, ADEN		; disable ADC
		in	temp1, SFIOR
		sbr	temp1, (1<<ACME)	; switch to comparator multiplexed
		out	SFIOR, temp1
		ret

.exit
