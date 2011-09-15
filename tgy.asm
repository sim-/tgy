;**** **** **** **** ****
;
;Die Benutzung der Software ist mit folgenden Bedingungen verbunden:
;
;1. Da ich alles kostenlos zur Verf�gung stelle, gebe ich keinerlei Garantie
;   und �bernehme auch keinerlei Haftung f�r die Folgen der Benutzung.
;
;2. Die Software ist ausschlie�lich zur privaten Nutzung bestimmt. Ich
;   habe nicht gepr�ft, ob bei gewerblicher Nutzung irgendwelche Patentrechte
;   verletzt werden oder sonstige rechtliche Einschr�nkungen vorliegen.
;
;3. Jeder darf �nderungen vornehmen, z.B. um die Funktion seinen Bed�rfnissen
;   anzupassen oder zu erweitern. Ich w�rde mich freuen, wenn ich weiterhin als
;   Co-Autor in den Unterlagen erscheine und mir ein Link zur entprechenden Seite
;   (falls vorhanden) mitgeteilt wird.
;
;4. Auch nach den �nderungen sollen die Software weiterhin frei sein, d.h. kostenlos bleiben.
;
;!! Wer mit den Nutzungbedingungen nicht einverstanden ist, darf die Software nicht nutzen !!
;
; tp-18a
; October 2004
; autor: Bernhard Konze
; email: bernhard.konze@versanet.de
;--
; Forked from "TGYP2010_416HzRCIntervalRate_PPM-Mod-only_NoCal" which removed
; OSCCAL setting from EEPROM and tweaked some startup parameters, all which
; have since been replaced.
;
; NO WARRANTY EXPRESSED OR IMPLIED. USE AT YOUR OWN RISK. Always test
; without propellers! Please respect Bernhard Konze's license above.
;--
; WARNING: I have blown FETs on Turnigy Plush 18A ESCs in previous versions
; of this code with my modifications. Some bugs have since been fixed, such
; as leaving PWM enabled while busy-looping forever outside of ISR code.
; However, this does run with higher PWM frequency than most original code,
; so higher FET temperatures may occur! USE AT YOUR OWN RISK, and maybe see
; how it compares and let me know!
;
; WARNING: This does not check temperature or voltage ADC inputs.
;
; NOTE: This version is using hardware 16-bit capable PWM mode on timer2,
; using a tcnt2h register to simulate the high byte, and so can operate at
; any frequency with the resolution of the clock. POWER_RANGE set to 400
; gives approximately 18kHz PWM output frequency due to cycles used in the
; interrupt before reloading TCNT2. This should allow compatibilty with 8MHz
; or 16MHz main clocks with minor adjustments.
;
; Simon Kirby <sim@simulated.ca>
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

.include "tgy.inc"		; TowerPro/Turnigy Basic/Plush (INT0 PPM)
;.include "afro.inc"		; AfroESC (ICP PPM)
;.include "bs.inc"		; HobbyKing BlueSeries *UNTESTED* (INT0 PPM)
;.include "bs_nfet.inc"		; HobbyKing BlueSeries with all nFETs (INT0 PPM)

.equ	MOT_BRAKE   	= 0	; Enable brake
.equ	RC_PULS 	= 1	; Enable PPM ("RC pulse") mode
.equ	RCP_TOT		= 32	; Number of timer1 overflows before considering rc pulse lost

.equ	MIN_RC_PULS	= 800	; Less than this is illegal pulse length
.equ	MAX_RC_PULS	= 2200	; More than this is illegal pulse length
.equ	STOP_RC_PULS	= 1060	; Stop motor at or below this pulse length

.equ	POWER_RANGE	= 400	; Number of PWM steps (if adjusted, see scaling in evaluate_rc_puls)
.equ	MIN_DUTY	= 12	; Minimum duty before starting when stopped
.equ	MAX_POWER	= (POWER_RANGE-1)

.equ	PWR_RANGE_RUN	= 0x20	; 2048 microseconds per commutation
.equ	PWR_RANGE1	= 0x40	; 4096 microseconds per commutation
.equ	PWR_RANGE2	= 0x20	; 2048 microseconds per commutation

.equ	PWR_MAX_RPM1	= (POWER_RANGE/4) ; Power limit when running slower than PWR_RANGE1
.equ	PWR_MAX_RPM2	= (POWER_RANGE/2) ; Power limit when running slower than PWR_RANGE2

.equ	timeoutSTART	= 48000	; ~833 RPM
.equ	timeoutMIN	= 36000	; ~1111 RPM

;**** **** **** **** ****
; Register Definitions
.def	zero		= r0		; stays at 0
.def	i_sreg		= r1		; status register save in interrupts
.def	duty_l		= r2		; on duty cycle low, one's complement
.def	duty_h		= r3		; on duty cycle high
.def	com_duty_l 	= r4		; off duty cycle low, one's complement
.def	com_duty_h	= r5		; off duty cycle high
.def	start_rcpuls_l	= r6
.def	start_rcpuls_h	= r7
.def	tcnt2h		= r8
;.def		 	= r9
.def	uart_cnt	= r10
.def	rcpuls_timeout	= r11
.def	sys_control_l	= r12		; duty limit low (word register aligned)
.def	sys_control_h	= r13		; duty limit high
.def	acsr_save	= r14		; saved ACSR register value

.def	temp1		= r16		; main temporary (L)
.def	temp2		= r17		; main temporary (H)
.def	temp3		= r18		; main temporary (L)
.def	temp4		= r19		; main temporary (H)
.def	temp5		= r9		; aux temporary (limited operations)

.def	i_temp1		= r20		; interrupt temporary
.def	i_temp2		= r15		; interrupt temporary (limited operations)
.def	i_temp3		= r22		; interrupt temporary

.def	flags0	= r23	; state flags
	.equ	OCT1_PENDING	= 0	; if set, output compare interrunpt is pending
;	.equ	UB_LOW 		= 1	; set if accu voltage low
;	.equ	I_pFET_HIGH	= 2	; set if over-current detect
;	.equ	GET_STATE	= 3	; set if state is to be send
;	.equ	C_FET		= 4	; if set, C-FET state is to be changed
;	.equ	A_FET		= 5	; if set, A-FET state is to be changed
	     ; if neither 1 nor 2 is set, B-FET state is to be changed
	.equ	I_FET_ON	= 6	; if set, fets off
	.equ	I_ON_CYCLE	= 7	; if set, current on cycle is active (optimized as MSB)

.def	flags1	= r24	; state flags
	.equ	POWER_OFF	= 0	; switch fets on disabled
	.equ	FULL_POWER	= 1	; 100% on - don't switch off, but do OFF_CYCLE working
;	.equ	CALC_NEXT_OCT1	= 2	; calculate OCT1 offset, when wait_OCT1_before_switch is called
	.equ	RC_PULS_UPDATED	= 3	; new rc-puls value available
;	.equ	EVAL_RC_PULS	= 4	; if set, new rc puls is evaluated, while waiting for OCT1
;	.equ	EVAL_SYS_STATE	= 5	; if set, overcurrent and undervoltage are checked
;	.equ	EVAL_RPM	= 6	; if set, next PWM on should look for current
;	.equ	EVAL_UART	= 7	; if set, next PWM on should look for uart

.def	flags2	= r25
;	.equ	RPM_RANGE1	= 0	; if set RPM is lower than 1831 RPM
;	.equ	RPM_RANGE2	= 1	; if set RPM is lower than 3662 RPM
	.equ	RC_INTERVAL_OK	= 2
;	.equ	POFF_CYCLE	= 3	; if set one commutation cycle is performed without power
;	.equ	COMP_SAVE	= 4	; if set ACO was high
;	.equ	COMP_SAVE_READY	= 5	; if acsr_save was set by PWM interrupt
;	.equ	STARTUP		= 6	; if set startup-phase is active
	.equ	SCAN_TIMEOUT	= 7	; if set a startup timeout occurred

; here the XYZ registers are placed ( r26-r31)

; X: general temporary
; Y: general temporary
; Z: interrupt-accessed address of current PWM FET ON routine (eg: pwm_afet_on)

;**** **** **** **** ****
; RAM Definitions
.dseg				; DATA segment
.org SRAM_START

last_tcnt1_l:	.byte	1	; last timer1 value
last_tcnt1_h:	.byte	1
timing_l:	.byte	1	; holds time of 4 commutations
timing_h:	.byte	1
timing_x:	.byte	1

wt_comp_scan_l:	.byte	1	; time from switch to comparator scan
wt_comp_scan_h:	.byte	1
com_timing_l:	.byte	1	; time from zero-crossing to switch of the appropriate FET
com_timing_h:	.byte	1
wt_OCT1_tot_l:	.byte	1	; OCT1 waiting time
wt_OCT1_tot_h:	.byte	1
zero_wt_l:	.byte	1
zero_wt_h:	.byte	1

stop_rcpuls_l:	.byte	1
stop_rcpuls_h:	.byte	1
new_rcpuls_l:	.byte	1
new_rcpuls_h:	.byte	1
rc_duty_l:	.byte	1	; desired duty cycle
rc_duty_h:	.byte	1

;duty_offset:	.byte	1
goodies:	.byte	1
comp_state:	.byte	1
gp_cnt:		.byte	1

uart_data:	.byte	100	; only for debug requirements


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
		rjmp reset
		rjmp_ext_int0	; ext_int0
		nop		; ext_int1
		nop		; t2oc_int
		rjmp t2ovfl_int	; t2ovfl_int
		rjmp_icp1_int	; icp1_int
		rjmp t1oca_int
		nop		; t1ocb_int
		rjmp t1ovfl_int
		nop		; t0ovfl_int
		nop		; spi_int
		nop		; urxc
		nop		; udre
		nop		; utxc

; not used	nop		; adc_int
; not used	nop		; eep_int
; not used	nop		; aci_int
; not used	nop		; wire2_int
; not used	nop		; spmc_int

;-----bko-----------------------------------------------------------------
; init after reset

reset:
		clr	zero

		ldi	temp1, high(RAMEND)	; stack = RAMEND
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

	; clear all registers r0 through r26
		ldi	XH, high(27)
		ldi	XL, low (27)
clear_regs:	st	-X, zero
		tst	XL
		brne	clear_regs

	; clear RAM
		ldi	XH, high(SRAM_START)
		ldi	XL, low (SRAM_START)
clear_ram:	st	X+, zero
		cpi	XL, uart_data+1
		brlo	clear_ram

	; power off (sys_control is zero)
		rcall	switch_power_off
		rcall	set_new_duty

	; timer0: clk/1 for beep control and waiting
		ldi	temp1, (1<<CS00)
		out	TCCR0, temp1

	; timer1: clk/8 for commutation control, RC pulse measurement
		ldi	temp1, (1<<CS11)+T1ICP
		out	TCCR1B, temp1

	; timer2: clk/1 for output PWM
		ldi	temp1, (1<<CS20)
		out	TCCR2, temp1

	; startup beeps
		rcall	wait260ms	; wait a while

		rcall	beep_f1
		rcall	wait30ms
		rcall	beep_f2
		rcall	wait30ms
		rcall	beep_f3
		rcall	wait30ms

		; status led on
		GRN_on

control_start:
	; init registers and interrupts
		ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<OCIE2)
		out	TIFR, temp1		; clear TOIE1,OCIE1A & OCIE2
		out	TIMSK, temp1		; enable TOIE1,OCIE1A & OCIE2 interrupts

		sei				; enable all interrupts

; init rc-puls
		rcp_int_rising_edge temp1
		rcp_int_enable temp1
i_rc_puls1:	ldi	temp3, 10		; wait for this count of receiving power off
i_rc_puls2:	sbrs	flags1, RC_PULS_UPDATED
		rjmp	i_rc_puls2
		lds	temp1, new_rcpuls_l
		lds	temp2, new_rcpuls_h
		cbr	flags1, (1<<RC_PULS_UPDATED) ; rc impuls value is read out
		subi	temp1, low  (MIN_RC_PULS) ; valid RC pulse?
		sbci	temp2, high (MIN_RC_PULS)
		brcs	i_rc_puls1		; no - reset counter
		subi	temp1, low  (STOP_RC_PULS - MIN_RC_PULS) ; power off received?
		sbci	temp2, high (STOP_RC_PULS - MIN_RC_PULS)
		brcc	i_rc_puls1		; no - reset counter
		dec	temp3			; yes - decrement counter
		brne	i_rc_puls2		; repeat until zero
		cli				; disable all interrupts
		rcall	beep_f4			; signal: rcpuls ready
		rcall	beep_f4
		rcall	beep_f4
		sei				; enable all interrupts

		rjmp	init_startup

;-----bko-----------------------------------------------------------------
; NOTE: This interrupt uses the 16-bit atomic timer read/write register
; by reading TCNT1L and TCNT1H, so this interrupt must be disabled before
; any other 16-bit timer options happen that might use the same register
; (see "Accessing 16-bit registers" in the Atmel documentation)
.if USE_ICP
; icp1 = rc pulse input, if enabled
icp1_int:	in	i_temp3, TCCR1B
		in	i_sreg, SREG

; get captured timer values
		in	i_temp1, ICR1L
		in	i_temp2, ICR1H

; evaluate edge of this interrupt
		sbrs	i_temp3, ICES1

.else
;-----bko-----------------------------------------------------------------
ext_int0:
		in	i_temp3, PIND
		in	i_sreg, SREG

; get timer1 values
		in	i_temp1, TCNT1L
		in	i_temp2, TCNT1H

; evaluate edge of this interrupt
		sbrs	i_temp3, rcp_in
.endif

		rjmp	falling_edge		; bit is clear = falling edge
; rc impuls is at high state
		rcp_int_falling_edge i_temp3	; set next int to falling edge

		mov	start_rcpuls_l, i_temp1
		mov	start_rcpuls_h, i_temp2
; test rcpulse low interval
		cbr	flags2, (1<<RC_INTERVAL_OK) ; preset to not ok
		lds	i_temp3, stop_rcpuls_l
		sub	i_temp1, i_temp3
		lds	i_temp3, stop_rcpuls_h
		sbc	i_temp2, i_temp3
		cpi	i_temp1, low (5)	; 200 ok for 417Hz, 5 for 495Hz
		ldi	i_temp3, high(5)	; test range low
		cpc	i_temp2, i_temp3
		brlo	rcpint_fail		; throw away
		sbr	flags2, (1<<RC_INTERVAL_OK) ; set to rc impuls value is ok !
		rjmp	rcpint_exit

rcpint_fail:	cpse	rcpuls_timeout, zero
		dec	rcpuls_timeout
		rjmp	rcpint_exit

; rc impuls is at low state
falling_edge:
		rcp_int_rising_edge i_temp3	; set next int to rising edge
		sbrc	flags1, RC_PULS_UPDATED
		rjmp	rcpint_exit

; get timer1 values
		sts	stop_rcpuls_l, i_temp1	; prepare next interval evaluation
		sts	stop_rcpuls_h, i_temp2

		sbrs	flags2, RC_INTERVAL_OK
		rjmp	rcpint_exit
		cbr	flags2, (1<<RC_INTERVAL_OK) ; flag is evaluated

		sub	i_temp1, start_rcpuls_l
		sbc	i_temp2, start_rcpuls_h

; save impuls length
		sts	new_rcpuls_l, i_temp1
		sts	new_rcpuls_h, i_temp2
		cpi	i_temp1, low (MAX_RC_PULS)
		ldi	i_temp3, high(MAX_RC_PULS)	; test range high
		cpc	i_temp2, i_temp3
		brsh	rcpint_fail			; throw away
		cpi	i_temp1, low (MIN_RC_PULS)
		ldi	i_temp3, high(MIN_RC_PULS)	; test range low
		cpc	i_temp2, i_temp3
		brlo	rcpint_fail			; throw away
		sbr	flags1, (1<<RC_PULS_UPDATED) ; set to rc impuls value is ok !
		mov	i_temp1, rcpuls_timeout
		cpi	i_temp1, RCP_TOT
		adc	rcpuls_timeout, zero
rcpint_exit:	out	SREG, i_sreg
		reti

;-----bko-----------------------------------------------------------------
; timer output compare interrupt
t1oca_int:	in	i_sreg, SREG
		cbr	flags0, (1<<OCT1_PENDING) ; signal OCT1 passed
		out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; timer1 overflow interrupt (happens every 65536�s)
t1ovfl_int:	in	i_sreg, SREG
		cpse	rcpuls_timeout, zero
		dec	rcpuls_timeout
pwm_exit:	out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; timer2 overflow compare interrupt (output PWM)
t2ovfl_int:
		in	i_sreg, SREG
		dec	tcnt2h			; decrement tcnt2 "high byte"
		brpl	pwm_exit		; exit if >= 0
		subi	flags0, (1<<I_ON_CYCLE)	; fast toggle of msb
		brmi	pwm_off			; fast branch on msb

; ON cycle: switch appropriate nFET on as soon as possible
pwm_on:
		sbrs	flags1, POWER_OFF
		ijmp	; Z should be set to one of the below labels
pwm_off:
		in	acsr_save, ACSR
		sbrc	flags1, FULL_POWER
		rjmp	pwm_off_exit
		cbr	flags0, (1<<I_FET_ON)
	; We can just turn all nFETs off as we only have one nFET on at a
	; time, and interrupts are disabled during beeps. Doing this with
	; three cbi instructions is 6 cycles, while in/cbr/out is 3 cycles.
		all_nFETs_off	i_temp1
pwm_off_exit:
		mov	tcnt2h, com_duty_h
		out	SREG, i_sreg
		out	TCNT2, com_duty_l
		reti
pwm_afet_on:
		BnFET_off
		AnFET_on
		sbr	flags0, (1<<I_FET_ON)
		mov	tcnt2h, duty_h
		out	SREG, i_sreg
		out	TCNT2, duty_l
		reti
pwm_bfet_on:
		CnFET_off
		BnFET_on
		sbr	flags0, (1<<I_FET_ON)
		mov	tcnt2h, duty_h
		out	SREG, i_sreg
		out	TCNT2, duty_l
		reti
pwm_cfet_on:
		AnFET_off
		CnFET_on
		sbr	flags0, (1<<I_FET_ON)
		mov	tcnt2h, duty_h
		out	SREG, i_sreg
		out	TCNT2, duty_l
		reti


;-----bko-----------------------------------------------------------------
; beeper: timer0 is set to 1�s/count
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
		out	TCNT0, temp1
		BpFET_on		; BpFET on
		AnFET_on		; CnFET on
beep_BpCn10:	in	temp1, TCNT0
		cpi	temp1, 127		; 32�s on (was 32)
		brlo	beep_BpCn10
		BpFET_off		; BpFET off
		AnFET_off		; CnFET off
		ldi	temp3, 64		; 2040�s off (was 8)
beep_BpCn12:	clr	temp1
		out	TCNT0, temp1
beep_BpCn13:	in	temp1, TCNT0
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
		out	TCNT0, temp1
		out	TIFR, temp1
beep_BpCn22:	in	temp1, TIFR
		sbrs	temp1, TOV0
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
		out	TCNT0, temp1
beep2_BpCn22:	in	temp1, TCNT0
		cpi	temp1, 200
		brlo	beep2_BpCn22
		dec	temp3
		brne	beep2_BpCn21
		dec	temp2
		brne	beep2_BpCn20
		ret
;-----bko-----------------------------------------------------------------
evaluate_rc_puls:
		sbrs	flags1, RC_PULS_UPDATED
		ret
		lds	temp1, new_rcpuls_l
		lds	temp2, new_rcpuls_h
		cbr	flags1, (1<<RC_PULS_UPDATED) ; rc impuls value is read out
		subi	temp1, low  (STOP_RC_PULS)
		sbci	temp2, high (STOP_RC_PULS)
		brcc	eval_rc_nonzero
		clr	temp1
		clr	temp2
eval_rc_nonzero:
		cpi	temp1, low (POWER_RANGE*2)
		ldi	temp3, high(POWER_RANGE*2)
		cpc	temp2, temp3
		brlo	eval_rc_not_full
		ldi	temp1, low (POWER_RANGE*2)
		ldi	temp2, high(POWER_RANGE*2)
eval_rc_not_full:
		lsr	temp2
		ror	temp1
		sts	rc_duty_l, temp1
		sts	rc_duty_h, temp2
		rjmp	set_new_duty
;-----bko-----------------------------------------------------------------
;evaluate_uart:	cbr	flags1, (1<<EVAL_UART)
;		ret
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

set_timing_v:	ldi	temp4, 0x01
		mov	temp5, temp4
		sts	timing_x, temp5
		ldi	temp4, 0xff
		sts	timing_h, temp4
		ldi	temp3, 0xff
		sts	timing_l, temp3

		ret
;-----bko-----------------------------------------------------------------
update_timing:	cli
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		add	YL, temp1
		adc	YH, temp2
		out	OCR1AH, YH
		out	OCR1AL, YL
		sei
		sbr	flags0, (1<<OCT1_PENDING)

	; calculate this commutation time
		lds	temp3, last_tcnt1_l
		lds	temp4, last_tcnt1_h
		sts	last_tcnt1_l, temp1
		sts	last_tcnt1_h, temp2
		sub	temp1, temp3
		sbc	temp2, temp4

	; calculate next waiting times - timing(-l-h-x) holds the time of 4 commutations
		lds	temp3, timing_l
		lds	temp4, timing_h
		lds	temp5, timing_x

		sts	zero_wt_l, temp3	; save for zero crossing timeout
		sts	zero_wt_h, temp4
		tst	temp5
		breq	update_t00
		ldi	YL, 0xff
		sts	zero_wt_l, YL		; save for zero crossing timeout
		sts	zero_wt_h, YL
update_t00:
		movw	YL, temp3		; copy timing to Y
		lsr	temp5			; build a quarter
		ror	YH
		ror	YL
		lsr	temp5
		ror	YH			; temp5 no longer needed (should be 0)
		ror	YL

		lds	temp5, timing_x		; reload original timing_x

		sub	temp3, YL		; subtract quarter from timing
		sbc	temp4, YH
		sbc	temp5, zero

		add	temp3, temp1		; .. and add the new time
		adc	temp4, temp2
		adc	temp5, zero

	; limit RPM to 120.000
		cpi	temp3, 0x4c		; 0x14c = 120.000 RPM
		ldi	temp1, 0x1
		cpc	temp4, temp1
		cpc	temp5, zero
		brcc	update_t90

		lsr	sys_control_h		; limit by reducing power
		ror	sys_control_l
		adc	sys_control_l, zero

update_t90:	sts	timing_l, temp3
		sts	timing_h, temp4
		sts	timing_x, temp5
		ldi	temp2, 2
		cp	temp5, temp2		; limit range to 0x1ffff
		brcs	update_t99
		rcall	set_timing_v

update_t99:
		lsr	temp5			; a 16th is the next wait before scan
		ror	temp4
		ror	temp3
		lsr	temp5
		ror	temp4
		ror	temp3
		lsr	temp5
		ror	temp4
		ror	temp3
		lsr	temp5
		ror	temp4
		ror	temp3
		sts	wt_comp_scan_l, temp3
		sts	wt_comp_scan_h, temp4

	; use the same value for commutation timing (15�)
		sts	com_timing_l, temp3
		sts	com_timing_h, temp4

		ret
;-----bko-----------------------------------------------------------------
calc_next_timing:
		lds	YL, wt_comp_scan_l	; holds wait-before-scan value
		lds	YH, wt_comp_scan_h
		rjmp	update_timing

wait_OCT1_tot:	sbrc	flags0, OCT1_PENDING
		rjmp	wait_OCT1_tot

set_OCT1_tot:
		lds	YH, zero_wt_h
		lds	YL, zero_wt_l
		cli
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		add	temp1, YL
		adc	temp2, YH
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sei
		sbr	flags0, (1<<OCT1_PENDING)

		ret
;-----bko-----------------------------------------------------------------
wait_OCT1_before_switch:
		lds	YL, com_timing_l
		lds	YH, com_timing_h
		cli
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		add	temp1, YL
		adc	temp2, YH
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sei
		sbr	flags0, (1<<OCT1_PENDING)

	; don't waste time while waiting - do some controls, if indicated

		ldi	temp1, low(MAX_POWER)
		cp	sys_control_l, temp1
		ldi	temp1, high(MAX_POWER)
		cpc	sys_control_h, temp1
		adc	sys_control_l, zero	; increment sys_control
		adc	sys_control_h, zero	; if not yet at MAX_POWER
		rcall	evaluate_rc_puls

OCT1_wait:	sbrc	flags0, OCT1_PENDING
		rjmp	OCT1_wait
		ret
;-----bko-----------------------------------------------------------------
start_timeout:	lds	YL, wt_OCT1_tot_l
		lds	YH, wt_OCT1_tot_h
		rcall	update_timing

		lds	YH, wt_OCT1_tot_h
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
		rcall	evaluate_rc_puls
;		rcall	evaluate_uart

		ret
;-----bko-----------------------------------------------------------------
set_new_duty:	lds	YL, rc_duty_l
		cp	YL, sys_control_l
		lds	YH, rc_duty_h
		cpc	YH, sys_control_h
		brcs	set_new_duty10
		movw	YL, sys_control_l	; Limit duty to sys_control
set_new_duty10:	lds	temp2, timing_x
		tst	temp2
		brne	set_new_duty12		; on carry - very slow timing
		lds	temp2, timing_h		; work with the timing high byte
		cpi	temp2, PWR_RANGE1	; timing longer than PWR_RANGE1?
		brcs	set_new_duty25		; on carry - test next range
set_new_duty12:	cpi	YL, low(PWR_MAX_RPM1)	; higher than range1 power max ?
		ldi	temp1, high(PWR_MAX_RPM1)
		cpc	YH, temp1
		brcs	set_new_duty31		; on carry - not longer, no restriction
		ldi	YL, low(PWR_MAX_RPM1)	; low (range1) RPM - set PWR_MAX_RPM1
		ldi	YH, high(PWR_MAX_RPM1)
		rjmp	set_new_duty31
set_new_duty25:	cpi	temp2, PWR_RANGE2	; timing longer than PWR_RANGE2?
		brcs	set_new_duty31		; on carry - not longer, no restriction
		cpi	YL, low(PWR_MAX_RPM2)	; higher than range2 power max ?
		ldi	temp1, high(PWR_MAX_RPM2)
		cpc	YH, temp1
		brcs	set_new_duty31		; on carry - not shorter, no restriction
		ldi	YL, low(PWR_MAX_RPM2)	; low (range2) RPM - set PWR_MAX_RPM2
		ldi	YH, high(PWR_MAX_RPM2)
set_new_duty31: 
		cp	YL, zero
		cpc	YH, zero
		breq	set_new_duty_zero
		cbr	flags1, (1<<POWER_OFF)	; atomic clear of POWER_OFF for PWM interrupt
set_new_duty32:
		ldi	temp1, low(MAX_POWER)
		ldi	temp2, high(MAX_POWER)
		sub	temp1, YL		; Calculate OFF duty
		sbc	temp2, YH
		breq	set_new_duty_full
		cbr	flags1, (1<<FULL_POWER)	; atomic clear of FULL_POWER for PWM interrupt
set_new_duty33:
		com	YL			; Save one's complement of both
		com	temp1			; low bytes for up-counting TCNT2
		movw	duty_l, YL		; atomic set of new ON duty for PWM interrupt
		movw	com_duty_l, temp1	; atomic set of new OFF duty for PWM interrupt
		ret
set_new_duty_full:
		sbr	flags1, (1<<FULL_POWER)	; atomic set of FULL_POWER for PWM interrupt
		rjmp	set_new_duty33
set_new_duty_zero:
		sbr	flags1, (1<<POWER_OFF)	; atomic set of POWER_OFF for PWM interrupt
		rjmp	set_new_duty32
;-----bko-----------------------------------------------------------------
switch_power_off:
		ldi	XL, low  (pwm_off)
		ldi	XH, high (pwm_off)
		movw	ZL, XL			; atomic set of FET for PWM interrupt
		all_nFETs_off temp1
		all_pFETs_off temp1
		ret				; motor is off
;-----bko-----------------------------------------------------------------
wait_if_spike:	ldi	temp1, 4
wait_if_spike2:	dec	temp1
		brne	wait_if_spike2
		ret
;-----bko-----------------------------------------------------------------
sync_with_poweron:
		ldi	temp1, 0xff
		mov	acsr_save, temp1	; ACSR will never be 0xff
wait_for_poweroff:
		sbrs	flags0, OCT1_PENDING
		ret
		cp	acsr_save, temp1
		breq	wait_for_poweroff
		ret
;-----bko-----------------------------------------------------------------
motor_brake:
.if MOT_BRAKE == 1
		clr	sys_control
		all_nFETs_off temp1
		all_pFETs_off temp1
		in	temp2, TCNT1L
		sbrc	temp2, 6
		rjmp	brake_off_cycle
		nFET_brake temp1
brake_off_cycle:
		rcall	evaluate_rc_puls
		lds	temp1, rc_duty_l
		cpi	temp1, low(MIN_DUTY)
		lds	temp1, rc_duty_h
		ldi	temp2, high(MIN_DUTY)
		cpc	temp1, temp1
		brcs	motor_brake
		all_nFETs_off temp1
.endif	; MOT_BRAKE == 1
		ret

;-----bko-----------------------------------------------------------------
; **** startup loop ****
init_startup:
		rcall	switch_power_off
wait_for_power_on:
		rcall	motor_brake
		rcall	evaluate_rc_puls
		lds	temp1, rc_duty_l
		cpi	temp1, low(MIN_DUTY)
		lds	temp1, rc_duty_h
		ldi	temp2, high(MIN_DUTY)
		cpc	temp1, temp1
		brcs	wait_for_power_on
		ldi	temp1, RCP_TOT - 1	; allow some racing with t1ovfl_int
		cp	rcpuls_timeout, temp1
		brcs	wait_for_power_on

		comp_init temp1			; init comparator

		ldi	temp1, 27		; wait about 5mikosec
FETs_off_wt:	dec	temp1
		brne	FETs_off_wt

		ldi	YL, low(MAX_POWER)
		ldi	YH, high(MAX_POWER)
		movw	sys_control_l, YL

		cbr	flags2, (1<<SCAN_TIMEOUT)
		sts	goodies, zero

		rcall	set_new_duty
		rcall	set_all_timings

		rcall	com5com6
		rcall	com6com1
		RED_off
		rcall	start_timeout

	; fall through start1

;-----bko-----------------------------------------------------------------
; **** start control loop ****

; state 1 = B(p-on) + C(n-choppered) - comparator A evaluated
; out_cA changes from low to high
start1:		rcall	start_step
		brcs	start1_9
; do the special 120� switch
		sts	goodies, zero
		rcall	com1com2
		rcall	com2com3
		rcall	com3com4

		rcall	start_timeout
		rjmp	start4
start1_9:
		rcall	com1com2
		rcall	start_timeout

; state 2 = A(p-on) + C(n-choppered) - comparator B evaluated
; out_cB changes from high to low

start2:		rcall	start_step
		rcall	com2com3
		rcall	start_timeout

; state 3 = A(p-on) + B(n-choppered) - comparator C evaluated
; out_cC changes from low to high

start3:		rcall	start_step
		rcall	com3com4
		rcall	start_timeout

; state 4 = C(p-on) + B(n-choppered) - comparator A evaluated
; out_cA changes from high to low

start4:		rcall	start_step
		rcall	com4com5
		rcall	start_timeout

; state 5 = C(p-on) + A(n-choppered) - comparator B evaluated
; out_cB changes from low to high

start5:		rcall	start_step
		rcall	com5com6
		rcall	start_timeout

; state 6 = B(p-on) + A(n-choppered) - comparator C evaluated
; out_cC changes from high to low

start6:		rcall	start_step
		rcall	com6com1

		sbrc	flags1, POWER_OFF	; Check if power turned off
		rjmp	init_startup

		tst	rcpuls_timeout		; Check for RC timeout
		brne	s6_test_rpm
		rjmp	restart_control

s6_test_rpm:	lds	temp1, timing_h		; get actual RPM reference high
		cpi	temp1, PWR_RANGE_RUN
		lds	temp1, timing_x
		cpc	temp1, zero
		brcc	s6_start1

s6_run1:	rcall	calc_next_timing
		rcall	set_OCT1_tot
		rjmp	run1			; running state begins

s6_start1:	rcall	start_timeout		; need to be here for a correct temp1=comp_state
		rjmp	start1			; go back to state 1

start_step:
		sbrc	flags0, OCT1_PENDING
		rjmp	start_1
start_0:
		sbr	flags2, (1<<SCAN_TIMEOUT)
		ret
start_1:	rcall	sync_with_poweron
		rcall	sync_with_poweron
		sbrs	acsr_save, ACO
		rjmp	start_3

start_2:	sbrs	flags0, OCT1_PENDING
		rjmp	start_0
		sbrc	acsr_save, ACO
		rjmp	start_2
		sec
		ret

start_3:	sbrs	flags0, OCT1_PENDING
		rjmp	start_0
		sbrs	acsr_save, ACO
		rjmp	start_3
		clc
		ret

;-----bko-----------------------------------------------------------------
; **** running control loop ****

; run 1 = B(p-on) + C(n-choppered) - comparator A evaluated
; out_cA changes from low to high

run1:
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		rcall	wait_OCT1_before_switch
		rcall	com1com2
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 2 = A(p-on) + C(n-choppered) - comparator B evaluated
; out_cB changes from high to low

run2:
		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		rcall	wait_OCT1_before_switch
		rcall	com2com3
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 3 = A(p-on) + B(n-choppered) - comparator C evaluated
; out_cC changes from low to high

run3:
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		rcall	wait_OCT1_before_switch
		rcall	com3com4
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 4 = C(p-on) + B(n-choppered) - comparator A evaluated
; out_cA changes from high to low
run4:
		rcall	wait_for_low
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		rcall	wait_OCT1_before_switch
		rcall	com4com5
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 5 = C(p-on) + A(n-choppered) - comparator B evaluated
; out_cB changes from low to high

run5:
		rcall	wait_for_high
		sbrs	flags0, OCT1_PENDING
		rjmp	run_to_start

		rcall	wait_OCT1_before_switch
		rcall	com5com6
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 6 = B(p-on) + A(n-choppered) - comparator C evaluated
; out_cC changes from high to low

run6:
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
run_to_start:	rjmp	init_startup

run6_2:		rjmp	run1			; go back to run 1

restart_control:
		cli				; disable all interrupts
		rcall	switch_power_off
		clr	sys_control_l
		clr	sys_control_h
		rcall	set_new_duty
		rcall	wait30ms
		rcall	beep_f3
		rcall	beep_f2
		rcall	wait30ms
		sei
		rjmp	init_startup

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
com1com2:
		BpFET_off			; bP off
		sbrs	flags1, POWER_OFF
		ApFET_on			; aP on
		set_comp_phase_b temp1		; Set comparator to phase B
		ret

com2com3:
		ldi	XL, low  (pwm_bfet_on)
		ldi	XH, high (pwm_bfet_on)
		movw	ZL, XL			; Atomic set (read by ISR)
		cli
		sbrc	flags0, I_FET_ON
		icall				; bN on
		sei
		set_comp_phase_c temp1		; Set comparator to phase C
		ret

com3com4:
		ApFET_off			; aP off
		sbrs	flags1, POWER_OFF
		CpFET_on			; cP on
		set_comp_phase_a temp1		; Set comparator to phase A
		ret

com4com5:
		ldi	XL, low  (pwm_afet_on)
		ldi	XH, high (pwm_afet_on)
		movw	ZL, XL			; Atomic set (read by ISR)
		cli
		sbrc	flags0, I_FET_ON
		icall				; aN on
		sei
		set_comp_phase_b temp1		; Set comparator to phase B
		ret

com5com6:
		CpFET_off			; Cp off
		sbrs	flags1, POWER_OFF
		BpFET_on			; Bp on
		set_comp_phase_c temp1		; Set comparator to phase C
		ret

com6com1:
		ldi	XL, low  (pwm_cfet_on)
		ldi	XH, high (pwm_cfet_on)
		movw	ZL, XL			; Atomic set (read by ISR)
		cli
		sbrc	flags0, I_FET_ON
		icall				; cN on
		sei
		set_comp_phase_a temp1		; Set comparator to phase A
		ret

.exit
