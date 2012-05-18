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
; NOTE: We do 16-bit PWM on timer2 at the full CPU clock rate, using tcnt2h
; to simulate the high byte. An input FULL to STOP range of 800 plus a
; MIN_DUTY of 63 (a POWER_RANGE of 863) gives 800 unique PWM steps at an
; about 18kHz on a 16MHz CPU clock. The output frequency is slightly lower
; than F_CPU / POWER_RANGE due to cycles used in the interrupt before
; reloading TCNT2.
;
; Simon Kirby <sim@simulated.ca>
;
;-- Device ----------------------------------------------------------------
;
.include "m8def.inc"
;
; 8K Bytes of In-System Self-Programmable Flash
; 512 Bytes EEPROM
; 1K Byte Internal SRAM
;
;-- Fuses -----------------------------------------------------------------
;
; Old fuses for internal RC oscillator at 8 MHz were lfuse=0xa4 hfuse=0xdf,
; but since we now set OSCCAL to actually run at about 16 MHz, we'd better
; set brown-out detection to 4.0V. This code should work without changes on
; boards with external 16MHz crystals / resonators; just set lfuse=0x3f.
;
; Suggested fuses with 4.0V brown-out voltage:
; Without external crystal: avrdude -U lfuse:w:0x24:m -U hfuse:w:0xd7:m
;    With external crystal: avrdude -U lfuse:w:0x3f:m -U hfuse:w:0xc7:m
;
; Testing fuses with 2.7V brown-out voltage (unsafe at 16MHz):
; Without external crystal: avrdude -U lfuse:w:0xa4:m -U hfuse:w:0xd7:m
;    With external crystal: avrdude -U lfuse:w:0xbf:m -U hfuse:w:0xc7:m
;
;-- Board -----------------------------------------------------------------
;
; The following only works with avra or avrasm2.
; For avrasm32, just comment out all but the include you need.
.if defined(afro_esc)
.include "afro.inc"		; AfroESC (ICP PWM, I2C, UART)
.elif defined(afro2_esc)
.include "afro2.inc"		; AfroESC 2 (ICP PWM, I2C, UART)
.elif defined(birdie70a_esc)
.include "birdie70a.inc"	; Birdie 70A with all nFETs (INT0 PWM)
.elif defined(bs_esc)
.include "bs.inc"		; HobbyKing BlueSeries / Mystery (INT0 PWM)
.elif defined(bs_nfet_esc)
.include "bs_nfet.inc"		; HobbyKing BlueSeries / Mystery with all nFETs (INT0 PWM)
.elif defined(bs40a_esc)
.include "bs40a.inc"		; HobbyKing BlueSeries / Mystery 40A (INT0 PWM)
.elif defined(dlu40a_esc)
.include "dlu40a.inc"		; Pulso Advance Plus 40A DLU40A inverted-PWM-opto (INT0 PWM)
.elif defined(hk200a_esc)
.include "hk200a.inc"		; Hobbyking SS Series 190-200A with all nFETs (INT0 PWM)
.elif defined(kda_esc)
.include "kda.inc"		; Keda Model 12A - 30A (INT0 PWM)
.elif defined(rb50a_esc)
.include "rb50a.inc"		; Red Brick 50A with all nFETs (INT0 PWM)
.elif defined(rb70a_esc)
.include "rb70a.inc"		; Red Brick 70A with all nFETs (INT0 PWM)
.elif defined(rct50a_esc)
.include "rct50a.inc"		; RCTimer 50A with all nFETs (INT0 PWM)
.elif defined(tp_esc)
.include "tp.inc"		; TowerPro 25A/HobbyKing 18A "type 1" (INT0 PWM)
.elif defined(tp_i2c_esc)
.include "tp_i2c.inc"		; TowerPro 25A/HobbyKing 18A "type 1" (I2C)
.elif defined(tp_nfet_esc)
.include "tp_nfet.inc"		; TowerPro 25A with all nFETs "type 3" (INT0 PWM)
.elif defined(tgy6a_esc)
.include "tgy6a.inc"		; Turnigy Plush 6A (INT0 PWM)
.else
.include "tgy.inc"		; TowerPro/Turnigy Basic/Plush "type 2" (INT0 PWM)
.endif

.equ	I2C_ADDR	= 0x50	; MK-style I2C address
.equ	MOTOR_ID	= 1	; MK-style I2C motor ID, or UART motor number

.equ	TIME_ACCUMULATE	= 0	; Accumulate 4 commutations timing method
.equ	TIME_HALFADD	= 0	; Update half timing method
.equ	TIME_QUARTERADD	= 1	; Update quarter timing method (original)

.equ	MOTOR_BRAKE	= 0	; Enable brake
.equ	MOTOR_REVERSE	= 0	; Reverse normal commutation direction
.equ	RC_PULS_REVERSE	= 0	; Enable RC-car style forward/reverse throttle
.equ	SLOW_THROTTLE	= 0	; Limit maximum throttle jump to try to prevent overcurrent

.equ	RCP_TOT		= 16	; Number of 65536us periods before considering rc pulse lost
.equ	CPU_MHZ		= F_CPU / 1000000

; These are now defaults which can be adjusted via throttle calibration
; (stick high, stick low, (stick neutral) at start).
.if defined(ultrapwm)
.equ	STOP_RC_PULS	= 200	; Support for http://www.xaircraft.com/wiki/UltraPWM/en
.equ	FULL_RC_PULS	= 1200	; which says motors should start at 200us,
.equ	MAX_RC_PULS	= 1400	; but does not define min/max pulse width.
.else
; These might be a bit wide for most radios, but lines up with POWER_RANGE.
.equ	STOP_RC_PULS	= 1060	; Stop motor at or below this pulse length
.equ	FULL_RC_PULS	= 1860	; Full speed at or above this pulse length
.equ	MAX_RC_PULS	= 2400	; Throw away any pulses longer than this
.endif

.if	RC_PULS_REVERSE
.equ	RCP_DEADBAND	= 50	; Do not start until this much above or below neutral
.equ	PROGRAM_RC_PULS	= (STOP_RC_PULS + FULL_RC_PULS * 3) / 4	; Normally 1660
.else
.equ	RCP_DEADBAND	= 0
.equ	PROGRAM_RC_PULS	= (STOP_RC_PULS + FULL_RC_PULS) / 2	; Normally 1460
.endif
.equ	MAX_DRIFT_PULS	= 10	; Maximum jitter/drift microseconds during programming

; Minimum PWM on-time (too low and FETs won't turn on, hard starting)
.equ	MIN_DUTY	= 63 * CPU_MHZ / 16

; Number of PWM steps (too high and PWM frequency drops into audible range)
.equ	POWER_RANGE	= 800 * CPU_MHZ / 16 + MIN_DUTY

.equ	MAX_POWER	= (POWER_RANGE-1)
.equ	PWR_MIN_START	= (POWER_RANGE/6) ; Power limit while starting (to start)
.equ	PWR_MAX_START	= (POWER_RANGE/4) ; Power limit while starting (if still not running)
.equ	PWR_MAX_RPM1	= (POWER_RANGE/4) ; Power limit when running slower than TIMING_RANGE1
.equ	PWR_MAX_RPM2	= (POWER_RANGE/2) ; Power limit when running slower than TIMING_RANGE2

.equ	TIMING_MIN	= 0x8000 ; 8192us per commutation
.equ	TIMING_RUN	= 0x1000 ; 1024us per commutation
.equ	TIMING_RANGE1	= 0x4000 ; 4096us per commutation
.equ	TIMING_RANGE2	= 0x2000 ; 2048us per commutation
.equ	TIMING_MAX	= 0x0050 ; 20us per commutation

.equ	timeoutSTART	= 48000 ; 48ms per commutation
.equ	timeoutMIN	= 36000	; 36ms per commutation

.equ	ENOUGH_GOODIES	= 12	; This many start cycles without timeout will transition to running mode

.equ	T0CLK		= (1<<CS01)	; clk/8 == 2Mhz
.equ	T1CLK		= (1<<CS10)+(USE_ICP<<ICES1)+(USE_ICP<<ICNC1)	; clk/1 == 16MHz
.equ	T2CLK		= (1<<CS20)	; clk/1 == 16MHz

.equ	EEPROM_SIGN	= 31337		; Random 16-bit value
.equ	EEPROM_OFFSET	= 0x80		; Offset into 512-byte space (why not)

;**** **** **** **** ****
; Register Definitions
.def	temp5		= r0		; aux temporary (L) (limited operations)
.def	temp6		= r1		; aux temporary (H) (limited operations)
.def	duty_l		= r2		; on duty cycle low, one's complement
.def	duty_h		= r3		; on duty cycle high
.def	off_duty_l	= r4		; off duty cycle low, one's complement
.def	off_duty_h	= r5		; off duty cycle high
.def	rx_l		= r6		; received throttle low
.def	rx_h		= r7		; received throttle high
.def	tcnt2h		= r8		; timer2 high byte
.def	i_sreg		= r9		; status register save in interrupts
;.def			= r10
.def	rc_timeout	= r11
.def	sys_control_l	= r12		; duty limit low (word register aligned)
.def	sys_control_h	= r13		; duty limit high
.def	temp7		= r14		; really aux temporary (limited operations)
;.def			= r15

.def	nfet_on		= r18
.def	nfet_off	= r19
.def	i_temp1		= r20		; interrupt temporary
.def	i_temp2		= r21		; interrupt temporary
.def	temp3		= r22		; main temporary (L)
.def	temp4		= r23		; main temporary (H)
.def	temp1		= r24		; main temporary (L), adiw-capable
.def	temp2		= r25		; main temporary (H), adiw-capable

.def	flags0	= r16	; state flags
	.equ	OCT1_PENDING	= 0	; if set, output compare interrupt is pending
;	.equ	OCT1B_PENDING	= 1	; if set, output compare interrupt B is pending
;	.equ	I_pFET_HIGH	= 2	; set if over-current detect
;	.equ	GET_STATE	= 3	; set if state is to be send
	.equ	I2C_FIRST	= 4	; if set, i2c will receive first byte next
	.equ	I2C_SPACE_LEFT	= 5	; if set, i2c buffer has room
	.equ	UART_SYNC	= 6	; if set, we are waiting for our serial throttle byte
	.equ	NO_CALIBRATION	= 7	; if set, disallow calibration (unsafe reset cause)

.def	flags1	= r17	; state flags
	.equ	POWER_OFF	= 0	; switch fets on disabled
	.equ	FULL_POWER	= 1	; 100% on - don't switch off, but do OFF_CYCLE working
	.equ	I2C_MODE	= 2	; if receiving updates via I2C
	.equ	UART_MODE	= 3	; if receiving updates via UART
	.equ	EVAL_RC		= 4	; if set, evaluate rc command while waiting for OCT1
	.equ	ACO_EDGE_HIGH	= 5	; if set, looking for ACO high - conviently located at the same bit position as ACO
	.equ	STARTUP		= 6	; if set, startup-phase is active
	.equ	REVERSE		= 7	; if set, do reverse commutation

;.def	flags2	= r25
;	.equ	RPM_RANGE1	= 0	; if set RPM is lower than 1831 RPM
;	.equ	RPM_RANGE2	= 1	; if set RPM is lower than 3662 RPM
;	.equ	RC_INTERVAL_OK	= 2
;	.equ	POFF_CYCLE	= 3	; if set one commutation cycle is performed without power
;	.equ	COMP_SAVE	= 4	; if set ACO was high
;	.equ	COMP_SAVE_READY	= 5	; if acsr_save was set by PWM interrupt
;	.equ	STARTUP		= 6	; if set startup-phase is active
;	.equ	SCAN_TIMEOUT	= 7	; if set a startup timeout occurred

; here the XYZ registers are placed ( r26-r31)

; XL: I/O address of PWM nFET port
; XH: I/O address of PWM nFET port
; YL: general temporary
; YH: general temporary
; ZL: Next PWM interrupt vector (low)
; ZH: Next PWM interrupt vector (high, stays at zero) -- used as "zero" register

;**** **** **** **** ****
; RAM Definitions
.dseg				; DATA segment
.org SRAM_START

orig_osccal:	.byte	1	; original OSCCAL value
goodies:	.byte	1
ocr1ax:		.byte	1	; 3rd byte of OCR1A
tcnt1x:		.byte	1	; 3rd byte of TCNT1
last_tcnt1_l:	.byte	1	; last timer1 value
last_tcnt1_h:	.byte	1
last_tcnt1_x:	.byte	1
timing_l:	.byte	1	; holds time of 4 commutations
timing_h:	.byte	1
timing_x:	.byte	1
timing_count:	.byte	1
wt_comp_scan_l:	.byte	1	; time from switch to comparator scan (blanking time)
wt_comp_scan_h:	.byte	1
wt_comp_scan_x:	.byte	1
com_timing_l:	.byte	1	; time from zero-crossing to switch of the appropriate FET
com_timing_h:	.byte	1
com_timing_x:	.byte	1
wt_OCT1_tot_l:	.byte	1	; time for each startup commutation
wt_OCT1_tot_h:	.byte	1
wt_OCT1_tot_x:	.byte	1
zero_wt_l:	.byte	1	; time to wait for zero-crossing while running
zero_wt_h:	.byte	1
zero_wt_x:	.byte	1
zc_filter_time:	.byte	1	; number of times to check zero-crossing
start_rcpuls_l:	.byte	1
start_rcpuls_h:	.byte	1
start_rcpuls_x:	.byte	1
rc_duty_l:	.byte	1	; desired duty cycle
rc_duty_h:	.byte	1
timing_duty_l:	.byte	1	; duty cycle limit based on timing
timing_duty_h:	.byte	1
fwd_scale_l:	.byte	1	; 16.16 multipliers to scale input RC pulse to POWER_RANGE
fwd_scale_h:	.byte	1
rev_scale_l:	.byte	1
rev_scale_h:	.byte	1
neutral_l:	.byte	1	; Offset for neutral throttle (in CPU_MHZ)
neutral_h:	.byte	1
max_pwm:	.byte	1	; MaxPWM for MK (NOTE: 250 while stopped is magic and enables v2)
motor_count:	.byte	1	; Motor number for serial control
;**** **** **** **** ****
; The following entries are block-copied from/to EEPROM
eeprom_sig_l:	.byte	1
eeprom_sig_h:	.byte	1
puls_high_l:	.byte	1	; -,
puls_high_h:	.byte	1	;  |
puls_low_l:	.byte	1	;  |- saved pulse lengths during throttle calibration
puls_low_h:	.byte	1	;  |  (order used by rc_prog)
puls_neutral_l:	.byte	1	;  |
puls_neutral_h:	.byte	1	; -'
eeprom_end:	.byte	1
;-----bko-----------------------------------------------------------------
;**** **** **** **** ****
.cseg
.org 0
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
; Reset and interrupt jump table
; When multiple interrupts are pending, the vectors are executed from top
; (ext_int0) to bottom.
		rjmp reset	; reset
		rjmp rcp_int	; ext_int0
		reti		; ext_int1
		reti		; t2oc_int
		ijmp		; t2ovfl_int
		rjmp rcp_int	; icp1_int
		rjmp t1oca_int	; t1oca_int
		reti		; t1ocb_int
		rjmp t1ovfl_int	; t1ovfl_int
		reti		; t0ovfl_int
		reti		; spi_int
		rjmp urxc_int	; urxc
		reti		; udre
		reti		; utxc
		reti		; adc_int
		reti		; eep_int
		reti		; aci_int
		rjmp i2c_int	; twi_int
		reti		; spmc_int

eeprom_defaults_w:
	.db low(EEPROM_SIGN), high(EEPROM_SIGN)
	.db byte1(FULL_RC_PULS * CPU_MHZ), byte2(FULL_RC_PULS * CPU_MHZ)
	.db byte1(STOP_RC_PULS * CPU_MHZ), byte2(STOP_RC_PULS * CPU_MHZ)
	.db byte1((FULL_RC_PULS + STOP_RC_PULS) * CPU_MHZ / 2), byte2((FULL_RC_PULS + STOP_RC_PULS) * CPU_MHZ / 2)

;-----bko-----------------------------------------------------------------
; init after reset

reset:		clr	r0
		out	SREG, r0		; Clear interrupts and flags

	; Set up stack
		ldi	ZH, high(RAMEND)
		ldi	ZL, low(RAMEND)
		out	SPH, ZH
		out	SPL, ZL
	; Clear RAM and all registers
clear_loop:	st	-Z, r0
		cpi	ZL, SRAM_START
		cpc	ZH, r0
		brne	clear_loop1
		ldi	ZL, 30			; Start clearing registers
clear_loop1:	cp	ZL, r0
		cpc	ZH, r0
		brne	clear_loop		; Leaves with all registers (r0 through ZH) at 0

	; Save original OSCCAL and reset cause
		in	i_sreg, OSCCAL
		sts	orig_osccal, i_sreg
		in	i_sreg, MCUCSR
		out	MCUCSR, r0

	; portB - all FETs off
		ldi	temp1, INIT_PB
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

	; Start timers except output PWM
		ldi	temp1, T0CLK		; timer0: beep control, delays
		out	TCCR0, temp1
		ldi	temp1, T1CLK		; timer1: commutation timing,
		out	TCCR1B, temp1		; RC pulse measurement
		out	TCCR2, ZH		; timer2: PWM, stopped

	; Enable watchdog (WDTON may be set or unset)
		ldi	temp1, (1<<WDCE)+(1<<WDE)
		out	WDTCR, temp1
		ldi	temp1, (1<<WDE)		; Fastest option: ~16.3ms timeout
		out	WDTCR, temp1

	; Read EEPROM block to RAM
		rcall	wait120ms
		rcall	eeprom_read_block	; Also calls osccal_set

	; Check EEPROM signature
		ldi	XL, low(eeprom_sig_l) + 2
		ld	temp2, -X
		ld	temp1, -X		; Leave X at eeprom_sig_l
		subi	temp1, low(EEPROM_SIGN)
		sbci	temp2, high(EEPROM_SIGN)
		breq	eeprom_good

	; Signature not good: set defaults in RAM, but do not write
	; to the EEPROM until we actually set something non-default
		ldi	ZL, low(eeprom_defaults_w << 1)
eeprom_default:	lpm	temp1, Z+
		st	X+, temp1
		cpi	XL, low(eeprom_end)
		brne	eeprom_default
eeprom_good:

	; Check reset cause
		sbrs	i_sreg, PORF		; Power-on reset
		rjmp	init_no_porf
		rcall	beep_f1			; Usual startup beeps
		rcall	beep_f2
		rcall	beep_f3
		rjmp	control_start
init_no_porf:
		sbrs	i_sreg, BORF		; Brown-out reset
		rjmp	init_no_borf
		rcall	beep_f3			; "dead cellphone"
		rcall	beep_f1
		sbr	flags0, (1<<NO_CALIBRATION)
		rjmp	control_start
init_no_borf:
		sbrs	i_sreg, EXTRF		; External reset
		rjmp	init_no_extrf
		rcall	beep_f4			; Single beep
		rjmp	control_start
init_no_extrf:
		sbrs	i_sreg, WDRF		; Watchdog reset
		rjmp	init_no_wdrf
init_wdrf1:	rcall	beep_f1			; "siren"
		rcall	beep_f1
		rcall	beep_f3
		rcall	beep_f3
		rjmp	init_wdrf1		; Loop forever
init_no_wdrf:

	; Unknown reset cause: Beep out all 8 bits
	; Sometimes I can cause this by touching the oscillator.
init_bitbeep1:	rcall	wait240ms
		mov	nfet_on, i_sreg
		ldi	nfet_off, 8
init_bitbeep2:	sbrs	nfet_on, 0
		rcall	beep_f2
		sbrc	nfet_on, 0
		rcall	beep_f4
		rcall	wait120ms
		lsr	nfet_on
		dec	nfet_off
		brne	init_bitbeep2
		rjmp	init_bitbeep1		; Loop forever

;-----bko-----------------------------------------------------------------
; timer2 overflow compare interrupt (output PWM) -- the interrupt vector
; actually "ijmp"s to Z which should point to one of these entry points.
;
; We try to avoid clobbering (and thus needing to save/restore) flags;
; in, out, mov, ldi, etc. do not modify any flags, while dec does.
;
; The comparator (ACSR) is saved at the very end of the ON cycle, but
; since the nFET takes at least half a microsecond to turn off and the
; AVR buffers ACO for a few cycles, we do it after turning off the drive
; pin. For low duty cycles (with a longer off period), testing shows that
; waiting an extra 0.5us - 0.75us (8-12 cycles at 16MHz) actually helps
; to improve zero-crossing detection accuracy significantly, perhaps
; because the driven-low phase has had a chance to finish swinging down.
; However, some tiny boards such as 10A or less may have very low gate
; charge/capacitance, and so can turn off faster. We used to wait 8/9
; cycles, but now we wait 5 cycles (5/16ths of a microsecond), which
; still helps on ~30A boards without breaking 10A boards.
;
; We reload TCNT2 as the very last step so as to reduce PWM dead areas
; between the reti and the next interrupt vector execution, which still
; takes a good 4 (reti) + 4 (interrupt call) + 2 (ijmp) cycles. We also
; try to keep the fet switch off as close to this as possible to avoid a
; significant bump at FULL_POWER.
;
; The pwm_*_high entry points are only called when the particular on/off
; cycle is longer than 8 bits. This is tracked in tcnt2h.

pwm_on_high:
		in	i_sreg, SREG
		dec	tcnt2h
		brne	pwm_on_again
		ldi	ZL, pwm_on
pwm_on_again:	out	SREG, i_sreg
		reti
pwm_off_high:
		in	i_sreg, SREG
		dec	tcnt2h
		brne	pwm_off_again
		ldi	ZL, pwm_off
pwm_off_again:	out	SREG, i_sreg
		reti

pwm_on:
		st	X, nfet_on
		ldi	ZL, pwm_off
		cpse	duty_h, ZH
		ldi	ZL, pwm_off_high
		mov	tcnt2h, duty_h
		out	TCNT2, duty_l
		reti

pwm_off:
		ldi	ZL, pwm_on		; 1 cycle
		cpse	off_duty_h, ZH		; 1 cycle if not zero, 2 if zero
		ldi	ZL, pwm_on_high		; 1 cycle
		mov	tcnt2h, off_duty_h	; 1 cycle
		wdr				; 1 cycle
		st	X, nfet_off		; 2 cycles (off at 7 cycles from entry)
		out	TCNT2, off_duty_l	; 1 cycle
		reti				; 4 cycles

.if high(pwm_off)
.error "high(pwm_off) is non-zero; please move code closer to start or use 16-bit (ZH) jump registers"
.endif
;-----bko-----------------------------------------------------------------
; timer output compare interrupt
t1oca_int:	in	i_sreg, SREG
		lds	i_temp1, ocr1ax
		subi	i_temp1, 1
		brcc	t1oca_int1
		cbr	flags0, (1<<OCT1_PENDING)	; signal OCT1A passed
t1oca_int1:	sts	ocr1ax, i_temp1
		out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; timer1 overflow interrupt (happens every 4096�s)
t1ovfl_int:	in	i_sreg, SREG
		lds	i_temp1, tcnt1x
		inc	i_temp1
		sts	tcnt1x, i_temp1
		andi	i_temp1, 15			; Every 16 overflows
		brne	t1ovfl_int1
		cpse	rc_timeout, ZH
		dec	rc_timeout
t1ovfl_int1:	out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; NOTE: This interrupt uses the 16-bit atomic timer read/write register
; by reading TCNT1L and TCNT1H, so this interrupt must be disabled before
; any other 16-bit timer options happen that might use the same register
; (see "Accessing 16-bit registers" in the Atmel documentation)
; icp1 = rc pulse input, if enabled
rcp_int:
	.if USE_ICP || USE_INT0
		.if USE_ICP
		in	i_temp1, ICR1L		; get captured timer values
		in	i_temp2, ICR1H
		in	i_sreg, TCCR1B		; abuse i_sreg to hold value
		sbrs	i_sreg, ICES1		; evaluate edge of this interrupt
		.else
;-----bko-----------------------------------------------------------------
		in	i_temp1, TCNT1L		; get timer1 values
		in	i_temp2, TCNT1H
		.if USE_INT0 == 1
		sbis	PIND, rcp_in		; evaluate edge of this interrupt
		.else
		sbic	PIND, rcp_in		; inverted signalling
		.endif
		.endif
		rjmp	falling_edge		; bit is clear = falling edge
rising_edge:					; Flags not saved here!
		sts	start_rcpuls_l, i_temp1	; Save pulse start time
		rcp_int_falling_edge i_temp1	; Set next int to falling edge
		sts	start_rcpuls_h, i_temp2
		lds	i_temp1, tcnt1x
		sts	start_rcpuls_x, i_temp1
		sbrc	i_temp2, 7
		reti
		in	i_temp2, TIFR
		sbrs	i_temp2, TOV1
		reti
		in	i_sreg, SREG
		inc	i_temp1			; Compensate for postponed tcnt1x update
		sts	start_rcpuls_x, i_temp1
		out	SREG, i_sreg
		reti

rcpint_fail:	cpse	rc_timeout, ZH
		dec	rc_timeout
		rjmp	rcpint_exit

falling_edge:
		in	i_sreg, SREG
		rcp_int_rising_edge XH		; Set next int to rising edge
		lds	XH, tcnt1x		; We borrow XH and ZH (normally 0)
		sbrc	i_temp2, 7		; as additional registers and
		rjmp	falling_edge1		; clear them before returning.
		in	ZH, TIFR
		sbrc	ZH, TOV1
		inc	XH			; Compensate for postponed tcnt1x update
falling_edge1:	lds	ZH, start_rcpuls_l
		sub	i_temp1, ZH
		lds	ZH, start_rcpuls_h
		sbc	i_temp2, ZH
		lds	ZH, start_rcpuls_x
		sbc	XH, ZH

.if byte3(MAX_RC_PULS*CPU_MHZ)
.error "MAX_RC_PULS too high: adjust it or the pulse length checking code"
.endif
		cpi	i_temp1, byte1(MAX_RC_PULS*CPU_MHZ)
		ldi	ZH, byte2(MAX_RC_PULS*CPU_MHZ)
		cpc	i_temp2, ZH
		ldi	ZH, 0			; Return ZH to 0; clr clobbers flags
		cpc	XH, ZH
		ldi	XH, 0			; Return XH to 0
		brsh	rcpint_fail		; throw away (too long pulse)

.if 0
		cpi	i_temp1, byte1(180*CPU_MHZ)
		ldi	XH, byte2(180*CPU_MHZ)
		cpc	i_temp2, XH
		ldi	XH, 0			; Return XH to 0
		brlo	rcpint_fail		; throw away (too short pulse)
.endif

		movw	rx_l, i_temp1
		sbr	flags1, (1<<EVAL_RC)

rcpint_exit:	out	SREG, i_sreg
		reti
	.endif
;-----bko-----------------------------------------------------------------
i2c_int:
	.if USE_I2C
		in	i_sreg, SREG
		in	i_temp1, TWSR
		cpi	i_temp1, 0x00		; 00000000b bus error due to illegal start/stop condition
		breq	i2c_io_error
		cpi	i_temp1, 0x60		; 01100000b rx-mode: own SLA+W
		breq	i2c_rx_init
		cpi	i_temp1, 0x80		; 10000000b rx-mode: data available
		breq	i2c_rx_data
		cpi	i_temp1, 0xa0		; 10100000b stop/restart condition (end of message)
		breq	i2c_rx_stop
		cpi	i_temp1, 0xa8		; 10101000b tx-mode: own SLA+R
		breq	i2c_ack
		cpi	i_temp1, 0xb8		; 10111000b tx-mode: data request
		breq	i2c_tx_data
		cpi	i_temp1, 0xf8		; 11111000b no relevant state information
		breq	i2c_io_error
		brne	i2c_unknown		; unknown state, reset all ;-)
i2c_rx_stop:	sbrs	flags0, I2C_FIRST
		sbr	flags1, (1<<EVAL_RC)+(1<<I2C_MODE)	; i2c message received
i2c_unknown:	ldi	i_temp1, (1<<TWIE)+(1<<TWEN)+(1<<TWEA)+(1<<TWINT)
		rjmp	i2c_out
i2c_rx_init:	sbrs	flags1, EVAL_RC		; Skip this message if last one not received
		sbr	flags0, (1<<I2C_FIRST)+(1<<I2C_SPACE_LEFT)
		rjmp	i2c_ack
i2c_rx_data:	sbrs	flags0, I2C_SPACE_LEFT	; Receive buffer has room?
		rjmp	i2c_ack			; No, skip
		sbrs	flags0, I2C_FIRST
		rjmp	i2c_rx_data1
		in	rx_h, TWDR		; Receive high byte from bus
		mov	rx_l, ZH		; Zero low byte (we may not receive it)
		cbr	flags0, (1<<I2C_FIRST)
		rjmp	i2c_ack
i2c_rx_data1:	in	rx_l, TWDR		; Receive low byte from bus (MK FlightCtrl "new protocol")
		cbr	flags0, (1<<I2C_SPACE_LEFT)
		rjmp	i2c_ack
i2c_tx_init:	out	TWDR, ZH		; Send 0 as Current (dummy)
		ldi	i_temp1, 250		; Prepare MaxPWM value (250 when stopped enables proto v2 for MK)
		sbrc	flags1, POWER_OFF
i2c_tx_datarep:	ldi	i_temp1, 255		; Send MaxPWM 255 when running (and repeat for Temperature)
		sts	max_pwm, i_temp1
		rjmp	i2c_ack
i2c_tx_data:	lds	i_temp1, max_pwm	; MaxPWM value (has special meaning for MK)
		out	TWDR, i_temp1
		rjmp	i2c_tx_datarep		; Send 255 for Temperature for which we should get a NACK (0xc0)
i2c_io_error:	in	i_temp1, TWCR
		sbr	i_temp1, (1<<TWSTO)+(1<<TWINT)
		rjmp	i2c_out
i2c_ack:	in	i_temp1, TWCR
		sbr	i_temp1, (1<<TWINT)
i2c_out:	out	TWCR, i_temp1
i2c_ret:	out	SREG, i_sreg
		reti
	.endif
;-----bko-----------------------------------------------------------------
urxc_int:
	; This is Bernhard's serial protocol implementation in the UART
	; version here: http://home.versanet.de/~b-konze/blc_6a/blc_6a.htm
 	; This seems to be implemented for a project described here:
	; http://www.control.aau.dk/uav/reports/10gr833/10gr833_student_report.pdf
	; The UART runs at 38400 baud, N81. Input is ignored until >= 0xf5
	; is received, where we start counting to MOTOR_ID, at which
	; the received byte is used as throttle input. 0 is POWER_OFF,
	; >= 200 is FULL_POWER.
	.if USE_UART
		in	i_sreg, SREG
		in	i_temp1, UDR
		cpi	i_temp1, 0xf5		; Start throttle byte sequence
		breq	urxc_x3d_sync
		sbrs	flags0, UART_SYNC
		rjmp	urxc_exit		; Throw away if not UART_SYNC
		brcc	urxc_unknown
		lds	i_temp2, motor_count
		dec	i_temp2
		brne	urxc_set_exit		; Skip when motor_count != 0
		mov	rx_h, i_temp1		; Save 8-bit input
		sbr	flags1, (1<<EVAL_RC)+(1<<UART_MODE)
urxc_unknown:	cbr	flags0, (1<<UART_SYNC)
		rjmp	urxc_exit
urxc_x3d_sync:	sbr	flags0, (1<<UART_SYNC)
		ldi	i_temp2, MOTOR_ID	; Start counting down from MOTOR_ID
urxc_set_exit:	sts	motor_count, i_temp2
urxc_exit:	out	SREG, i_sreg
		reti
	.endif
;-----bko-----------------------------------------------------------------
; beeper: timer0 is set to 1�s/count
beep_f1:	ldi	temp4, 200
		ldi	temp2, 80
		BpFET_on
		AnFET_on
		rjmp	beep

beep_f2:	ldi	temp4, 180
		ldi	temp2, 100
		CpFET_on
		BnFET_on
		rjmp	beep

beep_f3:	ldi	temp4, 160
		ldi	temp2, 120
		ApFET_on
		CnFET_on
		rjmp	beep

beep_f4:	ldi	temp4, 140
		ldi	temp2, 140
		CpFET_on
		AnFET_on
		; Fall through
;-----bko-----------------------------------------------------------------
; Interrupts no longer need to be disabled to beep, but the PWM interrupt
; must be muted first
beep:		in	temp5, PORTB		; Save ON state
		in	temp6, PORTC
		in	temp7, PORTD
beep_on:	out	PORTB, temp5		; Restore ON state
		out	PORTC, temp6
		out	PORTD, temp7
		out	TCNT0, ZH
beep_BpCn10:	in	temp1, TCNT0
		cpi	temp1, 2*CPU_MHZ	; 32�s on
		brlo	beep_BpCn10
		all_nFETs_off temp3
		all_pFETs_off temp3
		ldi	temp3, CPU_MHZ		; 2040�s off
beep_BpCn12:	out	TCNT0, ZH
		wdr
beep_BpCn13:	in	temp1, TCNT0
		cp	temp1, temp4
		brlo	beep_BpCn13
		dec	temp3
		brne	beep_BpCn12
		dec	temp2
		brne	beep_on
		ret

wait240ms:	rcall	wait120ms
wait120ms:	rcall	wait60ms
wait60ms:	rcall	wait30ms
wait30ms:	ldi	temp2, 15
beep_BpCn20:	ldi	temp3, CPU_MHZ
beep_BpCn21:	out	TCNT0, ZH
		ldi	temp1, (1<<TOV0)	; Clear TOV0 by setting it
		out	TIFR, temp1
		wdr
beep_BpCn22:	in	temp1, TIFR
		sbrs	temp1, TOV0
		rjmp	beep_BpCn22
		dec	temp3
		brne	beep_BpCn21
		dec	temp2
		brne	beep_BpCn20
		ret
;-----bko-----------------------------------------------------------------
; Read from or write to the EEPROM block. To avoid duplication, we use the
; global interrupts flag (I) to enable writing versus reading mde. Only
; changed bytes are written. We restore OSCCAL to the boot-time value as
; the EEPROM timing is affected by it. We always return by falling through
; to osccal_set.
eeprom_read_block:				; When interrupts disabled
eeprom_write_block:				; When interrupts enabled
		lds	temp1, orig_osccal
		out	OSCCAL, temp1
		ldi	YL, low(eeprom_sig_l)
		ldi	YH, high(eeprom_sig_l)
		ldi	temp1, low(EEPROM_OFFSET)
		ldi	temp2, high(EEPROM_OFFSET)
eeprom_rw1:	wdr
		sbic	EECR, EEWE
		rjmp	eeprom_rw1		; Loop while writing EEPROM
		in	temp3, SPMCR
		sbrc	temp3, SPMEN
		rjmp	eeprom_rw1		; Loop while flashing
		cpi	YL, low(eeprom_end)
		breq	eeprom_rw4
		out	EEARH, temp2
		out	EEARL, temp1
		adiw	temp1, 1
		sbi	EECR, EERE		; Read existing EEPROM byte
		in	temp3, EEDR
		brie	eeprom_rw2
		st	Y+, temp3		; Store the byte to RAM
		rjmp	eeprom_rw1
eeprom_rw2:	ld	temp4, Y+		; Compare with the byte in RAM
		out	EEDR, temp4
		cli
		sbi	EECR, EEMWE
		cpse	temp3, temp4
		sbi	EECR, EEWE
		sei
		rjmp	eeprom_rw1
eeprom_rw4:	rcall	wait30ms
		; Fall through to set the oscillator calibration
;-----bko-----------------------------------------------------------------
; Set the oscillator calibration for 8MHz operation, or set it to 0xff for
; approximately 16MHz operation even without an external oscillator. This
; should be safe as long as we restore it during EEPROM accesses. This
; will have no effect on boards with external oscillators, except that
; the EEPROM still uses the internal oscillator (at 1MHz).
osccal_set:
.if CPU_MHZ == 16
		ldi	temp1, 0xff		; Almost 16MHz
.else
		ldi	temp1, 0x9f		; Almost 8MHz
.endif
		out	OSCCAL, temp1
		ret
;-----bko-----------------------------------------------------------------
; Shift left temp7:temp6:temp5 temp1 times.
lsl_temp567:
		lsl	temp5
		rol	temp6
		rol	temp7
		dec	temp1
		brne	lsl_temp567
		ret
;-----bko-----------------------------------------------------------------
; Multiply temp1:temp2 by temp3:temp4 and adds high 16 bits of result to Y.
; Clobbers temp5, temp6, temp7.
mul_y_12x34:
		mul	temp1, temp3		; Scale raw pulse length to POWER_RANGE: 16x16->32 (bottom 16 discarded)
		mov	temp7, temp6		; Save byte 2 of result, discard byte 1 already
		mul	temp2, temp3
		add	temp7, temp5
		adc	YL, temp6
		adc	YH, ZH
		mul	temp1, temp4
		add	temp7, temp5
		adc	YL, temp6
		adc	YH, ZH
		mul	temp2, temp4
		add	YL, temp5
		adc	YH, temp6		; Product is now in Y, flags set
		ret
;-----bko-----------------------------------------------------------------
; Unlike the normal evaluate_rc, we look here for programming mode (pulses
; above PROGRAM_RC_PULS), unless we have received I2C or UART input.
;
; With pulse width modulation (PWM) input, we have to be careful about
; oscillator drift. If we are running on a board without an external
; crystal/resonator/oscillator, the internal RC oscillator must be used,
; which can drift significantly with temperature and voltage. So, we must
; use some margins while calibrating. The internal RC speeds up when cold,
; causing arming problems if the learned pulse is too low. Likewise, the
; internal RC slows down when hot, making it impossible to reach full
; throttle.
evaluate_rc_init:
		.if USE_UART
		sbrc	flags1, UART_MODE
		rjmp	evaluate_rc_uart
		.endif
		.if USE_I2C
		sbrc	flags1, I2C_MODE
		rjmp	evaluate_rc_i2c
		.endif
		.if USE_ICP || USE_INT0
		cbr	flags1, (1<<EVAL_RC)
	; If input is above PROGRAM_RC_PULS, we try calibrating throttle
		ldi	YL, low(puls_high_l)	; Start with high pulse calibration
		ldi	YH, high(puls_high_l)
		sbrc	flags0, NO_CALIBRATION	; Is it safe to calibrate now?
		rjmp	evaluate_rc_puls
		rjmp	rc_prog1
rc_prog0:	rcall	wait240ms		; Wait for stick movement to settle
	; Collect average of throttle input pulse length
rc_prog1:	movw	temp3, rx_l		; Save the starting pulse length
		wdr
rc_prog2:	mul	ZH, ZH			; Clear 24-bit result registers (0 * 0 -> temp5:temp6)
		clr	temp7
		cpi	YL, low(puls_high_l)	; Are we learning the high pulse?
		brne	rc_prog3		; No, maybe the low pulse
		cpi	temp3, byte1(PROGRAM_RC_PULS * CPU_MHZ)
		ldi	temp1, byte2(PROGRAM_RC_PULS * CPU_MHZ)
		cpc	temp4, temp1
		brcs	evaluate_rc_puls	; Lower than PROGRAM_RC_PULS - exit programming
		ldi	temp1, 32 * 31/32	; Full speed pulse averaging count (slightly below exact)
		rjmp	rc_prog5
rc_prog3:	lds	temp1, puls_high_l	; If not learning the high pulse, we should stay below it
		cp	temp3, temp1
		lds	temp1, puls_high_h
		cpc	temp4, temp1
		brcc	rc_prog1		; Restart while pulse not lower than learned high pulse
		cpi	YL, low(puls_low_l)	; Are we learning the low pulse?
		brne	rc_prog4		; No, must be the neutral pulse
		ldi	temp1, 32 * 17/16	; Stop/reverse pulse (slightly above exact)
		rjmp	rc_prog5
rc_prog4:	lds	temp1, puls_low_l
		cp	temp3, temp1
		lds	temp1, puls_low_h
		cpc	temp4, temp1
		brcs	rc_prog1		; Restart while pulse lower than learned low pulse
		ldi	temp1, 32		; Neutral pulse measurement (exact)
rc_prog5:	mov	tcnt2h, temp1		; Abuse tcnt2h as pulse counter
rc_prog6:	wdr
		sbrs	flags1, EVAL_RC		; Wait for next pulse
		rjmp	rc_prog6
		cbr	flags1, (1<<EVAL_RC)
		movw	temp1, rx_l		; Atomic copy of new rc pulse length
		add	temp5, temp1		; Accumulate 24-bit average
		adc	temp6, temp2
		adc	temp7, ZH
		sub	temp1, temp3		; Subtract the starting pulse from this one
		sbc	temp2, temp4		; to find the drift since the starting pulse
	; Check for excessive drift with an emulated signed comparison -
	; add the drift amount to offset the negative side to 0
		subi	temp1, byte1(-MAX_DRIFT_PULS * CPU_MHZ)
		sbci	temp2, -1 - byte2(MAX_DRIFT_PULS * CPU_MHZ)
	; ..then subtract the 2*drift + 1 -- carry will be clear if
	; we drifted outside of the range
		subi	temp1, byte1(2 * MAX_DRIFT_PULS * CPU_MHZ + 1)
		sbci	temp2, byte2(2 * MAX_DRIFT_PULS * CPU_MHZ + 1)
		brcc	rc_prog0		; Wait and start over if input moved
		dec	tcnt2h
		brne	rc_prog6		; Loop until average accumulated
		ldi	temp1, 3
		rcall	lsl_temp567		; Multiply by 8 (so that 32 loops makes average*256)
		st	Y+, temp6		; Save the top 16 bits as the result
		st	Y+, temp7
	; One beep: high (full speed) pulse received
		rcall	beep_f3
		cpi	YL, low(puls_high_l+2)
		breq	rc_prog1		; Go back to get low pulse
	; Two beeps: low (stop/reverse) pulse received
		rcall	wait30ms
		rcall	beep_f3
		cpi	YL, low(puls_low_l+2)
		.if RC_PULS_REVERSE
		breq	rc_prog1		; Go back to get neutral pulse
		.else
		breq	rc_prog_done
		.endif
	; Three beeps: neutral pulse received
		rcall	wait30ms
		rcall	beep_f3
rc_prog_done:	rcall	eeprom_write_block
		rjmp	puls_scale		; Calculate the new scaling factors
		.endif
;-----bko-----------------------------------------------------------------
evaluate_rc:
		.if USE_UART
		sbrc	flags1, UART_MODE
		rjmp	evaluate_rc_uart
		.endif
		.if USE_I2C
		sbrc	flags1, I2C_MODE
		rjmp	evaluate_rc_i2c
		.endif
	; Fall through to evaluate_rc_puls
;-----bko-----------------------------------------------------------------
.if USE_ICP || USE_INT0
evaluate_rc_puls:
		cbr	flags1, (1<<EVAL_RC)
		lds	YL, neutral_l
		lds	YH, neutral_h
		movw	temp1, rx_l		; Atomic copy of rc pulse length
		sub	temp1, YL
		sbc	temp2, YH
		brcc	puls_plus
		.if RC_PULS_REVERSE
		.if MOTOR_REVERSE
		cbr	flags1, (1<<REVERSE)
		.else
		sbr	flags1, (1<<REVERSE)
		.endif
		com	temp2
		neg	temp1
		sbci	temp2, -1
		lds	temp3, rev_scale_l
		lds	temp4, rev_scale_h
		rjmp	puls_not_zero
		.endif
		; Fall through
puls_zero:	clr	YL
		clr	YH
		rjmp	rc_not_full
puls_plus:
		.if MOTOR_REVERSE
		sbr	flags1, (1<<REVERSE)
		.else
		cbr	flags1, (1<<REVERSE)
		.endif
		lds	temp3, fwd_scale_l
		lds	temp4, fwd_scale_h
puls_not_zero:
		.if RCP_DEADBAND
		subi	temp1, byte1(RCP_DEADBAND * CPU_MHZ)
		sbci	temp2, byte2(RCP_DEADBAND * CPU_MHZ)
		brmi	puls_zero
		.endif
.endif
	; The following is used by all input modes
rc_do_scale:	ldi	YL, byte1(MIN_DUTY)	; Offset result so that 0 is MIN_DUTY
		ldi	YH, byte2(MIN_DUTY)
		rcall	mul_y_12x34		; Scaled result is now in Y
		cpi	YL, byte1(MAX_POWER)
		ldi	temp1, byte2(MAX_POWER)
		cpc	YH, temp1
		brlo	rc_not_full
		ldi	YL, byte1(MAX_POWER)
		ldi	YH, byte2(MAX_POWER)
rc_not_full:	ldi	temp1, RCP_TOT		; Check rc_timeout
		cp	rc_timeout, temp1
		adc	rc_timeout, ZH		; Increment if not at RCP_TOT
		sts	rc_duty_l, YL
		sts	rc_duty_h, YH
		rjmp	set_new_duty_l		; Skip reload into YL:YH
;-----bko-----------------------------------------------------------------
.if USE_I2C
evaluate_rc_i2c:
		movw	YL, rx_l		; Atomic copy of 16-bit input
		cbr	flags1, (1<<EVAL_RC)+(1<<REVERSE)
		.if MOTOR_REVERSE
		sbr	flags1, (1<<REVERSE)
		.endif
	; MK sends one or two bytes, if supported, and if low bits are
	; non-zero. We store the first received byte in rx_h, second
	; in rx_l. There are 3 low bits which are stored at the low
	; side of the second byte, so we must shift them to line up with
	; the high byte. The high bits become less significant, if set.
		lsl	YL			; 00000xxxb -> 0000xxx0b
		swap	YL			; 0000xxx0b -> xxx00000b
		adiw	YL, 0			; 16-bit zero-test
		breq	rc_not_full
	; Scale so that YH == 247 is MAX_POWER, to support reaching full
	; power from the highest MaxGas setting in MK-Tools. Bernhard's
	; original version reaches full power at around 245.
		movw	temp1, YL
		ldi	temp3, low(0x100 * (POWER_RANGE - MIN_DUTY) / 247)
		ldi	temp4, high(0x100 * (POWER_RANGE - MIN_DUTY) / 247)
		rjmp	rc_do_scale		; The rest of the code is common
.endif
;-----bko-----------------------------------------------------------------
.if USE_UART
evaluate_rc_uart:
		mov	YH, rx_h		; Copy 8-bit input
		cbr	flags1, (1<<EVAL_RC)+(1<<REVERSE)
		.if MOTOR_REVERSE
		sbr	flags1, (1<<REVERSE)
		.endif
		ldi	YL, 0
		cpi	YH, 0
		breq	rc_not_full
	; Scale so that YH == 200 is MAX_POWER.
		movw	temp1, YL
		ldi	temp3, low(0x100 * (POWER_RANGE - MIN_DUTY) / 200)
		ldi	temp4, high(0x100 * (POWER_RANGE - MIN_DUTY) / 200)
		rjmp	rc_do_scale		; The rest of the code is common
.endif
;-----bko-----------------------------------------------------------------
; Calculate the neutral offset and forward (and reverse) scaling factors
; to line up with the high/low (and neutral) pulse lengths.
puls_scale:
		.if RC_PULS_REVERSE
		lds	temp1, puls_neutral_l
		lds	temp2, puls_neutral_h
		.else
		lds	temp1, puls_low_l
		lds	temp2, puls_low_h
		.endif
		sts	neutral_l, temp1
		sts	neutral_h, temp2
	; Find the distance to full throttle and fit it to match the
	; distance between FULL_RC_PULS and STOP_RC_PULS by walking
	; for the lowest 16.16 multiplier that just brings us in range.
		lds	temp3, puls_high_l
		lds	temp4, puls_high_h
		sub	temp3, temp1
		sbc	temp4, temp2
		rcall	puls_find_multiplicand
		sts	fwd_scale_l, temp1
		sts	fwd_scale_h, temp2
		.if RC_PULS_REVERSE
		lds	temp3, puls_neutral_l
		lds	temp4, puls_neutral_h
		lds	temp1, puls_low_l
		lds	temp2, puls_low_h
		sub	temp3, temp1
		sbc	temp4, temp2
		rcall	puls_find_multiplicand
		sts	rev_scale_l, temp1
		sts	rev_scale_h, temp2
		.endif
		ret
;-----bko-----------------------------------------------------------------
; Find the lowest 16.16 multiplicand that brings us to full throttle
; (POWER_RANGE - MIN_DUTY) when multplied by temp3:temp4.
; The range we are looking for is around 3000 - 10000:
; m = (POWER_RANGE - MIN_DUTY) * 65536 / (1000us * 16MHz)
; If the input range is < 100us at 8MHz, < 50us at 16MHz, we return
; too low a multiplicand (higher won't fit in 16 bits).
puls_find_multiplicand:
		.if RCP_DEADBAND
		subi	temp3, byte1(RCP_DEADBAND * CPU_MHZ)
		sbci	temp4, byte2(RCP_DEADBAND * CPU_MHZ)
		.endif
		ldi	temp1, byte1(1999)
		ldi	temp2, byte2(1999)
puls_find1:	adiw	temp1, 1
		wdr
		cpi	temp2, 0xff
		cpc	temp1, temp2
		breq	puls_find_fail		; Return if we reached 0xffff
	; Start with negative POWER_RANGE so that 0 is full throttle
		ldi	YL, low(MIN_DUTY - POWER_RANGE)
		ldi	YH, high(MIN_DUTY - POWER_RANGE)
		rcall	mul_y_12x34
	; We will always be increasing the result in steps of less than 1,
	; so we can test for just zero rather than a range.
		brne	puls_find1
puls_find_fail:	ret
;-----bko-----------------------------------------------------------------
update_timing:
		rcall	set_ocr1a		; Returns TCNT1L/H/X in temp1, temp2, temp3
	; tcnt1x may not be updated until many instructions later, even though
	; interrupts have been enabled, because the AVR always executes one
	; non-interrupt instruction between interrupts, and several other
	; higher-priority interrupts may (have) come up. So, we must save
	; tcnt1x and TIFR with interrupts disabled, then do a correction.
		sbrc	temp2, 7		; If highest bit of TCNT1H is set,
		rjmp	update_timing1		; we assume tcnt1x must be right.
		sbrc	temp4, TOV1		; If TOV1 is/was pending,
		inc	temp3			; increment our copy of tcnt1x.
update_timing1:

.if TIME_ACCUMULATE
		lds	temp4, timing_count
		cpi	temp4, 4
		brsh	update_timing2
		inc	temp4
		sts	timing_count, temp4
		ret
update_timing2:
.endif
	; calculate this commutation time
		lds	YL, last_tcnt1_l
		lds	YH, last_tcnt1_h
		lds	temp5, last_tcnt1_x
		sts	last_tcnt1_l, temp1
		sts	last_tcnt1_h, temp2
		sts	last_tcnt1_x, temp3
		sub	temp1, YL
		sbc	temp2, YH
		sbc	temp3, temp5

	; calculate next waiting times - timing(-l-h-x) holds the time of 4 commutations

.if TIME_ACCUMULATE
	; Accumulation method: Wait for 4 commutations, and that's all.
		movw	YL, temp1		; Copy new timing to YL:YH:temp5
		mov	temp5, temp3
.else
		lds	YL, timing_l		; Load old timing into YL:YH:temp5
		lds	YH, timing_h
		lds	temp5, timing_x

.if TIME_HALFADD
	; Half method: New timing is sum of half of old and twice of last commutation time
		lsr	temp5
		ror	YH
		ror	YL
		lsl	temp1
		rol	temp2
		rol	temp3
		add	YL, temp1
		adc	YH, temp2
		adc	temp5, temp3
.elif TIME_QUARTERADD
	; Quarter method: Subtract quart from old timing, add new commutation time
		lsr	temp5			; build a quater
		ror	YH
		ror	YL
		lsr	temp5
		ror	YH
		ror	YL
		sub	temp1, YL		; subtract old quarter from new timing
		sbc	temp2, YH
		sbc	temp3, temp5
		lds	YL, timing_l
		lds	YH, timing_h
		lds	temp5, timing_x
		add	YL, temp1		; add the new timing difference
		adc	YH, temp2
		adc	temp5, temp3
.endif
.endif
	; Limit maximum RPM (fastest timing)
		cpi	YL, byte1(TIMING_MAX*CPU_MHZ)
		ldi	temp4, byte2(TIMING_MAX*CPU_MHZ)
		cpc	YH, temp4
		ldi	temp4, byte3(TIMING_MAX*CPU_MHZ)
		cpc	temp5, temp4
		brcc	update_timing3
		lsr	sys_control_h		; limit by reducing power
		ror	sys_control_l
		rjmp	update_timing5
update_timing3:
	; Limit minimum RPM (slowest timing)
		ldi	temp4, byte3(TIMING_MIN*CPU_MHZ)
		cp	temp5, temp4
		brcs	update_timing5
		mov	temp5, temp4
		ldi	YH, byte2(TIMING_MIN*CPU_MHZ)
		ldi	YL, byte1(TIMING_MIN*CPU_MHZ)
update_timing5:
.if TIME_ACCUMULATE
		sts	timing_count, ZH
.endif
		sts	timing_l, YL		; save new timing
		sts	timing_h, YH
		sts	timing_x, temp5

		sts	zero_wt_l, YL		; save zero crossing timeout
		sts	zero_wt_h, YH
		sts	zero_wt_x, temp5

	; Calculate a hopefully sane duty cycle limit from this timing,
	; to prevent excessive current if high duty is requested when the
	; current duty is low. This is the best we can do without a current
	; sensor. The actual current will depend on motor KV and voltage,
	; so this is just an approximation. It would be nice if we could
	; do this with math instead of two constants, but we need a divide.

		ldi	temp1, low(MAX_POWER)
		ldi	temp2, high(MAX_POWER)
		cpi	YH, byte2(TIMING_RANGE2*CPU_MHZ)
		ldi	temp4, byte3(TIMING_RANGE2*CPU_MHZ)
		cpc	temp5, temp4
		brcs	update_timing6
		ldi	temp1, low(PWR_MAX_RPM2)
		ldi	temp2, high(PWR_MAX_RPM2)
		cpi	YH, byte2(TIMING_RANGE1*CPU_MHZ)
		ldi	temp4, byte3(TIMING_RANGE1*CPU_MHZ)
		cpc	temp5, temp4
		brcs	update_timing6
		ldi	temp1, low(PWR_MAX_RPM1)
		ldi	temp2, high(PWR_MAX_RPM1)
update_timing6:	sts	timing_duty_l, temp1	; Save new duty limit by timing
		sts	timing_duty_h, temp2

		mov	temp1, YH		; Copy high and check extended byte
		tst	temp5			; We work with 1/256th of timing
		breq	update_timing7
		ldi	temp1, 0xff
.if TIMING_MAX*CPU_MHZ / 0xff < 3
.error "TIMING_MAX is too fast for at least 3 zero-cross checks -- increase it or adjust this"
.endif
update_timing7:	sts	zc_filter_time, temp1	; Save zero cross filter time

		lsr	temp5			; shift back to timing for one commutation
		ror	YH
		ror	YL
		lsr	temp5
		ror	YH
		ror	YL

		lsr	temp5			; a quarter is the next wait before scan
		ror	YH
		ror	YL
		lsr	temp5
		ror	YH
		ror	YL

		sts	wt_comp_scan_l, YL	; save zero-cross blanking wait time (15�)
		sts	wt_comp_scan_h, YH
		sts	wt_comp_scan_x, temp5

		sts	com_timing_l, YL	; use the same value for commutation timing (15�)
		sts	com_timing_h, YH
		sts	com_timing_x, temp5

		sbrc	flags1, EVAL_RC
		rjmp	evaluate_rc		; Set new duty either way
		rjmp	set_new_duty
;-----bko-----------------------------------------------------------------
calc_next_timing_and_wait:
		sbrc	flags1, STARTUP
		rjmp	start_timeout

		lds	YL, wt_comp_scan_l	; holds wait-before-scan value
		lds	YH, wt_comp_scan_h
		lds	temp5, wt_comp_scan_x
		rcall	update_timing

		rcall	wait_OCT1_tot		; Wait for zero blanking completion

		lds	YL, zero_wt_l		; Set OCT1 for zero-crossing timeout
		lds	YH, zero_wt_h
		lds	temp5, zero_wt_x
set_ocr1a:	adiw	YL, 7			; Compensate for timer increment during in-add-out
		ldi	temp4, (1<<OCF1A)
		cli
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		add	YL, temp1
		adc	YH, temp2
		out	OCR1AH, YH
		out	OCR1AL, YL
		out	TIFR, temp4		; Clear any pending OCF1A interrupt (7 cycles from TCNT1 read)
		sts	ocr1ax, temp5
		sbr	flags0, (1<<OCT1_PENDING)
		lds	temp3, tcnt1x
		in	temp4, TIFR
		sei				; We could use reti here, but that's
		ret				; 4 more cycles of interrupt latency
;-----bko-----------------------------------------------------------------
set_com_timing_and_wait:
		lds	YL, com_timing_l
		lds	YH, com_timing_h
		lds	temp5, com_timing_x
		rcall	set_ocr1a
wait_OCT1_tot:	sbrc	flags1, EVAL_RC
		rcall	evaluate_rc
		sbrc	flags0, OCT1_PENDING
		rjmp	wait_OCT1_tot		; Wait for commutation time
		ret
;-----bko-----------------------------------------------------------------
start_timeout_start:
		ldi	YL, byte1(timeoutSTART*CPU_MHZ)
		ldi	YH, byte2(timeoutSTART*CPU_MHZ)
		ldi	temp1, byte3(timeoutSTART*CPU_MHZ)
		mov	temp5, temp1
		ret
;-----bko-----------------------------------------------------------------
start_timeout:
		lds	YL, wt_OCT1_tot_l	; Load the start commutation
		lds	YH, wt_OCT1_tot_h	; timeout into YL:YH:temp5 and
		lds	temp5, wt_OCT1_tot_x	; subtract a "random" amount
		in	temp1, TCNT0
		andi	temp1, 0x1f
		sub	YH, temp1
		sbc	temp5, ZH
		brcs	start_timeout1
		cpi	YL, byte1(timeoutMIN*CPU_MHZ)
		ldi	temp1, byte2(timeoutMIN*CPU_MHZ)
		cpc	YH, temp1
		ldi	temp1, byte3(timeoutMIN*CPU_MHZ)
		cpc	temp5, temp1
		brcc	start_timeout2
start_timeout1:	rcall	start_timeout_start
start_timeout2:	sts	wt_OCT1_tot_l, YL	; Return with YL:YH:temp5 set
		sts	wt_OCT1_tot_h, YH	; to new wt_OCT1_tot value
		sts	wt_OCT1_tot_x, temp5
		rjmp	update_timing
;-----bko-----------------------------------------------------------------
set_new_duty:	lds	YL, rc_duty_l
		lds	YH, rc_duty_h
set_new_duty_l:	lds	temp1, timing_duty_l
		lds	temp2, timing_duty_h
		cp	YL, temp1
		cpc	YH, temp2
		brcs	set_new_duty10
		movw	YL, temp1		; Limit duty to timing_duty
set_new_duty10:	cp	YL, sys_control_l
		cpc	YH, sys_control_h
		brcs	set_new_duty11
		movw	YL, sys_control_l	; Limit duty to sys_control
set_new_duty11:
.if SLOW_THROTTLE
		; If sys_control is higher than twice the current duty,
		; limit it to that. This means that a steady-state duty
		; cycle can double at any time, but any larger change will
		; be rate-limited.
		ldi	temp1, low(PWR_MIN_START)
		ldi	temp2, high(PWR_MIN_START)
		cp	YL, temp1
		cpc	YH, temp2
		brcs	set_new_duty12
		movw	temp1, YL		; temp1:temp2 >= PWR_MIN_START
set_new_duty12:	lsl	temp1
		rol	temp2
		cp	sys_control_l, temp1
		cpc	sys_control_h, temp2
		brcs	set_new_duty13
		movw	sys_control_l, temp1
set_new_duty13:
.endif
		ldi	temp1, low(MAX_POWER)
		ldi	temp2, high(MAX_POWER)
		sub	temp1, YL		; Calculate OFF duty
		sbc	temp2, YH
		breq	set_new_duty_full
		cbr	flags1, (1<<FULL_POWER)
		cp	YL, ZH
		cpc	YH, ZH
		breq	set_new_duty_zero
		; Not off and not full power
		; Halve PWM frequency when starting (helps hard drive startup)
		lds	temp3, goodies
		cpi	temp3, ENOUGH_GOODIES
		brcc	set_new_duty_set
		lsl	temp1
		rol	temp2
		lsl	YL
		rol	YH
set_new_duty_set:
		cbr	flags1, (1<<POWER_OFF)
set_new_duty_set_off:
		com	YL			; Save one's complement of both
		com	temp1			; low bytes for up-counting TCNT2
		movw	duty_l, YL		; Atomic set of new ON duty for PWM interrupt
		movw	off_duty_l, temp1	; Atomic set of new OFF duty for PWM interrupt
		ret
set_new_duty_full:
		; Full power
		sbr	flags1, (1<<FULL_POWER)
		rjmp	set_new_duty_set
set_new_duty_zero:
		; Power off
		sbr	flags1, (1<<POWER_OFF)
		rjmp	set_new_duty_set_off
;-----bko-----------------------------------------------------------------
switch_power_off:
		out	TCCR2, ZH		; Disable PWM
		ldi	temp1, (1<<TOV2)
		out	TIFR, temp1		; Clear pending PWM interrupts
		ldi	ZL, low(pwm_off)	; Set PWM interrupt vector
		all_pFETs_off temp1
		all_nFETs_off temp1
		ret
;-----bko-----------------------------------------------------------------
control_start:
	; status led on
		GRN_on

		rcall	puls_scale

	; init registers and interrupts
		ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE2)
		out	TIFR, temp1		; clear TOIE1, OCIE1A & TOIE2
		out	TIMSK, temp1		; enable TOIE1, OCIE1A & TOIE2 interrupts

		.if defined(HK_PROGRAM_CARD)
	; This program card seems to send data at 1200 baud N81,
	; Messages start with 0xdd 0xdd, have 7 bytes of config,
	; and end with 0xde, sent two seconds after power-up or
	; after any jumper change.
		.equ	BAUD_RATE = 1200
		.equ	UBRR_VAL = F_CPU / BAUD_RATE / 16 - 1
		ldi	temp1, high(UBRR_VAL)
		out	UBRRH, temp1
		ldi	temp1, low(UBRR_VAL)
		out	UBRRL, temp1
		ldi	temp1, (1<<RXEN)	; Do programming card rx by polling
		out	UCSRB, temp1
		ldi	temp1, (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0)
		out	UCSRC, temp1
		.endif

	; init input sources (i2c and/or rc-puls)
		.if USE_UART && !defined(HK_PROGRAM_CARD)
		.equ	BAUD_RATE = 38400
		.equ	UBRR_VAL = F_CPU / BAUD_RATE / 16 - 1
		ldi	temp1, high(UBRR_VAL)
		out	UBRRH, temp1
		ldi	temp1, low(UBRR_VAL)
		out	UBRRL, temp1
		ldi	temp1, (1<<RXEN)+(0<<RXCIE)	; We don't actually tx
		out	UCSRB, temp1
		ldi	temp1, (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0)	; N81
		out	UCSRC, temp1
		in	temp1, UDR
		in	temp1, UDR
		in	temp1, UDR
                sbi     UCSRA, RXC              ; clear flag
                sbi     UCSRB, RXCIE            ; enable reception irq
		.endif
		.if USE_I2C
		sbr	flags0, (1<<I2C_FIRST)+(1<<I2C_SPACE_LEFT)
		ldi	temp1, I2C_ADDR + (MOTOR_ID << 1)
		out	TWAR, temp1
		ldi	temp1, (1<<TWIE)+(1<<TWEN)+(1<<TWEA)+(1<<TWINT)
		out	TWCR, temp1
		.endif
		.if USE_INT0 || USE_ICP
		rcp_int_rising_edge temp1
		rcp_int_enable temp1
		.endif

		sei				; enable all interrupts

i_rc_puls1:	clr	rc_timeout
		cbr	flags1, (1<<EVAL_RC)+(1<<I2C_MODE)+(1<<UART_MODE)
i_rc_puls2:	wdr
		.if defined(HK_PROGRAM_CARD)
		.endif
		sbrs	flags1, EVAL_RC
		rjmp	i_rc_puls2
		rcall	evaluate_rc_init
		lds	YL, rc_duty_l
		lds	YH, rc_duty_h
		adiw	YL, 0			; Test for zero
		brne	i_rc_puls1
		ldi	temp1, 10		; wait for this count of receiving power off
		cp	rc_timeout, temp1
		brlo	i_rc_puls2
		.if USE_I2C
		sbrs	flags1, I2C_MODE
		out	TWCR, ZH		; Turn off I2C and interrupt
		.endif
		.if USE_UART
		sbrs	flags1, UART_MODE
		out	UCSRB, ZH		; Turn off UART and interrupt
		.endif
		.if USE_INT0 || USE_ICP
		mov	temp1, flags1
		andi	temp1, (1<<I2C_MODE)+(1<<UART_MODE)
		breq	i_rc_puls3
		rcp_int_disable temp1		; Turn off RC pulse interrupt
i_rc_puls3:
		.endif
		rcall	beep_f4			; signal: rcpuls ready
		rcall	beep_f4
		rcall	beep_f4
	; Fall through to init_startup
;-----bko-----------------------------------------------------------------
init_startup:
		rcall	switch_power_off	; Disables PWM timer, turns off all FETs
.if MOTOR_BRAKE
		nFET_brake temp1
.endif
wait_for_power_on:
		wdr
		sbrs	flags1, EVAL_RC
		rjmp	wait_for_power_on
		rcall	evaluate_rc
		lds	YL, rc_duty_l
		lds	YH, rc_duty_h
		adiw	YL, 0			; Test for zero
		breq	wait_for_power_on
		ldi	temp1, RCP_TOT - 1	; allow some racing with t1ovfl_int
		cp	rc_timeout, temp1
		brcs	wait_for_power_on

start_from_running:
		rcall	switch_power_off
		comp_init temp1			; init comparator
		RED_off

		ldi	temp1, 27		; wait about 5mikosec
FETs_off_wt:	dec	temp1
		brne	FETs_off_wt

		ldi	YL, low(PWR_MIN_START)	; Start with limited power to
		ldi	YH, high(PWR_MIN_START) ; reduce the chance that we
		movw	sys_control_l, YL	; align to a timing harmonic

		sts	goodies, ZH
		sbr	flags1, (1<<STARTUP)
		ldi	temp1, 4
		sts	timing_count, temp1
		rcall	start_timeout_start
		rcall	update_timing		; Clears POWER_OFF, sets duty, sets last_tcnt1
		rcall	start_timeout_start
		sts	timing_l, YL		; Reset timing to timeoutSTART
		sts	timing_h, YH
		sts	timing_x, temp1
		sts	timing_count, ZH

		rcall	com5com6		; Enable pFET if not POWER_OFF
		rcall	com6com1		; Set comparator phase and nFET vector

		ldi	temp1, T2CLK
		out	TCCR2, temp1		; Enable PWM (ZL and XL have been set)

;-----bko-----------------------------------------------------------------
; **** running control loop ****

run1:		sbrc	flags1, REVERSE
		rjmp	run_reverse

run_forward:	rcall	wait_for_high
		rcall	com1com2
		rcall	wait_for_low
		rcall	com2com3
		rcall	wait_for_high
		rcall	com3com4
		rcall	wait_for_low
		rcall	com4com5
		rcall	wait_for_high
		rcall	com5com6
		rcall	wait_for_low
		rcall	com6com1
		rjmp	run6

run_to_start:	rcall	switch_power_off
		rjmp	start_from_running

run_reverse:	rcall	wait_for_low
		rcall	com1com6
		rcall	wait_for_high
		rcall	com6com5
		rcall	wait_for_low
		rcall	com5com4
		rcall	wait_for_high
		rcall	com4com3
		rcall	wait_for_low
		rcall	com3com2
		rcall	wait_for_high
		rcall	com2com1
run6:
		tst	rc_timeout
		breq	restart_control

.if MOTOR_BRAKE
		; Brake immediately whenever power is off
		sbrc	flags1, POWER_OFF
		rjmp	run_to_brake
.else
		; If timing is too slow and power is off, return to init_startup
		lds	temp1, timing_x
		cpi	temp1, byte3(TIMING_MIN*CPU_MHZ)
		sbrc	flags1, POWER_OFF
		brsh	run_to_brake
.endif
		cp	sys_control_l, ZH
		cpc	sys_control_h, ZH
		breq	run_to_start

		movw	YL, sys_control_l
		lds	temp1, goodies
		cpi	temp1, ENOUGH_GOODIES
		brcc	run6_2
		inc	temp1
		sts	goodies, temp1
		; Build up sys_control to PWR_MAX_START in steps.
		adiw	YL, ((PWR_MAX_START - PWR_MIN_START) + 15) / 16
		ldi	temp1, low(PWR_MAX_START)
		ldi	temp2, high(PWR_MAX_START)
		rjmp	run6_3

run6_2:		cbr	flags1, (1<<STARTUP)
		; Build up sys_control to MAX_POWER in steps.
		; If SLOW_THROTTLE is disabled, this only limits
		; initial start ramp-up; once running, sys_control
		; will stay at MAX_POWER unless timing is lost.
		.equ SLOW_THROTTLE_STEPS = (POWER_RANGE + 31) / 32
		.if SLOW_THROTTLE_STEPS > 63
		subi	YL, byte1(-SLOW_THROTTLE_STEPS)
		sbci	YH, -1 - byte2(SLOW_THROTTLE_STEPS)
		.else
		adiw	YL, SLOW_THROTTLE_STEPS
		.endif
		ldi	temp1, low(MAX_POWER)
		ldi	temp2, high(MAX_POWER)
run6_3:		cp	YL, temp1
		cpc	YH, temp2
		brcs	run6_4
		movw	sys_control_l, temp1
		rjmp	run1
run6_4:		movw	sys_control_l, YL
		rjmp	run1

restart_control:
		rcall	switch_power_off
		rcall	wait30ms
		rcall	beep_f3
		rcall	beep_f2
		rcall	wait30ms
run_to_brake:	rjmp	init_startup

;-----bko-----------------------------------------------------------------
wait_timeout:	sts	goodies, ZH
		sbr	flags1, (1<<STARTUP)
		ret
;-----bko-----------------------------------------------------------------
wait_for_low:	cbr	flags1, (1<<ACO_EDGE_HIGH)
		rjmp	wait_for_edge
;-----bko-----------------------------------------------------------------
wait_for_high:	sbr	flags1, (1<<ACO_EDGE_HIGH)
;-----bko-----------------------------------------------------------------
wait_for_edge:	rcall	calc_next_timing_and_wait
		lds	temp1, zc_filter_time
wait_for_edge2:	lds	temp2, zc_filter_time
wait_for_edge3:	sbrs	flags0, OCT1_PENDING
		rjmp	wait_timeout
		in	temp3, ACSR
		eor	temp3, flags1
		sbrc	temp3, ACO
		rjmp	wait_for_edge4
		cp	temp1, temp2		; Not yet crossed
		adc	temp1, ZH		; Increment temp1 if < temp2
		sbrs	flags1, EVAL_RC
		rjmp	wait_for_edge3
		push	temp1
		rcall	evaluate_rc
		pop	temp1
		rjmp	wait_for_edge2		; Restore temp2 and loop
wait_for_edge4:	dec	temp1			; Zero-cross has happened
		brne	wait_for_edge3		; Check again unless temp1 is zero
		sbrs	flags1, STARTUP		; Skip commutation wait during startup
		rjmp	set_com_timing_and_wait
		ret
;-----bko-----------------------------------------------------------------
; *** commutation utilities ***
com1com2:	; Bp off, Ap on
		set_comp_phase_b temp1
		.if CnFET_port == BpFET_port
		BpFET_off_reg nfet_on
		BpFET_off_reg nfet_off
		.endif
		BpFET_off
		sbrc	flags1, POWER_OFF
		ret
		.if CnFET_port == ApFET_port
		ApFET_on_reg nfet_on
		ApFET_on_reg nfet_off
		.endif
		ApFET_on
		ret

com2com1:	; Bp on, Ap off
		set_comp_phase_a temp1
		.if CnFET_port == ApFET_port
		ApFET_off_reg nfet_on
		ApFET_off_reg nfet_off
		.endif
		ApFET_off
		sbrc	flags1, POWER_OFF
		ret
		.if CnFET_port == BpFET_port
		BpFET_on_reg nfet_on
		BpFET_on_reg nfet_off
		.endif
		BpFET_on
		ret

com2com3:	; Cn off, Bn on
		set_comp_phase_c temp1
		cli
		in	temp1, CnFET_port
		CnFET_off
		in	temp2, CnFET_port
		in	nfet_on, BnFET_port
		cpse	temp1, temp2
		BnFET_on
		sbrs	flags1, POWER_OFF
		BnFET_on_reg nfet_on
		mov	nfet_off, nfet_on
		sbrs	flags1, FULL_POWER
		BnFET_off_reg nfet_off
		ldi	XL, BnFET_port+0x20
		sei
		ret

com3com2:	; Cn on, Bn off
		set_comp_phase_b temp1
		cli
		in	temp1, BnFET_port
		BnFET_off
		in	temp2, BnFET_port
		in	nfet_on, CnFET_port
		cpse	temp1, temp2
		CnFET_on
		sbrs	flags1, POWER_OFF
		CnFET_on_reg nfet_on
		mov	nfet_off, nfet_on
		sbrs	flags1, FULL_POWER
		CnFET_off_reg nfet_off
		ldi	XL, CnFET_port+0x20
		sei
		ret

com3com4:	; Ap off, Cp on
		set_comp_phase_a temp1
		.if BnFET_port == ApFET_port
		ApFET_off_reg nfet_on
		ApFET_off_reg nfet_off
		.endif
		ApFET_off
		sbrc	flags1, POWER_OFF
		ret
		.if BnFET_port == CpFET_port
		CpFET_on_reg nfet_on
		CpFET_on_reg nfet_off
		.endif
		CpFET_on
		ret

com4com3:	; Ap on, Cp off
		set_comp_phase_c temp1
		.if BnFET_port == CpFET_port
		CpFET_off_reg nfet_on
		CpFET_off_reg nfet_off
		.endif
		CpFET_off
		sbrc	flags1, POWER_OFF
		ret
		.if BnFET_port == ApFET_port
		ApFET_on_reg nfet_on
		ApFET_on_reg nfet_off
		.endif
		ApFET_on
		ret

com4com5:	; Bn off, An on
		set_comp_phase_b temp1
		cli
		in	temp1, BnFET_port
		BnFET_off
		in	temp2, BnFET_port
		in	nfet_on, AnFET_port
		cpse	temp1, temp2
		AnFET_on
		sbrs	flags1, POWER_OFF
		AnFET_on_reg nfet_on
		mov	nfet_off, nfet_on
		sbrs	flags1, FULL_POWER
		AnFET_off_reg nfet_off
		ldi	XL, AnFET_port+0x20
		sei
		ret

com5com4:	; Bn on, An off
		set_comp_phase_a temp1
		cli
		in	temp1, AnFET_port
		AnFET_off
		in	temp2, AnFET_port
		in	nfet_on, BnFET_port
		cpse	temp1, temp2
		BnFET_on
		sbrs	flags1, POWER_OFF
		BnFET_on_reg nfet_on
		mov	nfet_off, nfet_on
		sbrs	flags1, FULL_POWER
		BnFET_off_reg nfet_off
		ldi	XL, BnFET_port+0x20
		sei
		ret

com5com6:	; Cp off, Bp on
		set_comp_phase_c temp1
		.if AnFET_port == CpFET_port
		CpFET_off_reg nfet_on
		CpFET_off_reg nfet_off
		.endif
		CpFET_off
		sbrc	flags1, POWER_OFF
		ret
		.if AnFET_port == BpFET_port
		BpFET_on_reg nfet_on
		BpFET_on_reg nfet_off
		.endif
		BpFET_on
		ret

com6com5:	; Cp on, Bp off
		set_comp_phase_b temp1
		.if AnFET_port == BpFET_port
		BpFET_off_reg nfet_on
		BpFET_off_reg nfet_off
		.endif
		BpFET_off
		sbrc	flags1, POWER_OFF
		ret
		.if AnFET_port == CpFET_port
		CpFET_on_reg nfet_on
		CpFET_on_reg nfet_off
		.endif
		CpFET_on
		ret

com6com1:	; An off, Cn on
		set_comp_phase_a temp1
		cli
		in	temp1, AnFET_port
		AnFET_off
		in	temp2, AnFET_port
		in	nfet_on, CnFET_port
		cpse	temp1, temp2
		CnFET_on
		sbrs	flags1, POWER_OFF
		CnFET_on_reg nfet_on
		mov	nfet_off, nfet_on
		sbrs	flags1, FULL_POWER
		CnFET_off_reg nfet_off
		ldi	XL, CnFET_port+0x20
		sei
		ret

com1com6:	; An on, Cn off
		set_comp_phase_c temp1
		cli
		in	temp1, CnFET_port
		CnFET_off
		in	temp2, CnFET_port
		in	nfet_on, AnFET_port
		cpse	temp1, temp2
		AnFET_on
		sbrs	flags1, POWER_OFF
		AnFET_on_reg nfet_on
		mov	nfet_off, nfet_on
		sbrs	flags1, FULL_POWER
		AnFET_off_reg nfet_off
		ldi	XL, AnFET_port+0x20
		sei
		ret

.exit
