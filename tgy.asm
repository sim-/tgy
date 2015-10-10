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
;--
; Based on upon Bernhard's "tp-18a" and others; see
; http://home.versanet.de/~b-konze/blc_18a/blc_18a.htm
; Copyright (C) 2004 Bernhard Konze
; Copyright (C) 2011-2012 Simon Kirby and other contributors
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
; NOTE: We do 16-bit PWM on timer2 at full CPU clock rate resolution, using
; tcnt2h to simulate the high byte. An input FULL to STOP range of 800 plus
; a MIN_DUTY of 56 (a POWER_RANGE of 856) gives 800 unique PWM steps at an
; about 18kHz on a 16MHz CPU clock. The output frequency is slightly lower
; than F_CPU / POWER_RANGE due to cycles used in the interrupt as TCNT2 is
; reloaded.
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
; Old fuses for internal RC oscillator at 8MHz were lfuse=0xa4 hfuse=0xdf,
; but since we now set OSCCAL to 0xff (about 16MHz), running under 4.5V is
; officially out of spec. We'd better set the brown-out detection to 4.0V.
; The resulting code works with or without external 16MHz oscillators.
; Boards with external oscillators can use lfuse=0x3f.
;
; If the boot loader is enabled, the last nibble of the hfuse should be set
; to 'a' or '2' to also enable EESAVE - save EEPROM on chip erase. This is
; a 512-word boot flash section (0xe00), and enable BOOTRST to jump to it.
; Setting these fuses actually has no harm even without the boot loader,
; since 0xffff is nop, and it will just nop-sled around into normal code.
;
; Suggested fuses with 4.0V brown-out voltage:
; Without external oscillator: avrdude -U lfuse:w:0x24:m -U hfuse:w:0xda:m
;    With external oscillator: avrdude -U lfuse:w:0x3f:m -U hfuse:w:0xca:m
;
; Don't set WDTON if using the boot loader. We will enable it on start.
;
;-- Board -----------------------------------------------------------------
;
; The following only works with avra or avrasm2.
; For avrasm32, just comment out all but the include you need.
#if defined(afro_esc)
#include "afro.inc"		; AfroESC (ICP PWM, I2C, UART)
#elif defined(afro2_esc)
#include "afro2.inc"		; AfroESC 2 (ICP PWM, I2C, UART)
#elif defined(afro_hv_esc)
#include "afro_hv.inc"		; AfroESC HV with drivers (ICP PWM, I2C, UART)
#elif defined(afro_nfet_esc)
#include "afro_nfet.inc"	; AfroESC 3 with all nFETs (ICP PWM, I2C, UART)
#elif defined(afro_pr0_esc)
#include "afro_pr0.inc"		; AfroESC prototype rev0 with NCP5911 (ICP PWM)
#elif defined(afro_pr1_esc)
#include "afro_pr1.inc"		; AfroESC prototype rev1 with NCP5911 (ICP PWM)
#elif defined(arctictiger_esc)
#include "arctictiger.inc"	; Arctic Tiger 30A ESC with all nFETs (ICP PWM)
#elif defined(birdie70a_esc)
#include "birdie70a.inc"	; Birdie 70A with all nFETs (INT0 PWM)
#elif defined(mkblctrl1_esc)
#include "mkblctrl1.inc"	; MK BL-Ctrl v1.2 (ICP PWM, I2C, UART, high side PWM, sense hack)
#elif defined(bs_esc)
#include "bs.inc"		; HobbyKing BlueSeries / Mystery (INT0 PWM)
#elif defined(bs_nfet_esc)
#include "bs_nfet.inc"		; HobbyKing BlueSeries / Mystery with all nFETs (INT0 PWM)
#elif defined(bs40a_esc)
#include "bs40a.inc"		; HobbyKing BlueSeries / Mystery 40A (INT0 PWM)
#elif defined(dlu40a_esc)
#include "dlu40a.inc"		; Pulso Advance Plus 40A DLU40A inverted-PWM-opto (INT0 PWM)
#elif defined(dlux_esc)
#include "dlux.inc"		; HobbyKing Dlux Turnigy ESC 20A
#elif defined(diy0_esc)
#include "diy0.inc"		; HobbyKing DIY Open ESC (unreleased rev 0)
#elif defined(dys_nfet_esc)
#include "dys_nfet.inc"		; DYS 30A ESC with all nFETs (ICP PWM, I2C, UART)
#elif defined(hk200a_esc)
#include "hk200a.inc"		; HobbyKing SS Series 190-200A with all nFETs (INT0 PWM)
#elif defined(hm135a_esc)
#include "hm135a.inc"		; Hacker/Jeti Master 135-O-F5B 135A inverted-PWM-opto (INT0 PWM)
#elif defined(hxt200a_esc)
#include "hxt200a.inc"		; HexTronik F3J HXT200A HV ESC (INT0 PWM, I2C, UART)
#elif defined(kda_esc)
#include "kda.inc"		; Keda/Multistar 12A, 20A, 30A (original) (inverted INT0 PWM)
#elif defined(kda_8khz_esc)
#include "kda_8khz.inc"		; Keda/Multistar 30A (early 2014) (inverted INT0 PWM)
#elif defined(kda_nfet_esc)
#include "kda_nfet.inc"		; Keda/Multistar 30A with all nFETs (inverted INT0 PWM)
#elif defined(kda_nfet_ni_esc)
#include "kda_nfet_ni.inc"	; Keda/Multistar/Sunrise ~30A with all nFETs (INT0 PWM)
#elif defined(rb50a_esc)
#include "rb50a.inc"		; Red Brick 50A with all nFETs (INT0 PWM)
#elif defined(rb70a_esc)
#include "rb70a.inc"		; Red Brick 70A with all nFETs (INT0 PWM)
#elif defined(rb70a2_esc)
#include "rb70a2.inc"		; Newer Red Brick 70A with blue pcb and all nFETs (INT0 PWM)
#elif defined(rct50a_esc)
#include "rct50a.inc"		; RCTimer 50A (MLF version) with all nFETs (INT0 PWM)
#elif defined(tbs_esc)
#include "tbs.inc"		; TBS 30A ESC (Team BlackSheep) with all nFETs (ICP PWM, UART)
#elif defined(tbs_hv_esc)
#include "tbs_hv.inc"		; TBS high voltage ESC (Team BlackSheep) with all nFETs (ICP PWM, UART)
#elif defined(tp_esc)
#include "tp.inc"		; TowerPro 25A/HobbyKing 18A "type 1" (INT0 PWM)
#elif defined(tp_8khz_esc)
#include "tp_8khz.inc"		; TowerPro 25A/HobbyKing 18A "type 1" (INT0 PWM) at 8kHz PWM
#elif defined(tp_i2c_esc)
#include "tp_i2c.inc"		; TowerPro 25A/HobbyKing 18A "type 1" (I2C)
#elif defined(tp_nfet_esc)
#include "tp_nfet.inc"		; TowerPro 25A with all nFETs "type 3" (INT0 PWM)
#elif defined(tp70a_esc)
#include "tp70a.inc"		; TowerPro 70A with BL8003 FET drivers (INT0 PWM)
#elif defined(tgy6a_esc)
#include "tgy6a.inc"		; Turnigy Plush 6A (INT0 PWM)
#elif defined(tgy_8mhz_esc)
#include "tgy_8mhz.inc"		; TowerPro/Turnigy Basic/Plush "type 2" w/8MHz oscillator (INT0 PWM)
#elif defined(tgy_esc)
#include "tgy.inc"		; TowerPro/Turnigy Basic/Plush "type 2" (INT0 PWM)
#else
#error "Unrecognized board type."
#endif

.equ	CPU_MHZ		= F_CPU / 1000000

.equ	BOOT_LOADER	= 1	; Include Turnigy USB linker STK500v2 boot loader on PWM input pin
.equ	BOOT_JUMP	= 1	; Jump to any boot loader when PWM input stays high
.equ	BOOT_START	= THIRDBOOTSTART

.if !defined(COMP_PWM)
.equ	COMP_PWM	= 0	; During PWM off, switch high side on (unsafe on some boards!)
.endif
.if !defined(DEAD_LOW_NS)
.equ	DEAD_LOW_NS	= 300	; Low-side dead time w/COMP_PWM (62.5ns steps @ 16MHz, max 2437ns)
.equ	DEAD_HIGH_NS	= 300	; High-side dead time w/COMP_PWM (62.5ns steps @ 16MHz, max roughly PWM period)
.endif
.equ	DEAD_TIME_LOW	= DEAD_LOW_NS * CPU_MHZ / 1000
.equ	DEAD_TIME_HIGH	= DEAD_HIGH_NS * CPU_MHZ / 1000

.if !defined(MOTOR_ADVANCE)
.equ	MOTOR_ADVANCE	= 18	; Degrees of timing advance (0 - 30, 30 meaning no delay)
.endif
.if !defined(TIMING_OFFSET)
.equ	TIMING_OFFSET	= 0	; Motor timing offset in microseconds
.endif
.equ	MOTOR_BRAKE	= 0	; Enable brake during neutral/idle ("motor drag" brake)
.equ	LOW_BRAKE	= 0	; Enable brake on very short RC pulse ("thumb" brake like on Airtronics XL2P)
.if !defined(MOTOR_REVERSE)
.equ	MOTOR_REVERSE	= 0	; Reverse normal commutation direction
.endif
.equ	RC_PULS_REVERSE	= 0	; Enable RC-car style forward/reverse throttle
.equ	RC_CALIBRATION	= 1	; Support run-time calibration of min/max pulse lengths
.equ	SLOW_THROTTLE	= 0	; Limit maximum throttle jump to try to prevent overcurrent
.equ	BEACON		= 1	; Beep periodically when RC signal is lost
.equ	BEACON_IDLE	= 0	; Beep periodically if idle for a long period
.if !defined(CHECK_HARDWARE)
.equ	CHECK_HARDWARE	= 0	; Check for correct pin configuration, sense inputs, and functioning MOSFETs
.endif
.equ	CELL_MAX_DV	= 43	; Maximum battery cell deciV
.equ	CELL_MIN_DV	= 35	; Minimum battery cell deciV
.equ	CELL_COUNT	= 0	; 0: auto, >0: hard-coded number of cells (for reliable LVC > ~4S)
.equ	BLIP_CELL_COUNT	= 0	; Blip out cell count before arming
.equ	DEBUG_ADC_DUMP	= 0	; Output an endless loop of all ADC values (no normal operation)
.equ	MOTOR_DEBUG	= 0	; Output sync pulses on MOSI or SCK, debug flag on MISO

.equ	I2C_ADDR	= 0x50	; MK-style I2C address
.equ	MOTOR_ID	= 1	; MK-style I2C motor ID, or UART motor number

.equ	RCP_TOT		= 2	; Number of 65536us periods before considering rc pulse lost

; These are now defaults which can be adjusted via throttle calibration
; (stick high, stick low, (stick neutral) at start).
; These might be a bit wide for most radios, but lines up with POWER_RANGE.
.equ	STOP_RC_PULS	= 1060	; Stop motor at or below this pulse length
.equ	FULL_RC_PULS	= 1860	; Full speed at or above this pulse length
.equ	MAX_RC_PULS	= 2400	; Throw away any pulses longer than this
.equ	MIN_RC_PULS	= 768	; Throw away any pulses shorter than this
.equ	MID_RC_PULS	= (STOP_RC_PULS + FULL_RC_PULS) / 2	; Neutral when RC_PULS_REVERSE = 1
.equ	RCP_ALIAS_SHIFT	= 3	; Enable 1/8th PWM input alias ("oneshot125")
.equ	BEEP_RCP_ERROR	= 0	; Beep at stop if invalid PWM pulses were received

.if	RC_PULS_REVERSE
.equ	RCP_DEADBAND	= 50	; Do not start until this much above or below neutral
.equ	PROGRAM_RC_PULS	= (STOP_RC_PULS + FULL_RC_PULS * 3) / 4	; Normally 1660
.else
.equ	RCP_DEADBAND	= 0
.equ	PROGRAM_RC_PULS	= (STOP_RC_PULS + FULL_RC_PULS) / 2	; Normally 1460
.endif

.if	LOW_BRAKE
.equ	RCP_LOW_DBAND	= 60	; Brake at this many microseconds below low pulse
.endif

.equ	MAX_DRIFT_PULS	= 10	; Maximum jitter/drift microseconds during programming

; Minimum PWM on-time (too low and FETs won't turn on, hard starting)
.if !defined(MIN_DUTY)
.equ	MIN_DUTY	= 56 * CPU_MHZ / 16
.endif

; Number of PWM steps (too high and PWM frequency drops into audible range)
.if !defined(POWER_RANGE)
.equ	POWER_RANGE	= 800 * CPU_MHZ / 16 + MIN_DUTY
.endif

.equ	MAX_POWER	= (POWER_RANGE-1)
.equ	PWR_COOL_START	= (POWER_RANGE/24) ; Power limit while starting to reduce heating
.equ	PWR_MIN_START	= (POWER_RANGE/6) ; Power limit while starting (to start)
.equ	PWR_MAX_START	= (POWER_RANGE/4) ; Power limit while starting (if still not running)
.equ	PWR_MAX_RPM1	= (POWER_RANGE/4) ; Power limit when running slower than TIMING_RANGE1
.equ	PWR_MAX_RPM2	= (POWER_RANGE/2) ; Power limit when running slower than TIMING_RANGE2

.equ	BRAKE_POWER	= MAX_POWER*2/3	; Brake force is exponential, so start fairly high
.equ	BRAKE_SPEED	= 3		; Speed to reach MAX_POWER, 0 (slowest) - 8 (fastest)
.equ	LOW_BRAKE_POWER	= MAX_POWER*2/3
.equ	LOW_BRAKE_SPEED	= 5

.equ	TIMING_MIN	= 0x8000 ; 8192us per commutation
.equ	TIMING_RANGE1	= 0x4000 ; 4096us per commutation
.equ	TIMING_RANGE2	= 0x2000 ; 2048us per commutation
.equ	TIMING_RANGE3	= 0x1000 ; 1024us per commutation
.equ	TIMING_MAX	= 0x0080 ; 32us per commutation (312,500eRPM)

.equ	TIMEOUT_START	= 48000	; Timeout per commutation for ZC during starting
.if !defined(START_DELAY_US)
.equ	START_DELAY_US	= 0	; Initial post-commutation wait during starting
.endif
.equ	START_DSTEP_US	= 8	; Microseconds per start delay step
.equ	START_DELAY_INC	= 15	; Wait step count increase (wraps in a byte)
.equ	START_MOD_INC	= 4	; Start power modulation step count increase (wraps in a byte)
.equ	START_MOD_LIMIT	= 48	; Value at which power is reduced to avoid overheating
.equ	START_FAIL_INC	= 16	; start_tries step count increase (wraps in a byte, upon which we disarm)

.equ	ENOUGH_GOODIES	= 12	; This many start cycles without timeout will transition to running mode
.equ	ZC_CHECK_FAST	= 12	; Number of ZC check loops under which PWM noise should not matter
.equ	ZC_CHECK_MAX	= POWER_RANGE / 32 ; Limit ZC checking to about 1/2 PWM interval
.equ	ZC_CHECK_MIN	= 3

.equ	T0CLK		= (1<<CS01)	; clk/8 == 2MHz
.equ	T1CLK		= (1<<CS10)+(USE_ICP<<ICES1)+(USE_ICP<<ICNC1)	; clk/1 == 16MHz
.equ	T2CLK		= (1<<CS20)	; clk/1 == 16MHz

.equ	EEPROM_SIGN	= 31337		; Random 16-bit value
.equ	EEPROM_OFFSET	= 0x80		; Offset into 512-byte space (why not)

; Conditional code inclusion
.set	DEBUG_TX	= 0		; Output debugging on UART TX pin
.set	ADC_READ_NEEDED	= 0		; Reading from ADCs

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
.def	temp7		= r10		; really aux temporary (limited operations)
.def	rc_timeout	= r11
.def	sys_control_l	= r12		; duty limit low (word register aligned)
.def	sys_control_h	= r13		; duty limit high
.def	timing_duty_l	= r14		; timing duty limit low
.def	timing_duty_h	= r15		; timing duty limit high
.def	flags0		= r16	; state flags
	.equ	OCT1_PENDING	= 0	; if set, output compare interrupt is pending
	.equ	SET_DUTY	= 1	; if set when armed, set duty during evaluate_rc
	.equ	RCP_ERROR	= 2	; if set, corrupted PWM input was seen
	.equ	RCP_ALIAS	= 3	; if set, rc alias (shifted) range is active
	.equ	EEPROM_RESET	= 4	; if set, reset EEPROM
	.equ	EEPROM_WRITE	= 5	; if set, save settings to EEPROM
	.equ	UART_SYNC	= 6	; if set, we are waiting for our serial throttle byte
	.equ	NO_CALIBRATION	= 7	; if set, disallow calibration (unsafe reset cause)
.def	flags1		= r17	; state flags
	.equ	POWER_ON	= 0	; if set, switching fets is enabled
	.equ	FULL_POWER	= 1	; 100% on - don't switch off, but do OFF_CYCLE working
	.equ	I2C_MODE	= 2	; if receiving updates via I2C
	.equ	UART_MODE	= 3	; if receiving updates via UART
	.equ	EVAL_RC		= 4	; if set, evaluate rc command while waiting for OCT1
	.equ	ACO_EDGE_HIGH	= 5	; if set, looking for ACO high - same bit position as ACO
	.equ	STARTUP		= 6	; if set, startup-phase is active
	.equ	REVERSE		= 7	; if set, do reverse commutation
.def	flags2		= r18
	.equ	A_FET		= 0	; if set, A FET is being PWMed
	.equ	B_FET		= 1	; if set, B FET is being PWMed
	.equ	C_FET		= 2	; if set, C FET is being PWMed
	.equ	ALL_FETS	= (1<<A_FET)+(1<<B_FET)+(1<<C_FET)
	.equ	TIMING_FAST	= 6	; if set, timing fits in 16 bits
	.equ	SKIP_CPWM	= 7	; if set, skip complementary PWM (for short off period)
;.def			= r19
.def	i_temp1		= r20		; interrupt temporary
.def	i_temp2		= r21		; interrupt temporary
.def	temp3		= r22		; main temporary (L)
.def	temp4		= r23		; main temporary (H)
.def	temp1		= r24		; main temporary (L), adiw-capable
.def	temp2		= r25		; main temporary (H), adiw-capable

; XL: general temporary
; XH: general temporary
; YL: general temporary
; YH: general temporary
; ZL: Next PWM interrupt vector (low)
; ZH: Next PWM interrupt vector (high, stays at zero) -- used as "zero" register

;**** **** **** **** ****
; RAM Definitions
.dseg				; DATA segment
.org SRAM_START

orig_osccal:	.byte	1	; original OSCCAL value
goodies:	.byte	1	; Number of rounds without timeout
powerskip:	.byte	1	; Skip power through this number of steps
ocr1ax:		.byte	1	; 3rd byte of OCR1A
tcnt1x:		.byte	1	; 3rd byte of TCNT1
pwm_on_ptr:	.byte	1	; Next PWM ON vector
rct_boot:	.byte	1	; Counter which increments while rc_timeout is 0 to jump to boot loader
rct_beacon:	.byte	1	; Counter which increments while rc_timeout is 0 to disarm and beep occasionally
idle_beacon:	.byte	1	; Counter which increments while armed and idle to beep occasionally
last_tcnt1_l:	.byte	1	; last timer1 value
last_tcnt1_h:	.byte	1
last_tcnt1_x:	.byte	1
l2_tcnt1_l:	.byte	1	; last last timer1 value
l2_tcnt1_h:	.byte	1
l2_tcnt1_x:	.byte	1
timing_l:	.byte	1	; interval of 2 commutations
timing_h:	.byte	1
timing_x:	.byte	1
com_time_l:	.byte	1	; time of last commutation
com_time_h:	.byte	1
com_time_x:	.byte	1
start_delay:	.byte	1	; delay count after starting commutations before checking back-EMF
start_modulate:	.byte	1	; Start modulation counter (to reduce heating from PWR_MAX_START if stuck)
start_fail:	.byte   1	; Number of start_modulate loops for eventual failure and disarm
rcp_beep_count:	.byte	1	; Number of RC pulse error beeps
rc_duty_l:	.byte	1	; desired duty cycle
rc_duty_h:	.byte	1
fwd_scale_l:	.byte	1	; 16.16 multipliers to scale input RC pulse to POWER_RANGE
fwd_scale_h:	.byte	1
rev_scale_l:	.byte	1
rev_scale_h:	.byte	1
neutral_l:	.byte	1	; Offset for neutral throttle (in CPU_MHZ)
neutral_h:	.byte	1
.if RCP_DEADBAND && defined(RCP_ALIAS_SHIFT)
deadband_l:	.byte	1	; Deadband scaled for possible input alias
deadband_h:	.byte	1
.endif
.if LOW_BRAKE
low_brake_l:	.byte	1	; Low brake position with deadband applied
low_brake_h:	.byte	1
.endif
.if USE_I2C
i2c_max_pwm:	.byte	1	; MaxPWM for MK (NOTE: 250 while stopped is magic and enables v2)
i2c_rx_state:	.byte	1
i2c_blc_offset:	.byte	1
.endif
motor_count:	.byte	1	; Motor number for serial control
brake_sub:	.byte	1	; Brake speed subtrahend (power of two)
brake_want:	.byte	1	; Type of brake desired
brake_active:	.byte	1	; Type of brake active
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
.if USE_I2C
blc_revision:	.byte	1	; BLConfig revision
blc_setmask:	.byte	1	; BLConfig settings mask
blc_pwmscaling:	.byte	1	; BLConfig pwm scaling
blc_currlimit:	.byte	1	; BLConfig current limit
blc_templimit:	.byte	1	; BLConfig temperature limit
blc_currscale:	.byte	1	; BLConfig current scaling
blc_bitconfig:	.byte	1	; BLConfig bitconfig (1 == MOTOR_REVERSE)
blc_checksum:	.byte	1	; BLConfig checksum (0xaa + above bytes)
.endif
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
	.db byte1(MID_RC_PULS * CPU_MHZ), byte2(MID_RC_PULS * CPU_MHZ)
.if USE_I2C
.equ	BL_REVISION	= 2
	.db BL_REVISION, 144	; Revision, SetMask -- Settings mask should encode MOTOR_REVERSE bit
	.db 255, 255		; PwmScaling, CurrentLimit
	.db 127, 0		; TempLimit, CurrentScaling
	.db 0, byte1(0xaa + BL_REVISION + 144 + 255 + 255 + 127 + 0 + 0)	; BitConfig, crc (0xaa + sum of above bytes)
.endif

;-- Instruction extension macros -----------------------------------------

; Add any 16-bit immediate to a register pair (@0:@1 += @2), no Z flag
.macro adi2
	.if byte1(-@2)
		subi	@0, byte1(-@2)
		sbci	@1, byte1(-byte2(@2 + 0xff))
	.else
		subi	@1, byte1(-byte2(@2 + 0xff))
	.endif
.endmacro

; Smaller version for r24 and above, Z flag not reliable
.macro adiwx
	.if !(@2 & 0xff) || (@2) & ~0x3f
		adi2	@0, @1, @2
	.else
		adiw	@0, @2
	.endif
.endmacro

; Compare any 16-bit immediate from a register pair (@0:@1 -= @2, maybe clobbering @3)
.macro cpiz2
		cpi	@0, byte1(@2)
	.if byte2(@2)
		ldi	@3, byte2(@2)
		cpc	@1, @3
	.else
		cpc	@1, ZH
	.endif
.endmacro

; Compare any 16-bit immediate from a register pair (@0:@1 -= @2, maybe clobbering @3), no Z flag
; Do not follow by Z flag tests like breq, brne, brlt, brge, brlo, brsh!
; The idea here is that the low byte being compared with (subtracted by)
; 0 will never set carry, so skipping it and cpi'ing the high byte is the
; same other than the result of the Z flag.
.macro cpi2
	.if byte1(@2)
		cpiz2	@0, @1, @2, @3
	.else
		cpi	@1, byte2(@2)
	.endif
.endmacro

; Compare any 24-bit immediate from a register triplet (@0:@1:@2 -= @3, maybe clobbering @4)
.macro cpiz3
		cpi	@0, byte1(@3)
	.if byte2(@3)
		ldi	@4, byte2(@3)
		cpc	@1, @4
	.else
		cpc	@1, ZH
	.endif
	.if byte3(@3)
		ldi	@4, byte3(@3)
		cpc	@2, @4
	.else
		cpc	@2, ZH
	.endif
.endmacro

; Compare any 24-bit immediate from a register triplet (@0:@1:@2 -= @3, maybe clobbering @4)
; May not set Z flag, as above.
.macro cpi3
	.if byte1(@3)
		cpiz3	@0, @1, @2, @3, @4
	.else
		cpi2	@1, @2, @3 >> 8, @4
	.endif
.endmacro

; Subtract any 16-bit immediate from a register pair (@0:@1 -= @2), no Z flag
.macro sbi2
	.if byte1(@2)
		subi	@0, byte1(@2)
		sbci	@1, byte2(@2)
	.else
		subi	@1, byte2(@2)
	.endif
.endmacro

; Smaller version for r24 and above, Z flag not reliable
.macro sbiwx
	.if !(@2 & 0xff) || (@2) & ~0x3f
		sbi2	@0, @1, @2
	.else
		sbiw	@0, @2
	.endif
.endmacro

; Load 2-byte immediate
.macro ldi2
		ldi	@0, byte1(@2)
		ldi	@1, byte2(@2)
.endmacro

; Load 3-byte immediate
.macro ldi3
		ldi	@0, byte1(@3)
		ldi	@1, byte2(@3)
		ldi	@2, byte3(@3)
.endmacro

; Register out to any address (memory-mapped if necessary)
.macro outr
	.if @0 < 64
		out	@0, @1
	.else
		sts	@0, @1
	.endif
.endmacro

; Register in from any address (memory-mapped if necessary)
.macro inr
	.if @1 < 64
		in	@0, @1
	.else
		lds	@0, @1
	.endif
.endmacro

; Immediate out to any port (possibly via @2 as a temporary)
.macro outi
	.if @1
		ldi	@2, @1
		outr	@0, @2
	.else
		outr	@0, ZH
	.endif
.endmacro

;-- LED macros -----------------------------------------------------------

.if !defined(red_led)
	.macro RED_on
	.endmacro
	.macro RED_off
	.endmacro
.endif

.if !defined(green_led)
	.macro GRN_on
	.endmacro
	.macro GRN_off
	.endmacro
.endif

.if !defined(blue_led)
	.macro BLUE_on
	.endmacro
	.macro BLUE_off
	.endmacro
.endif

;-- FET driving macros ---------------------------------------------------
; Careful: "if" conditions split over multiple lines (with backslashes)
; work with arva, but avrasm2.exe silently produces wrong results.

.if !defined(HIGH_SIDE_PWM)
.equ HIGH_SIDE_PWM = 0
.endif

.if defined(AnFET)
; Traditional direct high and low side drive, inverted if INIT_Px is set.
; Dead-time insertion is supported for COMP_PWM.

	.equ CPWM_SOFT = COMP_PWM

	.macro FET_on
		.if (INIT_PB & ((@0 == PORTB) << @1)) | (INIT_PC & ((@0 == PORTC) << @1)) | (INIT_PD & ((@0 == PORTD) << @1))
		cbi	@0, @1
		.else
		sbi	@0, @1
		.endif
	.endmacro

	.macro FET_off
		.if (INIT_PB & ((@0 == PORTB) << @1)) | (INIT_PC & ((@0 == PORTC) << @1)) | (INIT_PD & ((@0 == PORTD) << @1))
		sbi	@0, @1
		.else
		cbi	@0, @1
		.endif
	.endmacro

	.macro AnFET_on
		FET_on	AnFET_port, AnFET
	.endmacro
	.macro AnFET_off
		FET_off	AnFET_port, AnFET
	.endmacro
	.macro ApFET_on
		FET_on	ApFET_port, ApFET
	.endmacro
	.macro ApFET_off
		FET_off	ApFET_port, ApFET
	.endmacro
	.macro BnFET_on
		FET_on	BnFET_port, BnFET
	.endmacro
	.macro BnFET_off
		FET_off	BnFET_port, BnFET
	.endmacro
	.macro BpFET_on
		FET_on	BpFET_port, BpFET
	.endmacro
	.macro BpFET_off
		FET_off	BpFET_port, BpFET
	.endmacro
	.macro CnFET_on
		FET_on	CnFET_port, CnFET
	.endmacro
	.macro CnFET_off
		FET_off	CnFET_port, CnFET
	.endmacro
	.macro CpFET_on
		FET_on	CpFET_port, CpFET
	.endmacro
	.macro CpFET_off
		FET_off	CpFET_port, CpFET
	.endmacro

	.macro PWM_FOCUS_A_on
		.if COMP_PWM
		cpse	temp3, temp4
		PWM_COMP_A_on
		.endif
	.endmacro
	.macro PWM_FOCUS_A_off
		.if COMP_PWM
		in	temp3, PWM_COMP_A_PORT_in
		PWM_COMP_A_off
		in	temp4, PWM_COMP_A_PORT_in
		.endif
	.endmacro
	.macro PWM_FOCUS_B_on
		.if COMP_PWM
		cpse	temp3, temp4
		PWM_COMP_B_on
		.endif
	.endmacro
	.macro PWM_FOCUS_B_off
		.if COMP_PWM
		in	temp3, PWM_COMP_B_PORT_in
		PWM_COMP_B_off
		in	temp4, PWM_COMP_B_PORT_in
		.endif
	.endmacro
	.macro PWM_FOCUS_C_on
		.if COMP_PWM
		cpse	temp3, temp4
		PWM_COMP_C_on
		.endif
	.endmacro
	.macro PWM_FOCUS_C_off
		.if COMP_PWM
		in	temp3, PWM_COMP_C_PORT_in
		PWM_COMP_C_off
		in	temp4, PWM_COMP_C_PORT_in
		.endif
	.endmacro

	; For PWM state mirroring in commutation routines
	.if HIGH_SIDE_PWM
		.equ	PWM_A_PORT_in = ApFET_port
		.equ	PWM_B_PORT_in = BpFET_port
		.equ	PWM_C_PORT_in = CpFET_port
		.equ	PWM_COMP_A_PORT_in = AnFET_port
		.equ	PWM_COMP_B_PORT_in = BnFET_port
		.equ	PWM_COMP_C_PORT_in = CnFET_port
	.else
		.equ	PWM_A_PORT_in = AnFET_port
		.equ	PWM_B_PORT_in = BnFET_port
		.equ	PWM_C_PORT_in = CnFET_port
		.equ	PWM_COMP_A_PORT_in = ApFET_port
		.equ	PWM_COMP_B_PORT_in = BpFET_port
		.equ	PWM_COMP_C_PORT_in = CpFET_port
	.endif

	.macro PWM_ALL_off
		.if HIGH_SIDE_PWM
		all_pFETs_off @0
		.else
		all_nFETs_off @0
		.endif
	.endmacro

	.macro all_pFETs_off
	.if ApFET_port != BpFET_port || ApFET_port != CpFET_port
		ApFET_off
		BpFET_off
		CpFET_off
	.else
		in	@0, ApFET_port
		.if (INIT_PB & ((ApFET_port == PORTB) << ApFET)) | (INIT_PC & ((ApFET_port == PORTC) << ApFET)) | (INIT_PD & ((ApFET_port == PORTD) << ApFET))
		sbr	@0, (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
		.else
		cbr	@0, (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
		.endif
		out	ApFET_port, @0
	.endif
	.endmacro

	.macro all_nFETs_off
	.if AnFET_port != BnFET_port || AnFET_port != CnFET_port
		AnFET_off
		BnFET_off
		CnFET_off
	.else
		in	@0, AnFET_port
		.if (INIT_PB & ((AnFET_port == PORTB) << AnFET)) | (INIT_PC & ((AnFET_port == PORTC) << AnFET)) | (INIT_PD & ((AnFET_port == PORTD) << AnFET))
		sbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
		.else
		cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
		.endif
		out	AnFET_port, @0
	.endif
	.endmacro

	.macro nFET_brake
	.if AnFET_port != BnFET_port || AnFET_port != CnFET_port
		AnFET_on
		BnFET_on
		CnFET_on
	.else
		in	@0, AnFET_port
		.if (INIT_PB & ((AnFET_port == PORTB) << AnFET)) | (INIT_PC & ((AnFET_port == PORTC) << AnFET)) | (INIT_PD & ((AnFET_port == PORTD) << AnFET))
		cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
		.else
		sbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
		.endif
		out	AnFET_port, @0
	.endif
	.endmacro

.elif defined(ENABLE_ALL)
; Three logic level PWM/ENABLE-style driver, with diode emulation mode or
; off state at the middle level on the PWM pin. This is accomplished by
; setting a pull-up rather than drive high. With this method, every FET
; toggle can be a single I/O instruction, rather than having to select
; high or low in advance to toggling enable. The following macros are used:
;
; XnFET_on  -> cbi PWM_X_PORT, PWM_X (drain through ext pull-down)
; XnFET_off -> sbi PWM_X_PORT, PWM_X (pull-up pin with ext pull-down)
; XpFET_on  -> sbi PWM_X_DDR, PWM_X (drive-up pin)
; XpFET_off -> cbi PWM_X_DDR, PWM_X (pull-up pin with ext pull-down)
;
; COMP_PWM on these is done in hardware rather than software, so we can
; just toggle the PORT value after PWM_FOCUS sets DDR (output mode).
; This results in the following macro arrangement:
;
; TRI	CPWM	HIGH_SIDE_PWM	: PWM ON	PWM OFF		PWM_X_PORT_in
; 0	0	0		: XnFET_on	XnFET_off	XnFET_port
; 0	0	1		: XpFET_on	XpFET_off	XpFET_port
; 0	1	0		: XnFET_on	XnFET_off	XnFET_port
; 0	1	1		: XpFET_on	XpFET_off	XpFET_port
; 1	0	0		: XnFET_on	XnFET_off	PWM_X_PORT
; 1	0	1		: XpFET_on	XpFET_off	PWM_X_DDR
; 1	1	0		: XnFET_on	XnFET_off	PWM_X_PORT
; 1	1	1		: XnFET_off	XnFET_on	PWM_X_PORT
;
; For the last case, PWM_X_off actually turns low side on which isn't how
; we want to leave the phase after commutating. PWM_X_clear will take care
; of this.
;
; We leave ENABLE high once initialized as some drivers will actually
; shut down rather than just using the input as a logic gate.
;
; We prefer HIGH_SIDE_PWM as diode emulation mode in these drivers
; typically allows active freewheeling only in this orientation.

	.equ CPWM_SOFT = 0

	.macro AnFET_on
		cbi	PWM_A_PORT, PWM_A
	.endmacro
	.macro AnFET_off
		sbi	PWM_A_PORT, PWM_A
	.endmacro
	.macro ApFET_on
		sbi	PWM_A_DDR, PWM_A
	.endmacro
	.macro ApFET_off
		cbi	PWM_A_DDR, PWM_A
	.endmacro
	.macro BnFET_on
		cbi	PWM_B_PORT, PWM_B
	.endmacro
	.macro BnFET_off
		sbi	PWM_B_PORT, PWM_B
	.endmacro
	.macro BpFET_on
		sbi	PWM_B_DDR, PWM_B
	.endmacro
	.macro BpFET_off
		cbi	PWM_B_DDR, PWM_B
	.endmacro
	.macro CnFET_on
		cbi	PWM_C_PORT, PWM_C
	.endmacro
	.macro CnFET_off
		sbi	PWM_C_PORT, PWM_C
	.endmacro
	.macro CpFET_on
		sbi	PWM_C_DDR, PWM_C
	.endmacro
	.macro CpFET_off
		cbi	PWM_C_DDR, PWM_C
	.endmacro

	.macro PWM_FOCUS_A_on
		.if COMP_PWM
		sbrc	flags1, POWER_ON
		ApFET_on
		.endif
	.endmacro
	.macro PWM_FOCUS_A_off
		.if COMP_PWM
		ApFET_off
		.endif
	.endmacro

	.macro PWM_FOCUS_B_on
		.if COMP_PWM
		sbrc	flags1, POWER_ON
		BpFET_on
		.endif
	.endmacro
	.macro PWM_FOCUS_B_off
		.if COMP_PWM
		BpFET_off
		.endif
	.endmacro

	.macro PWM_FOCUS_C_on
		.if COMP_PWM
		sbrc	flags1, POWER_ON
		CpFET_on
		.endif
	.endmacro
	.macro PWM_FOCUS_C_off
		.if COMP_PWM
		CpFET_off
		.endif
	.endmacro

	; For PWM state mirroring in commutation routines
	.if COMP_PWM || !HIGH_SIDE_PWM
		.equ	PWM_A_PORT_in = PWM_A_PORT
		.equ	PWM_B_PORT_in = PWM_B_PORT
		.equ	PWM_C_PORT_in = PWM_C_PORT
	.else
		.equ	PWM_A_PORT_in = PWM_A_DDR
		.equ	PWM_B_PORT_in = PWM_B_DDR
		.equ	PWM_C_PORT_in = PWM_C_DDR
	.endif

	.macro PWM_ALL_off
		all_nFETs_off @0
	.endmacro

	.macro all_pFETs_off
		.if PWM_A_DDR != PWM_B_DDR || PWM_A_DDR != PWM_C_DDR
			ApFET_off
			BpFET_off
			CpFET_off
		.else
			in	@0, PWM_A_DDR
			cbr	@0, (1<<PWM_A)+(1<<PWM_B)+(1<<PWM_C)
			out	PWM_A_DDR, @0
		.endif
	.endmacro

	.macro all_nFETs_off
		.if PWM_A_PORT != PWM_B_PORT || PWM_A_PORT != PWM_C_PORT
			AnFET_off
			BnFET_off
			CnFET_off
		.else
			in	@0, PWM_A_PORT
			sbr	@0, (1<<PWM_A)+(1<<PWM_B)+(1<<PWM_C)
			out	PWM_A_PORT, @0
		.endif
	.endmacro

	.macro nFET_brake
		.if PWM_A_PORT != PWM_B_PORT || PWM_A_PORT != PWM_C_PORT
			AnFET_on
			BnFET_on
			CnFET_on
		.else
			in	@0, PWM_A_PORT
			cbr	@0, (1<<PWM_A)+(1<<PWM_B)+(1<<PWM_C)
			out	PWM_A_PORT, @0
		.endif
	.endmacro

.endif

;-- Commutation drive macros ---------------------------------------------

.if HIGH_SIDE_PWM
	.macro COMMUTATE_A_on
		AnFET_on
	.endmacro
	.macro COMMUTATE_A_off
		AnFET_off
	.endmacro
	.macro COMMUTATE_B_on
		BnFET_on
	.endmacro
	.macro COMMUTATE_B_off
		BnFET_off
	.endmacro
	.macro COMMUTATE_C_on
		CnFET_on
	.endmacro
	.macro COMMUTATE_C_off
		CnFET_off
	.endmacro
.else
	.macro COMMUTATE_A_on
		ApFET_on
	.endmacro
	.macro COMMUTATE_A_off
		ApFET_off
	.endmacro
	.macro COMMUTATE_B_on
		BpFET_on
	.endmacro
	.macro COMMUTATE_B_off
		BpFET_off
	.endmacro
	.macro COMMUTATE_C_on
		CpFET_on
	.endmacro
	.macro COMMUTATE_C_off
		CpFET_off
	.endmacro
.endif

;-- PWM macros -----------------------------------------------------------

.macro PWM_A_on
	.if !defined(AnFET) && COMP_PWM && HIGH_SIDE_PWM
		AnFET_off
	.elif HIGH_SIDE_PWM
		ApFET_on
	.else
		AnFET_on
	.endif
.endmacro
.macro PWM_A_off
	.if !defined(AnFET) && COMP_PWM && HIGH_SIDE_PWM
		AnFET_on
	.elif HIGH_SIDE_PWM
		ApFET_off
	.else
		AnFET_off
	.endif
.endmacro
.macro PWM_A_clear
		in	temp1, PWM_A_PORT_in
	.if !defined(AnFET) && COMP_PWM && HIGH_SIDE_PWM
		AnFET_off
	.elif HIGH_SIDE_PWM
		ApFET_off
	.else
		AnFET_off
	.endif
		in	temp2, PWM_A_PORT_in
.endmacro
.macro PWM_A_copy
		cpse	temp1, temp2
	.if !defined(AnFET) && COMP_PWM && HIGH_SIDE_PWM
		AnFET_on
	.elif HIGH_SIDE_PWM
		ApFET_on
	.else
		AnFET_on
	.endif
.endmacro

.macro PWM_B_on
	.if !defined(BnFET) && COMP_PWM && HIGH_SIDE_PWM
		BnFET_off
	.elif HIGH_SIDE_PWM
		BpFET_on
	.else
		BnFET_on
	.endif
.endmacro
.macro PWM_B_off
	.if !defined(BnFET) && COMP_PWM && HIGH_SIDE_PWM
		BnFET_on
	.elif HIGH_SIDE_PWM
		BpFET_off
	.else
		BnFET_off
	.endif
.endmacro
.macro PWM_B_clear
		in	temp1, PWM_B_PORT_in
	.if !defined(BnFET) && COMP_PWM && HIGH_SIDE_PWM
		BnFET_off
	.elif HIGH_SIDE_PWM
		BpFET_off
	.else
		BnFET_off
	.endif
		in	temp2, PWM_B_PORT_in
.endmacro
.macro PWM_B_copy
		cpse	temp1, temp2
	.if !defined(BnFET) && COMP_PWM && HIGH_SIDE_PWM
		BnFET_on
	.elif HIGH_SIDE_PWM
		BpFET_on
	.else
		BnFET_on
	.endif
.endmacro

.macro PWM_C_on
	.if !defined(CnFET) && COMP_PWM && HIGH_SIDE_PWM
		CnFET_off
	.elif HIGH_SIDE_PWM
		CpFET_on
	.else
		CnFET_on
	.endif
.endmacro
.macro PWM_C_off
	.if !defined(CnFET) && COMP_PWM && HIGH_SIDE_PWM
		CnFET_on
	.elif HIGH_SIDE_PWM
		CpFET_off
	.else
		CnFET_off
	.endif
.endmacro
.macro PWM_C_clear
		in	temp1, PWM_C_PORT_in
	.if !defined(CnFET) && COMP_PWM && HIGH_SIDE_PWM
		CnFET_off
	.elif HIGH_SIDE_PWM
		CpFET_off
	.else
		CnFET_off
	.endif
		in	temp2, PWM_C_PORT_in
.endmacro
.macro PWM_C_copy
		cpse	temp1, temp2
	.if !defined(CnFET) && COMP_PWM && HIGH_SIDE_PWM
		CnFET_on
	.elif HIGH_SIDE_PWM
		CpFET_on
	.else
		CnFET_on
	.endif
.endmacro

;-- Complementary PWM macros ---------------------------------------------

.if CPWM_SOFT
	.macro PWM_COMP_A_on
		.if HIGH_SIDE_PWM
			AnFET_on
		.else
			ApFET_on
		.endif
	.endmacro

	.macro PWM_COMP_A_off
		.if HIGH_SIDE_PWM
			AnFET_off
		.else
			ApFET_off
		.endif
	.endmacro

	.macro PWM_COMP_B_on
		.if HIGH_SIDE_PWM
			BnFET_on
		.else
			BpFET_on
		.endif
	.endmacro

	.macro PWM_COMP_B_off
		.if HIGH_SIDE_PWM
			BnFET_off
		.else
			BpFET_off
		.endif
	.endmacro

	.macro PWM_COMP_C_on
		.if HIGH_SIDE_PWM
			CnFET_on
		.else
			CpFET_on
		.endif
	.endmacro

	.macro PWM_COMP_C_off
		.if HIGH_SIDE_PWM
			CnFET_off
		.else
			CpFET_off
		.endif
	.endmacro
.endif

;-- RC pulse setup and edge handling macros ------------------------------

.if USE_ICP
.macro rcp_int_enable
		in	@0, TIMSK
		sbr	@0, (1<<TICIE1)	; enable icp1_int
		out	TIMSK, @0
.endmacro
.macro rcp_int_disable
		in	@0, TIMSK
		cbr	@0, (1<<TICIE1)	; disable icp1_int
		out	TIMSK, @0
.endmacro
.macro rcp_int_rising_edge
		ldi	@0, T1CLK
		out	TCCR1B, @0
.endmacro
.macro rcp_int_falling_edge
		ldi	@0, T1CLK & ~(1<<ICES1)
		out	TCCR1B, @0
.endmacro
.elif USE_INT0
.macro rcp_int_enable
		ldi	@0, (1<<INT0)	; enable ext_int0
		out	GICR, @0
.endmacro
.macro rcp_int_disable
		out	GICR, ZH	; disable ext_int0
.endmacro
.if USE_INT0 == 1
.macro rcp_int_rising_edge
		ldi	@0, (1<<ISC01)+(1<<ISC00)
		out	MCUCR, @0	; set next int0 to rising edge
.endmacro
.macro rcp_int_falling_edge
		ldi	@0, (1<<ISC01)
		out	MCUCR, @0	; set next int0 to falling edge
.endmacro
.elif USE_INT0 == 2
.macro rcp_int_rising_edge
		ldi	@0, (1<<ISC01)
		out	MCUCR, @0	; set next int0 to falling edge
.endmacro
.macro rcp_int_falling_edge
		ldi	@0, (1<<ISC01)+(1<<ISC00)
		out	MCUCR, @0	; set next int0 to rising edge
.endmacro
.endif
.endif

;-- Analog comparator sense macros ---------------------------------------
; We enable and disable the ADC to override ACME when one of the sense
; pins is AIN1 instead of an ADC pin. In the future, this will allow
; reading from the ADC at the same time.

.macro comp_init
		in	@0, SFIOR
		sbr	@0, (1<<ACME)	; set Analog Comparator Multiplexer Enable
		out	SFIOR, @0
	.if defined(mux_a) && defined(mux_b) && defined(mux_c)
		cbi	ADCSRA, ADEN	; Disable ADC to make sure ACME works
	.endif
.endmacro
.macro comp_adc_disable
	.if !defined(mux_a) || !defined(mux_b) || !defined(mux_c)
		cbi	ADCSRA, ADEN	; Disable ADC if we enabled it to get AIN1
	.endif
.endmacro
.macro comp_adc_enable
		sbi	ADCSRA, ADEN	; Eisable ADC to effectively disable ACME
.endmacro
.macro set_comp_phase_a
	.if defined(mux_a)
		ldi	@0, mux_a	; set comparator multiplexer to phase A
		out	ADMUX, @0
		comp_adc_disable
	.else
		comp_adc_enable
	.endif
.endmacro
.macro set_comp_phase_b
	.if defined(mux_b)
		ldi	@0, mux_b	; set comparator multiplexer to phase B
		out	ADMUX, @0
		comp_adc_disable
	.else
		comp_adc_enable
	.endif
.endmacro
.macro set_comp_phase_c
	.if defined(mux_c)
		ldi	@0, mux_c	; set comparator multiplexer to phase C
		out	ADMUX, @0
		comp_adc_disable
	.else
		comp_adc_enable
	.endif
.endmacro

;-- Timing and motor debugging macros ------------------------------------

.macro flag_on
	.if MOTOR_DEBUG && (DIR_PB & (1<<4)) == 0
		sbi	PORTB, 4
	.endif
.endmacro
.macro flag_off
	.if MOTOR_DEBUG && (DIR_PB & (1<<4)) == 0
		cbi	PORTB, 4
	.endif
.endmacro
.macro sync_on
	.if MOTOR_DEBUG && (DIR_PB & (1<<3)) == 0
		sbi	PORTB, 3
	.elif MOTOR_DEBUG && (DIR_PB & (1<<5)) == 0
		sbi	PORTB, 5
	.endif
.endmacro
.macro sync_off
	.if MOTOR_DEBUG && (DIR_PB & (1<<3)) == 0
		cbi	PORTB, 3
	.elif MOTOR_DEBUG && (DIR_PB & (1<<5)) == 0
		cbi	PORTB, 5
	.endif
.endmacro

; Short cycle delay without clobbering flags
.equ	MAX_BUSY_WAIT_CYCLES	= 32
.macro cycle_delay
.if @0 >= MAX_BUSY_WAIT_CYCLES
.error "cycle_delay too long"
.endif
.if @0 > 0
	.if @0 & 1
		nop
	.endif
	.if @0 & 2
		rjmp	PC + 1
	.endif
	.if @0 & 4
		rjmp	PC + 1
		rjmp	PC + 1
	.endif
	.if @0 & 8
		nop
		rcall	wait_ret		; 3 cycles to call + 4 to return
	.endif
	.if @0 & 16
		rjmp	PC + 1
		rcall	wait_ret
		rcall	wait_ret
	.endif
.endif
.endmacro

;-----bko-----------------------------------------------------------------
; Timer2 overflow interrupt (output PWM) -- the interrupt vector actually
; "ijmp"s to Z, which should point to one of these entry points.
;
; We try to avoid clobbering (and thus needing to save/restore) flags;
; in, out, mov, ldi, cpse, etc. do not modify any flags, while dec does.
;
; We used to check the comparator (ACSR) here to help starting, since PWM
; switching is what introduces noise that affects the comparator result.
; However, timing of this is very sensitive to FET characteristics, and
; would work well on some boards but not at all on others without waiting
; another 100-200ns, which was enough to break other boards. So, instead,
; we do all of the ACSR sampling outside of the interrupt and do digital
; filtering. The AVR interrupt overhead also helps to shield the noise.
;
; We reload TCNT2 as the very last step so as to reduce PWM dead areas
; between the reti and the next interrupt vector execution, which still
; takes a good 4 (reti) + 4 (interrupt call) + 2 (ijmp) cycles. We also
; try to keep the switch on close to the start of pwm_on and switch off
; close to the end of pwm_aff to minimize the power bump at full power.
;
; pwm_*_high and pwm_again are called when the particular on/off cycle
; is longer than will fit in 8 bits. This is tracked in tcnt2h.

.if MOTOR_BRAKE || LOW_BRAKE
pwm_brake_on:
		cpse	tcnt2h, ZH
		rjmp	pwm_again
		in	i_sreg, SREG
		nFET_brake i_temp1
		ldi	i_temp1, 0xff
		cp	off_duty_l, i_temp1	; Check for 0 off-time
		cpc	off_duty_h, ZH
		breq	pwm_brake_on1
		ldi	ZL, pwm_brake_off	; Not full on, so turn it off next
		lds	i_temp2, brake_sub
		sub	sys_control_l, i_temp2
		brne	pwm_brake_on1
		neg	duty_l			; Increase duty
		sbc	duty_h, i_temp1		; i_temp1 is 0xff aka -1
		com	duty_l
		com	off_duty_l		; Decrease off duty
		sbc	off_duty_l, ZH
		sbc	off_duty_h, ZH
		com	off_duty_l
pwm_brake_on1:	mov	tcnt2h, duty_h
		out	SREG, i_sreg
		out	TCNT2, duty_l
		reti

pwm_brake_off:
		cpse	tcnt2h, ZH
		rjmp	pwm_again
		in	i_sreg, SREG
		ldi	ZL, pwm_brake_on
		mov	tcnt2h, off_duty_h
		all_nFETs_off i_temp1
		out	SREG, i_sreg
		out	TCNT2, off_duty_l
		reti
.endif

.if DEAD_TIME_HIGH > 7
.equ	EXTRA_DEAD_TIME_HIGH = DEAD_TIME_HIGH - 7
.else
.equ	EXTRA_DEAD_TIME_HIGH = 0
.endif

pwm_on_fast_high:
.if CPWM_SOFT && EXTRA_DEAD_TIME_HIGH > MAX_BUSY_WAIT_CYCLES
		in	i_sreg, SREG
		dec	tcnt2h
		brne	pwm_on_fast_high_again
		ldi	ZL, pwm_on_fast
pwm_on_fast_high_again:
		out	SREG, i_sreg
		reti
.endif

pwm_on_high:
		in	i_sreg, SREG
		dec	tcnt2h
		brne	pwm_on_again
		ldi	ZL, pwm_on
pwm_on_again:	out	SREG, i_sreg
		reti

pwm_again:
		in	i_sreg, SREG
		dec	tcnt2h
		out	SREG, i_sreg
		reti

pwm_on:
.if CPWM_SOFT
		sbrc	flags2, A_FET
		PWM_COMP_A_off
		sbrc	flags2, B_FET
		PWM_COMP_B_off
		sbrc	flags2, C_FET
		PWM_COMP_C_off
	.if EXTRA_DEAD_TIME_HIGH > MAX_BUSY_WAIT_CYCLES
		; Reschedule to interrupt once the dead time has passed
		.if high(EXTRA_DEAD_TIME_HIGH)
		ldi	i_temp1, high(EXTRA_DEAD_TIME_HIGH)
		mov	tcnt2h, i_temp1
		ldi	ZL, pwm_on_fast_high
		.else
		ldi	ZL, pwm_on_fast
		.endif
		ldi	i_temp1, 0xff - low(EXTRA_DEAD_TIME_HIGH)
		out	TCNT2, i_temp1
		reti				; Do something else while we wait
		.equ	CPWM_OVERHEAD_HIGH = 7 + 8 + EXTRA_DEAD_TIME_HIGH
	.else
		; Waste cycles to wait for the dead time
		cycle_delay EXTRA_DEAD_TIME_HIGH
		.equ	CPWM_OVERHEAD_HIGH = 7 + EXTRA_DEAD_TIME_HIGH
		; Fall through
	.endif
.endif
pwm_on_fast:
		sbrc	flags2, A_FET
		PWM_A_on
		sbrc	flags2, B_FET
		PWM_B_on
		sbrc	flags2, C_FET
		PWM_C_on
		ldi	ZL, pwm_off
		mov	tcnt2h, duty_h
		out	TCNT2, duty_l
		reti

pwm_wdr:					; Just reset watchdog
		wdr
		reti

pwm_off:
		cpse	tcnt2h, ZH		; 2 cycles to skip when tcnt2h is 0
		rjmp	pwm_again
		wdr				; 1 cycle: watchdog reset
		sbrc	flags1, FULL_POWER	; 2 cycles to skip if not full power
		rjmp	pwm_on			; None of this off stuff if full power
		lds	ZL, pwm_on_ptr		; 2 cycles
		mov	tcnt2h, off_duty_h	; 1 cycle
		sbrc	flags2, A_FET		; 2 cycles if skip, 1 cycle otherwise
		PWM_A_off			; 2 cycles (off at 12 cycles from entry)
		sbrc	flags2, B_FET		; Offset by 2 cycles here,
		PWM_B_off			; but still equal on-time
		sbrc	flags2, C_FET
		PWM_C_off
		out	TCNT2, off_duty_l	; 1 cycle
		.if CPWM_SOFT
		sbrc	flags2, SKIP_CPWM	; 2 cycles if skip, 1 cycle otherwise
		reti
		.if DEAD_TIME_LOW > 9
		.equ	EXTRA_DEAD_TIME_LOW = DEAD_TIME_LOW - 9
		.else
		.equ	EXTRA_DEAD_TIME_LOW = 0
		.endif
		cycle_delay EXTRA_DEAD_TIME_LOW - 2
		.equ	CPWM_OVERHEAD_LOW = 9 + EXTRA_DEAD_TIME_LOW
		sbrc	flags2, A_FET
		PWM_COMP_A_on
		sbrc	flags2, B_FET
		PWM_COMP_B_on
		sbrc	flags2, C_FET
		PWM_COMP_C_on
		.endif
		reti				; 4 cycles

.if high(pwm_off)
.error "high(pwm_off) is non-zero; please move code closer to start or use 16-bit (ZH) jump registers"
.endif
;-----bko-----------------------------------------------------------------
; timer1 output compare interrupt
t1oca_int:	in	i_sreg, SREG
		lds	i_temp1, ocr1ax
		subi	i_temp1, 1
		brcc	t1oca_int1
		cbr	flags0, (1<<OCT1_PENDING)	; signal OCT1A passed
t1oca_int1:	sts	ocr1ax, i_temp1
		out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; timer1 overflow interrupt (happens every 4096µs)
t1ovfl_int:	in	i_sreg, SREG
		lds	i_temp1, tcnt1x
		inc	i_temp1
		sts	tcnt1x, i_temp1
		.if BEACON_IDLE
		brne	t1ovfl_int0
		lds	i_temp2, idle_beacon
		inc	i_temp2
		sts	idle_beacon, i_temp2
t1ovfl_int0:
		.endif
		andi	i_temp1, 15			; Every 16 overflows
		brne	t1ovfl_int1
		tst	rc_timeout
		breq	t1ovfl_int2
		dec	rc_timeout
t1ovfl_int1:	out	SREG, i_sreg
		reti
t1ovfl_int2:	lds	i_temp1, rct_boot
		inc	i_temp1
		sts	rct_boot, i_temp1
		lds	i_temp1, rct_beacon
		inc	i_temp1
		sts	rct_beacon, i_temp1
		rjmp	t1ovfl_int1
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
		in	i_temp1, TCNT1L		; get timer1 values
		in	i_temp2, TCNT1H
		.if USE_INT0 == 1
		sbis	PIND, rcp_in		; evaluate edge of this interrupt
		.else
		sbic	PIND, rcp_in		; inverted signalling
		.endif
		.endif
		rjmp	falling_edge
rising_edge:
		in	i_sreg, SREG
		; Stuff this rise time plus MAX_RC_PULS into OCR1B.
		; We use this both to save the time it went high and
		; to get an interrupt to indicate high timeout.
		adiwx	i_temp1, i_temp2, MAX_RC_PULS * CPU_MHZ
		out	OCR1BH, i_temp2
		out	OCR1BL, i_temp1
		rcp_int_falling_edge i_temp1	; Set next int to falling edge
		ldi	i_temp1, (1<<OCF1B)	; Clear OCF1B flag
		out	TIFR, i_temp1
		out	SREG, i_sreg
		reti

rcpint_fail:
		in	i_sreg, SREG
		sbr	flags0, (1<<RCP_ERROR)
		rjmp	rcpint_exit

falling_edge:
		in	i_sreg, TIFR
		sbrc	i_sreg, OCF1B		; Too long high would set OCF1B
		rjmp	rcpint_fail
		in	i_sreg, SREG
		movw	rx_l, i_temp1		; Guaranteed to be valid, store immediately
		in	i_temp1, OCR1BL		; No atomic temp register used to read OCR1* registers
		in	i_temp2, OCR1BH
		sbi2	i_temp1, i_temp2, MAX_RC_PULS * CPU_MHZ	; Put back to start time
		sub	rx_l, i_temp1		; Subtract start time from current time
		sbc	rx_h, i_temp2
.if MAX_RC_PULS * CPU_MHZ > 0xffff
.error "MAX_RC_PULS * CPU_MHZ too big to fit in two bytes -- adjust it or the rcp_int code"
.endif
		sbr	flags1, (1<<EVAL_RC)
rcpint_exit:	rcp_int_rising_edge i_temp1	; Set next int to rising edge
		out	SREG, i_sreg
		reti
	.endif
;-----bko-----------------------------------------------------------------
; MK BL-Ctrl v1, v2 compatible input control
; Ctrl-click Settings in MKTool for reversing and additional settings
i2c_int:
	.if USE_I2C
		in	i_sreg, SREG
		in	i_temp1, TWSR
		cpi	i_temp1, 0x60		; rx: received our SLA+W
		breq	i2c_ack
		cpi	i_temp1, 0x80		; rx: data available, previously ACKed
		breq	i2c_rx_data
		cpi	i_temp1, 0xa0		; rx: stop/restart condition (end of message)
		breq	i2c_rx_stop
		cpi	i_temp1, 0xa8		; tx: received our SLA+R
		breq	i2c_tx_init
		cpi	i_temp1, 0xb8		; tx: data request, previously ACKed
		breq	i2c_tx_data
		cpi	i_temp1, 0xf8		; tx: no relevant state information
		breq	i2c_io_error
		cpse	i_temp1, ZH		; Bus error due to illegal start/stop condition
		rjmp	i2c_ack			; 0x88, 0xc0, etc.: enable listening
i2c_io_error:	ldi	i_temp1, (1<<TWIE)|(1<<TWEN)|(1<<TWSTO)|(1<<TWEA)|(1<<TWINT)
		rjmp	i2c_out

i2c_tx_init:	sbrc	rx_l, 7			; BLConfig struct requested?
		rjmp	i2c_tx_blconfig
		out	TWDR, ZH		; Send 0 as Current (dummy)
		ldi	i_temp1, 250		; Prepare MaxPWM value (250 when stopped enables MK BL-Ctrl proto v2)
		sbrc	flags1, POWER_ON
i2c_tx_datarep:	ldi	i_temp1, 255		; Send MaxPWM 255 when running (and repeat for Temperature)
		sts	i2c_max_pwm, i_temp1
		rjmp	i2c_ack
i2c_tx_data:	sbrc	rx_l, 7			; BLConfig struct requested?
		rjmp	i2c_tx_blconfig1
		lds	i_temp1, i2c_max_pwm	; MaxPWM value (has special meaning for MK)
		out	TWDR, i_temp1
		rjmp	i2c_tx_datarep		; Send 255 for Temperature for which we should get a NACK (0xc0)

i2c_tx_blconfig:
		ldi	i_temp1, blc_revision	; First BLConfig structure member
		sts	i2c_blc_offset, i_temp1
i2c_tx_blconfig1:
		mov	i_temp2, ZL		; Save Z
		lds	ZL, i2c_blc_offset
		ld	i_temp1, Z+
		sts	i2c_blc_offset, ZL
		out	TWDR, i_temp1
		cpi	ZL, blc_checksum + 1	; Past last structure member?
		mov	ZL, i_temp2		; Restore Z
		breq	i2c_nack		; No more space
		rjmp	i2c_ack

i2c_nack:	ldi	i_temp1, (1<<TWIE)|(1<<TWEN)|(1<<TWINT)
		rjmp	i2c_out
i2c_ack:	ldi	i_temp1, (1<<TWIE)|(1<<TWEN)|(1<<TWEA)|(1<<TWINT)
i2c_out:	out	TWCR, i_temp1
		out	SREG, i_sreg
		reti

i2c_rx_stop:	lds	i_temp1, i2c_rx_state
		cpse	i_temp1, ZH		; Skip if empty message or we were writing
		sbr	flags1, (1<<EVAL_RC)|(1<<I2C_MODE)	; i2c message received
		sts	i2c_rx_state, ZH
		rjmp	i2c_ack
i2c_rx_data:	lds	i_temp1, i2c_rx_state
		inc	i_temp1
		sts	i2c_rx_state, i_temp1
		cpi	i_temp1, 1
		brne	i2c_rx_data1
		in	rx_h, TWDR		; Receive high byte from bus
		mov	rx_l, ZH		; Zero low byte (we may not receive it)
		rjmp	i2c_ack
i2c_rx_data1:	cpi	i_temp1, 2
		brne	i2c_rx_blc
		in	rx_l, TWDR		; Receive low byte from bus
		rjmp	i2c_ack

i2c_rx_blc:	cpi	i_temp1, 3		; BLConfig revision
		brne	i2c_rx_blc1
		in	i_temp1, TWDR
		cpi	i_temp1, 2
		brne	i2c_nack		; We support only BLConfig revision 2
		rjmp	i2c_ack
i2c_rx_blc1:	cpi	i_temp1, 3 + blc_checksum - blc_revision	; Checksum field?
		breq	i2c_rx_blccsum
		mov	i_temp2, ZL		; Save Z
		ldi	ZL, blc_revision - 3
		add	ZL, i_temp1		; Z now points to BLConfig structure member
		in	i_temp1, TWDR		; Read BLConfig data byte
		st	Z, i_temp1		; Update structure member
		mov	ZL, i_temp2		; Restore Z
		lds	i_temp2, blc_checksum
		add	i_temp2, i_temp1
		sts	blc_checksum, i_temp2
		rjmp	i2c_ack			; More expected
i2c_rx_blccsum:	in	i_temp1, TWDR		; We can't do anything with the checksum, so just update it to remove setmask
		lds	i_temp2, blc_setmask	; After receiving, zero the settings mask
;out UDR, i_temp2
		sbrc	i_temp2, 6		; Reset EEPROM if bit 6 set in blc_setmask
		sbr	flags0, (1<<EEPROM_RESET)
		sbrc	i_temp2, 7		; Write EEPROM if bit 7 set in blc_setmask
		sbr	flags0, (1<<EEPROM_WRITE)
		sub	i_temp1, i_temp2
		ldi	i_temp2, 0b10010000	; Default to write EEPROM and Reverse direction options selected
		add	i_temp1, i_temp2
		sts	blc_setmask, i_temp2
		sts	blc_checksum, i_temp1
		rjmp	i2c_ack
	.endif
;-----bko-----------------------------------------------------------------
urxc_int:
; This is Bernhard's serial protocol implementation in the UART
; version here: http://home.versanet.de/~b-konze/blc_6a/blc_6a.htm
; This seems to be implemented for a project described here:
; http://www.control.aau.dk/uav/reports/10gr833/10gr833_student_report.pdf
; The UART runs at 38400 baud, N81. Input is ignored until >= 0xf5
; is received, where we start counting to MOTOR_ID, at which
; the received byte is used as throttle input. 0 is neutral,
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
; beeper: timer0 is set to 1µs/count
beep_f1:	ldi	temp2, 80
		ldi	temp4, 200
		RED_on
		BLUE_on
beep_f1_on:	BpFET_on
		AnFET_on
		rcall	beep
		brne	beep_f1_on
		BLUE_off
		RED_off
		ret

beep_f2:	ldi	temp2, 100
		ldi	temp4, 180
		GRN_on
		BLUE_on
beep_f2_on:	CpFET_on
		BnFET_on
		rcall	beep
		brne	beep_f2_on
		BLUE_off
		GRN_off
		ret

beep_f3:	ldi	temp2, 120
		ldi	temp4, 160
		BLUE_on
beep_f3_on:	ApFET_on
		CnFET_on
		rcall	beep
		brne	beep_f3_on
		BLUE_off
		ret

beep_f4:	ldi	temp2, 140
beep_f4_freq:	ldi	temp4, 140
beep_f4_fets:	RED_on
		GRN_on
		BLUE_on
beep_f4_on:	CpFET_on
		AnFET_on
		rcall	beep
		brne	beep_f4_on
		BLUE_off
		GRN_off
		RED_off
		ret

		; Fall through
;-----bko-----------------------------------------------------------------
; Interrupts no longer need to be disabled to beep, but the PWM interrupt
; must be muted first
beep:		out	TCNT0, ZH
beep1:		in	temp1, TCNT0
		cpi	temp1, 2*CPU_MHZ	; 32µs on
		brlo	beep1
		all_pFETs_off temp3
		all_nFETs_off temp3
		ldi	temp3, CPU_MHZ
beep2:		out	TCNT0, ZH
		wdr
beep3:		in	temp1, TCNT0
		cp	temp1, temp4
		brlo	beep3
		dec	temp3
		brne	beep2
		dec	temp2
		ret

wait240ms:	rcall	wait120ms
wait120ms:	rcall	wait60ms
wait60ms:	rcall	wait30ms
wait30ms:	ldi	temp2, 15
wait1:		ldi	temp3, CPU_MHZ
wait2:		out	TCNT0, ZH
		ldi	temp1, (1<<TOV0)	; Clear TOV0 by setting it
		out	TIFR, temp1
		wdr
wait3:		in	temp1, TIFR
		sbrs	temp1, TOV0
		rjmp	wait3
		dec	temp3
		brne	wait2
		dec	temp2
		brne	wait1
wait_ret:	ret

;-- EEPROM functions -----------------------------------------------------
; Interrupts must be disabled to avoid Z conflicts and content changes.
eeprom_check_reset:
	; Check EEPROM signature
		lds	temp1, eeprom_sig_l
		lds	temp2, eeprom_sig_h
		subi	temp1, low(EEPROM_SIGN)
		sbci	temp2, high(EEPROM_SIGN)
		breq	eeprom_good

	; Signature not good: set defaults in RAM, but do not write
	; to the EEPROM until we actually set something non-default
eeprom_reset1:	ldi2	YL, YH, eeprom_sig_l
		ldi	ZL, low(eeprom_defaults_w << 1)
eeprom_reset2:	lpm	temp1, Z+
		st	Y+, temp1
		cpi	YL, low(eeprom_end)
		brne	eeprom_reset2
eeprom_good:	ret

.if USE_I2C
eeprom_reset_block:
		cli
		push	ZL
		rcall	eeprom_reset1
		pop	ZL
		sei
		ret
.endif

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
		cbr	flags0, (1<<EEPROM_WRITE)
		ldi2	YL, YH, eeprom_sig_l
		ldi2	temp1, temp2, EEPROM_OFFSET
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
; Multiply temp1:temp2 by temp3:temp4 and add high 16 bits of result to Y.
; Clobbers temp5, temp6, and leaves the lower byte in temp7.
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

;-- Hardware diagnostics -------------------------------------------------
; Any brushless ESC based on the ATmega8 or similar must tie the sense
; neutral star to AIN0, and the three sense lines to three ADC pins or
; two ADC pins and AIN1. All of these pins are also normal I/O pins, so
; we can drive them and see if the ADC values move. A value that does not
; move indicates a shorted FET or that an incorrect board target has been
; flashed.
;
; Note: Some FET drivers such as the LM5109 can pull up the output a bit,
; making the "stuck high" test return a false positive. Perhaps it would
; be sufficient to test that the phases read as 0 _or_ can be pulled down
; by driving AIN0 to ground.
;
; In typical conditions on the ATmega8, I/O pins transition to low at
; about 1.42V and to high at about 1.86V. The ADC is 10-bit, however, and
; should work even with a strong sense voltage divider.
;
; Throughout all of this, the motor may be spinning. If so, we should wait
; long enough that each phase falls to 0V and all tests succeed.
;
.if CHECK_HARDWARE
.set ADC_READ_NEEDED = 1
.equ MAX_CHECK_LOOPS = 5000			; ADC check takes ~200us

hardware_check:
		clt

		; First, check that all sense lines are low.
		.if defined(mux_a)
		ldi	XL, 1			; Error code 1: Phase A stuck high
		ldi	temp4, mux_a
		rcall	check_sense_low
		.endif

		.if defined(mux_b)
		ldi	XL, 2			; Error code 2: Phase B stuck high
		ldi	temp4, mux_b
		rcall	check_sense_low
		.endif

		.if defined(mux_c)
		ldi	XL, 3			; Error code 3: Phase C stuck high
		ldi	temp4, mux_c
		rcall	check_sense_low
		.endif

		.if !defined(mux_a) || !defined(mux_b) || !defined(mux_c)
		ldi	XL, 4			; Error code 4: AIN1 stuck high
		ldi2	YL, YH, MAX_CHECK_LOOPS
check_ain1_low:	sbiw	YL, 1
		sbic	PIND, 7			; Skip loop if AIN1 low
		brne	check_ain1_low
		rcall	hw_error_eq
		.endif

		ldi	XL, 5			; Error code 5: AIN0 stuck high
		ldi2	YL, YH, MAX_CHECK_LOOPS
check_ain0_low:	sbiw	YL, 1
		sbic	PIND, 6			; Skip loop if AIN0 low
		brne	check_ain0_low
		rcall	hw_error_eq

		brts	hardware_check		; Do not allow further tests if stuck high

		; If nothing is stuck high, pull up the motor by driving
		; the emulated neutral (AIN0) high, and try driving each
		; phase low. While driven, leakage through the star is
		; eliminated, so one phase will not influence another
		; unless a motor is connected. A phase on AIN1 cannot be
		; read by the ADC, so we must skip it.

		sbi	DDRD, 6
		sbi	PORTD, 6		; Drive AIN0 high

		.if defined(mux_a)
		rcall	wait30ms		; There might be some capacitance
		ldi	XL, 6			; Error code 6: Phase A low-side drive broken
		ldi	temp4, mux_a
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		AnFET_on			; Drive down this phase (we've established that it was 0V above).
		rcall	adc_read		; FET turn-on will easily beat ADC initialization
		AnFET_off
		rcall	hw_error_y_le_temp12
		.endif

		.if defined(mux_b)
		rcall	wait30ms
		ldi	XL, 7			; Error code 7: Phase B low-side drive broken
		ldi	temp4, mux_b
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		BnFET_on			; Drive down this phase (we've established that it was 0V above).
		rcall	adc_read		; FET turn-on will easily beat ADC initialization
		BnFET_off
		rcall	hw_error_y_le_temp12
		.endif

		.if defined(mux_c)
		rcall	wait30ms
		ldi	XL, 8			; Error code 8: Phase C low-side drive broken
		ldi	temp4, mux_c
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		CnFET_on			; Drive down this phase (we've established that it was 0V above).
		rcall	adc_read		; FET turn-on will easily beat ADC initialization
		CnFET_off
		rcall	hw_error_y_le_temp12
		.endif

		cbi	PORTD, 6		; Sink on AIN0 (help to pull down the outputs)
		rcall	wait30ms

		.if defined(mux_a)
		ldi	XL, 9			; Error code 9: Phase A high-side drive broken
		ldi	temp4, mux_a
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		ApFET_on			; Drive up this phase.
		rcall	adc_read		; Waste time for high side to turn off
		ApFET_off
		rcall	hw_error_temp12_le_y
		.endif

		.if defined(mux_b)
		ldi	XL, 10			; Error code 10: Phase B high-side drive broken
		ldi	temp4, mux_b
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		BpFET_on			; Drive up this phase.
		rcall	adc_read		; Waste time for high side to turn off
		BpFET_off
		rcall	hw_error_temp12_le_y
		.endif

		.if defined(mux_c)
		ldi	XL, 11			; Error code 11: Phase C high-side drive broken
		ldi	temp4, mux_c
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		CpFET_on			; Drive up this phase.
		rcall	adc_read		; Waste time for high side to turn off
		CpFET_off
		rcall	hw_error_temp12_le_y
		.endif

		cbi	DDRD, 6			; Restore tristated AIN0
		ret

check_sense_low:
		ldi2	YL, YH, MAX_CHECK_LOOPS
check_sense_low1:
		rcall	adc_read
		; Up to 3.5V to account for ADC offset or driver pull-up.
		.equ	OFF_MAX_ADC = 35 * 1024 * O_GROUND / (50 * (O_POWER + O_GROUND))
		sbiwx	temp1, temp2, OFF_MAX_ADC
		brcs	check_sense_low_ret	; Return if pin reads low
		sbiw	YL, 1
		brne	check_sense_low1	; Loop until timeout
		rjmp	hw_error
check_sense_low_ret:
		ret

hw_error_temp12_le_y:
		cp	YL, temp1
		cpc	YH, temp2
		brcc	hw_error
		ret

hw_error_y_le_temp12:
		cp	temp1, YL
		cpc	temp2, YH
		brcc	hw_error
		ret

;-- Hardware error -------------------------------------------------------
; Blink an LED or beep XL times to indicate a hardware error.
; Beeping is possibly unsafe. The only other option is to stop.
hw_error_eq:
		brne	hw_error_ret
hw_error:
		mov	YL, XL
hw_error1:
		.if defined(red_led)
		RED_on
		rcall	wait120ms
		RED_off
		.elif defined(blue_led)
		BLUE_on
		rcall	wait120ms
		BLUE_off
		.elif defined(green_led)
		GRN_on
		rcall	wait120ms
		GRN_off
		.else
		rcall	beep_f1			; Low frequency is safer
		.endif
		rcall	wait240ms
		dec	YL
		brne	hw_error1
		rcall	wait240ms
		rcall	wait240ms
		set
hw_error_ret:	ret

.endif

;-------------------------------------------------------------------------
; ADC value dumping via the UART. Expects vt100ish.
.if DEBUG_ADC_DUMP
.set DEBUG_TX = 1
.set ADC_READ_NEEDED = 1

adc_input_dump:
		ldi	temp4, 27
		rcall	tx_byte
		ldi	temp4, '['
		rcall	tx_byte
		ldi	temp4, '2'
		rcall	tx_byte
		ldi	temp4, 'J'
		rcall	tx_byte

adc_input_dump1:
		ldi	temp2, 5
		rcall	wait1
		ldi	temp4, 27
		rcall	tx_byte
		ldi	temp4, '['
		rcall	tx_byte
		ldi	temp4, 'H'
		rcall	tx_byte

		.if defined(mux_a)
		ldi	temp4, 'A'
		rcall	tx_byte
		ldi	temp4, mux_a
		rcall	adc_read
		rcall	colon_hex_write
		.endif
		.if defined(mux_b)
		ldi	temp4, 'B'
		rcall	tx_byte
		ldi	temp4, mux_b
		rcall	adc_read
		rcall	colon_hex_write
		.endif
		.if defined(mux_c)
		ldi	temp4, 'C'
		rcall	tx_byte
		ldi	temp4, mux_c
		rcall	adc_read
		rcall	colon_hex_write
		.endif
		.if defined(mux_voltage)
		ldi	temp4, '#'
		rcall	tx_byte
		rcall	adc_cell_count
		clr	temp2
		rcall	colon_hex_write
		.endif

		rcall	tx_crlf

		clr	YL
adc_loop:
		ldi	temp4, 'A'
		rcall	tx_byte
		ldi	temp4, 'D'
		rcall	tx_byte
		ldi	temp4, 'C'
		rcall	tx_byte
		mov	temp4, YL
		rcall	tx_hex_nibble

		mov	temp4, YL
		rcall	adc_read
		rcall	colon_hex_write

		inc	YL
		andi	YL, 0xf
		breq	adc_input_dump1
		cpi	YL, 8
		brne	adc_loop
		ldi	YL, 0xe		; Jump to band-gap reference (no ADC8 - ADC13)
		rjmp	adc_loop

		ret
.endif

.if DEBUG_TX
init_debug_tx:
.if !defined(txd) && DIR_PD & (1<<1)
.error "Cannot use UART TX with this pin configuration"
.endif
	; Initialize TX for debugging on boards with free TX pin
		.equ	D_BAUD_RATE = 38400
		.equ	D_UBRR_VAL = F_CPU / D_BAUD_RATE / 16 - 1
		outi	UBRRH, high(D_UBRR_VAL), temp1
		outi	UBRRL, low(D_UBRR_VAL), temp1
;		sbi	UCSRA, U2X		; Double speed
		sbi	UCSRB, TXEN
		outi	UCSRC, (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0), temp1	; N81
		ret

tx_hex_byte:
		mov	temp4, temp3
		swap	temp4
		rcall	tx_hex_nibble
		mov	temp4, temp3
tx_hex_nibble:
		andi	temp4, 0xf
		ori	temp4, '0'
		cpi	temp4, '9' + 1
		brcs	tx_byte
		subi	temp4, '9' + 1 - 'a'
tx_byte:
		sbis	UCSRA, UDRE
		rjmp	tx_byte
		out	UDR, temp4
		ret

tx_crlf:
		ldi	temp4, 13
		rcall	tx_byte
		ldi	temp4, 10
		rjmp	tx_byte

tx_colonhex:
		ldi	temp4, ':'
		rcall	tx_byte
		ldi	temp4, ' '
		rcall	tx_byte
		ldi	temp4, '0'
		rcall	tx_byte
		ldi	temp4, 'x'
		rjmp	tx_byte

colon_hex_write:
		rcall	tx_colonhex
		mov	temp3, temp2
		rcall	tx_hex_byte
		mov	temp3, temp1
		rcall	tx_hex_byte
		rjmp	tx_crlf
.endif

;-- Battery cell count ---------------------------------------------------
; Assuming a LiPo cell will never exceed 4.3V, we can estimate
; the number of cells by dividing the measured voltage by 4.3.
.if defined(mux_voltage) && (DEBUG_ADC_DUMP || (!CELL_COUNT && BLIP_CELL_COUNT))
.set ADC_READ_NEEDED = 1

adc_cell_count:
		ldi	temp4, mux_voltage | (1<<ADLAR)
		rcall	adc_read
		ldi2	temp3, temp4, 256 * 50 * (O_POWER + O_GROUND) / (O_GROUND * CELL_MAX_DV)
		ldi2	YL, YH, 0x100		; Always at least one cell
		rcall	mul_y_12x34
		mov	temp1, YH
		ret
.endif

.if ADC_READ_NEEDED
;-- ADC input ------------------------------------------------------------
; Read ADC from the mux set in temp4 and return result in temp1:temp2.
;
; The ADC clock on the ATmega8 needs to run between 50kHz and 200kHz for
; full 10-bit sampling. At 8Mhz or 16MHz, we can use /128 to get 62.5kHz
; or 125kHz. The datasheet says we can overclock further at the expense
; of least significant bits. However, overclocked initialization appears
; to offset zero when running the ADC clock any faster than 500kHz. As we
; must turn off the ADC to use ADMUX for the comparator while driving the
; motor, the maximum usable speed appears to be 500kHz (16MHz/32). This
; results in minimum sample time of 25/500kHz == 50microseconds. Slowest
; sample time is 100microseconds at 8MHz or 200microseconds at 16MHz.
;
; If AVcc is tied to AREF, the ADC is intended to be used with no
; internal reference enabled (REFS0 and REFS1 not set). If a capacitor is
; at AREF, one of the internal references must be set, or all the
; capacitor will not charge and all ADC channels will read 0x3ff. REFS1
; can still be set with AVcc bridged to AREF, since then it just gets
; bridged internally and externally. The internal 2.56V reference (REFS0
; and REFS1 set) can only be enabled if AVcc is NOT bridged.
adc_read:
		out	SFIOR, ZH		; Disable the Analog Comparator Multiplexer
		sbr	temp4, (1<<REFS0)	; Enable AVCC (5.0V) reference
		out	ADMUX, temp4		; Set ADC channel, AVcc reference with cap at AREF (should be safe if bridged)
		ldi	temp1, (1<<ADEN)+(1<<ADSC)+(1<<ADPS2)+(1<<ADPS1)+(1<<ADPS0)
		out	ADCSRA, temp1		; Enable the ADC, start conversion
		wdr				; Will wait 25*128 cycles
adc_wait:	sbic	ADCSRA, ADSC
		rjmp	adc_wait
		in	temp1, ADCL
		in	temp2, ADCH
		out	ADCSRA, ZH		; Disable the ADC (next enable and sample will take 25 ADC cycles)
		ret
.endif

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
	; Check if pulse is in aliased input range and toggle RCP_ALIAS to match
		.if defined(RCP_ALIAS_SHIFT)
		movw	temp1, rx_l		; Atomic copy of rc pulse length
		sbiwx	temp1, temp2, MIN_RC_PULS * CPU_MHZ
		sbc	temp3, temp3		; All temp3 bits <- carry (set if in aliased range)
		eor	temp3, flags0		; XOR against current flags0 value
		sbrc	temp3, RCP_ALIAS	; Skip if zero (no change)
		rjmp	puls_scale_alias_toggle	; or toggle RCP_ALIAS and rescale
		.endif
		.if RC_CALIBRATION
		sbrc	flags0, NO_CALIBRATION	; Is it safe to calibrate now?
		rjmp	evaluate_rc_puls
	; If input is above PROGRAM_RC_PULS, we try calibrating throttle
		ldi2	YL, YH, puls_high_l	; Start with high pulse calibration
		rjmp	rc_prog1
rc_prog0:	rcall	wait240ms		; Wait for stick movement to settle
	; Collect average of throttle input pulse length
rc_prog1:	rcall	rc_prog_get_rx		; Pulse length into temp1:temp2
		movw	temp3, temp1		; Save starting pulse length
		wdr
rc_prog2:	mul	ZH, ZH			; Clear 24-bit result registers (0 * 0 -> temp5:temp6)
		clr	temp7
		cpi	YL, low(puls_high_l)	; Are we learning the high pulse?
		brne	rc_prog3		; No, maybe the low pulse
		ldi	temp2, 32 * 31/32	; Full speed pulse averaging count (slightly below exact)
		cpi2	temp3, temp4, PROGRAM_RC_PULS * CPU_MHZ, temp1
		brcc	rc_prog5		; Equal to or higher than PROGRAM_RC_PULS - start measuring
		rjmp	evaluate_rc_puls	; Lower than PROGRAM_RC_PULS - exit programming
rc_prog3:	lds	temp1, puls_high_l	; If not learning the high pulse, we should stay below it
		cp	temp3, temp1
		lds	temp1, puls_high_h
		cpc	temp4, temp1
rc_prog_brcc_prog1:				; Branch trampoline
		brcc	rc_prog1		; Restart while pulse not lower than learned high pulse
		ldi	temp2, 32 * 17/16	; Stop/reverse pulse (slightly above exact)
		cpi	YL, low(puls_low_l)	; Are we learning the low pulse?
		breq	rc_prog5		; Yes, start measuring
rc_prog4:	lds	temp1, puls_low_l
		cp	temp3, temp1
		lds	temp1, puls_low_h
		cpc	temp4, temp1
		brcs	rc_prog1		; Restart while pulse lower than learned low pulse
		ldi	temp2, 32		; Neutral pulse measurement (exact)
rc_prog5:	mov	tcnt2h, temp2		; Abuse tcnt2h as pulse counter
rc_prog6:	wdr
		sbrs	flags1, EVAL_RC		; Wait for next pulse
		rjmp	rc_prog6
		cbr	flags1, (1<<EVAL_RC)
		rcall	rc_prog_get_rx		; Pulse length into temp1:temp2
		add	temp5, temp1		; Accumulate 24-bit average
		adc	temp6, temp2
		adc	temp7, ZH
		sub	temp1, temp3		; Subtract the starting pulse from this one
		sbc	temp2, temp4		; to find the drift since the starting pulse
	; Check for excessive drift with an emulated signed comparison -
	; add the drift amount to offset the negative side to 0
		adiwx	temp1, temp2, MAX_DRIFT_PULS * CPU_MHZ
	; ..then subtract the 2*drift + 1 -- carry will be clear if
	; we drifted outside of the range
		sbiwx	temp1, temp2, 2 * MAX_DRIFT_PULS * CPU_MHZ + 1
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
		breq	rc_prog_brcc_prog1	; Go back to get low pulse
	; Two beeps: low (stop/reverse) pulse received
		rcall	wait30ms
		rcall	beep_f3
		cpi	YL, low(puls_low_l+2)
		.if RC_PULS_REVERSE
		breq	rc_prog_brcc_prog1	; Go back to get neutral pulse
		.else
		breq	rc_prog_done
		.endif
	; Three beeps: neutral pulse received
		rcall	wait30ms
		rcall	beep_f3
rc_prog_done:	rcall	eeprom_write_block
		rjmp	puls_scale		; Calculate the new scaling factors
rc_prog_get_rx:
		movw	temp1, rx_l		; Atomic copy of rc pulse length
		.if defined(RCP_ALIAS_SHIFT)
		sbrs	flags0, RCP_ALIAS
		ret
		ldi	XL, RCP_ALIAS_SHIFT
rc_prog_get_rx1:
		lsl	temp1
		rol	temp2
		dec	XL
		brne	rc_prog_get_rx1
		.endif
		ret
		.endif
		.endif
	; Fall through from evaluate_rc_init
;-----bko-----------------------------------------------------------------
; These routines may clobber temp* and Y, but not X.
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
		cbr	flags1, (1<<EVAL_RC)+(1<<REVERSE)
		.if MOTOR_BRAKE || LOW_BRAKE
		sts	brake_want, ZH
		.endif
		movw	temp1, rx_l		; Atomic copy of rc pulse length
		cpi2	temp1, temp2, MIN_RC_PULS * CPU_MHZ, temp3
		brcs	puls_length_short	; Branch if short pulse
		.if defined(RCP_ALIAS)
		sbrc	flags0, RCP_ALIAS
		rjmp	puls_length_error	; Throw away long pulse if alias range expected
		.endif
puls_alias:
		.if LOW_BRAKE
		lds	YL, low_brake_l
		lds	YH, low_brake_h
		cp	temp1, YL
		cpc	temp2, YH
		brcc	puls_not_low_brake
		ldi	YL, 2
		sts	brake_want, YL		; Set desired brake to 2 (low brake)
		rjmp	puls_zero
puls_not_low_brake:
		.endif
		lds	YL, neutral_l
		lds	YH, neutral_h
		sub	temp1, YL		; Offset input to neutral
		sbc	temp2, YH
		brcc	puls_plus
		.if RC_PULS_REVERSE
		sbr	flags1, (1<<REVERSE)
		com	temp2			; Negate 16-bit value to get positive duty cycle
		neg	temp1
		sbci	temp2, -1
		lds	temp3, rev_scale_l	; Load reverse scaling factor
		lds	temp4, rev_scale_h
		rjmp	puls_not_zero
		.endif
		; Fall through to stop/zero in no reverse case
puls_zero_brake:
		.if MOTOR_BRAKE
		ldi	YL, 1
		sts	brake_want, YL		; Set desired brake to 1 (neutral brake)
		.endif
puls_zero:	clr	YL
		clr	YH
		rjmp	rc_duty_set
puls_length_short:
		.if defined(RCP_ALIAS_SHIFT)
		; Additional length checks for aliased range
		cpi2	temp1, temp2, (MIN_RC_PULS * CPU_MHZ) >> RCP_ALIAS_SHIFT, temp3
		brcs	puls_length_error
		cpi2	temp1, temp2, (MAX_RC_PULS * CPU_MHZ) >> RCP_ALIAS_SHIFT, temp3
		sbrc	flags0, RCP_ALIAS	; If alias range expected
		brcs	puls_alias
		.endif
puls_length_error:
		sbr	flags0, (1<<RCP_ERROR)
		ret
puls_plus:
		lds	temp3, fwd_scale_l	; Load forward scaling factor
		lds	temp4, fwd_scale_h
puls_not_zero:
		.if RCP_DEADBAND
		.if defined(RCP_ALIAS_SHIFT)
		lds	YL, deadband_l		; Load dynamic deadband
		lds	YH, deadband_h
		sub	temp1, YL
		sbc	temp2, YH
		.else
		sbiwx	temp1, temp2, RCP_DEADBAND * CPU_MHZ
		.endif
		brmi	puls_zero_brake
		.endif
.endif
	; The following is used by all input modes
rc_do_scale:	ldi2	YL, YH, MIN_DUTY	; Offset result so that 0 is MIN_DUTY
		rcall	mul_y_12x34		; Scaled result is now in Y
		cpi2	YL, YH, MAX_POWER, temp1
		brcs	rc_duty_set
		ldi2	YL, YH, MAX_POWER
rc_duty_set:	sts	rc_duty_l, YL
		sts	rc_duty_h, YH
		sbrs	flags0, SET_DUTY
		rjmp	rc_no_set_duty
		ldi	temp1, RCP_TOT
		mov	rc_timeout, temp1	; Short rc_timeout when driving
		rjmp	set_new_duty_l		; Skip reload into YL:YH
rc_no_set_duty:	ldi	temp1, 12		; More than 10 needed to arm
		cp	rc_timeout, temp1
		adc	rc_timeout, ZH
		ret
;-----bko-----------------------------------------------------------------
.if USE_I2C
evaluate_rc_i2c:
		movw	YL, rx_l		; Atomic copy of 16-bit input
		cbr	flags1, (1<<EVAL_RC)
	; Load settings from BLConfig structure (BL-Ctrl v2)
		lds	temp1, blc_bitconfig
		bst	temp1, 0		; BitConfig bit 0: Reverse
		bld	flags1, REVERSE
	; MK sends one or two bytes, if supported, and if low bits are
	; non-zero. We store the first received byte in rx_h, second
	; in rx_l. There are 3 low bits which are stored at the low
	; side of the second byte, so we must shift them to line up with
	; the high byte. The high bits become less significant, if set.
		lsl	YL			; 00000xxxb -> 0000xxx0b
		swap	YL			; 0000xxx0b -> xxx00000b
		adiw	YL, 0			; 16-bit zero-test
		breq	rc_duty_set		; Power off
	; Scale so that YH == 247 is MAX_POWER, to support reaching full
	; power from the highest MaxGas setting in MK-Tools. Bernhard's
	; original version reaches full power at around 245.
		movw	temp1, YL
		ldi2	temp3, temp4, 0x100 * (POWER_RANGE - MIN_DUTY) / 247
		rjmp	rc_do_scale		; The rest of the code is common
.endif
;-----bko-----------------------------------------------------------------
.if USE_UART
evaluate_rc_uart:
		mov	YH, rx_h		; Copy 8-bit input
		cbr	flags1, (1<<EVAL_RC)+(1<<REVERSE)
		ldi	YL, 0
		cpi	YH, 0
		breq	rc_duty_set		; Power off
	; Scale so that YH == 200 is MAX_POWER.
		movw	temp1, YL
		ldi2	temp3, temp4, 0x100 * (POWER_RANGE - MIN_DUTY) / 200
		rjmp	rc_do_scale		; The rest of the code is common
.endif
;-----bko-----------------------------------------------------------------
; If an input alias is defined, shift the values in temp3:temp4 and
; temp1:temp2 to follow.
.if defined(RCP_ALIAS_SHIFT)
puls_scale_alias_shift:
		ldi	YL, RCP_ALIAS_SHIFT
puls_scale_alias_shift1:
		lsr	temp4
		ror	temp3
		lsr	temp2
		ror	temp1
		dec	YL
		brne	puls_scale_alias_shift1
		ret
;-----bko-----------------------------------------------------------------
; Toggle RCP_ALIAS and fall through to pulse_scale
puls_scale_alias_toggle:
		ldi	temp1, (1<<RCP_ALIAS)
		eor	flags0, temp1
.endif
;-----bko-----------------------------------------------------------------
; Calculate the neutral offset and forward (and reverse) scaling factors
; to line up with the high/low (and neutral) pulse lengths.
puls_scale:
	; If reversible input is enabled, scale from neutral; otherwise,
	; scale from the lowest point.
		.if RC_PULS_REVERSE
		lds	temp1, puls_neutral_l
		lds	temp2, puls_neutral_h
		.else
		lds	temp1, puls_low_l
		lds	temp2, puls_low_h
		.endif
		lds	temp3, puls_high_l
		lds	temp4, puls_high_h
		.if defined(RCP_ALIAS_SHIFT)
		sbrc	flags0, RCP_ALIAS
		rcall	puls_scale_alias_shift
		.endif
	; Find the distance to full throttle and fit it to match the
	; distance between FULL_RC_PULS and STOP_RC_PULS by walking
	; for the lowest 16.16 multiplier that just brings us in range.
		sts	neutral_l, temp1
		sts	neutral_h, temp2
		sub	temp3, temp1
		sbc	temp4, temp2
		rcall	puls_find_multiplicand
		sts	fwd_scale_l, temp1
		sts	fwd_scale_h, temp2

	; If reversible input is enabled, calculate the reverse scaling
	; factor.
		.if RC_PULS_REVERSE
		lds	temp3, puls_neutral_l
		lds	temp4, puls_neutral_h
		lds	temp1, puls_low_l
		lds	temp2, puls_low_h
		.if defined(RCP_ALIAS_SHIFT)
		sbrc	flags0, RCP_ALIAS
		rcall	puls_scale_alias_shift
		.endif
		sub	temp3, temp1
		sbc	temp4, temp2
		rcall	puls_find_multiplicand
		sts	rev_scale_l, temp1
		sts	rev_scale_h, temp2
		.endif

	; If a deadband is enabled and input aliases are supported,
	; calculate the scaled deadband in advance.
		.if RCP_DEADBAND && defined(RCP_ALIAS_SHIFT)
		ldi2	temp1, temp2, RCP_DEADBAND * CPU_MHZ
		sbrc	flags0, RCP_ALIAS
		rcall	puls_scale_alias_shift
		sts	deadband_l, temp1
		sts	deadband_h, temp2
		.endif

	; If low brake is enabled, store the position where it starts
	; after subtracting the deadband. This avoids the calculation
	; in the receive path.
		.if LOW_BRAKE
		lds	temp1, puls_low_l		; Lowest calibrated pulse (regardless of RC_PULS_REVERSE)
		lds	temp2, puls_low_h
		ldi2	temp3, temp4, RCP_LOW_DBAND * CPU_MHZ
		.if defined(RCP_ALIAS_SHIFT)
		sbrc	flags0, RCP_ALIAS
		rcall	puls_scale_alias_shift
		.endif
		sub	temp1, temp3
		sbc	temp2, temp4
		brcc	puls_low_brake_save
		ldi2	temp1, temp2, 0
puls_low_brake_save:
		sts	low_brake_l, temp1
		sts	low_brake_h, temp2
		.endif

		ret
;-----bko-----------------------------------------------------------------
; Find the lowest 16.16 multiplicand that brings us to full throttle
; (POWER_RANGE - MIN_DUTY) when multiplied by temp3:temp4.
; The range we are looking for is around 3000 - 10000:
; m = (POWER_RANGE - MIN_DUTY) * 65536 / (1000us * 16MHz)
; If the input range is < 100us at 8MHz, < 50us at 16MHz, we return
; too low a multiplicand (higher won't fit in 16 bits).
; We preload temp1:temp2 with the weakest possible scale based on the
; fact that we can't accept a wider range than MAX_RC_PULS microseconds.
puls_find_multiplicand:
		.if RCP_DEADBAND
		sbi2	temp3, temp4, RCP_DEADBAND * CPU_MHZ
		.endif
		ldi2	temp1, temp2, (POWER_RANGE - MIN_DUTY) * 65536 / (MAX_RC_PULS * CPU_MHZ)
puls_find1:	adiw	temp1, 1
		wdr
		cpi	temp2, 0xff
		cpc	temp1, temp2
		breq	puls_find_fail		; Return if we reached 0xffff
	; Start with negative POWER_RANGE so that 0 is full throttle
		ldi2	YL, YH, MIN_DUTY - POWER_RANGE
		rcall	mul_y_12x34
	; We will always be increasing the result in steps of less than 1,
	; so we can test for just zero rather than a range.
		brne	puls_find1
puls_find_fail:	ret
;-----bko-----------------------------------------------------------------
update_timing:
		cli
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		lds	temp3, tcnt1x
		in	temp4, TIFR
		sei
		cpi	temp2, 0x80		; tcnt1x is right when TCNT1h[7] set;
		sbrc	temp4, TOV1		; otherwise, if TOV1 is/was pending,
		adc	temp3, ZH		; increment our copy of tcnt1x.

	; Calculate the timing from the last two zero-crossings
		lds	YL, last_tcnt1_l	; last -> Y
		lds	YH, last_tcnt1_h
		lds	temp7, last_tcnt1_x
		sts	last_tcnt1_l, temp1
		sts	last_tcnt1_h, temp2
		sts	last_tcnt1_x, temp3
		lds	temp5, l2_tcnt1_l	; last2 -> temp5
		lds	temp6, l2_tcnt1_h
		lds	temp4, l2_tcnt1_x
		sts	l2_tcnt1_l, YL
		sts	l2_tcnt1_h, YH
		sts	l2_tcnt1_x, temp7

	; Cancel DC bias by starting our timing from the average of the
	; last two zero-crossings. Commutation phases always alternate.
	; Next start = (cur(c) - last2(a)) / 2 + last(b)
	; -> start=(c-b+(c-a)/2)/2+b
	;
	;                  (c - a)
	;         (c - b + -------)
	;                     2
	; start = ----------------- + b
	;                 2

		sub	temp1, temp5		; c' = c - a
		sbc	temp2, temp6
		sbc	temp3, temp4

	; Limit maximum RPM (fastest timing)
		cpi3	temp1, temp2, temp3, TIMING_MAX * CPU_MHZ / 2, temp4
		brcc	update_timing1
		ldi3	temp1, temp2, temp3, TIMING_MAX * CPU_MHZ / 2
		lsr	sys_control_h		; limit by reducing power
		ror	sys_control_l
update_timing1:

	; Calculate a hopefully sane duty cycle limit from this timing,
	; to prevent excessive current if high duty is requested at low
	; speed. This is the best we can do without a current sensor.
	; The actual current peak will depend on motor KV and voltage,
	; so this is just an approximation. This is calculated smoothly
	; with a (very slow) software divide only if timing permits.
		cpi2	temp2, temp3, (TIMING_RANGE3 * CPU_MHZ / 2) >> 8, temp4
		ldi2	XL, XH, MAX_POWER
		brcs	update_timing4	; Fast timing: no duty limit.

		; 24.8-bit fixed-point unsigned divide, inlined with available registers:
		; duty (XL:XH) = MAX_POWER * (TIMING_RANGE3 * CPU_MHZ / 2) / period (temp1:temp2:temp3)
		; This takes about one microsecond per loop, but we only take this path
		; when the motor is spinning slowly.

		ldi	XL, byte3(MAX_POWER * (TIMING_RANGE3 * CPU_MHZ / 2) / 0x100)
		ldi	XH, 33		; Iteration counter
		movw	timing_duty_l, XL
		ldi2	XL, XH, MAX_POWER * (TIMING_RANGE3 * CPU_MHZ / 2) / 0x100

		mul	ZH, ZH		; Zero temp5, temp6
		sub	temp4, temp4	; Zero temp4, clear carry
		rjmp	fudiv24_ep	; Jump with carry clear
fudiv24_loop:
		rol	temp4
		rol	temp5
		rol	temp6
		cp	temp4, temp1	; Divide by commutation period
		cpc	temp5, temp2
		cpc	temp6, temp3
		brcs	fudiv24_ep
		sub	temp4, temp1
		sbc	temp5, temp2
		sbc	temp6, temp3
fudiv24_ep:
		rol	XL
		rol	XH
		rol	timing_duty_l
		dec	timing_duty_h
		brne	fudiv24_loop
		com	XL
		com	XH

		cpi2	XL, XH, PWR_MAX_RPM1, temp4
		brcc	update_timing4
		ldi2	XL, XH, PWR_MAX_RPM1
update_timing4:	movw	timing_duty_l, XL

		sts	timing_l, temp1		; Store timing (120 degrees)
		sts	timing_h, temp2
		sts	timing_x, temp3

		lsr	temp3			; c'>>= 1 (shift to 60 degrees)
		ror	temp2
		ror	temp1

.if defined(DC_BIAS_CANCEL)
		lds	temp5, last_tcnt1_l	; restore original c as a'
		lds	temp6, last_tcnt1_h
		lds	temp4, last_tcnt1_x
		sub	temp5, YL		; a'-= b
		sbc	temp6, YH
		sbc	temp4, temp7

		add	temp5, temp1		; a'+= c'
		adc	temp6, temp2
		adc	temp4, temp3
		lsr	temp4			; a'>>= 1
		ror	temp6
		ror	temp5
		add	YL, temp5		; b+= a' -> YL:YH:temp7 become filtered ZC time
		adc	YH, temp6
		adc	temp7, temp4
.else
		lds	YL, last_tcnt1_l	; restore original c as a'
		lds	YH, last_tcnt1_h
		lds	temp7, last_tcnt1_x
.endif

		ldi	temp4, (30 - MOTOR_ADVANCE) * 256 / 60
		rcall	update_timing_add_degrees
.if TIMING_OFFSET
		sbiwx	YL, YH, TIMING_OFFSET * CPU_MHZ
		ldi	temp4, byte3(TIMING_OFFSET * CPU_MHZ)
		sbc	temp7, temp4
.endif
		sts	com_time_l, YL		; Store start of next commutation
		sts	com_time_h, YH
		sts	com_time_x, temp7

		cpi	temp2, 0x10		; Will 240 degrees fit in 15 bits?
		cpc	temp3, ZH
		brcs	update_timing_fast
		cbr	flags2, (1<<TIMING_FAST)
		rcall	set_ocr1a_abs_slow	; Set timer for start of next commutation
		rjmp	update_timing_out
update_timing_fast:
		sbr	flags2, (1<<TIMING_FAST)
		rcall	set_ocr1a_abs_fast
update_timing_out:

		sbrc	flags1, EVAL_RC
		rjmp	evaluate_rc		; Set new duty either way
	; Fall through to set_new_duty
;-----bko-----------------------------------------------------------------
; Unlike update_timing above, we try not to clobber XL and XH which are
; used for loop-counting in wait_for_edge.
set_new_duty:	lds	YL, rc_duty_l
		lds	YH, rc_duty_h
set_new_duty_l:	cp	YL, timing_duty_l
		cpc	YH, timing_duty_h
		brcs	set_new_duty10
		movw	YL, timing_duty_l	; Limit duty to timing_duty
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
		ldi2	temp1, temp2, PWR_MIN_START
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
		ldi2	temp1, temp2, MAX_POWER
		sub	temp1, YL		; Calculate OFF duty
		sbc	temp2, YH
		breq	set_new_duty_full
		adiw	YL, 0
		breq	set_new_duty_zero
		; Not off and not full power
		cbr	flags1, (1<<FULL_POWER)
		sbr	flags1, (1<<POWER_ON)
		; At higher PWM frequencies, halve the frequency
		; when starting -- this helps hard drive startup
		.if POWER_RANGE < 1000 * CPU_MHZ / 16
		sbrs	flags1, STARTUP
		rjmp	set_new_duty_set
		lsl	temp1
		rol	temp2
		lsl	YL
		rol	YH
		.endif
set_new_duty_set:
		; When off duty is short, skip complementary PWM; otherwise,
		; compensate the off_duty time to account for the overhead.
	.if CPWM_SOFT
		set
		ldi	temp4, pwm_on_fast	; Short off period: skip complementary PWM
		cpse	temp2, ZH
		ldi	temp4, pwm_on_fast_high	; Off period >= 0x100
		cpi2	temp1, temp2, CPWM_OVERHEAD_HIGH + CPWM_OVERHEAD_LOW, temp3
		brcs	set_new_duty21		; Off period < off-to-on cycle count plus interrupt overhead
		clt				; Not short off period, unset SKIP_CPWM
		sbiwx	temp1, temp2, CPWM_OVERHEAD_HIGH
	.endif
		ldi	temp4, pwm_on		; Off period < 0x100
		cpse	temp2, ZH
		ldi	temp4, pwm_on_high	; Off period >= 0x100
set_new_duty21:
		com	YL			; Save one's complement of both
		com	temp1			; low bytes for up-counting TCNT2
		movw	duty_l, YL		; Atomic set new ON duty for PWM interrupt
		cli				; Critical section (off_duty & flags together)
		movw	off_duty_l, temp1	; Set new OFF duty for PWM interrupt
		sts	pwm_on_ptr, temp4	; Set Next PWM ON interrupt vector
	.if CPWM_SOFT
		bld	flags2, SKIP_CPWM	; If to skip complementary PWM
	.endif
		sei
		ret
set_new_duty_full:
		; Full power
		sbr	flags1, (1<<FULL_POWER)+(1<<POWER_ON)
		rjmp	set_new_duty_set
set_new_duty_zero:
		; Power off
		cbr	flags1, (1<<FULL_POWER)+(1<<POWER_ON)
		ldi	temp4, pwm_off		; Skip the on phase entirely
	.if CPWM_SOFT
		set				; Skip complementary PWM
	.endif
		rjmp	set_new_duty21
;-----bko-----------------------------------------------------------------
; Multiply the 24-bit timing in temp1:temp2:temp3 by temp4 and add the top
; 24-bits to YL:YH:temp7.
update_timing_add_degrees:
		mul	temp1, temp4
		add	YL, temp6		; Discard byte 1 already
		adc	YH, ZH
		adc	temp7, ZH
		mul	temp2, temp4
		add	YL, temp5
		adc	YH, temp6
		adc	temp7, ZH
		mul	temp3, temp4
		add	YH, temp5
		adc	temp7, temp6
		ret
load_timing:
		lds	temp1, timing_l
		lds	temp2, timing_h
		lds	temp3, timing_x
		lds	YL, com_time_l
		lds	YH, com_time_h
		lds	temp7, com_time_x
		ret
;-----bko-----------------------------------------------------------------
; Set zero-crossing timeout to 240 degrees
set_ocr1a_zct_slow:
		rcall	load_timing
		add	YL, temp1
		adc	YH, temp2
		adc	temp7, temp3
		add	YL, temp1
		adc	YH, temp2
		adc	temp7, temp3
		rjmp	set_ocr1a_abs_slow
set_timing_degrees_slow:
		rcall	load_timing
		rcall	update_timing_add_degrees
	; Fall through to set_ocr1a_abs_slow
;-----bko-----------------------------------------------------------------
; Set OCT1_PENDING until the absolute time specified by YL:YH:temp7 passes.
; Returns current TCNT1(L:H:X) value in temp1:temp2:temp3.
;
; tcnt1x may not be updated until many instructions later, even with
; interrupts enabled, because the AVR always executes one non-interrupt
; instruction between interrupts, and several other higher-priority
; interrupts may (have) come up. So, we must save tcnt1x and TIFR with
; interrupts disabled, then do a correction.
set_ocr1a_abs_slow:
		in	temp4, TIMSK
		mov	temp5, temp4
		cbr	temp4, (1<<TOIE1)+(1<<OCIE1A)
		out	TIMSK, temp4		; Disable TOIE1 and OCIE1A temporarily
		ldi	temp4, (1<<OCF1A)
		cli
		out	OCR1AH, YH
		out	OCR1AL, YL
		out	TIFR, temp4		; Clear any pending OCF1A interrupt
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		sei
		sbr	flags0, (1<<OCT1_PENDING)
		lds	temp3, tcnt1x
		in	temp4, TIFR
		cpi	temp2, 0x80		; tcnt1x is right when TCNT1h[7] set;
		sbrc	temp4, TOV1		; otherwise, if TOV1 is/was pending,
		adc	temp3, ZH		; increment our copy of tcnt1x.
		sub	YL, temp1		; Check that time might have already
		sbc	YH, temp2		; passed -- if so, clear pending flag.
		sbc	temp7, temp3
		sts	ocr1ax, temp7
		brpl	set_ocr1a_abs_slow1	; Skip set if time has passed
		cbr	flags0, (1<<OCT1_PENDING)
set_ocr1a_abs_slow1:
		out	TIMSK, temp5		; Enable TOIE1 and OCIE1A again
		ret

; Variants of the above with a fast path when timing is guaranteed to fit
; within 15 bits.
set_ocr1a_zct:
		sbrs	flags2, TIMING_FAST
		rjmp	set_ocr1a_zct_slow
		lds	temp1, timing_l
		lds	temp2, timing_h
		lds	YL, com_time_l
		lds	YH, com_time_h
		add	YL, temp1
		adc	YH, temp2
		add	YL, temp1
		adc	YH, temp2
		rjmp	set_ocr1a_abs_fast
set_timing_degrees:
		sbrs	flags2, TIMING_FAST
		rjmp	set_timing_degrees_slow
		lds	temp1, timing_l
		lds	temp2, timing_h
		lds	YL, com_time_l
		lds	YH, com_time_h
		mul	temp1, temp4
		add	YL, temp6
		adc	YH, ZH
		mul	temp2, temp4
		add	YL, temp5
		adc	YH, temp6
set_ocr1a_abs_fast:
		ldi	temp4, (1<<OCF1A)
		cli
		out	OCR1AH, YH
		out	OCR1AL, YL
		out	TIFR, temp4		; Clear any pending OCF1A interrupt
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		sbr	flags0, (1<<OCT1_PENDING)
		sts	ocr1ax, ZH
		sei
		sub	YL, temp1		; Check that time might have already
		sbc	YH, temp2		; passed -- if so, clear pending flag.
		brpl	set_ocr1a_abs_fast1	; Skip set if time has passed
		cbr	flags0, (1<<OCT1_PENDING)
set_ocr1a_abs_fast1:
		ret
;-----bko-----------------------------------------------------------------
; Set OCT1_PENDING until the relative time specified by YL:YH:temp7 passes.
set_ocr1a_rel:	adiw	YL, 7			; Compensate for timer increment during in-add-out
		ldi	temp4, (1<<OCF1A)
		cli
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		add	YL, temp1
		adc	YH, temp2
		out	OCR1AH, YH
		out	OCR1AL, YL
		out	TIFR, temp4		; Clear any pending OCF1A interrupt (7 cycles from TCNT1 read)
		sts	ocr1ax, temp7
		sbr	flags0, (1<<OCT1_PENDING)
		sei
		ret
;-----bko-----------------------------------------------------------------
wait_OCT1_tot:	sbrc	flags1, EVAL_RC
		rcall	evaluate_rc
		sbrc	flags0, OCT1_PENDING
		rjmp	wait_OCT1_tot		; Wait for commutation time
		ret
;-----bko-----------------------------------------------------------------
switch_power_off:
		out	TCCR2, ZH		; Disable PWM
		ldi	temp1, (1<<TOV2)
		out	TIFR, temp1		; Clear pending PWM interrupts
		ldi	ZL, low(pwm_wdr)	; Stop PWM switching
		all_pFETs_off temp1
		all_nFETs_off temp1
		.if MOTOR_BRAKE || LOW_BRAKE
		sts	brake_active, ZH	; No active brake
		.endif
		ret
;-----bko-----------------------------------------------------------------
.if BOOT_JUMP
boot_loader_test:
		.if USE_ICP
		sbis	PINB, rcp_in		; Skip clear if ICP pin high
		.elif USE_INT0 == 1
		sbis	PIND, rcp_in		; Skip clear if INT0 pin high
		.else
		sbic	PIND, rcp_in		; Skip clear if INT0 pin low (inverted)
		.endif
		sts	rct_boot, ZH		; Clear rct_count when low
		lds	temp1, rct_boot
		sbrs	temp1, 5 		; Wait 32 * 16 * 65536us (~2s) before jumping
boot_ret:	ret
; Check for boot loader presence
		ldi	ZL, low(BOOT_START << 1)
		cli				; Interrupts depend on ZH being 0
		ldi	ZH, high(BOOT_START << 1)
		lpm	temp1, Z+
		lpm	temp2, Z
		ldi	ZH, 0
		sei
		adiw	temp1, 1		; Check flash contents for 0xffff or 0x0000
		sbiw	temp1, 2
		brcs	boot_ret		; Return if boot loader area is empty
boot_loader_jump:
		cli
		out	DDRB, ZH		; Tristate pins
		out	DDRC, ZH
		out	DDRD, ZH
		outi	WDTCR, (1<<WDCE)+(1<<WDE), temp1
		out	WDTCR, ZH		; Disable watchdog
		lds	temp1, orig_osccal
		out	OSCCAL, temp1		; Restore OSCCAL
		rjmp	BOOT_START		; Jump to boot loader
.endif
;-----bko-----------------------------------------------------------------
.if USE_I2C
i2c_init:
		ldi	temp1, I2C_ADDR + (MOTOR_ID << 1)
		.if defined(MK_ADDRESS_PADS)
		sbis	PINB, adr1		; Offset MOTOR_ID by address pads
		subi	temp1, -1
		sbis	PINB, adr2
		subi	temp1, -2
		.endif
		out	TWAR, temp1
		outi	TWCR, (1<<TWIE)+(1<<TWEN)+(1<<TWEA)+(1<<TWINT), temp1
		ret
.endif
;-----bko-----------------------------------------------------------------
.if BEEP_RCP_ERROR
rcp_error_beep:
		rcall	switch_power_off	; Brake may have been on
		rcall	wait30ms
		lds	temp4, rcp_beep_count
		subi	temp4, -4
		sts	rcp_beep_count, temp4
		brne	rcp_error_beep_more
		cbr	flags0, (1<<RCP_ERROR)
rcp_error_beep_more:
		ldi	temp2, 18		; Short beep pulses to indicate corrupted PWM input
		rjmp	beep_f4_freq
.endif
;-----bko-----------------------------------------------------------------
main:
		clr	r0
		out	SREG, r0		; Clear interrupts and flags

	; Set up stack
		ldi2	ZL, ZH, RAMEND
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
		in	temp1, OSCCAL
		sts	orig_osccal, temp1
		in	temp7, MCUCSR		; Store reset reason in register not used for a while
		out	MCUCSR, ZH

	; Initialize ports
		outi	PORTB, INIT_PB, temp1
		outi	DDRB, DIR_PB | (MOTOR_DEBUG<<3) | (MOTOR_DEBUG<<4) | (MOTOR_DEBUG<<5), temp1
		outi	PORTC, INIT_PC, temp1
		outi	DDRC, DIR_PC, temp1
		outi	PORTD, INIT_PD, temp1
		outi	DDRD, DIR_PD, temp1

	; For PWM-enable drivers, set enable after initializing ports
		.if defined(ENABLE_ALL_PORT)
		sbi	ENABLE_ALL_PORT, ENABLE_ALL
		.endif
		.if defined(ENABLE_A_PORT)
		sbi     ENABLE_A_PORT, ENABLE_A
		.endif
		.if defined(ENABLE_B_PORT)
		sbi     ENABLE_B_PORT, ENABLE_B
		.endif
		.if defined(ENABLE_C_PORT)
		sbi     ENABLE_C_PORT, ENABLE_C
		.endif

		.if DEBUG_TX
		rcall	init_debug_tx
		.endif

	; Start timers except output PWM
		outi	TCCR0, T0CLK, temp1	; timer0: beep control, delays
		outi	TCCR1B, T1CLK, temp1	; timer1: commutation timing, RC pulse measurement
		out	TCCR2, ZH		; timer2: PWM, stopped

	; Enable watchdog (WDTON may be set or unset)
		ldi	temp1, (1<<WDCE)+(1<<WDE)
		out	WDTCR, temp1
		ldi	temp1, (1<<WDE)		; Fastest option: ~16.3ms timeout
		out	WDTCR, temp1

	; Wait for power to settle -- this must be no longer than 64ms
	; (with 64ms delayed start fuses) for i2c V2 protocol detection
		rcall	wait30ms		; Running at unadjusted speed(!)

	; Debugging hooks
		.if DEBUG_ADC_DUMP
		rcall	adc_input_dump
		.endif

	; Read EEPROM block to RAM
		rcall	eeprom_read_block	; Also calls osccal_set
		rcall	eeprom_check_reset

	; Early input initialization is required for i2c BL-Ctrl V2 detection
	; This serves data from the EEPROM, so this is as early as possible.
		.if USE_I2C
		rcall	i2c_init
		.endif

	; Enable interrupts for early input (i2c)
		sei

	; Check hardware (before making any beeps)
		.if CHECK_HARDWARE
		rcall	hardware_check
		.endif

	; Check reset cause
		bst	temp7, PORF		; Power-on reset
		cpse	temp7, ZH		; or zero
		brtc	init_no_porf
		rcall	beep_f1			; Usual startup beeps
		rcall	beep_f2
		rcall	beep_f3
		rjmp	control_start
init_no_porf:
		sbrs	temp7, BORF		; Brown-out reset
		rjmp	init_no_borf
		rcall	beep_f3			; "dead cellphone"
		rcall	beep_f1
		sbr	flags0, (1<<NO_CALIBRATION)
		rjmp	control_start
init_no_borf:
		sbrs	temp7, EXTRF		; External reset
		rjmp	init_no_extrf
		rcall	beep_f4			; Single beep
		rjmp	control_start
init_no_extrf:
		cli				; Disable interrupts for terminal reset causes

		sbrs	temp7, WDRF		; Watchdog reset
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
		mov	i_temp1, temp7
		ldi	i_temp2, 8
init_bitbeep2:	sbrs	i_temp1, 0
		rcall	beep_f2
		sbrc	i_temp1, 0
		rcall	beep_f4
		rcall	wait120ms
		lsr	i_temp1
		dec	i_temp2
		brne	init_bitbeep2
		rjmp	init_bitbeep1		; Loop forever

control_start:

; Check cell count
.if BLIP_CELL_COUNT
	.if defined(mux_voltage) && !CELL_COUNT
		rcall	adc_cell_count
		cpi	temp1, 5
		brlo	cell_count_good		; Detection of >=~5 LiPo cells becomes ambiguous based on charge state
		ldi	temp1, 0
cell_count_good:
	.else
		ldi	temp1, CELL_COUNT
	.endif
		mov	YL, temp1		; Beep clobbers temp1-temp5
		cpi	YL, 0
		breq	cell_blipper1
cell_blipper:
		rcall	wait120ms
		ldi	temp2, 10		; Short blip (not too long for this)
		rcall	beep_f4_freq
		dec	YL
		brne	cell_blipper
		rcall	wait120ms
cell_blipper1:

.endif

control_disarm:
	; LEDs off while disarmed
		BLUE_off
		GRN_off
		RED_off

		cbr	flags0, (1<<SET_DUTY)	; We need to count a full rc_timeout for safe arming
		rcall	puls_scale

	; Enable timer interrupts (we only do this late to improve beep quality)
		ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE2)
		out	TIFR, temp1		; Clear TOIE1, OCIE1A, and TOIE2 flags
		out	TIMSK, temp1		; Enable t1ovfl_int, t1oca_int, t2ovfl_int

		.if defined(HK_PROGRAM_CARD)
	; This program card seems to send data at 1200 baud N81,
	; Messages start with 0xdd 0xdd, have 7 bytes of config,
	; and end with 0xde, sent two seconds after power-up or
	; after any jumper change.
		.equ	BAUD_RATE = 1200
		.equ	UBRR_VAL = F_CPU / BAUD_RATE / 16 - 1
		outi	UBRRH, high(UBRR_VAL), temp1
		outi	UBRRL, low(UBRR_VAL), temp1
		sbi	UCSRB, RXEN		; Do programming card rx by polling
		outi	UCSRC, (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0), temp1	; N81
		.endif

	; Initialize input sources (i2c and/or rc-puls)
		.if USE_UART && !defined(HK_PROGRAM_CARD)
		.equ	BAUD_RATE = 38400
		.equ	UBRR_VAL = F_CPU / BAUD_RATE / 16 - 1
		outi	UBRRH, high(UBRR_VAL), temp1
		outi	UBRRL, low(UBRR_VAL), temp1
		sbi	UCSRB, RXEN		; We don't actually tx
		outi	UCSRC, (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0), temp1	; N81
		in	temp1, UDR
		sbi	UCSRA, RXC		; clear flag
		sbi	UCSRB, RXCIE		; enable reception irq
		.endif
		.if USE_I2C
		rcall	i2c_init
		.endif
		.if USE_INT0 || USE_ICP
		rcp_int_rising_edge temp1
		rcp_int_enable temp1
		.endif

	; Wait for one of the input sources to give arming input

i_rc_puls1:	clr	rc_timeout
		cbr	flags1, (1<<EVAL_RC)+(1<<I2C_MODE)+(1<<UART_MODE)
		sts	rct_boot, ZH
		sts	rct_beacon, ZH
i_rc_puls2:	wdr
		.if defined(HK_PROGRAM_CARD)
		.endif
		sbrc	flags1, EVAL_RC
		rjmp	i_rc_puls_rx
		.if BOOT_JUMP
		rcall	boot_loader_test
		.endif
		.if BEACON
		lds	temp1, rct_beacon
		cpi	temp1, 120		; Beep every 120 * 16 * 65536us (~8s)
		brne	i_rc_puls2
		ldi	temp1, 60
		sts	rct_beacon, temp1	; Double rate after the first beep
		rcall	beep_f3			; Beacon
		.endif
		rjmp	i_rc_puls2
i_rc_puls_rx:	rcall	evaluate_rc_init
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
		cbi	UCSRB, RXEN		; Turn off receiver
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
		cbr     flags0, (1<<RCP_ERROR)

	; Fall through to restart_control
;-----bko-----------------------------------------------------------------
restart_control:
		rcall	switch_power_off	; Disables PWM timer, turns off all FETs
		cbr	flags0, (1<<SET_DUTY)	; Do not yet set duty on input
		.if BEACON_IDLE
		sts	idle_beacon, ZH
		.endif
		GRN_on				; Green on while armed and idle or braking
		RED_off
		BLUE_off
wait_for_power_on_init:
		sts	rct_boot, ZH
		sts	rct_beacon, ZH

		.if MOTOR_BRAKE || LOW_BRAKE
		lds	temp3, brake_want
		lds	temp4, brake_active
		cp	temp3, temp4
		breq	wait_for_power_on

		rcall	switch_power_off	; Disable any active brake
		sts	brake_active, temp3	; Set new brake_active to brake_want

		cpi	temp3, 1		; Neutral brake
		brne	set_brake1
		ldi	YL, 1 << low(BRAKE_SPEED)
		sts	brake_sub, YL
		ldi2	YL, YH, BRAKE_POWER
		rjmp	set_brake_duty

set_brake1:	cpi	temp3, 2		; Thumb brake
		brne	wait_for_power_on
		ldi	YL, 1 << low(LOW_BRAKE_SPEED)
		sts	brake_sub, YL
		ldi2	YL, YH, LOW_BRAKE_POWER

set_brake_duty:	ldi2	temp1, temp2, MAX_POWER
		sub	temp1, YL		; Calculate OFF duty
		sbc	temp2, YH
		rcall	set_new_duty_set
		ldi	ZL, low(pwm_brake_off)	; Enable PWM brake mode
		clr	tcnt2h
		clr	sys_control_l		; Abused as duty update divisor
		outi	TCCR2, T2CLK, temp1	; Enable PWM, cleared later by switch_power_off
		.endif

wait_for_power_on:
		wdr
		sbrc	flags1, EVAL_RC
		rjmp	wait_for_power_rx
		.if BEEP_RCP_ERROR
		sbrc	flags0, RCP_ERROR	; Check if we've seen bad PWM edges
		rcall	rcp_error_beep
		.endif
		.if BEACON_IDLE
		lds	temp1, idle_beacon
		cpi	temp1, 240		; Beep after ~4 minutes
		brcs	no_idle_beep
		ldi	temp1, 238
		sts	idle_beacon, temp1
		rcall	switch_power_off	; Brake may have been on
		rcall	wait30ms
		rcall	beep_f4
no_idle_beep:
		.endif
		tst	rc_timeout
		brne	wait_for_power_on	; Tight loop unless rc_timeout is zero
		.if BOOT_JUMP
		rcall	boot_loader_test
		.endif
		lds	temp1, rct_beacon
		cpi	temp1, 30		; Disarm after ~2 seconds of no signal
		brne	wait_for_power_on
		rcall	switch_power_off	; Brake may have been on
		rcall	wait30ms
		rcall	beep_f3			; Play beeps for signal lost, disarming
		rcall	beep_f2
		rjmp	control_disarm		; Do not start motor until neutral signal received once again

wait_for_power_rx:
		.if USE_I2C
		sbrc	flags0, EEPROM_RESET
		rcall	eeprom_reset_block
		sbrc	flags0, EEPROM_WRITE
		rcall	eeprom_write_block
		.endif
		rcall	evaluate_rc		; Only get rc_duty, don't set duty
		tst	rc_timeout		; If not a valid signal, loop
		breq	wait_for_power_on	; while increasing boot/beacon timers
		adiw	YL, 0			; If no power requested yet, loop
		breq	wait_for_power_on_init	; while resetting boot/beacon timers

start_from_running:
		rcall	switch_power_off
		comp_init temp1			; init comparator
		RED_off
		GRN_off
		BLUE_on

		ldi2	YL, YH, PWR_MIN_START	; Start with limited power to reduce the chance that we
		movw	sys_control_l, YL	; align to a timing harmonic

		sbr	flags0, (1<<SET_DUTY)
		; Set STARTUP flag and call update_timing which will set
		; last_tcnt1 and set the duty (limited by STARTUP) and
		; set POWER_ON.
		rcall	wait_timeout_init
		sts	start_delay, ZH
		sts	start_modulate, ZH
		sts	start_fail, ZH
		ldi	temp1, RCP_TOT		; Start with a short timeout to stop quickly
		mov	rc_timeout, temp1	; if we see no further pulses after the first.
		ldi	temp1, 6		; Do not enable FETs during first cycle to
		sts	powerskip, temp1	; see if motor is running, and align to it.
		ldi	temp1, ENOUGH_GOODIES	; If we can follow without a timeout, do not
		sts	goodies, temp1		; continue in startup mode (long ZC filtering).
		outi	TCCR2, T2CLK, temp1	; Enable PWM (ZL has been set to pwm_wdr)

;-----bko-----------------------------------------------------------------
; *** commutation utilities ***

.macro com1com2
		; Bp off, Ap on
		set_comp_phase_b temp1
		COMMUTATE_B_off
		sbrc	flags1, POWER_ON
		COMMUTATE_A_on
.endmacro

.macro com2com1
		; Bp on, Ap off
		set_comp_phase_a temp1
		COMMUTATE_A_off
		sbrc	flags1, POWER_ON
		COMMUTATE_B_on
.endmacro

.macro com2com3
		; Cn off, Bn on
		set_comp_phase_c temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<B_FET)
		PWM_FOCUS_C_off
		PWM_C_clear
		PWM_B_copy
		PWM_FOCUS_B_on
		sei
.endmacro

.macro com3com2
		; Cn on, Bn off
		set_comp_phase_b temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<C_FET)
		PWM_FOCUS_B_off
		PWM_B_clear
		PWM_C_copy
		PWM_FOCUS_C_on
		sei
.endmacro

.macro com3com4
		; Ap off, Cp on
		set_comp_phase_a temp1
		COMMUTATE_A_off
		sbrc	flags1, POWER_ON
		COMMUTATE_C_on
.endmacro

.macro com4com3
		; Ap on, Cp off
		set_comp_phase_c temp1
		COMMUTATE_C_off
		sbrc	flags1, POWER_ON
		COMMUTATE_A_on
.endmacro

.macro com4com5
		; Bn off, An on
		set_comp_phase_b temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<A_FET)
		PWM_FOCUS_B_off
		PWM_B_clear
		PWM_A_copy
		PWM_FOCUS_A_on
		sei
.endmacro

.macro com5com4
		; Bn on, An off
		set_comp_phase_a temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<B_FET)
		PWM_FOCUS_A_off
		PWM_A_clear
		PWM_B_copy
		PWM_FOCUS_B_on
		sei
.endmacro

.macro com5com6
		; Cp off, Bp on
		set_comp_phase_c temp1
		COMMUTATE_C_off
		sbrc	flags1, POWER_ON
		COMMUTATE_B_on
.endmacro

.macro com6com5
		; Cp on, Bp off
		set_comp_phase_b temp1
		COMMUTATE_B_off
		sbrc	flags1, POWER_ON
		COMMUTATE_C_on
.endmacro

.macro com6com1
		; An off, Cn on
		set_comp_phase_a temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<C_FET)
		PWM_FOCUS_A_off
		PWM_A_clear
		PWM_C_copy
		PWM_FOCUS_C_on
		sei
.endmacro

.macro com1com6
		; An on, Cn off
		set_comp_phase_c temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<A_FET)
		PWM_FOCUS_C_off
		PWM_C_clear
		PWM_A_copy
		PWM_FOCUS_A_on
		sei
.endmacro

;-----bko-----------------------------------------------------------------
; **** running control loop ****

run1:		.if MOTOR_REVERSE
		sbrs	flags1, REVERSE
		.else
		sbrc	flags1, REVERSE
		.endif
		rjmp	run_reverse

run_forward:
		rcall	wait_for_high
		com1com2
		sync_off
		rcall	wait_for_low
		com2com3
		rcall	wait_for_high
		com3com4
		rcall	wait_for_low
		com4com5
		sync_on
		rcall	wait_for_high
		com5com6
		rcall	wait_for_low
		com6com1
		rjmp	run6

run_reverse:
		rcall	wait_for_low
		com1com6
		sync_on
		rcall	wait_for_high
		com6com5
		rcall	wait_for_low
		com5com4
		rcall	wait_for_high
		com4com3
		sync_off
		rcall	wait_for_low
		com3com2
		rcall	wait_for_high
		com2com1

run6:
		.if MOTOR_BRAKE || LOW_BRAKE
		lds	temp1, brake_want
		cpse	temp1, ZH
		rjmp	run_to_brake
		.endif
		lds	temp1, goodies
		.if !MOTOR_BRAKE
		; If last commutation timed out and power is off, return to restart_control
		cpi	temp1, 0
		sbrs	flags1, POWER_ON
		breq	run_to_brake
		.endif
		movw	YL, sys_control_l	; Each time TIMING_MAX is hit, sys_control is lsr'd
		adiw	YL, 0			; If zero, try starting over (with powerskipping)
		breq	restart_run
		cpi	temp1, ENOUGH_GOODIES
		brcc	run6_2
		inc	temp1
		sts	goodies, temp1
		; Build up sys_control to PWR_MAX_START in steps.
		adiwx	YL, YH, (POWER_RANGE + 47) / 48
		ldi2	temp1, temp2, PWR_MAX_START
		; If we've been trying to start for a while,
		; modulate power to reduce heating.
		lds	temp3, start_fail
		lds	temp4, start_modulate
		subi	temp4, -START_MOD_INC
		sts	start_modulate, temp4
		brne	run6_1
		; If we've been trying for a long while, give up.
		subi	temp3, -START_FAIL_INC
		breq	start_failed
		sts	start_fail, temp3
run6_1:		; Allow first loop at full power, then modulate.
		cpi	temp3, START_FAIL_INC
		brcs	run6_3
		cpi	temp4, START_MOD_LIMIT
		brcs	run6_3
		ldi2	temp1, temp2, PWR_COOL_START
		rjmp	run6_3

run6_2:
		cbr	flags1, (1<<STARTUP)
		sts	start_fail, ZH
		sts	start_modulate, ZH
		RED_off
		; Build up sys_control to MAX_POWER in steps.
		; If SLOW_THROTTLE is disabled, this only limits
		; initial start ramp-up; once running, sys_control
		; will stay at MAX_POWER unless timing is lost.
		adiwx	YL, YH, (POWER_RANGE + 31) / 32
		ldi2	temp1, temp2, MAX_POWER
run6_3:		cp	YL, temp1
		cpc	YH, temp2
		brcs	run6_4
		movw	sys_control_l, temp1
		rjmp	run1
run6_4:		movw	sys_control_l, YL
		rjmp	run1

run_to_brake:	rjmp	restart_control
restart_run:	rjmp	start_from_running

;-----bko-----------------------------------------------------------------
; If we were unable to start for a long time, just sit and beep unless
; input goes back to no power. This might help us get found if crashed.
start_failed:
		rcall	switch_power_off
		cbr	flags0, (1<<SET_DUTY)
start_fail_beep:
		rcall	wait240ms
		rcall	wait240ms
		rcall	beep_f4			; "Start failed" beacon
		rcall	evaluate_rc		; Returns rc_duty when !SET_DUTY
		adiw	YL, 0			; Test for zero
		brne	start_fail_beep
		rjmp	control_disarm

;-----bko-----------------------------------------------------------------
demag_timeout:
		ldi	ZL, low(pwm_wdr)	; Stop PWM switching
		; Interrupts will not turn on any FETs now
		.if COMP_PWM && CPWM_SOFT
		; Turn off complementary PWM if it was on,
		; but leave on the high side commutation FET.
		sbrc	flags2, A_FET
		PWM_COMP_A_off
		sbrc	flags2, B_FET
		PWM_COMP_B_off
		sbrc	flags2, C_FET
		PWM_COMP_C_off
		.elif COMP_PWM && !CPWM_SOFT
		sbrc	flags2, A_FET
		PWM_FOCUS_A_off
		sbrc	flags2, B_FET
		PWM_FOCUS_B_off
		sbrc	flags2, C_FET
		PWM_FOCUS_C_off
		.endif
		PWM_ALL_off temp1
		RED_on
		; Skip power for the next commutation. Note that this
		; won't decrease a non-zero powerskip because demag
		; checking is skipped when powerskip is non-zero.
		ldi	temp1, 1
		sts	powerskip, temp1
		rjmp	wait_commutation
;-----bko-----------------------------------------------------------------
wait_timeout:
		sbrc	flags1, STARTUP
		rjmp	wait_timeout_start
		cpi	XH, ZC_CHECK_FAST
		brcs	wait_timeout_run
		ldi	XH, ZC_CHECK_FAST - 1	; Limit back-tracking
		cp	XL, XH
		brcc	wait_timeout1
		mov	XL, XH			; Clip current distance from crossing
wait_timeout1:	rcall	set_ocr1a_zct		; Set zero-crossing timeout
		rjmp	wait_for_edge2
wait_timeout_run:
		RED_on				; Turn on red LED
wait_timeout_start:
		sts	goodies, ZH		; Clear good commutation count
		lds	temp4, start_delay
		subi	temp4, -START_DELAY_INC	; Increase start (blanking) delay,
		sbrc	flags1, STARTUP		; unless we were running
		sts	start_delay, temp4
wait_timeout_init:
		sbr	flags1, (1<<STARTUP)	; Leave running mode
		rjmp	wait_commutation	; Update timing and duty.
;-----bko-----------------------------------------------------------------
wait_for_low:	cbr	flags1, (1<<ACO_EDGE_HIGH)
		rjmp	wait_for_edge
;-----bko-----------------------------------------------------------------
wait_for_high:	sbr	flags1, (1<<ACO_EDGE_HIGH)
;-----bko-----------------------------------------------------------------
; Here we wait for the zero-crossing on the undriven phase to synchronize
; with the motor timing. The voltage of the undriven phase should cross
; the average of all three phases at half of the way into the 60-degree
; commutation period.
;
; The voltage on the undriven phase is affected by noise from PWM (mutual
; inductance) and also the demagnetization from the previous commutation
; step. Demagnetization time is proportional to motor current, and in
; extreme cases, may take more than 30 degrees to complete. To avoid
; sensing erroneous early zero-crossings in this case and losing motor
; synchronization, we check that demagnetization has finished after the
; minimum blanking period. If we do not see it by the maximum blanking
; period (about 30 degrees since we commutated last), we turn off power
; and continue as if the ZC had occurred. PWM is enabled again after the
; next commutation step.
;
; Normally, we wait for the blanking window to pass, look for the
; comparator to swing as the sign of the zero crossing, wait for the
; timing delay, and then commutate.
;
; Simulations show that the demagnetization period shows up on the phase
; being monitored by the comparator with no PWM-induced noise. As such,
; we do not need any filtering. However, it may not show up immediately
; due to filtering capacitors, hence the initial blind minimum blanking
; period.
;
; Special case: powerskipping during start. The idea here is to learn the
; timing of a possibly-spinning motor while not driving it, which would
; induce demagnetization and PWM noise that we cannot ignore until we
; know the timing. We use twice the timeout that would otherwise bring
; ZC check count to 0xff. A motor spinning twice the speed or slower will
; fall through to regular startup with ZC check count at 0xff. This lets
; us start from braking, RC timeout, or power-up without misaligning.
;
wait_for_edge:
		lds	temp1, powerskip	; Are we trying to track a maybe running motor?
		subi	temp1, 1
		brcs	wait_pwm_enable
		sts	powerskip, temp1
		sbrs	flags1, STARTUP
		rjmp	wait_for_edge0
		ldi	YL, byte1(0xff * 0x100)	; Timing is 120 degrees, so wait for
		ldi	YH, byte2(0xff * 0x100)	; what would be 0xff at 60 degrees
		mov	temp7, ZH
		rcall	set_ocr1a_rel
		ldi	XL, ZC_CHECK_MIN	; There shouldn't be much noise with no power
		rjmp	wait_for_edge1
wait_pwm_enable:
		cpi	ZL, low(pwm_wdr)
		brne	wait_pwm_running
		ldi	ZL, low(pwm_off)	; Re-enable PWM if disabled for powerskip or sync loss avoidance
		RED_off				; wait_timeout would have happened if motor not spinning during powerskip
wait_pwm_running:
		sbrc	flags1, STARTUP
		rjmp	wait_startup
		ldi	temp4, 13 * 256 / 120
		rcall	set_timing_degrees
		rcall	wait_OCT1_tot		; Wait for the minimum blanking period

		ldi	temp4, 42 * 256 / 120
		rcall	set_timing_degrees	; Set timeout for maximum blanking period
wait_for_demag:
		sbrs	flags0, OCT1_PENDING
		rjmp	demag_timeout
		sbrc	flags1, EVAL_RC
		rcall	evaluate_rc
		in	temp3, ACSR
		eor	temp3, flags1
		.if HIGH_SIDE_PWM
		sbrs	temp3, ACO		; Check for opposite level (demagnetization)
		.else
		sbrc	temp3, ACO		; Check for opposite level (demagnetization)
		.endif
		rjmp	wait_for_demag
wait_for_edge0:
		lds	XL, timing_h		; Copy high and extended byte
		lds	XH, timing_x		; to calculate the ZC check count
		lsr	XH			; Quarter to obtain timing / 1024
		ror	XL
		lsr	XH
		ror	XL
		cpi	XL, ZC_CHECK_MIN
		cpc	XH, ZH
		brcs	wait_for_edge_fast_min
		breq	wait_for_edge_fast
.if ZC_CHECK_MAX < 256
		cpi	XL, ZC_CHECK_MAX
.else
		cpi	XL, 255
.endif
		cpc	XH, ZH
		brcs	wait_for_edge_below_max
		ldi	XL, ZC_CHECK_MAX	; Limit to ZC_CHECK_MAX
wait_for_edge_below_max:
		cpi	XL, ZC_CHECK_FAST	; For faster timing, set normal ZC timeout
		brcs	wait_for_edge_fast
		ldi	temp4, 24 * 256 / 120	; With slower (longer) timing, set timer
		rcall	set_timing_degrees	; for limited backtracing
		rjmp	wait_for_edge1
wait_for_edge_fast_min:
		ldi	XL, ZC_CHECK_MIN
wait_for_edge_fast:
		rcall	set_ocr1a_zct		; Set zero-crossing timeout

wait_for_edge1:	mov	XH, XL
wait_for_edge2:	sbrs	flags0, OCT1_PENDING
		rjmp	wait_timeout
		sbrc	flags1, EVAL_RC
		rcall	evaluate_rc
		in	temp3, ACSR
.if 0
		; Visualize comparator output on the flag pin
		sbrc	temp3, ACO
		flag_on
		nop
		flag_off
		sbrs	temp3, ACO
		flag_on
.endif
		eor	temp3, flags1
		.if HIGH_SIDE_PWM
		sbrs	temp3, ACO
		.else
		sbrc	temp3, ACO
		.endif
		rjmp	wait_for_edge3
		cp	XL, XH			; Not yet crossed
		adc	XL, ZH			; Increment if not at zc_filter
		rjmp	wait_for_edge2
wait_for_edge3:	dec	XL			; Zero-cross has happened
		brne	wait_for_edge2		; Check again unless temp1 is zero

wait_commutation:
		flag_on
		rcall	update_timing
		sbrs	flags1, STARTUP
		rcall	wait_OCT1_tot
		flag_off
		lds	temp1, powerskip
		cpse	temp1, ZH
		cbr	flags1, (1<<POWER_ON)	; Disable power when powerskipping
		cpse	rc_timeout, ZH
		ret
		pop	temp1			; Throw away return address
		pop	temp1
		rjmp	restart_control		; Restart control immediately on RC timeout

; When starting, we have no idea what sort of motor may be connected,
; or how much time it will take for the previous commutation current
; to stop flowing. If the motor is not spinning and the sensing while
; powerskipping fails, we apply power and check for back-EMF after an
; increasing delay, building up further if the motor remains stalled.
; This will stretch out short commutations caused by the comparator
; sitting low or high when inputs are below the minimum offset.
wait_startup:
		ldi3	YL, YH, temp4, START_DELAY_US * CPU_MHZ
		mov	temp7, temp4
		lds	temp1, goodies
		cpi	temp1, 2		; After some good commutations,
		brcc	wait_startup1		; skip additional delay injection
		lds	temp4, start_delay
		ldi3	temp1, temp2, temp3, START_DSTEP_US * CPU_MHZ * 0x100
		rcall	update_timing_add_degrees	; Add temp4 (start_delay) START_DSTEPs of wait
wait_startup1:	rcall	set_ocr1a_rel
		rcall	wait_OCT1_tot
		ldi3	YL, YH, temp4, TIMEOUT_START * CPU_MHZ
		mov	temp7, temp4
		rcall	set_ocr1a_rel
; Powered startup: Use a fixed (long) ZC check count until goodies reaches
; ENOUGH_GOODIES and the STARTUP flag is cleared.
		ldi	XL, 0xff * CPU_MHZ / 16
		rjmp	wait_for_edge1

;-----bko-----------------------------------------------------------------
; init after reset

; The reset vector points here, right at the end of the main program.
; This nop-sleds to the boot loader if the program flash was incomplete.
reset:
		rjmp	main

.if BOOT_LOADER
.include "boot.inc"
.endif
