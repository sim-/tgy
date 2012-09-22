; Simple boot loader on PWM input pin.
;
; We stay here as long as the input pin is pulled high, which is typical
; for the Turnigy USB Linker. The Turnigy USB Linker sports a SiLabs MCU
; (5V tolerant I/O) which converts 9600baud serial output from a SiLabs
; CP2102 USB-to-serial converter to a half duplex wire encoding which
; avoids signalling that can look like valid drive pulses. All bits are
; either one or two pulses, as opposed to a serial UART which could go
; high or low for a long time. This means it _should_ be safe to signal
; even to an armed ESC, as long as the low end has not been calibrated
; or set to start at pulses shorter than the linker timing.
;
; All transmissions have a leader of three 0xff bytes plus one 0-bit.
; Bit encoding starts at the least significant bit and is 8 bits wide.
; 1-bits are encoded as 64.0us high, 72.8us low (135.8us total).
; 0-bits are encoded as 27.8us high, 34.5us low, 34.4us high, 37.9 low
; (134.6us total)
; End of encoding adds 34.0us high, then return to input mode.
; The last 0-bit low time is 32.6us instead of 37.9us, for some reason.
;
; We always learn the actual timing from the host's leader. It seems to
; be possible to respond faster or slower, but faster will cause drops
; between the host and its serial-to-USB conversion at 9600baud. It does
; seem to work to use an average of high and low times as the actual bit
; timing, but since it doesn't quite fit in one byte at clk/8 at 16MHz,
; we store the high and low times separately, and copy the same timings.
; We should still work even at many times the bit rate.
;
; We support self-flashing ourselves (yo dawg), but doing so in a way
; that can still respond after each page update is a bit tricky. Some
; nops are present for future expansion without bumping addresses.
;
; We implement STK500v2, as recommended by the avrdude author, rather
; than implementing a random new protocol. STK500v2 protocol is the only
; serial protocol that passes the chip signature bytes directly instead
; of using a lookup table. However, avrdude uses CMD_SPI_MULTI to get
; these, which is for direct SPI access. We have to catch this and fake
; the response. We respond to CMD_SIGN_ON with "AVRISP_2", which keeps
; all messaes in the same format and with xor-checksums. We could say
; "AVRISP_MK2" and drop the message structure after sign-on, but then
; there is nothing to synchronize messages or do checksums.
;
; Note that to work with the Turnigy USB linker, the baud rate must be
; set to 9600.
;
; Registers:
; r0: Temporary, spm data (temp5)
; r1: Temporary, spm data (temp6)
; r2: Half-bit low time in timer2 ticks
; r3: Half-bit high time in timer2 ticks
; r4: Quarter-bit average time in timer2 ticks
; r5: stk500v2 message checksum (xor)
; r6: stk500v2 message length low
; r7: stk500v2 message length high
; r8: 7/8th bit time in timer2 ticks
; r9: Unused
; r10: Doubled (word) address l
; r11: Doubled (word) address h
; r12: Address l
; r13: Address h
; r14: Temporary (for checking TIFR, Z storage) (temp7)
; r15: Temporary (Z storage)
; r16: Zero
; r17: EEPROM read/write flags
; r18: Unused
; r19: Unused
; r20: Set for clearing TOV2/OCF2 flags
; r21: Timeout
; r22: Byte storage for bit shifting rx/tx (temp3)
; r23: Temporary (temp4)
; r24: Loop counter (temp1)
; r25: Loop counter (temp2)
; X: TX pointer
; Y: RX pointer
; Z: RX jump state pointer
;
; We keep the RX buffer just past start of RAM,
; and start building the response at the start of ram.
; The whole RAM area is used as the RX/TX buffer.
.equ	RX_BUFFER = SRAM_START + 32
.equ	TX_BUFFER = SRAM_START

; Number of RX timeouts / unsuccessful restarts before exiting boot loader
; If we get stray pulses or continuous high/low with no successful bytes
; received, we will exit the boot loader after this many tries.
.equ	BOOT_RX_TRIES = 20

; STK message constants
.equ	MESSAGE_START			= 0x1b
.equ	TOKEN				= 0x0e

; STK general command constants
.equ	CMD_SIGN_ON			= 0x01
.equ	CMD_SET_PARAMETER		= 0x02
.equ	CMD_GET_PARAMETER		= 0x03
.equ	CMD_SET_DEVICE_PARAMETERS	= 0x04
.equ	CMD_OSCCAL			= 0x05
.equ	CMD_LOAD_ADDRESS		= 0x06
.equ	CMD_FIRMWARE_UPGRADE		= 0x07
.equ	CMD_CHECK_TARGET_CONNECTION	= 0x0d
.equ	CMD_LOAD_RC_ID_TABLE		= 0x0e
.equ	CMD_LOAD_EC_ID_TABLE		= 0x0f

; STK ISP command constants
.equ	CMD_ENTER_PROGMODE_ISP		= 0x10
.equ	CMD_LEAVE_PROGMODE_ISP		= 0x11
.equ	CMD_CHIP_ERASE_ISP		= 0x12
.equ	CMD_PROGRAM_FLASH_ISP		= 0x13
.equ	CMD_READ_FLASH_ISP		= 0x14
.equ	CMD_PROGRAM_EEPROM_ISP		= 0x15
.equ	CMD_READ_EEPROM_ISP		= 0x16
.equ	CMD_PROGRAM_FUSE_ISP		= 0x17
.equ	CMD_READ_FUSE_ISP		= 0x18
.equ	CMD_PROGRAM_LOCK_ISP		= 0x19
.equ	CMD_READ_LOCK_ISP		= 0x1a
.equ	CMD_READ_SIGNATURE_ISP		= 0x1b
.equ	CMD_READ_OSCCAL_ISP		= 0x1c
.equ	CMD_SPI_MULTI			= 0x1d

; STK status constants
.equ	STATUS_CMD_OK			= 0x00
.equ	STATUS_CMD_TOUT			= 0x80
.equ	STATUS_RDY_BSY_TOUT		= 0x81
.equ	STATUS_SET_PARAM_MISSING	= 0x82
.equ	STATUS_CMD_FAILED		= 0xc0
.equ	STATUS_CKSUM_ERROR		= 0xc1
.equ	STATUS_CMD_UNKNOWN		= 0xc9
.equ	STATUS_CMD_ILLEGAL_PARAMETER	= 0xca

; STK parameter constants
.equ	PARAM_BUILD_NUMBER_LOW		= 0x80
.equ	PARAM_BUILD_NUMBER_HIGH		= 0x81
.equ	PARAM_HW_VER			= 0x90
.equ	PARAM_SW_MAJOR			= 0x91
.equ	PARAM_SW_MINOR			= 0x92
.equ	PARAM_VTARGET			= 0x94
.equ	PARAM_VADJUST			= 0x95 ; STK500 only
.equ	PARAM_OSC_PSCALE		= 0x96 ; STK500 only
.equ	PARAM_OSC_CMATCH		= 0x97 ; STK500 only
.equ	PARAM_SCK_DURATION		= 0x98 ; STK500 only
.equ	PARAM_TOPCARD_DETECT		= 0x9a ; STK500 only
.equ	PARAM_STATUS			= 0x9c ; STK500 only
.equ	PARAM_DATA			= 0x9d ; STK500 only
.equ	PARAM_RESET_POLARITY		= 0x9e ; STK500 only, and STK600 FW version <= 2.0.3
.equ	PARAM_CONTROLLER_INIT		= 0x9f

; Support listening on ICP pin (on AfroESCs)
.if defined(USE_ICP) && USE_ICP
.equ	RCP_PORT = PORTB
.equ	RCP_DDR = DDRB
.else
.equ	RCP_PORT = PORTD
.equ	RCP_DDR = DDRD
.endif

; THIRDBOOTSTART on the ATmega8 is 0xe00.
; Fuses shold have BOOTSZ1 set, BOOTSZ0 unset, BOOTRST set.
; Last nibble of hfuse should be A or 2 to save EEPROM on chip erase.
; Do not set WTDON. Implementing support for it here is big/difficult.
.equ BOOT_START = THIRDBOOTSTART
.org BOOT_START
boot_reset:	ldi	ZL, high(RAMEND)	; Set up stack
		ldi	ZH, low(RAMEND)
		out	SPH, ZH
		out	SPL, ZL
		ldi	r16, 0			; Use r16 as zero
		ldi	ZL, low(stk_rx_start)
		ldi	ZH, high(stk_rx_start)
		ldi	YL, low(RX_BUFFER)
		ldi	YH, high(RX_BUFFER)
		ldi	XL, low(TX_BUFFER)
		ldi	XH, high(TX_BUFFER)
		ldi	r20, (1<<CS21)		; timer2: clk/8 ... 256 ticks @ 16MHz = 128us; @ 8MHz = 256us
		out	TCCR2, r20
		ldi	r21, -BOOT_RX_TRIES
boot_rx_time:	inc	r21
		breq	boot_exit		; Exit if too many unsuccessful rx restarts
		ldi	r20, (1<<TOV2)+(1<<OCF2)
		out	TCNT2, r16
		out	TIFR, r20
boot_rx_time1:	cpi	XL, low(TX_BUFFER)
		breq	boot_rx_no_tx
		in	r14, TIFR
		sbrc	r14, TOV2
		rjmp	boot_tx_bytes
boot_rx_no_tx:	sbic	PIND, rcp_in
		rjmp	boot_rx_time1		; Loop while high, waiting for low edge
		out	TCNT2, r16
		out	TIFR, r20
boot_rx_time2:	in	r14, TIFR
		sbrc	r14, TOV2
boot_exit:	rjmp	FLASHEND + 1		; Low too long -- exit boot loader
		sbis	PIND, rcp_in		; Loop whlle low
		rjmp	boot_rx_time2
		out	TCNT2, r16
		out	TIFR, r20		; Start measuring high time
boot_rx_time3:	in	r14, TIFR
		sbrc	r14, TOV2
		rjmp	boot_rx_time		; High too long, start over
		sbic	PIND, rcp_in		; Loop whlle high, waiting for low edge
		rjmp	boot_rx_time3
		in	r3, TCNT2		; Save learned high time
		out	TCNT2, r16
		out	TIFR, r20		; Start measuring low time
boot_rx_time4:	in	r14, TIFR
		sbrc	r14, TOV2
		rjmp	FLASHEND + 1		; Low too long, exit boot loader
		sbis	PIND, rcp_in		; Loop whlle low, waiting for high edge
		rjmp	boot_rx_time4
		in	r2, TCNT2		; Save learned low time
		mov	r0, r2
		add	r0, r3
	; C:r0 now contains the number of timer2 ticks for one bit.
	; 7/8ths of this should be just enough to see two high to
	; low transitions for 0-bits, or one high-to-low for 1-bits.
	; Subtract 1/8th to get a time at which we check the edge
	; count and then wait for the next bit.
		mov	r8, r0			; C:r8 holds full time (9-bit)
		ror	r0			; r0 now holds half time (8-bit)
		lsr	r0
		mov	r4, r0			; Save quarter bit time (for tx)
		lsr	r0
		sbc	r8, r0			; Subtract 1/8th, rounding, unwrapping from 9th bit overflow
		com	r8			; Store one's complement for setting timer value
		com	r2			; Same for half-bit low time
		com	r3			; Same for half-bit high time
		com	r4			; Same for quarter-bit average time
		ldi	r22, 0b11100000		; Start with two leader bits and sentinel bit preloaded
		ldi	r24, 3			; Skip storing of 3 leader bytes
	; Bit-decoding: Set high-to-low edge counting timer (r8), and wait
	; for it to expire.
boot_rx:	out	TCNT2, r8
		out	TIFR, r20
boot_rx0:	in	r14, TIFR
		sbrc	r14, TOV2
		rjmp	FLASHEND + 1		; Low too long, exit boot loader
		sbis	PIND, rcp_in
		rjmp	boot_rx0
		out	TCNT2, r8		; Count falling edges for 75% of one bit time
		out	TIFR, r20
boot_rx1:	in	r14, TIFR
		sbrc	r14, TOV2
		rjmp	boot_rx_time		; High too long (or EOT), start over
		sbic	PIND, rcp_in
		rjmp	boot_rx1
		sec				; Receiving 1-bit
boot_rx2:	in	r14, TIFR
		sbrc	r14, TOV2
		rjmp	boot_rx_bit		; Timeout, must be 1-bit
		sbis	PIND, rcp_in
		rjmp	boot_rx2
boot_rx3:	in	r14, TIFR
		sbrc	r14, TOV2
		rjmp	boot_rx_time		; Hmm, timed out during second high
		sbic	PIND, rcp_in
		rjmp	boot_rx3
		clc				; Receiving 0-bit
boot_rx4:	in	r14, TIFR
		sbrc	r14, TOV2
		rjmp	boot_rx_bit		; Timeout, must be 0-bit
		sbis	PIND, rcp_in
		rjmp	boot_rx4

boot_tx_bytes:
		out	OCR2, r4		; Set OCF2 at quarter timing
		ldi	r24, 24			; Leader is 24 1-bits, 1 0-bit
boot_tx_leader:
		sbi	RCP_PORT, rcp_in	; Drive high
		sbi	RCP_DDR, rcp_in
		out	TCNT2, r3
		out	TIFR, r20
boot_tx_lead1:	in	r14, TIFR
		sbrs	r14, TOV2
		rjmp	boot_tx_lead1
		cbi	RCP_PORT, rcp_in	; Drive low
		out	TCNT2, r2
		out	TIFR, r20
boot_tx_lead2:	in	r14, TIFR
		sbrs	r14, TOV2
		rjmp	boot_tx_lead2
		dec	r24
		brne	boot_tx_leader

		ldi	YL, low(TX_BUFFER)
		ldi	YH, high(TX_BUFFER)

		ldi	r22, 0
		ldi	r24, 1
		rjmp	boot_tx_bits		; Send single start bit first

	; Interleaving rx/tx here to avoid branching trampolines.
boot_rx_bit:	ror	r22			; Roll rx bit in carry into r22
		brcc	boot_rx			; More bits to receive unless sentinel bit reached carry flag
		subi	r24, 1
		brcc	boot_rx_skip		; Don't store leader bytes
		ldi	r21, -BOOT_RX_TRIES	; Clear timeout on byte received
		ijmp				; Jump to current state handler

boot_tx:	cp	YL, XL
		cpc	YH, XH
		breq	boot_tx_end
		ld	r22, Y+
		ldi	r24, 8			; Send 8 bits
boot_tx_bits:	lsr	r22			; Put next bit in carry flag
		sbi	RCP_PORT, rcp_in	; Drive high
		out	TCNT2, r3
		out	TIFR, r20
boot_tx1:	in	r14, TIFR
		brcs	boot_tx2
		sbrc	r14, OCF2
		out	RCP_PORT, r16		; Drive low
boot_tx2:	sbrs	r14, TOV2
		rjmp	boot_tx1
		cbi	RCP_PORT, rcp_in
		brcs	boot_tx_low
		sbi	RCP_PORT, rcp_in	; Drive high
boot_tx_low:	out	TCNT2, r2
		out	TIFR, r20
boot_tx3:	in	r14, TIFR
		brcs	boot_tx4
		sbrc	r14, OCF2
		out	RCP_PORT, r16		; Drive low
boot_tx4:	sbrs	r14, TOV2
		rjmp	boot_tx3
		dec	r24
		brne	boot_tx_bits
		rjmp	boot_tx
	; Go high for a quarter bit time at the end
boot_tx_end:	sbi	RCP_PORT, rcp_in	; Drive high
		out	TCNT2, r3
		out	TIFR, r20
		ldi	YL, low(RX_BUFFER)
		ldi	YH, high(RX_BUFFER)
		ldi	XL, low(TX_BUFFER)
		ldi	XH, high(TX_BUFFER)
boot_tx_end1:	in	r14, TIFR
		sbrs	r14, OCF2
		rjmp	boot_tx_end1
		cbi	RCP_DDR, rcp_in		; Stop driving
		out	RCP_PORT, r16		; Turn off
		rjmp	boot_rx_time

boot_rx_cont:	ldi	r24, 0
boot_rx_skip:	ldi	r22, 0b10000000		; Restart with sentinel bit preloaded
		rjmp	boot_rx

; Simple implementation of stk500v2
; Do not clobber registers needed to reply: r2, r3, r8, r16, r20
stk_rx_restart:	ldi	ZL, low(stk_rx_start)
		ldi	ZH, high(stk_rx_start)
		ldi	YL, low(RX_BUFFER)
		ldi	YH, high(RX_BUFFER)
		rjmp	boot_rx_cont
		lds	r0, 0			; Future expansion nops
		lds	r0, 0
		lds	r0, 0
		lds	r0, 0
		lds	r0, 0
		lds	r0, 0
		lds	r0, 0
		lds	r0, 0
stk_rx_start:	nop				; Future expansion nops
		nop
		cpi	r22, MESSAGE_START
		brne	boot_rx_cont
		mov	r5, r22			; Start checksum in r5
		adiw	ZL, stk_rx_seq - stk_rx_start
		rjmp	boot_rx_cont
stk_rx_seq:	mov	i_sreg, r22		; Store sequence number in i_sreg
		eor	r5, r22
		adiw	ZL, stk_rx_size_h - stk_rx_seq
		rjmp	boot_rx_cont
stk_rx_size_h:	mov	r7, r22			; Store message length high in r7
		eor	r5, r22
		adiw	ZL, stk_rx_size_l - stk_rx_size_h
		rjmp	boot_rx_cont
stk_rx_size_l:	mov	r6, r22			; Store message length low in r6
		eor	r5, r22
		adiw	ZL, stk_rx_token - stk_rx_size_l
		rjmp	boot_rx_cont
stk_rx_token:	cpi	r22, TOKEN
		brne	stk_rx_restart
		eor	r5, r22
		adiw	ZL, stk_rx_body - stk_rx_token
		rjmp	boot_rx_cont
stk_rx_body:	st	Y+, r22
		eor	r5, r22
		cpi	YL, low(RAMEND)
		ldi	r24, high(RAMEND)
		cpc	YH, r24
		brcc	stk_rx_restart
		ldi	r24, 1
		sub	r6, r24
		sbc	r7, r16
		brne	stx_rx_cont
		adiw	ZL, stk_rx_cksum - stk_rx_body
stx_rx_cont:	rjmp	boot_rx_cont
stk_rx_cksum:	cpse	r22, r5
		rjmp	stk_rx_restart		; Restart if bad checksum
stk_rx:
	; Good checksum -- process message
	; We can use Z and Y now, since we will set it back to start in stk_rx_restart
	; Load the first three bytes into r22, r25, r24.
		ldi	YL, low(RX_BUFFER)	; Number of bytes to rx
		ldi	YH, high(RX_BUFFER)
		ld	r22, Y+			; Command byte
		ld	r25, Y+			; Parameter or address/count high,
		ld	r24, Y+			; Address/count low
	; Start the beginning of a typical response message
		movw	ZL, XL			; Start checksumming from here
		ldi	r23, MESSAGE_START
		st	Z, r23			; Message start
		std	Z+1, i_sreg		; Sequence number
		std	Z+2, r16		; Message body size high
		ldi	r23, 2
		std	Z+3, r23		; Message body size low
		ldi	r23, TOKEN
		std	Z+4, r23		; Message token
		std	Z+5, r22		; Command
		std	Z+6, r16		; Typical status OK (STATUS_CMD_OK)
		adiw	XL, 7
	; Check which command we received
		cpi	r22, CMD_SIGN_ON
		brne	scmd1			; Inverted tests for branch reach
		ldi	r24, SIGNATURE_LENGTH + 3
		std	Z+3, r24		; Messsage body size low
		ldi	r24, SIGNATURE_LENGTH
		st	X+, r24			; Signature size
		movw	YL, ZL
		ldi	ZL, low(avrisp_response_w << 1)
		ldi	ZH, high(avrisp_response_w << 1)
scmd_sign_on1:	lpm	r24, Z+
		st	X+, r24
		cpi	ZL, low((avrisp_response_w << 1) + SIGNATURE_LENGTH)
		brne	scmd_sign_on1
		movw	ZL, YL
scmd_send_chksum:
		ld	r24, Z+
chksum1:	ld	r22, Z+
		eor	r24, r22
		cp	ZL, XL
		cpc	ZH, XH
		brne	chksum1
		st	X+, r24			; Store xor checksum
		rjmp	stk_rx_restart
scmd1:		cpi	r22, CMD_SPI_MULTI
		brne	scmd2
	; avrdude uses spi_multi spi passthrough mode to check fuse bytes,
	; so we emulate this. Constants from the Arduino stk500v2 example
	; boot loader.
		mov	r23, r25		; Save NumTx in r23
		ldi	r25, 0			; Zero-extend r24
		adiw	r24, 3			; Command, status, rx'd bytes, status
		std	Z+3, r24		; Message body size low
		std	Z+2, r25		; Message body size high
		sbiw	r24, 3			; Back to just byte count
scmd_multi1:	st	X+, r16			; Fill return bufferwith zeroes
		dec	r24
		brne	scmd_multi1
	; Check for signature probe
	; Mirror address in result
		ld	r24, Y+			; RxStartAddr
		ld	r22, Y+			; TxData
		cpi	r22, 0x30		; Read signature bytes?
		cpc	r24, r16		; Only support RxStartAddr == 0
		ldi	r25, 4
		cpc	r23, r25		; Only support NumRx == 4
		brne	scmd_multi3
		std	Z+8, r22		; Echo back command
		ld	r24, Y+			; Address high
		cpi	r24, 0
		brne	scmd_multi3
		ld	r22, Y+			; Address low
		cpi	r22, 0
		ldi	r24, SIGNATURE_000	; atmega8 == 0x1e 0x93 0x07
		breq	scmd_multi2
		cpi	r22, 1
		ldi	r24, SIGNATURE_001
		breq	scmd_multi2
		cpi	r22, 2
		ldi	r24, SIGNATURE_002
		brne	scmd_multi3
scmd_multi2:	std	Z+10, r24		; Signature byte
scmd_multi3:	st	X+, r16			; STATUS_CMD_OK
		rjmp	scmd_send_chksum

scmd_load_address:
		cp	r24, r16
		cpc	r25, r16
		brne	scmd_fail
		ld	r13, Y+			; Save address
		ld	r12, Y+
		movw	r10, r12
		lsl	r10
		rol	r11
		rjmp	scmd_send_chksum
scmd2:
		cpi	r22, CMD_GET_PARAMETER
		breq	scmd_get_parameter
		cpi	r22, CMD_SET_PARAMETER
		breq	scmd_send_chksum	; Blind OK
		cpi	r22, CMD_ENTER_PROGMODE_ISP
		breq	scmd_send_chksum	; Blind OK
		cpi	r22, CMD_LEAVE_PROGMODE_ISP
		breq	scmd_send_chksum	; Blind OK
		cpi	r22, CMD_LOAD_ADDRESS
		breq	scmd_load_address
		cpi	r22, CMD_CHIP_ERASE_ISP
		breq	scmd_chip_erase
	; Commands after here are all read/write eeprom/flash types
		cpi	r24, low(RAMEND - TX_BUFFER - 12)
		ldi	r23, high(RAMEND - TX_BUFFER - 12)
		cpc	r25, r23
		brcc	scmd_fail		; Not enough RAM for that many bytes
		cpi	r22, CMD_READ_FLASH_ISP
		breq	scmd_read_flash
		cpi	r22, CMD_READ_EEPROM_ISP
		breq	scmd_read_eeprom
		adiw	YL, 7			; Skip useless write command bytes
		cpi	r22, CMD_PROGRAM_EEPROM_ISP
		breq	scmd_program_eeprom
		cpi	r22, CMD_PROGRAM_FLASH_ISP
		breq	scmd_program_flash
		nop				; Future expansion
		nop
scmd_fail:	ldi	r24, STATUS_CMD_FAILED
		std	Z+6, r24
		rjmp	scmd_send_chksum

scmd_get_parameter:
		cpi	r25, PARAM_HW_VER
		ldi	r24, 0xf
		breq	scmd_get_parameter_good
		cpi	r25, PARAM_SW_MAJOR
		ldi	r24, 0x2
		breq	scmd_get_parameter_good
		cpi	r25, PARAM_SW_MINOR
		ldi	r24, 0xa
		breq	scmd_get_parameter_good
		cpi	r25, PARAM_VTARGET
		ldi	r24, 50
		breq	scmd_get_parameter_good
		cpi	r25, PARAM_BUILD_NUMBER_LOW
		ldi	r24, 0
		breq	scmd_get_parameter_good
		cpi	r25, PARAM_BUILD_NUMBER_HIGH
		brne	scmd_fail
scmd_get_parameter_good:
		st	X+, r24
		ldi	r24, 3
		std	Z+3, r24		; Messsage body size low
		rjmp	scmd_send_chksum

scmd_read_flash:
		rcall	scmd_blob_message_size
		movw	YL, ZL			; Save Z
		movw	ZL, r10			; lpm can only use Z
scmd_read_rwwse_wait:
		rcall	boot_rwwsb_wt
		sbrc	r23, RWWSB
		rjmp	scmd_read_rwwse_wait	; Wait if flash still completing
scmd_read_fl1:	lpm	r22, Z+
		st	X+, r22
		sbiw	r24, 1
		brne	scmd_read_fl1
		movw	r10, ZL			; Save updated word address
		movw	ZL, YL			; Restore Z
		st	X+, r16			; STATUS_CMD_OK at end
		rjmp	scmd_send_chksum

scmd_read_eeprom:
		rcall	scmd_blob_message_size
		ldi	r17, (1<<EERE)
scmd_read_ee1:	rcall	boot_eeprom_rw		; Uses and increments byte address
		in	r22, EEDR
		st	X+, r22
		sbiw	r24, 1
		brne	scmd_read_ee1
		st	X+, r16			; STATUS_CMD_OK at end
		rjmp	scmd_send_chksum

; For chip erase, we just nuke the EEPROM.
scmd_chip_erase:
		clr	r12
		clr	r13
		ldi	r24, low(EEPROMEND+1)
		ldi	r25, high(EEPROMEND+1)
		set
scmd_program_eeprom:
		ldi	r17, (1<<EEMWE)+(1<<EEWE)
scmd_write_ee1:	ldi	r22, 0xff
		brts	scmd_write_ee2
		ld	r22, Y+
scmd_write_ee2:	rcall	boot_eeprom_rw
		sbiw	r24, 1
		brne	scmd_write_ee1
		clt
		rjmp	scmd_send_chksum

scmd_program_flash:
		cbr	r24, 0			; Round down
		ldi	r22, (1<<SPMEN)		; Store to temporary page buffer
		movw	r14, ZL			; Save Z
		movw	ZL, r10			; Load word address for page write
scmd_write_fl1:	ld	r0, Y+
		ld	r1, Y+
		rcall	boot_spm
		adiw	ZL, 2
		sbiw	r24, 2
		brne	scmd_write_fl1
		movw	r0, ZL			; Stash new address
		movw	ZL, r10			; Load old word address
		movw	r10, r0			; Save new word address
		ldi	r22, (1<<PGERS)+(1<<SPMEN)
		cpi	ZL, low(2*(boot_wr_flash & ~(PAGESIZE-1)))
		ldi	r23, high(2*(boot_wr_flash & ~(PAGESIZE-1)))
		cpc	ZH, r23
		breq	scmd_write_fl3		; Unless we are overwriting it,
		rcall	boot_wr_flash		; use the normal boot_wr_flash
scmd_write_fl2:	movw	ZL, r14			; Restore Z
		rjmp	scmd_send_chksum
scmd_write_fl3:	rcall	scmd_spm		; Erase page
		ldi	r22, (1<<PGWRT)+(1<<SPMEN)
		rcall	scmd_spm		; Write page
		ldi	r22, (1<<RWWSRE)+(1<<SPMEN)
		rcall	scmd_spm		; Re-enable RWW section
		rjmp	scmd_write_fl2
scmd_spm_wait:	in	r23, SPMCR		; Wait for previous SPM to finish
		sbrc	r23, SPMEN
		rjmp	scmd_spm_wait
scmd_ee_wait:	sbic	EECR, EEWE		; Wait for EEPROM write to finish
		rjmp	scmd_ee_wait
		ret
scmd_spm:	rcall	scmd_spm_wait
		out	SPMCR, r22		; Set SPM mode
		spm
		ret

scmd_blob_message_size:
		adiw	r24, 3			; Command, status, (data), status
		std	Z+2, r25		; Message body size high
		std	Z+3, r24		; Message body size low
		sbiw	r24, 3			; Back to just the byte count
		ret

boot_eeprom_rw:	rcall	boot_spm_wait
		out	EEARH, r13
		out	EEARL, r12
		sec
		adc	r12, r16		; Increment address
		adc	r13, r16
		mov	r23, r22		; Save desired value
		sbi	EECR, EERE		; Read existing EEPROM byte
		in	r22, EEDR
		cpse	r22, r23		; Return if byte matches
		sbrs	r17, EEMWE		; Return if only reading
		ret
		out	EEDR, r23		; Set new byte
		sbi	EECR, EEMWE		; Write arming
		out	EECR, r17		; Write
		ret

; Keep these addresses within a page so that we can self-update.
.org FLASHEND + 1 - 32
description:
	.db "http://github.com/sim-/tgy/", 0	; Hello!
avrisp_response_w:
	.equ SIGNATURE_LENGTH = 8
	.db "AVRISP_2"				; stk500v2 signature

boot_spm_wait:	in	r23, SPMCR		; Wait for previous SPM to finish
		sbrc	r23, SPMEN
		rjmp	boot_spm_wait
boot_ee_wait:	sbic	EECR, EEWE		; Wait for EEPROM write to finish
		rjmp	boot_ee_wait
		ret
boot_wr_flash:	rcall	boot_spm		; Erase page
		ldi	r22, (1<<PGWRT)+(1<<SPMEN)
		rcall	boot_spm		; Write page
boot_rwwsb_wt:	ldi	r22, (1<<RWWSRE)+(1<<SPMEN)
boot_spm:	rcall	boot_spm_wait
		out	SPMCR, r22		; Set SPM mode
		spm
		ret
.exit
