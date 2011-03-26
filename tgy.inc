;*********************************************************** 
;* tp-18a                                                  *
;* Mai.2007
;* http://home.versanet.de/~b-konze/                       *
;*
;* TGYP2010_416HzRCIntervalRate_PPM-Mod-only_NoCal
;* Renamed and renumbered to reflect changing of names
;* to reflect target ESC type and to harmonize version
;* numbers.
;* Arthur P. Meiners, 2010-01-26
;*********************************************************** 

;* only updated name

; activate Motor brake 

;.equ MOT_BRAKE      = 0

;Select Brake Power
;	1 = min Brake
;	2 = medium Brake
;  	3 = hard Brake

.equ BrakeSelect    = 2

.equ CALIBRATE			= 1      
.equ OSCAL_VALUE		= 0x01ff 	; eeprom position

.equ FORCE_LIPO 	= 1			; 1=Lipo und 0=NC wenn keine Buecke
.equ ActiveFreeRun	= 0

;**** **** **** **** ****
; PORT  definitions
;**** **** **** **** ****

;*********************
; PORT D 
;*********************  
.equ    mux_c   	= 7     ;i <plus> comparator input (AIN0)
.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	ApFET		= 5	;o
.equ	BpFET		= 4	;o
.equ	CpFET		= 3	;o
.equ	rcp_in		= 2	;i r/c pulse input

.equ	INIT_PD		= 0
.equ	DIR_PD		= (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
.equ	BRAKE_PD	= 0

.MACRO ApFET_on
	sbi	PORTD,5
.ENDMACRO
.MACRO ApFET_off
	cbi	PORTD,5
.ENDMACRO
.MACRO BpFET_on
	sbi	PORTD,4
.ENDMACRO
.MACRO BpFET_off
	cbi	PORTD,4
.ENDMACRO
.MACRO CpFET_on
	sbi	PORTD,3
.ENDMACRO
.MACRO CpFET_off
	cbi	PORTD,3
.ENDMACRO


;*********************
; PORT C definitions *
;*********************
.equ	mux_a		= 5	; ADC5 phase input
.equ	mux_b		= 4	; ADC4 phase input
;.equ    	  	= 3	; ADC3
;.equ    	  	= 2	; ADC2
.equ 	temp_adc	= 1 	; ADC1 temperature control input 
.equ 	accu_adc	= 0 	; ADC0 voltage control input 

;*********************
; ADC only
;*********************
;.equ    	  	= 7	; ADC7
;.equ    	  	= 6	; ADC6

.equ	INIT_PC		= 0
.equ	DIR_PC		= 0
.equ	BRAKE_PC	= 0


;*********************
; PORT B definitions *
;*********************
;.equ		= 7	
;.equ		= 6	
;.equ		= 5    	(sck stk200 interface) 
;.equ		= 4	(miso stk200 interface)
;.equ		= 3	(mosi stk200 interface) 
;.equ		= 2	
;.equ		= 1	
;.equ		= 0

.equ	AnFET		= 2	;o
.equ	BnFET		= 1	;o
.equ	CnFET		= 0	;o

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
.equ	BRAKE_PB	= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.MACRO AnFET_on
	sbi	PORTB,2
.ENDMACRO
.MACRO AnFET_off
	cbi	PORTB,2
.ENDMACRO
.MACRO BnFET_on
	sbi	PORTB,1
.ENDMACRO
.MACRO BnFET_off
	cbi	PORTB,1
.ENDMACRO
.MACRO CnFET_on
	sbi	PORTB,0
.ENDMACRO
.MACRO CnFET_off
	cbi	PORTB,0
.ENDMACRO



