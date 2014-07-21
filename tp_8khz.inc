;***********************************************************
;* TowerPro 17A / 25A / HK-18A "type 1" boards             *
;* with all FETs on PORTD                                  *
;***********************************************************

; Some TowerPro type 1 clones have a P-channel FET gate noise
; problem that causes self-destruction if the P-ch body diode
; is conducting when the PWM chops on. This happens less often
; at 8kHz, so, uhh...fixed!

#include "tp.inc"

.equ    T_CPU_MHZ       = F_CPU / 1000000
.equ    MIN_DUTY        = 56 * T_CPU_MHZ / 16
.equ    POWER_RANGE     = 2000 * T_CPU_MHZ / 16 + MIN_DUTY
