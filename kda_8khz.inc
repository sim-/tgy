;*************************************************
;* Keda/Multistar 12A, 20A, 30A with P/N-ch FETs *
;* 8kHz PWM variant for early-ish 2014 boards    *
;* before they went to an all-N-channel design   *
;*************************************************

; Like HK-SS TowerPro clones, it seems some Multistar 30A boards
; have a P-channel FET gate noise problem that causes overheating
; if the P-ch body diode is conducting when the PWM chops on.
; This happens less often at 8kHz, so, uhh...fixed!

#include "kda.inc"

.equ    T_CPU_MHZ	= F_CPU / 1000000
.equ    MIN_DUTY        = 56 * T_CPU_MHZ / 16
.equ    POWER_RANGE     = 2000 * T_CPU_MHZ / 16 + MIN_DUTY
