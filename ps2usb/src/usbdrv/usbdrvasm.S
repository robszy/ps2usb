/* Name: usbdrvasm.S
 * Project: V-USB, virtual USB port for Atmel's(r) AVR(r) microcontrollers
 * Author: Christian Starkjohann
 * Creation Date: 2007-06-13
 * Tabsize: 4
 * Copyright: (c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

/*
General Description:
This module is the assembler part of the USB driver. This file contains
general code (preprocessor acrobatics and CRC computation) and then includes
the file appropriate for the given clock rate.
*/

#define __SFR_OFFSET 0      /* used by avr-libc's register definitions */
#include "usbportability.h"
#include "usbdrv.h"         /* for common defs */

/* register names */
#define x1      r16
#define x2      r17
#define shift   r18
#define cnt     r19
#define x3      r20
#define x4      r21
#define x5		r22
#define bitcnt  x5
#define phase   x4
#define leap    x4

/* Some assembler dependent definitions and declarations: */

#ifdef __IAR_SYSTEMS_ASM__
    extern  usbRxBuf, usbDeviceAddr, usbNewDeviceAddr, usbInputBufOffset
    extern  usbCurrentTok, usbRxLen, usbRxToken, usbTxLen, inTokenRec, outTokenRec, intCounter, ackNackRes, state, storedState
    extern  usbTxBuf, usbTxStatus1, usbTxStatus3
#   if USB_COUNT_SOF
        extern usbSofCount
#   endif
    public  usbCrc16
    public  usbCrc16Append

    COMMON  INTVEC
#   ifndef USB_INTR_VECTOR
        ORG     INT0_vect
#   else /* USB_INTR_VECTOR */
        ORG     USB_INTR_VECTOR
#       undef   USB_INTR_VECTOR
#   endif /* USB_INTR_VECTOR */
#   define  USB_INTR_VECTOR usbInterruptHandler
    rjmp    USB_INTR_VECTOR
    RSEG    CODE

#else /* __IAR_SYSTEMS_ASM__ */

#   ifndef USB_INTR_VECTOR /* default to hardware interrupt INT0 */
#       ifdef INT0_vect
#           define USB_INTR_VECTOR  INT0_vect       // this is the "new" define for the vector
#       else
#           define USB_INTR_VECTOR  SIG_INTERRUPT0  // this is the "old" vector
#       endif
#   endif
    .text
    .global USB_INTR_VECTOR
    .type   USB_INTR_VECTOR, @function
    .global usbCrc16
    .global usbCrc16Append
	.global waitForIdle
#endif /* __IAR_SYSTEMS_ASM__ */


#if USB_INTR_PENDING < 0x40 /* This is an I/O address, use in and out */
#   define  USB_LOAD_PENDING(reg)   in reg, USB_INTR_PENDING
#   define  USB_STORE_PENDING(reg)  out USB_INTR_PENDING, reg
#else   /* It's a memory address, use lds and sts */
#   define  USB_LOAD_PENDING(reg)   lds reg, USB_INTR_PENDING
#   define  USB_STORE_PENDING(reg)  sts USB_INTR_PENDING, reg
#endif

#define usbTxLen1   usbTxStatus1
#define usbTxBuf1   (usbTxStatus1 + 1)
#define usbTxLen3   usbTxStatus3
#define usbTxBuf3   (usbTxStatus3 + 1)


;----------------------------------------------------------------------------
; Utility functions
;----------------------------------------------------------------------------

#ifdef __IAR_SYSTEMS_ASM__
/* Register assignments for usbCrc16 on IAR cc */
/* Calling conventions on IAR:
 * First parameter passed in r16/r17, second in r18/r19 and so on.
 * Callee must preserve r4-r15, r24-r29 (r28/r29 is frame pointer)
 * Result is passed in r16/r17
 * In case of the "tiny" memory model, pointers are only 8 bit with no
 * padding. We therefore pass argument 1 as "16 bit unsigned".
 */
RTMODEL "__rt_version", "3"
/* The line above will generate an error if cc calling conventions change.
 * The value "3" above is valid for IAR 4.10B/W32
 */
#   define argLen   r18 /* argument 2 */
#   define argPtrL  r16 /* argument 1 */
#   define argPtrH  r17 /* argument 1 */

#   define resCrcL  r16 /* result */
#   define resCrcH  r17 /* result */

#   define ptrL     ZL
#   define ptrH     ZH
#   define ptr      Z
#   define byte     r22
#   define bitCnt   r19
#   define polyL    r20
#   define polyH    r21
#   define scratch  r23

#else  /* __IAR_SYSTEMS_ASM__ */ 
/* Register assignments for usbCrc16 on gcc */
/* Calling conventions on gcc:
 * First parameter passed in r24/r25, second in r22/23 and so on.
 * Callee must preserve r1-r17, r28/r29
 * Result is passed in r24/r25
 */
#   define argLen   r22 /* argument 2 */
#   define argPtrL  r24 /* argument 1 */
#   define argPtrH  r25 /* argument 1 */

#   define resCrcL  r24 /* result */
#   define resCrcH  r25 /* result */

#   define ptrL     XL
#   define ptrH     XH
#   define ptr      x
#   define byte     r18
#   define bitCnt   r19
#   define polyL    r20
#   define polyH    r21
#   define scratch  r23

#endif


; Wait for idle on bus to exit usart interrupt
waitForIdle:
    clr		r18
	clr		r19
loop:
    sbis    USBIN, USBPLUS ; 0
	inc		r18				
	inc		r19				; 
	nop2					; 2
	sbis    USBIN, USBPLUS ; 0
	inc		r18				
	inc		r19				;
	cp		r19,r18			; 1
    brne	waitForIdle
	cpi		r19,14
	brne	loop
	ret

#if USB_USE_FAST_CRC

; This implementation is faster, but has bigger code size
; Thanks to Slawomir Fras (BoskiDialer) for this code!
; It implements the following C pseudo-code:
; unsigned table(unsigned char x)
; {
; unsigned    value;
; 
;     value = (unsigned)x << 6;
;     value ^= (unsigned)x << 7;
;     if(parity(x))
;         value ^= 0xc001;
;     return value;
; }
; unsigned usbCrc16(unsigned char *argPtr, unsigned char argLen)
; {
; unsigned crc = 0xffff;
; 
;     while(argLen--)
;         crc = table(lo8(crc) ^ *argPtr++) ^ hi8(crc);
;     return ~crc;
; }

; extern unsigned usbCrc16(unsigned char *argPtr, unsigned char argLen);
;   argPtr  r24+25 / r16+r17
;   argLen  r22 / r18
; temp variables:
;   byte    r18 / r22
;   scratch r23
;   resCrc  r24+r25 / r16+r17
;   ptr     X / Z
usbCrc16:
    mov     ptrL, argPtrL
    mov     ptrH, argPtrH
    ldi     resCrcL, 0xFF
    ldi     resCrcH, 0xFF
    rjmp    usbCrc16LoopTest
usbCrc16ByteLoop:
    ld      byte, ptr+
    eor     resCrcL, byte   ; resCrcL is now 'x' in table()
    mov     byte, resCrcL   ; compute parity of 'x'
    swap    byte
    eor     byte, resCrcL
    mov     scratch, byte
    lsr     byte
    lsr     byte
    eor     byte, scratch
    inc     byte
    lsr     byte
    andi    byte, 1         ; byte is now parity(x)
    mov     scratch, resCrcL
    mov     resCrcL, resCrcH
    eor     resCrcL, byte   ; low byte of if(parity(x)) value ^= 0xc001;
    neg     byte
    andi    byte, 0xc0
    mov     resCrcH, byte   ; high byte of if(parity(x)) value ^= 0xc001;
    clr     byte
    lsr     scratch
    ror     byte
    eor     resCrcH, scratch
    eor     resCrcL, byte
    lsr     scratch
    ror     byte
    eor     resCrcH, scratch
    eor     resCrcL, byte
usbCrc16LoopTest:
    subi    argLen, 1
    brsh    usbCrc16ByteLoop
    com     resCrcL
    com     resCrcH
    ret

#else   /* USB_USE_FAST_CRC */

; This implementation is slower, but has less code size
;
; extern unsigned usbCrc16(unsigned char *argPtr, unsigned char argLen);
;   argPtr  r24+25 / r16+r17
;   argLen  r22 / r18
; temp variables:
;   byte    r18 / r22
;   bitCnt  r19
;   poly    r20+r21
;   scratch r23
;   resCrc  r24+r25 / r16+r17
;   ptr     X / Z
usbCrc16:
    mov     ptrL, argPtrL
    mov     ptrH, argPtrH
    ldi     resCrcL, 0
    ldi     resCrcH, 0
    ldi     polyL, lo8(0xa001)
    ldi     polyH, hi8(0xa001)
    com     argLen      ; argLen = -argLen - 1: modified loop to ensure that carry is set
    ldi     bitCnt, 0   ; loop counter with starnd condition = end condition
    rjmp    usbCrcLoopEntry
usbCrcByteLoop:
    ld      byte, ptr+
    eor     resCrcL, byte
usbCrcBitLoop:
    ror     resCrcH     ; carry is always set here (see brcs jumps to here)
    ror     resCrcL
    brcs    usbCrcNoXor
    eor     resCrcL, polyL
    eor     resCrcH, polyH
usbCrcNoXor:
    subi    bitCnt, 224 ; (8 * 224) % 256 = 0; this loop iterates 8 times
    brcs    usbCrcBitLoop
usbCrcLoopEntry:
    subi    argLen, -1
    brcs    usbCrcByteLoop
usbCrcReady:
    ret
; Thanks to Reimar Doeffinger for optimizing this CRC routine!

#endif /* USB_USE_FAST_CRC */

; extern unsigned usbCrc16Append(unsigned char *data, unsigned char len);
usbCrc16Append:
    rcall   usbCrc16
    st      ptr+, resCrcL
    st      ptr+, resCrcH
    ret

#undef argLen
#undef argPtrL
#undef argPtrH
#undef resCrcL
#undef resCrcH
#undef ptrL
#undef ptrH
#undef ptr
#undef byte
#undef bitCnt
#undef polyL
#undef polyH
#undef scratch


#if USB_CFG_HAVE_MEASURE_FRAME_LENGTH
#ifdef __IAR_SYSTEMS_ASM__
/* Register assignments for usbMeasureFrameLength on IAR cc */
/* Calling conventions on IAR:
 * First parameter passed in r16/r17, second in r18/r19 and so on.
 * Callee must preserve r4-r15, r24-r29 (r28/r29 is frame pointer)
 * Result is passed in r16/r17
 * In case of the "tiny" memory model, pointers are only 8 bit with no
 * padding. We therefore pass argument 1 as "16 bit unsigned".
 */
#   define resL     r16
#   define resH     r17
#   define cnt16L   r30
#   define cnt16H   r31
#   define cntH     r18

#else  /* __IAR_SYSTEMS_ASM__ */ 
/* Register assignments for usbMeasureFrameLength on gcc */
/* Calling conventions on gcc:
 * First parameter passed in r24/r25, second in r22/23 and so on.
 * Callee must preserve r1-r17, r28/r29
 * Result is passed in r24/r25
 */
#   define resL     r24
#   define resH     r25
#   define cnt16L   r24
#   define cnt16H   r25
#   define cntH     r26
#endif
#   define cnt16    cnt16L

; extern unsigned usbMeasurePacketLength(void);
; returns time between two idle strobes in multiples of 7 CPU clocks
.global usbMeasureFrameLength
usbMeasureFrameLength:
    ldi     cntH, 6         ; wait ~ 10 ms for D- == 0
    clr     cnt16L
    clr     cnt16H
usbMFTime16:
    dec     cntH
    breq    usbMFTimeout
usbMFWaitStrobe:            ; first wait for D- == 0 (idle strobe)
    sbiw    cnt16, 1        ;[0] [6]
    breq    usbMFTime16     ;[2]
    sbic    USBIN, USBMINUS ;[3]
    rjmp    usbMFWaitStrobe ;[4]
usbMFWaitIdle:              ; then wait until idle again
    sbis    USBIN, USBMINUS ;1 wait for D- == 1
    rjmp    usbMFWaitIdle   ;2
    ldi     cnt16L, 1       ;1 represents cycles so far
    clr     cnt16H          ;1
usbMFWaitLoop:
    in      cntH, USBIN     ;[0] [7]
    adiw    cnt16, 1        ;[1]
    breq    usbMFTimeout    ;[3]
    andi    cntH, USBMASK   ;[4]
    brne    usbMFWaitLoop   ;[5]
usbMFTimeout:
#if resL != cnt16L
    mov     resL, cnt16L
    mov     resH, cnt16H
#endif
    ret

#undef resL
#undef resH
#undef cnt16
#undef cnt16L
#undef cnt16H
#undef cntH

#endif  /* USB_CFG_HAVE_MEASURE_FRAME_LENGTH */

;----------------------------------------------------------------------------
; Now include the clock rate specific code
;----------------------------------------------------------------------------

#ifndef USB_CFG_CLOCK_KHZ
#   ifdef F_CPU
#       define USB_CFG_CLOCK_KHZ (F_CPU/1000)
#   else
#       error "USB_CFG_CLOCK_KHZ not defined in usbconfig.h and no F_CPU set!"
#   endif
#endif

#if USB_CFG_CHECK_CRC   /* separate dispatcher for CRC type modules */
#   if USB_CFG_CLOCK_KHZ == 18000
#       include "usbdrvasm18-crc.inc"
#   else
#       error "USB_CFG_CLOCK_KHZ is not one of the supported crc-rates!"
#   endif
#else   /* USB_CFG_CHECK_CRC */
#   if USB_CFG_CLOCK_KHZ == 12000
#       include "usbdrvasm12.inc"
#   elif USB_CFG_CLOCK_KHZ == 12800
#       include "usbdrvasm128.inc"
#   elif USB_CFG_CLOCK_KHZ == 15000
#       include "usbdrvasm15.inc"
#   elif USB_CFG_CLOCK_KHZ == 16000
#       include "usbdrvasm16.inc"
#   elif USB_CFG_CLOCK_KHZ == 16500
#       include "usbdrvasm165.inc"
#   elif USB_CFG_CLOCK_KHZ == 20000
#       #include "usbdrvasm20.inc"
#		warning "20 MHz"
#   else
#       error "USB_CFG_CLOCK_KHZ is not one of the supported non-crc-rates!"
#   endif
#endif /* USB_CFG_CHECK_CRC */

/* Name: usbdrvasm20.inc
 * Project: V-USB, virtual USB port for Atmel's(r) AVR(r) microcontrollers
 * Author: Jeroen Benschop
 * Based on usbdrvasm16.inc from Christian Starkjohann
 * Creation Date: 2008-03-05
 * Tabsize: 4
 * Copyright: (c) 2008 by Jeroen Benschop and OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

/* Do not link this file! Link usbdrvasm.S instead, which includes the
 * appropriate implementation!
 */

/*
General Description:
This file is the 20 MHz version of the asssembler part of the USB driver. It
requires a 20 MHz crystal (not a ceramic resonator and not a calibrated RC
oscillator).

See usbdrv.h for a description of the entire driver.

Since almost all of this code is timing critical, don't change unless you
really know what you are doing! Many parts require not only a maximum number
of CPU cycles, but even an exact number of cycles!
*/

#define leap2   x3
#ifdef __IAR_SYSTEMS_ASM__
#define nextInst    $+2
#else
#define nextInst    .+0
#endif

macro NOP5
	nop2
	nop2
	nop
	endm

;max stack usage: [ret(2), YL, SREG, YH, bitcnt, shift, x1, x2, x3, x4, cnt] = 12 bytes
;nominal frequency: 20 MHz -> 13.333333 cycles per bit, 106.666667 cycles per byte
; Numbers in brackets are clocks counted from center of last sync bit
; when instruction starts
;register use in receive loop:
; shift assembles the byte currently being received
; x1 holds the D+ and D- line state
; x2 holds the previous line state
; x4 (leap)  is used to add a leap cycle once every three bytes received
; X3 (leap2) is used to add a leap cycle once every three stuff bits received
; bitcnt is used to determine when a stuff bit is due
; cnt holds the number of bytes left in the receive buffer

USB_INTR_VECTOR:
;order of registers pushed: YL, SREG YH, [sofError], bitcnt, shift, x1, x2, x3, x4, cnt
    push    YL                  ;[-28] push only what is necessary to sync with edge ASAP
    in      YL, SREG            ;[-26]
    push    YL                  ;[-25]
;----------------------------------------------------------------------------
; Synchronize with sync pattern:
;----------------------------------------------------------------------------
;sync byte (D-) pattern LSb to MSb: 01010100 [1 = idle = J, 0 = K]
;sync up with J to K edge during sync pattern -- use fastest possible loops
;The first part waits at most 1 bit long since we must be in sync pattern.
;YL is guarenteed to be < 0x80 because I flag is clear. When we jump to
;waitForJ, ensure that this prerequisite is met.
waitForJ:
    inc     YL
    sbis    USBIN, USBMINUS
    brne    waitForJ        ; just make sure we have ANY timeout
waitForK:
;The following code results in a sampling window of < 1/4 bit which meets the spec.
    sbis    USBIN, USBMINUS     ;[-19]
    rjmp    foundK              ;[-18]
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
#if USB_COUNT_SOF
    lds     YL, usbSofCount
    inc     YL
    sts     usbSofCount, YL
#endif  /* USB_COUNT_SOF */
#ifdef USB_SOF_HOOK
    USB_SOF_HOOK
#endif
    rjmp    sofError
foundK:                         ;[-16]
;{3, 5} after falling D- edge, average delay: 4 cycles [we want 4 for center sampling]
;we have 1 bit time for setup purposes, then sample again. Numbers in brackets
;are cycles from center of first sync (double K) bit after the instruction
	NOP5
	nop2
	nop2							; 8 nops 4 mov to center of first k and 8+5 to be in center of second
    push    YH                  ;2 [2]
    lds     YL, usbInputBufOffset;2 [4]
    clr     YH                  ;1 [5]
    subi    YL, lo8(-(usbRxBuf));1 [6]
    sbci    YH, hi8(-(usbRxBuf));1 [7]

    sbis    USBIN, USBMINUS ;1 [8] we want two bits K [sample 1 cycle too early]
    rjmp    haveTwoBitsK    ;2 [10]
    pop     YH              ;2 [11] undo the push from before
    rjmp    waitForK        ;2 [13] this was not the end of sync, retry
haveTwoBitsK:
;----------------------------------------------------------------------------
; push more registers and initialize values while we sample the first bits:
;----------------------------------------------------------------------------
	;nop	; 4 nopy w 2 push i jeszcze 1 w ldi r22
	push	bitcnt
	ldi		r22,1			; load 1 at 0 bit
    push    shift           ;2 [16]
	push	r23
    push    x1              ;2 [12]
    push    x2              ;2 [14]

    in      x1, USBIN       ;1 [17] <-- sample bit 0
    ldi     shift, 0xff     ;1 [18]
    bst     x1, USBMINUS    ;1 [19]
    bld     shift, 0        ;1 [20]
    push    x3              ;2 [22]
	push	x4
    push    cnt             ;2 [24]
	ldi		leap,0x55		; init leap
	;nop2	; 2 nopy w push x4 i 2 w push r24
    push	r24
    in      x2, USBIN       ;1 [25] <-- sample bit 1
    ser     x3              ;1 [26] [inserted init instruction]
    eor     x1, x2          ;1 [27]
    bst     x1, USBMINUS    ;1 [28]
    bld     shift, 1        ;1 [29]
    ldi     cnt, USB_BUFSIZE;1 [30] [inserted init instruction]
    rjmp    rxbit2          ;2 [32]

;----------------------------------------------------------------------------
; Receiver loop (numbers in brackets are cycles within byte after instr)
;----------------------------------------------------------------------------


unstuff1:               ;1 (branch taken)
    mov     x2, x1      ;1 [21] x1 contains last sampled (stuffed) bit
    andi    x3, ~0x02   ;1 [22]
    ori     shift, 0x02 ;1 [23]
    nop                 ;1 [24]
	rjmp	doDelayunstuff1
didunstuff1:
    in      x1, USBIN   ;1 [25] <-- sample bit 2 again
    rjmp    didUnstuff1 ;2 [27]

unstuff2:               ;1 (branch taken)
    andi    x3, ~0x04   ;1 [29]
    ori     shift, 0x04 ;1 [30]
    mov     x1, x2      ;1 [31] x2 contains last sampled (stuffed) bit
    nop                 ;1 [32]
	rjmp	doDelayunstuff2
didunstuff2:
    in      x2, USBIN   ;1 [33] <-- sample bit 3
    rjmp    didUnstuff2 ;2 [35]

unstuff3:               ;1 (branch taken)
    in      x2, USBIN   ;1 [34] <-- sample stuffed bit 3 [one cycle too late]
    andi    x3, ~0x08   ;1 [35]
    ori     shift, 0x08 ;1 [36]
	rjmp	doDelayunstuff3a
didunstuff3a:
    rjmp    didUnstuff3 ;2 [38]

unstuff4:               ;1 (branch taken)
    andi    x3, ~0x10   ;1 [40]
    in      x1, USBIN   ;1 [41] <-- sample stuffed bit 4
    ori     shift, 0x10 ;1 [42]
    rjmp    didUnstuff4 ;2 [44]

unstuff5:               ;1 (branch taken)
    andi    x3, ~0x20   ;1 [48]
    in      x2, USBIN   ;1 [49] <-- sample stuffed bit 5
    ori     shift, 0x20 ;1 [50]
    rjmp    didUnstuff5 ;2 [52]

overflowGate:
	rjmp overflow

; extra jobs done during bit interval:
; bit 0:    store, clear [SE0 is unreliable here due to bit dribbling in hubs]
; bit 1:    se0 check
; bit 2:    overflow check
; bit 3:    recovery from delay [bit 0 tasks took too long]
; bit 4:    none
; bit 5:    none
; bit 6:    none
; bit 7:    jump, eor

didrxbit2:
    in      x1, USBIN   ;1 [17] <-- sample bit 2 (or possibly bit 1 stuffed)
    andi    shift, 0xf3 ;1 [18]
    breq    unstuff1    ;1 [19] do remaining work for bit 1
didUnstuff1:
	rjmp	doDelayN1
didN1:
    subi    cnt, 1      ;1 [20]
    brcs    overflowGate    ;1 [21] loop control
    eor     x2, x1      ;1 [22]
    bst     x2, USBMINUS;1 [23]
    bld     shift, 2    ;1 [24]
    in      x2, USBIN   ;1 [25] <-- sample bit 3 (or possibly bit 2 stuffed)
    andi    shift, 0xe7 ;1 [26]
    breq    unstuff2    ;1 [27]
didUnstuff2:
	rjmp	doDelayN2
didN2:
    eor     x1, x2      ;1 [28]
    bst     x1, USBMINUS;1 [29]
    bld     shift, 3    ;1 [30]
didUnstuff3:
    andi    shift, 0xcf ;1 [31]
    breq    unstuff3    ;1 [32]
    in      x1, USBIN   ;1 [33] <-- sample bit 4
    eor     x2, x1      ;1 [34]
    bst     x2, USBMINUS;1 [35]
    bld     shift, 4    ;1 [36]
didUnstuff4:
	rjmp	doDelayN4
didN4:
    andi    shift, 0x9f ;1 [37]
    breq    unstuff4    ;1 [38]
    ;nop2                ;2 [40]
	sbrc	r22,3
	lsl		r22			; set state = 16 - end of second int 1
    in      x2, USBIN   ;1 [41] <-- sample bit 5
    eor     x1, x2      ;1 [42]
    bst     x1, USBMINUS;1 [43]
    bld     shift, 5    ;1 [44]
didUnstuff5:
	rjmp	doDelayN5
didN5:
    andi    shift, 0x3f ;1 [45]
    breq    unstuff5    ;1 [46]
    ;nop2                ;2 [48]
	subi    leap, 0x55          ; trick to introduce a leap cycle every 3 bytes
    brcc    nextInst
    in      x1, USBIN   ;1 [49] <-- sample bit 6
    eor     x2, x1      ;1 [50]
    bst     x2, USBMINUS;1 [51]
    bld     shift, 6    ;1 [52]
didUnstuff6:
	;rjmp	doDelayN6
;didN6:					; 5 taktów do zagospodarowania
	USB_LOAD_PENDING(r24)
	USB_STORE_PENDING(r24) ; nie bêdziemy tutaj robiæ wstrzymywania tego ca³ego inta tylko w obs³udze zrobimy ifa
	sbrc	r24,PS2_INTR_PENDING_BIT ; INT1 pending flag set ?
	lsl		r22
	nop
    cpi     shift, 0x02 ;1 [53]
    brlo    unstuff6    ;1 [54]
    ;nop                 ;2 [56] jeden nop wyciêty bo load int a drugi bo store
	nop2
    in      x2, USBIN   ;1 [57] <-- sample bit 7
    eor     x1, x2      ;1 [58]
    bst     x1, USBMINUS;1 [59]
    bld     shift, 7    ;1 [60]
didUnstuff7:
	;nop2
	sbrc	r22,1		; check if we have first bit set -> first int1
	in		r23,PIND	; if yes cp 
	nop2
	sbrc	r22,1
	lsl		r22			; go to state = 4
    cpi     shift, 0x04 ;1 [61]
    brsh    rxLoop      ;2 [63] loop control
unstuff7:
    andi    x3, ~0x80   ;1 [63]
    ori     shift, 0x80 ;1 [64]
    in      x2, USBIN   ;1 [65] <-- sample stuffed bit 7
    nop                 ;1 [66]
    rjmp    didUnstuff7 ;2 [68]

rxLoop:
    eor     x3, shift   ;1 [0] reconstruct: x3 is 0 at bit locations we changed, 1 at others
    in      x1, USBIN   ;1 [1] <-- sample bit 0
    st      y+, x3      ;2 [3] store data
    ser     x3          ;1 [4]
    nop                 ;1 [5]
    eor     x2, x1      ;1 [6]
    bst     x2, USBMINUS;1 [7]
    bld     shift, 0    ;1 [8]
	
	sbrs	r22,3		; check if we have second int1 state = 8 
	rjmp	normFlowRx
	sbic	PIND,5
	ori		r22,0x20	; set fifth bit if set in r22
	rjmp	normFlowRRx
normFlowRx:
	nop2
	nop ; za nopa
normFlowRRx:				; 6 cykli
    in      x2, USBIN   ;1 [9] <-- sample bit 1 (or possibly bit 0 stuffed)
    andi    x2, USBMASK ;1 [10]
    breq    se0         ;1 [11] SE0 check for bit 1
    andi    shift, 0xf9 ;1 [12]
didUnstuff0:
    breq    unstuff0    ;1 [13]
    eor     x1, x2      ;1 [14]
    bst     x1, USBMINUS;1 [15]
    bld     shift, 1    ;1 [16]
rxbit2:
	rjmp	doDelayrxbit2


	
unstuff0:               ;1 (branch taken)
	rjmp	doDelayunstuff0 ; nop5
didunstuff0:
    andi    x3, ~0x01   ;1 [15]
    mov     x1, x2      ;1 [16] x2 contains last sampled (stuffed) bit
    in      x2, USBIN   ;1 [17] <-- sample bit 1 again
    ori     shift, 0x01 ;1 [18]
    rjmp    didUnstuff0 ;2 [20]

unstuff6:               ;1 (branch taken)
    andi    x3, ~0x40   ;1 [56]
    in      x1, USBIN   ;1 [57] <-- sample stuffed bit 6
    ori     shift, 0x40 ;1 [58]
    rjmp    didUnstuff6 ;2 [60]

    
;----------------------------------------------------------------------------
; End of receive loop. Now start handling EOP
;----------------------------------------------------------------------------

macro POP_STANDARD ; 14 cycles
	pop		r24
    pop     cnt
    pop     x4
    pop     x3
    pop     x2
    pop     x1
	pop		r23
    pop     shift
    pop     bitcnt
	pop     YH
    endm
macro POP_RETI     ; 7 cycles
    
    pop     YL
    out     SREG, YL
    pop     YL
    endm


#include "asmcommon.inc"






; USB spec says:
; idle = J
; J = (D+ = 0), (D- = 1)
; K = (D+ = 1), (D- = 0)
; Spec allows 7.5 bit times from EOP to SOP for replies
; 7.5 bit times is 100 cycles. This implementation arrives a bit later at se0
; then specified in the include file but there is plenty of time


sendNakAndReti:                 ;0 [-19] 19 cycles until SOP
    ldi     x3, USBPID_NAK      ;1 [-18]
    rjmp    usbSendX3           ;2 [-16]
sendAckAndReti:                 ;0 [-19] 19 cycles until SOP
    ldi     x3, USBPID_ACK      ;1 [-18]
    rjmp    usbSendX3           ;2 [-16]
sendCntAndReti:                 ;0 [-17] 17 cycles until SOP
    mov     x3, cnt             ;1 [-16]
usbSendX3:                      ;0 [-16]
    ldi     YL, 20              ;1 [-15] 'x3' is R20
    ldi     YH, 0               ;1 [-14]
    ldi     cnt, 2              ;1 [-13]
    rjmp    usbSendAndReti      ;fallthrough


bitstuff0:                  ;1 (for branch taken)
    eor     x1, x4          ;1
    ldi     x2, 0           ;1
	nop2
	nop2
	nop
    out     USBOUT, x1      ;1 <-- out
    rjmp    didStuff0       ;2 branch back 2 cycles earlier
bitstuff1:                  ;1 (for branch taken)
    eor     x1, x4          ;1
    rjmp    didStuff1       ;2 we know that C is clear, jump back to do OUT and ror 0 into x2
bitstuff2:                  ;1 (for branch taken)
    eor     x1, x4          ;1
    rjmp    didStuff2       ;2 jump back 4 cycles earlier and do out and ror 0 into x2
bitstuff3:                  ;1 (for branch taken)
    eor     x1, x4          ;1
    rjmp    didStuff3       ;2 jump back earlier and ror 0 into x2



usbSendAndReti:             ;0 [-13] timing: 13 cycles until SOP
    in      x2, USBDDR      ;1 [-12]
    ori     x2, USBMASK     ;1 [-11]
    sbi     USBOUT, USBMINUS;2 [-9] prepare idle state; D+ and D- must have been 0 (no pullups)
    in      x1, USBOUT      ;1 [-8] port mirror for tx loop
    out     USBDDR, x2      ;1 [-7] <- acquire bus
; need not init x2 (bitstuff history) because sync starts with 0
    ;push    r24              ;2 [-5] // push x4
    ldi     x4, USBMASK     ;1 [-4] exor mask
    ldi     shift, 0x80     ;1 [-3] sync byte is first byte sent
txLoop:                     ;       [62]
    sbrs    shift, 0        ;1 [-2] [62]
    eor     x1, x4          ;1 [-1] [63]
    out     USBOUT, x1      ;1 [0] <-- out bit 0
    ror     shift           ;1 [1]
    ror     x2              ;1 [2]
didStuff0:
    cpi     x2, 0xfc        ;1 [3]
    brsh    bitstuff0       ;1 [4]
    sbrs    shift, 0        ;1 [5]
    eor     x1, x4          ;1 [6]
    ror     shift           ;1 [7]
	nop2					;  [8-9]
	nop2					;  [10-11]
	;nop2 ; +1 na pocz¹tku   ;  [12-13]
	nop2
didStuff1:
    out     USBOUT, x1      ;1 [8] <-- out bit 1
    ror     x2              ;1 [9]
    cpi     x2, 0xfc        ;1 [10]
    brsh    bitstuff1       ;1 [11]
    sbrs    shift, 0        ;1 [12]
    eor     x1, x4          ;1 [13]
    ror     shift           ;1 [14]
	nop2
	nop2
	nop
didStuff2:
    ror     x2              ;1 [15]
    out     USBOUT, x1      ;1 [16] <-- out bit 2
    cpi     x2, 0xfc        ;1 [17]
    brsh    bitstuff2       ;1 [18]
    sbrs    shift, 0        ;1 [19]
    eor     x1, x4          ;1 [20]
    ror     shift           ;1 [21]
didStuff3:
    ror     x2              ;1 [22]
    cpi     x2, 0xfc        ;1 [23]
	nop2
	nop2
	nop
    out     USBOUT, x1      ;1 [24] <-- out bit 3
    brsh    bitstuff3       ;1 [25]
    nop2                    ;2 [27]
    ld      x3, y+          ;2 [29]
    sbrs    shift, 0        ;1 [30]
    eor     x1, x4          ;1 [31]
	USB_LOAD_PENDING(r24)
	USB_STORE_PENDING(r24)
	sbrc	r24,PS2_INTR_PENDING_BIT ; INT1 pending flag set ?
	lsl		r22
	nop
    out     USBOUT, x1      ;1 [32] <-- out bit 4
    ror     shift           ;1 [33]
    ror     x2              ;1 [34]
didStuff4:
    cpi     x2, 0xfc        ;1 [35]
    brsh    bitstuff4       ;1 [36]
    sbrs    shift, 0        ;1 [37]
    eor     x1, x4          ;1 [38]
    ror     shift           ;1 [39]
didStuff5:
	nop2 ; 
	nop ;
	nop2
    out     USBOUT, x1      ;1 [40] <-- out bit 5
    ror     x2              ;1 [41]
    cpi     x2, 0xfc        ;1 [42]
    brsh    bitstuff5       ;1 [43]
    sbrs    shift, 0        ;1 [44]
    eor     x1, x4          ;1 [45]
    ror     shift           ;1 [46]
didStuff6:
    ror     x2              ;1 [47]
	;nop2
	;nop2
	
	sbrc	r22,1		; check if we have first bit set -> first int1
	in		r23,PIND	; if yes cp 
	nop2
	sbrc	r22,1
	lsl		r22			; go to state = 4

    out     USBOUT, x1      ;1 [48] <-- out bit 6
    cpi     x2, 0xfc        ;1 [49]
    brsh    bitstuff6       ;1 [50]
    sbrs    shift, 0        ;1 [51]
    eor     x1, x4          ;1 [52]
    ror     shift           ;1 [53]
didStuff7:
    ror     x2              ;1 [54]
	;nop2
	;nop2
	;nop2 ; +1 na koñcu
	
	sbrs	r22,3		; check if we have second int1 state = 8 
	rjmp	normFlow
	sbic	PIND,5
	ori		r22,0x20	; set fifth bit if set in r22
	rjmp	normFlowR
////////////////////////////////////////////////////////////// BIT STUFF CODE START
bitstuff5:                  ;1 (for branch taken)
    eor     x1, x4          ;1
    rjmp    didStuff5       ;2 same trick as in bitstuff1...

bitstuff4:                  ;1 (for branch taken)
    eor     x1, x4          ;1
    ldi     x2, 0           ;1
	nop2
	nop2
	nop
    out     USBOUT, x1      ;1 <-- out
    rjmp    didStuff4       ;2 jump back 2 cycles earlier

bitstuff6:                  ;1 (for branch taken)
    eor     x1, x4          ;1
    rjmp    didStuff6       ;2 same trick as above...
////////////////////////////////////////////////////////////////BIT STUFF CODE END
normFlow:
	nop2
	nop
normFlowR:
	cpi     x2, 0xfc        ;1 [55]
    out     USBOUT, x1      ;1 [56] <-- out bit 7
    brsh    bitstuff7       ;1 [57]
    mov     shift, x3       ;1 [58]
    dec     cnt             ;1 [59]
    brne    gateToByteLoop  ;1/2 [60/61]
							
;make SE0:
    cbr     x1, USBMASK     ;1 [61] prepare SE0 [spec says EOP may be 15 to 18 cycles]
    ;pop     r24              ;2 [63] // pop x4
	nop2					; zamiast pop r24
	sbrc	r22,3
	lsl		r22				; set state = 16 - end of second int 1
	;nop2					
	;nop						; 7 nopów do wyko¿ystania ale 3 wziête na sprawdzanie klaw
	ldi		r24,2
	sbrs	r22,0
	out		EIMSK,r24			; check if we have something to handle else skip masking int0
;brackets are cycles from start of SE0 now
    out     USBOUT, x1      ;1 [0] <-- out SE0 -- from now 2 bits = 16 cycles until bus idle
;2006-03-06: moved transfer of new address to usbDeviceAddr from C-Code to asm:
;set address only after data packet was sent, not after handshake
    lds     x2, usbNewDeviceAddr;2 [2]
    lsl     x2;             ;1 [3] we compare with left shifted address
    subi    YL, 20 + 2      ;1 [4] Only assign address on data packets, not ACK/NAK in x3
    sbci    YH, 0           ;1 [5]
    breq    skipAddrAssign  ;2 [7]
    sts     usbDeviceAddr, x2; if not skipped: SE0 is one cycle longer
skipAddrAssign:
;end of usbDeviceAddress transfer
    ldi     x2, 1;;;1 [8] int0 occurred during TX -- clear pending flag // clear all flags
    USB_STORE_PENDING(x2)   ;1 [9]
    ori     x1, USBIDLE     ;1 [10]
    in      x2, USBDDR      ;1 [11]
    cbr     x2, USBMASK     ;1 [12] set both pins to input
    mov     x3, x1          ;1 [13]
    cbr     x3, USBMASK     ;1 [14] configure no pullup on both pins
	;nop2
	;nop2
	;nop
	ldi		r24,1			; prepare value for port c to trigger pcint1
	; 5 do wykorzystania
	sbrs	r22,0
	sts		requestBit,r23
	sbrs	r22,0
	sts		requestBitPrev,r22
	sbrs	r22,0			; bit seven set ? 
;	out		PORTC,r24		; trigger pcint1
	sbi		PORTC,0			; zmiana w zwi¹zku z diod¹
	;nop ; zamiast outa
	nop
    out     USBOUT, x1      ;1 [15] <-- out J (idle) -- end of SE0 (EOP signal)
    out     USBDDR, x2      ;1 [16] <-- release bus now
    out     USBOUT, x3      ;1 [17] <-- ensure no pull-up resistors are active
	USB_STORE_PENDING(r24)
    rjmp    doReturn



gateToByteLoop:
	sbrc	r22,3
	lsl		r22			; set state = 16 - end of second int 1
	nop
	rjmp txLoop



bitstuff7:                  ;1 (for branch taken)
    eor     x1, x4          ;1
    rjmp    didStuff7       ;2 same trick as above...

doDelayunstuff0:
	nop
	rjmp	didunstuff0

doDelayunstuff1:
	nop
	rjmp	didunstuff1

doDelayunstuff2:
	nop
	rjmp	didunstuff2

doDelayunstuff3a:
	nop
	rjmp	didunstuff3a



doDelayrxbit2:
	nop
	rjmp	didrxbit2

doDelayN1:
	nop
	rjmp	didN1

doDelayN2:
	nop
	rjmp	didN2

doDelayN4:
	nop
	rjmp	didN4

doDelayN5:
	nop
	rjmp	didN5

