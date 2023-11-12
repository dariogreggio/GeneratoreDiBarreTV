;*************************************************************************
;*
;*   Emulating Data EEPROM for PIC24 Microcontrollers and
;*           dsPIC Digital Signal Controllers
;*
;*************************************************************************
;* FileName:     FlashOperations_DoubleWord.s
;* Compiler:     MPLAB XC16, v1.30 or higher
;* Company:      Microchip Technology, Inc.
;*
;* Software License Agreement
;*
;* Copyright © 2016 Microchip Technology Inc. All rights reserved.
;*
;* Microchip licenses to you the right to use, modify, copy and distribute
;* Software only when embedded on a Microchip microcontroller or digital
;* signal controller, which is integrated into your product or third party
;* product (pursuant to the sublicense terms in the accompanying license
;* agreement).
;*
;* You should refer to the license agreement accompanying this Software for
;* additional information regarding your rights and obligations.
;*
;* Author        Date        Comment
;*************************************************************************
;* P. Sinha      2011/01/20  Initial Release
;* Anantha R	 2011/11/30  Modified to ensure proper packing
;* P. Sinha      2016/12/20  Renamed to reflect support for new PIC24F devices
;  GD/C			2020/07/21	Little changes for dsPIC33E
;  GD/C			2021/08/09	more changes for dsPIC33E
;************************************************************************/

.ifdef __PIC24E
.include "p24Exxxx.inc"    
.endif
.ifdef __DSPIC33E
.include "p33Exxxx.inc"    
.endif
    
.global _ReadPMHigh
.global _ReadPMLow
.global _UnlockPM
.global _WritePMHigh
.global _WritePMLow

.section .text

_ReadPMHigh:
	tblrdh [W0],W0
	return

_ReadPMLow:
	tblrdl [W0],W0
	return		
	
_UnlockPM:
	push	W0
	disi	#5
	mov		#0x55,W0
	mov		W0, NVMKEY
	mov		#0xAA, W0
	mov		W0, NVMKEY
	bset	NVMCON, #15
	nop
	nop
	btsc	NVMCON, #15
	bra		$-2
	pop		W0
	return	

.ifdef ERASE
; Normal handling for devices with single-word writes
	
    _WritePMHigh:
	tblwth	W0,[W1]
	return

    _WritePMLow:
	tblwtl	W0,[W1]
	return
    
.else	
; Special handling for devices with double-word writes, while retaining same API
	
    _WritePMHigh:
	; Store current TBLPAG value and passed target address in NVMADRU and NVMADR respectively
	mov		TBLPAG, W2
	mov		W2, NVMADRU
	mov		W1, NVMADR
	; Set up latch pointer depending on whether required operation is Word Program or not
	mov		#0x4001, W4
	mov		NVMCON, W3
	cp		W3, W4
	bra		Z, ahead1
	and		#0xFF, W1
	bra		ahead2
    ahead1:
	; If the address to be written is an even word, then keep a copy of the corresponding odd word too
	; If the address to be written is an odd word, then keep a copy of the corresponding even word too
	mov		W1, W4
	btg		W4, #1 
	tblrdh	[W4],W5
; GD 10/8/21	, nel simulatore va con 0xff ma nell'hardware dspic33E no...
.ifdef __MPLAB_DEBUGGER_SIMULATOR
	and		#0xFF, W1
.else
	and		#0x03, W1
.endif
	; Save the current TBLPAG value and set up TBLPAG to point to the latch area
	push	TBLPAG
	mov		#0xFA, W6
	mov		W6, TBLPAG
	; Perform the required latch write
	tblwth	W0,[W1]
	; If the address to be written is an even word, then write the stored value of the odd word
	; If the address to be written is an odd word, then write the stored value of the even word
	btg		W1, #1 
	tblwth	W5,[W1]
	bra		ahead3
    ahead2:
	; Save the current TBLPAG value and set up TBLPAG to point to the latch area
	push	TBLPAG
	mov		#0xFA, W6
	mov		W6, TBLPAG
	tblwth	W0,[W1]
    ahead3:
	; Restore the original TBLPAG value
	pop		TBLPAG
	return

    _WritePMLow:
	; Store current TBLPAG value and passed target address in NVMADRU and NVMADR respectively
	mov		TBLPAG, W2
	mov		W2, NVMADRU
	mov		W1, NVMADR
	; Set up latch pointer depending on whether required operation is Word Program or not
	mov		#0x4001, W4
	mov		NVMCON, W3
	cp		W3, W4
	bra		Z, ahead4
	and		#0xFF, W1
	bra		ahead5
    ahead4:
	; If the address to be written is an even word, then keep a copy of the corresponding odd word too
	; If the address to be written is an odd word, then keep a copy of the corresponding even word too
	mov		W1, W4
	btg		W4, #1 
	tblrdl	[W4],W5
; GD 10/8/21	, nel simulatore va con 0xff ma nell'hardware dspic33E no...
.ifdef __MPLAB_DEBUGGER_SIMULATOR
	and		#0xFF, W1
.else
	and		#0x03, W1
.endif
	; Save the current TBLPAG value and set up TBLPAG to point to the latch area
	push	TBLPAG
	mov		#0xFA, W6
	mov		W6, TBLPAG
	; Perform the required latch write
	tblwtl	W0,[W1]
	; If the address to be written is an even word, then write the stored value of the odd word
	; If the address to be written is an odd word, then write the stored value of the even word
	btg		W1, #1 
	tblwtl	W5,[W1]
	bra		ahead6
    ahead5:
	; Save the current TBLPAG value and set up TBLPAG to point to the latch area
	push	TBLPAG
	mov		#0xFA, W6
	mov		W6, TBLPAG
	tblwtl	W0,[W1]
    ahead6:
	; Restore the original TBLPAG value
	pop		TBLPAG
	return

.endif
	
.end
