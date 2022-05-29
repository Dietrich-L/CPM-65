;STEP.ASM
;(c) by D. Lausberg

;set steprate for Floppies

;V1.0	01.05.21	Version A: 6ms B: 12ms C: 6ms D: 6ms

;Parameter

BDOS	= $F0
DMA	= $FE

TPA	= $200

;BDOS COMMANDS

CONIN	= $1
CONOUT	= $2
STROUT	= $9

;CONSTANTS

CR	= $D
LF	= $A
EOT	= $0

6MS	= $00
12MS	= $01

;6532 ADDRESSES

SCRATCH	= $F600

STPRATE	= SCRATCH+$54		;Steprate $00: 6ms, $01: 12 ms

;--------------------------------------------

	ORG TPA

	JMP START

Table	DB 6ms, 12ms, 6ms, 6ms

START	LDA #STARTM		;DISPLAY STARTUP MESSAGE
	LDY #STARTM/256
	JSR PRTSTR
	LDY #3
start1	lda Table,y
	sta stprate,y
	dey
	bpl start1
	rts

PRTSTR  LDX #STROUT
	JMP BDOS

STARTM	DB CR,LF,CR,LF,'STEP V1.0                '
	DB '  (c) D.Lausberg   2021',CR,LF
	DB CR,LF,'setting steprate to A: 6ms  B: 12 ms  C: 6ms  D: 6ms'
	DB CR,LF,CR,LF,EOT

	END
