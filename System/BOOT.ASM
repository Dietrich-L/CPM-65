;BOOT.ASM
;(c) by D. Lausberg

;Bootstrap Lader fuer den CP/M-Bootsector

;V1.0	30.10.91
;V1.1	10.11.91	the real thing
;V1.2	22.05.92	Bootprom version
;V1.3	20.02.95	new memory map
;V1.4	31.12.95	SCSI support
;V1.5	08.10.00	corrrected memory map for sector BIOS
;V1.6	27.04.21	4 Floppies + SCSI
;V1.7	12.07.21	3k BIOS

;Parameter

TRACK	= $0
SECTOR	= $1
PNT	= $2
MAXSEC	= $3


DMA	= $FE

TPA	= $200

;6532 ADDRESSES

SCRATCH	= $F600
DRIVE	= SCRATCH+$6B

;CONSTANTS

CR	= $D
LF	= $A
EOT	= $0

;ROM-BIOS ROUTINES

ROM_BOOT = $F800
SETSEC	= $F821		;BIOS ROUTINES OF MONITOR EPROM
SETTRK	= $F81E
READ	= $F824
PRTCHR	= $F80C

MONITOR	= $F830

;--------------------------------------------

	ORG TPA

	JMP BOOT

;---  CP/M-65 MEMORY MAP  -------------------

MAP	DB 0			;this Bootloader
CCP	DB $D8,$D9,$DA,$DB	;CCP
BDOS	DB $DC,$DD,$DE,$DF	;BDOS
	DB $E0,$E1,$E2,$E3
BIOS	DB $E4,$E5,$E6,$E7	;BIOS
	DB $E8,$E9,$EA,$EB
	DB $EC,$ED,$EE,$EF
	DB 0

;---  MESSAGES  -----------------------------

BOOTM	DB CR,LF,'CP/M-65 Bootstrap Loader V1.7',CR,LF,EOT
ERRM	DB CR,LF,'BOOT failed',CR,LF,EOT

;--------------------------------------------

BOOT	LDA #0
	STA TRACK
	STA SECTOR
	STA PNT
	STA DMA
	LDA DRIVE		;set MAXSEC
	LDX #16
	CMP #4
	BCC BOOT7
	LDX #64
BOOT7	STX MAXSEC

BOOT2	INC PNT
	INC SECTOR
	LDA SECTOR
	CMP MAXSEC
	BCC BOOT1
	INC TRACK
	LDA #0
	STA SECTOR
BOOT1	JSR SETSEC
	LDA TRACK
	JSR SETTRK
	LDY PNT			;SET DMA PAGE
	LDA MAP,Y
	BEQ BOOT3
	STA DMA+1
	JSR READ
	BCC BOOT2
	LDY #0
BOOT4	LDA ERRM,Y
	BEQ BOOT_X
	JSR PRTCHR
	INY
	BNE BOOT4

BOOT_X	JSR MONITOR
	JMP ROM_BOOT


BOOT3	LDA BIOS
	STA DMA+1
	LDY #0
BOOT5	LDA BOOTM,Y
	BEQ BOOT6
	JSR PRTCHR
	INY
	BNE BOOT5
BOOT6	JMP (DMA)		;COLD BOOT


	END
