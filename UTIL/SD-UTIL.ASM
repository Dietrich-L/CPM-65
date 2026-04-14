;FAT-UTIL.COM -- CP/M-65 SD UTILITY JC Version
;BY D. LAUSBERG	 (C)1988
;V1.0	18.07.23
;V1.1	26.07.23	error correction in SETLBA
;V1.2	16.01.24	fast SPI version in 4 us/ out 14 us clk T2 control
;V1.3	19.12.25	SD utility commands added

VERSION	= $13

;SYSTEM CONSTANTS
DMA	= $FE
DPH	= $F8
FCB1	= $F6
FCB2	= $F4
BDOS	= $F0
INBUF	= $EC
CCPV	= $E0
TPA	= $0200
BUFBEG	= $8000
BIOSV	= $E400		;BIOS start adress

;Zero Page adresses

CMDVEC	= $00
CMDKEY	= $01
PNT	= $02
CNT	= $03
MCNT	= $04
RCOUNT	= $05
BUFFER	= $08
NUMBER	= $09
HEXL	= $0A
HEXH	= $0B
HEXX	= $0C
HEXY	= $0D
DIVL	= $0E
DIVM	= $0F
DIVH	= $10

DRIVE	= $20
ERRNO	= $21
SECS	= $22		;sector # on SD

SYS	= $28
DIR_NP	= $2C		;pointer to nr of dir entry
DIR_FNP	= $2E		;dir filename pointer
IMAGE	= $30		;image to be mounted - Drive, LBA
CNT_I	= $35		;# images found
CNT_X	= $36		;counter
PNT_Y	= $37		;pointer

SD_CMD	= $81	;CMD Byte
SD_PB3  = $82   ;Parameter Byte 3
SD_PB2	= $83   ;Parameter Byte 2
SD_PB1  = $84   ;Parameter Byte 1
SD_PB0	= $85   ;Parameter Byte 0
SD_CRC	= $86   ;CRC Byte
PLBA	= $87	;LBA Pointer

XBUF	= $8A
YBUF	= $8B
TEMP	= $8C   ;temp storage
BLKBUF	= $92

;--- VIA 6522 adresses
;SPI driver  Mode 0 uses VIA DRB
;BIT 1	CS
;Bit 0	MOSI
; CB 1	CLK
; CB 2	MISO

DRB	= $F400
DDRB	= $F402
T2CL	= $F408
SR	= $F40A
ACR	= $F40B
IFR	= $F40D
IER	= $F40E

;--- SD Command Constants 

CMD0	= $40		; Reset SD-Card
CMD1	= CMD0 + 1	; Initialize MMC/SDC
CMD8	= CMD0 + 8	; Check voltage range
CMD13	= CMD0 + 13	;
CMD16	= CMD0 + 16	; Change block size
CMD17	= CMD0 + 17	; Read single block
CMD24	= CMD0 + 24	; Write single block
CMD55	= CMD0 + 55	; Application command prefix
CMD58	= CMD0 + 58	; Get OCR
ACMD41	= CMD0 + 41	; Initialize SDC

DATA_TOKEN	= $FE	; SD-Card Data Token Read Block

;--- constants ---

SYSSCS	= 32		;SYSTEM sectors CPM-65
DIRSCS	= 32		;directory sectors CPM-65

MAX_IMG	= 64		;max # of images on SD
SD1ST	= 8		;I: is 1. drive on SD
SDMAX	= 12		;drives I..L
SEC_BIOS0 = $0D		;BIOS 1. sector

;BDOS COMMANDS

BOOT	= $00
CONIN	= $01
CONOUT	= $02
STROUT	= $09
GETLIN	= $0A
BIOS	= $1A

_SELDSK	= $09
_SETSEC = $0B
_WRITE	= $0D
_BIOSVER = $0F
_MOUNT	= $20

;CONTROL CODES

EOT	= $00
BS	= $08
LF	= $0A
CLS	= $0C
CR	= $0D
EOF	= $1A
SPACE	= $20
SEMI	= $3B

;ERROR CODES

NOCOM	= $80
MISPAR	= $81
ILLPAR	= $82
TOOMCH	= $83
EOL	= $84
INVDRV	= $85
WRT_PROT = $86
SD_CHNG	= $87
NO_SD	= $88
NO_PART	= $89
TM_IMG	= $8A		;too many image files
NO_IMG	= $8B		;no image available
IMG_NOT_FOUND = $8C	;image not found in mounted drives list
BIOS_VER = $8D		;BIOS version not V3.3 or higher

	ORG TPA

DUTIL	JSR INIT 		;INIT VARIABLES

DUTILL	JSR STATUS
	LDA #PROMPT
	LDY #PROMPT/256
	JSR PRTSTR
	LDX #GETLIN
	JSR BDOS
	LDY #0			;SET COMMAND POINTER
	STY CMDVEC
DUTIL1	JSR DECODE		;DECODE AND EXEC COMMAND
	BCC DUTIL1
	CMP #EOL 		;END OF COMMAND
	BEQ DUTILL
	JSR ERROR
	JMP DUTILL


INIT	LDA #0			;RESET VARIABLES
	LDX #$40
INIT2	STA 0,X
	DEX
	BPL INIT2
	JSR CHK_BIOSVER		;BIOS V3.3 or higher
	BCS INIT1
	LDA #BUFBEG		;SET BUFFER VECTOR
	STA BUFFER
	LDA #BUFBEG/256
	STA BUFFER+1
	LDA #DMABUF		;set DMA
	STA DMA
	LDA #DMABUF/256
	STA DMA+1
	JSR SD_INIT		;initialize SD card
	BCC INIT1
	JSR DRVPAR		;GET DRIVE PARAM'S
	JSR READS		;read sector 0
	RTS

INIT1	JSR ERROR
	BCC INIT
	LDX #BOOT
	JMP BDOS


CHK_BIOSVER			;check for BIOS >= V3.3
	LDY #_BIOSVER
	JSR BIOSC
	CMP #$33		;is BIOS >= V3.3?
	BCS BVER1
	LDA #BIOS_VER
	SEC
	RTS

BVER1	CLC
	RTS


DRVPAR	LDA #0			;RESET VARIABLES
	STA SYS
	STA SYS+1
	STA SYS+2
	LDA #SD1ST
	STA DRIVE		;always first SD image mounted
	RTS


STATUS	LDA #STATM1		;DISPLAY STATUS
	LDY #STATM1/256
	JSR PRTSTR
	LDA DRIVE		;PRINT DRIVE
	CLC
	ADC #'A
	JSR PRTCHR
	LDA #STATM2
	LDY #STATM2/256
	JSR PRTSTR
	LDY #3
STATUS1	LDA SECS,Y
	JSR PRTHEX		;PRINT Sector #
	DEY
	BPL STATUS1
	LDA #STATM3
	LDY #STATM3/256
	JSR PRTSTR
	LDY #3
STATUS2	LDA LBA,Y
	JSR PRTHEX		;PRINT LBA #
	DEY
	BPL STATUS2
	RTS


PRTDEC	JSR COMNUM		;PRINTS DECIMAL
	PHA
	TYA
	JSR COMNUM
	PHA
	TYA
	JSR PRTNUM
	PLA
	JSR PRTNUM
	PLA
	JSR PRTNUM
	RTS


COMNUM	LDY #0			;DIV/10
COMNU1	SEC
	SBC #10
	BCC COMNU2
	INY
	BNE COMNU1

COMNU2	ADC #10
	RTS


PRTNUM	ORA #$30 		;PRINT A NUMBER
	JMP PRTCHR


DECODE	JSR GNCHR		;GET NEXT CHR
	BCS DECODX
	CMP #SEMI		;IS IT A SEPARATOR
	BNE DECOD1
	JSR GNCHR
	BCS DECODX
DECOD1	LDY #0			;SEARCH COMMAND
	STA CMDKEY
DECOD4	LDA CMDTAB,Y
	BEQ DECOD2		;COMMAND NOT FOUND
	CMP CMDKEY
	BEQ DECOD3		;FOUND
	INY
	INY
	INY
	BNE DECOD4

DECOD3	INY
	LDA CMDTAB,Y
	PHA
	INY
	LDA CMDTAB,Y
	PHA
	RTS			;EXEC

DECOD2	LDA #NOCOM		;UNKNOWN COMMAND
	SEC
DECODX	RTS

CMDTAB	DB 'S'
	DD SECRD-1
	DB 'L'
	DD LBARD-1
	DB 'D'
	DD DUMP-1
	DB 'C'
	DD CHANGE-1
	DB '+'
	DD PLUS-1
	DB '-'
	DD MINUS-1
	DB 'R'
	DD READS-1
	DB 'W'
	DD WRITES-1
	DB 'I'
	DD INFO-1
	DB 'M'
	DD MOUNT-1
	DB 'P'
	DD PUSH-1
	DB 'X'
	DD EXIT-1
	DB '?'
	DD HELP-1
	DB 'Z'
	DD WAIT-1
	DB '>'
	DD STOBUF-1
	DB '<'
	DD STODMA-1
	DB '/'
	DD REPEAT-1
	DB 'B'
	DD SETBUF-1

	DB $00

;==========  COMMANDS  ============= 

SECRD	JSR GETSEC		;GET Sector
	BCS SECRDX
	JSR READS		;READ SECTOR
SECRDX	RTS


LBARD	JSR GETSEC		;GET LBA
	BCS LBARDX
	ASL SECS		;SEC*2
	ROL SECS+1
	ROL SECS+2
	ROL SECS+3
	BCS LBARD1
	JSR READS		;READ SECTOR
LBARDX	RTS

LBARD1	LDA #ILLPAR		;LBA > 2 TB
	SEC
	RTS


DUMP	JSR STATUS		;DUMP DMA
	LDA #0
DUMP3	STA PNT			;SET POINTER
	JSR CRLF
	LDY PNT
DUMP1	LDA (DMA),Y
	JSR DISHEX
	INY
	TYA
	AND #$0F
	PHA
	CMP #8
	BNE DUMP4
	LDA #BS
	JSR PRTCHR
DUMP4	PLA
	BNE DUMP1
	LDY PNT
DUMP2	LDA (DMA),Y
	JSR DISASC
	INY
	TYA
	AND #$0F
	BNE DUMP2
	TYA
	BNE DUMP3
	CLC
	RTS


CHANGE	JSR GNCHR		;GET NEXT CHR
	BCS CHANGX
	CMP #'H
	BEQ CHEX
	CMP #'A
	BEQ CASC
	LDA #ILLPAR		;ILLEGAL PARAMETER
CHANGX	SEC
	RTS

CHEX	JSR GETHEX		;GET DMA POINTER
	BCS CHANGX
	STA PNT
CHEX1	JSR GETHEX		;GET HEX VALUE
	BCS CHEX2
	LDY PNT
	STA (DMA),Y		;REPLACE
	INC PNT
	BNE CHEX1
	LDA #TOOMCH		;TOO MUCH PARAMETERS
	SEC
	RTS

CHEX2	CMP #MISPAR
	BNE CHANGX
	CLC
	RTS

CASC	JSR GETHEX		;GET DMA POINTER
	BCS CHANGX
	STA PNT
CASC1	JSR GNCHR		;GET HEX VALUE
	BCS CHANGX
	LDY PNT
	STA (DMA),Y		;REPLACE
	INC PNT
	BNE CASC1
	LDA #TOOMCH		;TOO MUCH PARAMETERS
	SEC
	RTS


PLUS	LDX #0
PLUS2	INC SECS,X
	BNE PLUS1
	INX
	CPX #4
	BCC PLUS2
PLUS1	JSR READS
	RTS


MINUS	LDX #0
MINUS2	DEC SECS,X
	LDA SECS,X
	CMP #$FF
	BNE MINUS1
	INX
	CPX #4
	BCC MINUS2
MINUS1	JSR READS
	RTS


READS	JSR READ		;READ sector in SEC
	RTS


WRITES	JSR WRITE		;WRITE sector in SEC
	RTS


INFO	JSR SET_DIR_INFO	;set directory info parameters
	BCS INFOX
	JSR SET_IMG		;set loaded image table
	JSR DISP_INFO		;display files in directory
	BCC INFOX
	CMP #EOF
	BNE INFO1
	CLC
INFOX	RTS

INFO1	SEC
	RTS


MOUNT	JSR GNCHR		;GET drive
	JSR CHKDRV
	BCS MOUNTX
	STA IMAGE		;save drive
	LDA CNT_I
	BEQ MOUNT4
MOUNT3	JSR GNCHR		;search ',
	BCS MOUNT5
	CMP #',
	BNE MOUNT3
MOUNT1	JSR GETHEX		;GET image nr
	BCS MOUNTX
	CMP CNT_I
	BCS MOUNT4
	ASL A			;image index * 4
	ASL A
	TAY
	LDX #0
MOUNT2	LDA IMG_LBA,Y		;set image data
	STA IMAGE+1,X
	INY
	INX
	CPX #4
	BCC MOUNT2
MOUNTI	LDA #IMAGE
	LDY #_MOUNT		;mount drive
	JSR BIOSC
MOUNTX	RTS

MOUNT4	LDA #NO_IMG
	SEC
	RTS

MOUNT5	LDA IMAGE
	ORA #$80		;set delete bit
	STA IMAGE
	BNE MOUNTI


PUSH	LDA #0
	STA SECS+1		;reset sector nr
	STA SECS+2
	LDA #SEC_BIOS0		;calculate starting SECStor/track
	STA SECS
	LDA DMA			;save DMA
	PHA
	LDA DMA+1
	PHA
	LDA #BIOSV		;SET DMA TO BIOS
	STA DMA
	LDA #BIOSV/256
	STA DMA+1
	LDA #2			;push mod. BIOS to C:
	LDY #_SELDSK
	JSR BIOSC
	BCS PUSHX
	LDA #SECS
	LDY #_SETSEC
	JSR BIOSC
	LDY #_WRITE		;WRITE SECTOR
	JSR BIOSC
PUSHX	TAX
	PLA			;restore DMA
	STA DMA+1
	PLA
	STA DMA
	TXA
	RTS


EXIT	LDX #BOOT
	JSR BDOS


HELP	LDA #HELPM
	LDY #HELPM/256
	JSR PRTSTR
	RTS


WAIT	JSR GDEC 		;WAIT NN/10 S
	BCS WAITX
	STA CNT
WAIT3	LDA #10
	STA MCNT 		;100 MS
WAIT2	LDX #78
WAIT1	DEY
	BNE WAIT1
	DEX
	BNE WAIT1
	DEC MCNT
	BNE WAIT2
	DEC CNT
	BNE WAIT3
	CLC
WAITX	RTS


STOBUF	LDY #0			;SAVE DMA TO BUFFER
STOBU1	LDA (DMA),Y
	STA (BUFFER),Y
	INY
	BNE STOBU1
	CLC
	RTS


STODMA	LDY #0			;STORE BUFFER TO DMA
STODM1	LDA (BUFFER),Y
	STA (DMA),Y
	INY
	BNE STODM1
	CLC
	RTS


REPEAT	LDA RCOUNT		;REPEAT COMMAND
	BEQ REPEA1
REPEA3	DEC RCOUNT		;ONE MORE TIME
	BEQ REPEA5
	LDA #0
	STA CMDVEC
	CLC
	RTS

REPEA1	JSR GDEC 		;GET NO. OF REPEATS
	BCS REPEA2
	STA RCOUNT
	CMP #0
	BNE REPEA3
	LDA #ILLPAR
REPEA4	SEC
	RTS

REPEA2	CMP #MISPAR
	BNE REPEA4
	LDA #10			;DEFAULT 10 TIMES
	STA RCOUNT
	BNE REPEA3

REPEA5	LDA #EOL
	SEC
	RTS


SETBUF	JSR GETHEX		;GET BUFFER ADDRESS
	BCS SETBUX
	STY BUFFER+1
	STA BUFFER
SETBUX	RTS

;======  SUBROUTINES  ============== 

SET_DIR_INFO			;set dir params
	LDA #0
	LDY #3			;reset LBA, BASE, N32
SDIR1	STA LBA,Y
	STA BASE,Y
	STA N32,Y
	DEY
	BPL SDIR1
	JSR S_READ1		;read sector 0 to BLKB
	BCS SDIRX
	LDA BLKB+$1BE+4		;OS-Type
	BEQ SDIR9
	LDY #3
SDIR2	LDA BLKB+$1BE+8,Y
	STA BASE,Y		;BASE = begin of partition
	STA LBA,Y		;LBA = begin of partition
	DEY
	BPL SDIR2
	JSR S_READ1		;read sector 0 of partition 1
	BCS SDIRX
	LDA BLKB+$11		;save # dir entries
	STA NDIR
	LDA BLKB+$12
	STA NDIR+1
	LDA BLKB+$0D		;save # sectors/cluster
	STA SEC_CLUS
	LDA BLKB+$0E		;N32 = #reserved sectors
	STA N32
	LDA BLKB+$0F
	STA N32+1
	JSR ADD			;BASE = BASE + N32
	LDA BLKB+$10
	STA NFAT		;#FATs
	LDA BLKB+$16		;N32 = FAT size
	STA N32
	LDA BLKB+$17
	STA N32+1
	ORA N32			;N32 = 0? -> FAT32
	BEQ SDIR4
SDIR3	JSR ADD
	DEC NFAT
	BNE SDIR3
	JSR DIRB2SEC		;set sec = BASE*2
	JSR READS		;READ SECTOR
SDIRX	RTS

SDIR4	LDA BLKB+$24		;N32 = FAT32 size
	STA N32
	LDA BLKB+$25
	STA N32+1
	LDA BLKB+$26
	STA N32+2
	LDA BLKB+$27
	STA N32+3
	JMP SDIR3

SDIR9	LDA #NO_PART
	SEC
	RTS


DISP_INFO			;display directory of SD
	JSR SET_PARAMS		;set DIR_NP, CNT_I, BASE
DISINF1	JSR DIS_FN		;display filename indexed by DIR_NP if valid image
				;and store its LBA in FNAMES 
	JSR NXT_FN		;setup for next filename
	BCC DISINF1
	RTS


DIS_FN	LDA DMA			;DIR_FNP = DMA + PNT
	STA DIR_FNP
	LDA DMA+1
	STA DIR_FNP+1
	LDA DIR_NP		;PNT = DIR_NP*32 mod 256
	ASL A
	ASL A
	ASL A
	ASL A
	ASL A
	CLC
	ADC DIR_FNP
	STA DIR_FNP
	BCC DISFN1
	INC DIR_FNP+1
DISFN1	LDY #0
	LDA (DIR_FNP),Y		;check special byte
	CMP #$E5		;deleted?
	BEQ DISFNX
	LDY #$0B		;check attribute
	LDA (DIR_FNP),Y
	CMP #$20
	BNE DISFNX
	JSR CHK_FS		;check filesize
	BCS DISFNX
	JSR CRLF
	LDA CNT_I
	JSR PRTHEX		;print nr
	LDA #':
	JSR PRTCHR
	JSR PRT_FN
	JSR STO_LBA		;store LBA of 1. sector of the image
	LDA #STATM2
	LDY #STATM2/256
	JSR PRTSTR		;print ':  Sector '
	LDA CNT_I
	ASL A
	ASL A
	PHA			;save index
	TAY
	LDX #3
	CLC
DISFN3	LDA IMG_LBA,Y		;IMG_LBA * 2
	ROL A			;LSB..MSB
	PHA
	INY
	DEX
	BNE DISFN3
	LDY #3
DISFN4	PLA
	JSR PRTHEX		;MSB..LSB
	DEY
	BNE DISFN4
	PLA			;A is pointer to IMG_LBA
	JSR TST_DRV		;check if image is already mounted
	BCS DISFN5
	PHA			;save drive #
	LDA #SPACE		;print 2 spaces
	JSR PRTCHR
	LDA #SPACE
	JSR PRTCHR
	PLA
	ADC #SD1ST+'A		;convert to drive letter
	JSR PRTCHR
	LDA #':			;print X:
	JSR PRTCHR
DISFN5	INC CNT_I
	LDA CNT_I
	CMP #MAX_IMG
	BCS DISFN2
DISFNX	RTS

DISFN2	LDA #TM_IMG
	RTS


STO_LBA	LDY #$1A		;N32 = 1. cluster of image (LSB, LSB+1)
	LDA (DIR_FNP),Y
	STA N32
	INY	
	LDA (DIR_FNP),Y
	STA N32+1
	LDY #$14		;N32 = 1. cluster of image (LSB+2, LSB+3)
	LDA (DIR_FNP),Y
	STA N32+2
	INY	
	LDA (DIR_FNP),Y
	STA N32+3
	JSR MUL_SPC		;N32 = N32 * SEC_CLUS
	LDA CNT_I		;store LBA of 1. sector of the image
	ASL A
	ASL A
	TAY
	LDX #0
	LDA #4
	STA MCNT
	CLC
STOLBA1	LDA BASE,X
	ADC N32,X
	STA IMG_LBA,Y
	INY
	INX
	DEC MCNT
	BNE STOLBA1
	RTS


NXT_FN	INC DIR_NP		;setup for next filename
	BNE NXT_FN1		;DIR_NP = DIR_NP + 1
	INC DIR_NP+1
NXT_FN1	LDA DIR_NP+1
	CMP NDIR+1
	BNE NXT_FN4
	LDA DIR_NP
	CMP NDIR
	BCS NXT_FN3
NXT_FN4	LDA DIR_NP		;check for next sector
	AND #%00000111
	BNE NXT_FN2
	JSR PLUS	
	BCS NXT_FNX
NXT_FN2	CLC
	RTS	

NXT_FN3	LDA #EOF		;end of directory
NXT_FNX	RTS


SET_PARAMS			;set DIR_NP, CNT_I, BASE
	LDA #0			;DIR_NP=0
	STA DIR_NP
	STA DIR_NP+1
	STA N32+1
	STA CNT_I		;CNT_I = 0
	LDA NDIR+1		;BASE = BASE + NDIR*32/512
	STA N32			;N32 = NDIR/256
	LDY #4			;N32 = N32*16
SET_P1	JSR MUL2
	DEY
	BNE SET_P1
	JSR ADD
	LDY #3			;BASE = BASE - 2*SEC_CLUS
SET_P2	LDA MINUS_2,Y
	STA N32,Y
	DEY
	BPL SET_P2
	JSR MUL_SPC		;N32 * SEC_CLUS
	JSR ADD
	LDA #128		;max. 128 DIR entries
	STA NDIR
	LDA #0
	STA NDIR+1
	RTS


SET_IMG	LDA #SD1ST		;set loaded image table
SETIMG1	PHA
	ORA #%01000000		;get image data
	STA IMAGE
	JSR MOUNTI		;call BIOS MOUNT
	BCS SETIMG2
	PLA
	PHA
	SEC
	SBC #SD1ST
	TAX
	LDA #$FF		;set IMD_STAT,X valid
	STA IMGSTAT,X
	TXA
	ASL A			;*4
	ASL A
	TAY
	LDX #1			;set image table of mounted drives
SETIMG3	LDA IMAGE,X
	STA IMGTAB,Y
	INY
	INX
	CPX #5			;last byte?
	BCC SETIMG3
SETIMG4	PLA
	ADC #0			;+1
	CMP #SDMAX		;last drive reached?
	BCC SETIMG1
	CLC			;finished
	RTS

SETIMG2	PLA			;no image mounted
	PHA
	SEC
	SBC #SD1ST
	TAX
	LDA #0			;set table entry invalid
	STA IMGSTAT,X
	BEQ SETIMG4		;jump always


TST_DRV	LDX #0			;test for image assign to drive
	STX MCNT		;drive counter
	STX CNT_X		;IMG_TAB index
	STA PNT_Y		;save IMG_LBA pointer
TSTDRV2	LDX MCNT
	LDA IMGSTAT,X
	BEQ TSTDRV1
	LDX CNT_X
	LDY PNT_Y		;load IMG_LBA pointer
TSTDRV3	LDA IMG_LBA,Y		;get LBA image byte
	CMP IMGTAB,X		;compare with mounted LBA byte
	BNE TSTDRV1
	INX			;next byte
	INY
	TYA
	AND #%00000011
	BNE TSTDRV3
	LDA MCNT		;drive found
	CLC
	RTS

TSTDRV1	INC MCNT		;no match
	LDA CNT_X		;inc tab index=x by 4
	CLC
	ADC #4
	STA CNT_X
	CMP #4*4
	BCC TSTDRV2
	LDA #IMG_NOT_FOUND
	RTS



MUL_SPC	LDY SEC_CLUS		;N32 * SEC_CLUS
MULSPC1	TYA
	LSR A
	TAY
	BCS MULSPCX
	JSR MUL2
	JMP MULSPC1

MULSPCX	RTS
	

MUL2	ASL N32			;N32*2
	ROL N32+1
	ROL N32+2
	ROL N32+3
	RTS


ADD	CLC			;BASE = BASE + N32
	LDX #0
	LDY #3
ADD1	LDA BASE,X
	ADC N32,X
	STA BASE,X
	INX
	DEY
	BPL ADD1
	RTS


DIRB2SEC			;set SEC = BASE*2
	LDX #0
	CLC
DIRB2S1	LDA BASE,X
	ROL A
	STA SECS,X
	INX
	CPX #4
	BCC DIRB2S1
	RTS


CHK_FS	LDY #$1F		;check for valid image file size
	LDX #3
CHK_FS1	LDA (DIR_FNP),Y
	CMP IMG_SIZ,X
	BNE CHK_FSX
	DEY
	DEX
	BPL CHK_FS1
	CLC
	RTS

CHK_FSX	SEC
	RTS

PRT_FN	LDY #0			;print filename
PRT_FN1	LDA (DIR_FNP),Y
	JSR PRTCHR
	INY
	CPY #12
	BCS PRT_FNX
	CPY #8
	BNE PRT_FN1
	LDA #'.
	JSR PRTCHR
	JMP PRT_FN1

PRT_FNX	RTS


CHKDRV	BCS CHKDRV1		;check for valid drive
	SBC #$40
	BCC CHKDRV2
	AND #$1F
	CMP #SD1ST
	BCC CHKDRV2
	CMP #SDMAX		;Drives SD1ST..SDMAX
	BCS CHKDRV2
	RTS
	
CHKDRV1 LDA DRIVE		;default 
	CLC
	RTS

CHKDRV2	LDA #ILLPAR
	SEC
	RTS


DISHEX	JSR PRTHEX		;DISPLAYS A HEX
	LDA #SPACE
	JSR PRTCHR
	RTS


DISASC	AND #$7F
	CMP #$20
	BCC DISAS1
	CMP #$7F
	BCC DISAS2
DISAS1	LDA #'.
DISAS2	JSR PRTCHR
	RTS


GDEC	JSR GETHEX		;GETS A DECIMAL FROM LINE
	BCS GDECX
	AND #$0F
	CMP #10
	BCS GDEC1
	STA NUMBER		;EINER
	LDA HEXL
	AND #$F0
	LSR A			;ZEHNER*8
	PHA
	CLC
	ADC NUMBER
	STA NUMBER
	PLA
	LSR A
	LSR A			;ZEHNER*2
	CLC
	ADC NUMBER
	CPY #3			;Y= HEXH
	BCS GDEC1
GDEC5	DEY
	BMI GDEC4
	ADC #100
	BCC GDEC5
GDEC1	LDA #ILLPAR
	SEC
GDECX	RTS

GDEC4	STA NUMBER
	CLC
	RTS


GETSEC	JSR GETHEX		;get sector nr
	BCS GETHEX1
	LDX #3
GETSEC2	LDA HEXL,X		;copy HEX -> SEC
	STA SECS,X
	DEX
	BPL GETSEC2
GETHEX1	RTS


GETHEX	LDA #0			;GET 32 bit HEX NUMBER FROM LINE
	LDX #3			;reset HEX
GETHEX8	STA HEXL,X
	DEX
	BPL GETHEX8
	TAX
GETHE2	JSR GETNIB		;GET NIBBLE TO HEX
	BCS GETHE1
	INX
	CPX #9
	BCC GETHE2
GETHE5	LDA #ILLPAR		;ILLEGAL PARAMETER
	SEC
	RTS

GETHE1	CPX #0
	BNE GETHE3
	CMP #SPACE
	BEQ GETHE2
	CMP #EOL
	BEQ GETHE6
	CMP #SEMI
	BNE GETHE5
GETHE6	LDA #MISPAR
	SEC
	RTS

GETHE3	CMP #SEMI
	BEQ GETHE4
	CMP #',
	BEQ GETHE4
	CMP #EOL
	BNE GETHE5
GETHE4	LDA HEXL
	LDY HEXH
	CLC
	RTS


GETNIB	JSR GNCHR		;GET NIBBLE TO HEX
	BCS GETNIX
	CMP #'0
	BCC GETNI1
	CMP #$3A
	BCC GETNI2
	CMP #'A
	BCC GETNI1
	CMP #'G
	BCC GETNI3
GETNI1	SEC
GETNIX	RTS

GETNI3	CLC
	ADC #9
GETNI2	AND #$0F
	LDY #4
GETNI4	ASL HEXL
	ROL HEXH
	ROL HEXX
	DEY
	BNE GETNI4
	ORA HEXL
	STA HEXL
	CLC
	RTS


GNCHR	LDY #0
	INC CMDVEC
	LDA CMDVEC
	CMP (INBUF),Y
	BCS GNCHRX
	TAY
	LDA (INBUF),Y
	JSR UPCASE
	CLC
	RTS

GNCHRX	LDA #EOL
	RTS


UPCASE	CMP #'a			;LOWER --> UPPER CASE
	BCC UPCASX
	CMP #'~
	BCS UPCASX
	AND #$5F
UPCASX	RTS


BIOSC	LDX #BIOS		;BIOS call
	JMP BDOS

;--- SPI BIOS plugin ------

READ	JSR SETLBA		;SET LBA = IMGTAB + SECS/2
	JSR CHKBLK		;CHECK IF BLOCK VALID
	BCC SD_RD1
	JSR S_READ
	BCS SD_RDX
SD_RD1	LDA SECS
	LSR A
	LDY #0
	BCS SD_RD3
SD_RD2	LDA BLKB,Y
	STA (DMA),Y
	INY
	BNE SD_RD2
	CLC
	RTS

SD_RD3	LDA BLKB+256,Y
	STA (DMA),Y
	INY
	BNE SD_RD3
	CLC
SD_RDX	RTS


WRITE	JSR SETLBA		;WRITE SD card
	JSR CHKBLK
	BCC SD_WR1
	JSR S_READ		;write implicitely
	BCS SD_WRX		;when SD buffer dirty
SD_WR1	LDA SECS
	LSR A
	LDY #0
	BCS SD_WR2
SD_WR3	LDA (DMA),Y
	STA BLKB,Y
	INY
	BNE SD_WR3
	BEQ SD_WR4

SD_WR2	LDA (DMA),Y
	STA BLKB+256,Y
	INY
	BNE SD_WR2	
SD_WR4	LDA #$80
	STA LBA_F		;BLKB dirty
	LDA SECS+2		;is it a dir sector?
	BNE SD_WR5
	LDA SECS+1
	BNE SD_WR5
	LDA SECS
	CMP #SYSSCS+DIRSCS		
	BCS SD_WR5
	JSR SD_WB		;then write SD block immediate
SD_WRX	RTS

SD_WR5	CLC
	RTS


;====== SUBROUTINES =======

SETLBA	LDX #3			;LBA = SEC/2
	CLC
SETLBA0	LDA SECS,X		;SEC/2
	ROR A
	STA LBA,X
	DEX
	BPL SETLBA0
	CLC
	RTS


CHKBLK	LDY #3		;CHECK IF BLK IN BUFFER VALID
CHKBLK2	LDA LBA,Y	;C=1 invalid
	CMP LBA_B,Y
	BNE CHKBL1
	DEY
	BPL CHKBLK2
	LDA LBA_F
	ASL A
	RTS

CHKBL1	SEC
	RTS



S_READ	BIT LBA_F		;SD_ READ
	BPL S_READ1		;if buffer dirty
	JSR S_SAVE		;SAVE SD_BUFFER
	BCS S_READX
S_READ1	LDX #LBA		;load LBA to BLKB
	LDY #LBA/256
	JSR SD_RD_LBLK_BUF
	BCS S_READX
	SEC
	RTS

S_READX	JSR SD_UPD		;UPDATE BLK/FLAG INFO
	CLC
	RTS


S_SAVE	LDX #LBA_B		;save old buffer to SD
	LDY #LBA_B/256
	JSR SD_WR_LBLK_BUF
	BCC S_SAVEX
	CLC
	RTS

S_SAVEX	SEC
	RTS


SD_WB	LDX #LBA		;Write SD_ block pysically
	LDY #LBA/256
	JSR SD_WR_LBLK_BUF
	BCS S_READX
	SEC
	RTS


SD_UPD	LDA LBA+3		;save LBA AND update LBA_FLAG
	STA LBA_B+3
	LDA LBA+2
	STA LBA_B+2
	LDA LBA+1
	STA LBA_B+1
	LDA LBA
	STA LBA_B
	LDA #0
	STA LBA_F
	RTS


; *****************************************************************************
; Read Single Data Block from Logical Address to Std. Block Buffer
; Input:  X,Y = Ptr[LO:HI] to 32 Bit LBA Address
; Output: C = 0 Error, C = 1 Data OK
;	  A = Error Code
; *****************************************************************************

SD_RD_LBLK_BUF	JSR INIT_BLKBUF		; set pointer to block buffer
					; fall through to sd_rd_lblk

; *****************************************************************************
; Read Single Data Block from Logical Address
; Input:  X,Y = Ptr[LO:HI] to 32 Bit LBA Address
;	  BLKBUF,BLKBUFH = 16 Bit Destination Address
; Output: C = 0 Error, C = 1 Data OK
;	  A = Error Code
; *****************************************************************************

SD_RD_LBLK	JSR LOAD_LBA		; convert LBA CMD ADR
					; fall through to sd_rd_blk

; *****************************************************************************
; Read Single Data Block
; Input:  SD_PB3..SD_PB0 = 32 Bit Command Block Address
;         BLKBUF,BLKBUFH = 16 Bit Destination Address
; Output: C = 0 Error, C = 1 Read OK
;	  A = Error Code
; *****************************************************************************

SD_RD_BLK LDA #CMD17	; send CMD17 (blk read)
	JSR SD_SEND_BLK_CMD
	LDX #0
	STX T2CL	;set timer to 4 us
	LDX #32
SDRDB1	DEX		;32 us delay for VIA
	BNE SDRDB1
	LDA #DATA_TOKEN
	JSR SD_WAIT_TOKEN	; wait for data token $FE
	CMP #$FE	; is card ready for block read?
	CLC
	BNE SD_RD_END	; did not receive data token, exit with C = 0
	LDX #$01	; initialize page counter
	LDY #$00	; initialize byte counter
SD_RD_BLK0 JSR	SPI_BIN	; read a byte
	STA (BLKBUF),Y	; and store it into the block buffer
	INY		; increment destination pointer
	BNE SD_RD_BLK0	; pointer overflow? No, read next byte
	INC BLKBUF+1	; yes, increment block buffer page
	DEX
	BPL SD_RD_BLK0	; two pages read? no, read next byte
SD_RD_BLK1 JSR SPI_BIN	; yes, read 3 more bytes (CRC H, CRC L, dummy)
	INX
	CPX #$02	; all 3 bytes read?
	BNE SD_RD_BLK1	; no, read next byte
	LDX #5		;set timer to 14 us
	STX T2CL
	LDX #120
SDRDB2	DEX		;120 us delay for VIA
	BNE SDRDB2
	SEC		; yes, all data read, set C = 1
SD_RD_END RTS

; *****************************************************************************
; Write Single Data Block from Std. Block Buffer
; Input:  SD_PB3..SD_PB0 = 32 Bit Command Block Address
; Output: C = 0 Error, C = 1 Read OK
;	  A = Error Code
; *****************************************************************************

SD_WR_BLK_BUF	JSR	INIT_BLKBUF	; set pointer to block buffer
		BEQ	SD_WR_BLK

; *****************************************************************************
; Write Single Data Block from Std. Block Buffer to Logical Address
; Input:  X,Y = Ptr[LO:HI] to 32 Bit LBA Address
; Output: C = 0 Error, C = 1 Data OK
;	  A = Error Code
; *****************************************************************************

SD_WR_LBLK_BUF	JSR	INIT_BLKBUF	; set pointer to block buffer
					; fall through to sd_rd_lblk

; *****************************************************************************
; Write Single Data Block to Logical Address
; Input:  X,Y = Ptr[LO:HI] to 32 Bit LBA Address
;	  BLKBUF,BLKBUFH = 16 Bit Source Address
; Output: C = 0 Error, C = 1 Data OK
;	  A = Error Code
; *****************************************************************************

SD_WR_LBLK	JSR	LOAD_LBA		; convert LBA CMD ADR
						; fall through to wr_rd_blk

; *****************************************************************************
; Write Single Data Block
; Input:  SD_PB3..SD_PB0 = 32 Bit CommandBlock Address
; Output: C = 0 Error, C = 1 Read OK
;	  A = Error Code
; *****************************************************************************

SD_WR_BLK	LDA	#CMD24			; send CMD24 (blk write)
		JSR	SD_SEND_BLK_CMD
		CMP	#$FE			; is card ready for block write?
		CLC
		BNE SD_WR_END
		JSR	SPI_OUT		; write data token
		LDX	#$01			; initialize page counter
		STX XBUF
		LDY	#$00			; initialize byte counter
SD_WR_BLK0	LDA	(BLKBUF),Y		; read next byte from buffer
		STY YBUF
		JSR	SPI_OUT		; and write it to the card
		LDY YBUF
		INY				; increment source pointer
		BNE	SD_WR_BLK0		; pointer overflow? No, write next byte
		INC	BLKBUF+1			; yes, increment block buffer page
		DEC XBUF
		BPL	SD_WR_BLK0		; two pages written? no, write next byte
		JSR	SPI_BIN		; yes, send a (dummy) CRC ($FFFF)
		JSR	SPI_BIN	
		JSR	SPI_BIN			; read response byte
		PHA
SD_WR_BUSY	JSR	SPI_BIN			; read next byte
		TAX
		BEQ	SD_WR_BUSY		; check if busy ($00)
		PLA
		AND	#$1F			; mask result bits
		CMP	#$05			; data accepted?
		CLC
		BNE	SD_WR_END		; no, exit with C = 0
		SEC				; yes, exit with C = 1
SD_WR_END	RTS

; *****************************************************************************
; Send Block Read or Write Command
; Input :  A = Command (CMD17,CMD24)
; Output : -
; *****************************************************************************

SD_SEND_BLK_CMD	JSR	SD_SEND_CMD
		BNE	SD_RESP_ERR		; response <> 0 check error type
		LDA	#DATA_TOKEN
		RTS

; *****************************************************************************
; Check Error
; *****************************************************************************

SD_RESP_ERR	AND	#$01			; is card in idle mode?
		BEQ	SD_DISK_RW		; no, print error
		JSR	SD_INIT			; yes, maybe card changed, reset
		BCS	SD_DISK_CHNG
SD_NO_DISK	LDA	#$80
		RTS

SD_DISK_RW	LDA	#$86
		CLC
		RTS

SD_DISK_CHNG	LDA	#$87
		CLC
		RTS


; *****************************************************************************
; Load Logical Block Address into Command Address.
; Swap Endian and Shift Bits if Desired
; Input:  X,Y = Ptr[LO:HI] to 32 Bit LBA Address
; Output: ADR in SD_PB3..SD_PB0
; *****************************************************************************

LOAD_LBA	STX	PLBA
		STY	PLBA+1
		LDX	#$04
		LDY	#$00
		LDA	SD_TYPE
		BNE	BLK_MODE
		CLC
		TYA
                STA	SD_CMD,X
		DEX
BIT_MODE	LDA	(PLBA),Y
		ROL	A
                STA	SD_CMD,X
		INY
		DEX
		BNE	BIT_MODE
		RTS
BLK_MODE	LDA	(PLBA),Y
		STA	SD_CMD,X
		INY
		DEX
		BNE	BLK_MODE
		RTS

; *****************************************************************************
; Initialize SD-Card
; Output: C = 1 Init OK, C = 0 Error
; *****************************************************************************

SD_INIT	LDA #$00
	STA SD_TYPE
	JSR SD_RESET		; reset SD-Card
	CMP #$01		; SD-Card present?
	BNE SDC_NOT_FOUND	; invalid response, no usable card found
	JSR SD_GET_VERS		; get SD-Card version
	CMP #$05		; seems to be a version 1 card
	BEQ INIT_SD0		; so just try to initialize it
	CMP #$AA		; version 2 cards should response with $(01)AA
	BNE SDC_NOT_FOUND	; invalid response, no usable card found
	LDA #$40		; try ACMD41($40000000) init (SD Ver. 2+)
	BNE INIT_SD1

INIT_SD0 LDA #$00		; try ACMD41($00000000) init (SD Ver. 1)
INIT_SD1 JSR SD_CLEAR_CMD	; prepare for new command
	STA SD_PB3
INIT_SD2 LDA #CMD55		; send prefix CMD55 (application cmd)
	JSR SD_SEND_CMD
	CMP #$01
	BNE SDC_NOT_FOUND	; invalid response, no usable card found
	LDA #ACMD41		; send ACMD41 (initialize)
	JSR SD_SEND_CMD
	BEQ INIT_SD3		; response = 0 means card waked up,
	CMP #$01		; card still idle?
	BEQ INIT_SD2		; yes, try again
	BNE SDC_NOT_FOUND	; no, invalid response, no usable card found
INIT_SD3 LDA SD_PB3		; Ver. 2+ Card?
	BEQ INIT_SD4		; no, just set block size
	JSR SD_CLEAR_CMD	; prepare for new command
	LDA #CMD58		; send CMD58 (get OCR)
	JSR SD_SEND_CMD
	BNE SDC_NOT_FOUND	; invalid response, no usable card found
	JSR SD_WAIT_RESP3	; wait for OCR response
	LDA SD_PB3	 	; Test Bit 30
	AND #$40	 	; 1 if SDHC/SDXC card, 0 else
	STA SD_TYPE	 	; set type $00 Byte mode, $40 LBA mode
INIT_SD4 JSR SD_CLEAR_CMD		; prepare for new command
	LDA #$02	 	; set blocksize to 512 byte
	STA SD_PB1
	LDA #CMD16		; send CMD16 (set block size)
	JSR SD_SEND_CMD
	BNE SDC_NOT_FOUND ; invalid response, no usable card found
	SEC 	 	; everything gone well, set carry
	RTS

SDC_NOT_FOUND
	LDA #$88
	CLC  		; something went wrong, clear carry
	RTS 		; to signal error

; *****************************************************************************
; Get SD-Card Version
;  ****************************************************************************
SD_GET_VERS 
	LDA #$01	; set parameter byte 1
 	STA SD_PB1
 	LDA #$AA	; set parameter byte 0
 	STA SD_PB0
	LDA #$87	; set crc
	STA SD_CRC
	LDA #CMD8	; send CMD8($000001AA) (get version)
	JSR SD_SEND_CMD	; response should be $01
	CMP #$01	; SD-Card present?
	BNE END_GET_VERS ; no, exit with result <> $01
			; yes, fall through to sd_wait_resp

; *****************************************************************************
; Wait for a 32 Bit Command R3 Response from SD-Card
; *****************************************************************************

SD_WAIT_RESP3	LDY #$00
READ_RESP3	STY YBUF
		JSR SD_WAIT_RESP ;yes, receive 4 response bytes
		LDY YBUF
		STA SD_PB3,Y	;store response bytes in PB0..3
		INY
		CPY #$04
		BNE READ_RESP3
END_GET_VERS	RTS

; *****************************************************************************
; Clear SD-Card Command Parameters
; *****************************************************************************

SD_CLEAR_CMD	LDA	#$00
		LDY	#$04			; 4 parameter bytes to clear
NEXT_PARAM	STA	SD_CMD,Y		; clear parameter byte
		DEY
		BNE	NEXT_PARAM		; more to clear?
		LDA	#$FF
		STA	SD_CRC			; no, finally set CRC byte to $FF
		RTS

; *****************************************************************************
; Send Command to SD-Card
; Input: A = Command Index
; *****************************************************************************

SD_SEND_CMD	STA	SD_CMD
		LDA #$01
		STA DRB			;CS = 0, MOSI = 1
		JSR	SPI_BIN		; send one dummy
		LDX	#$00
SEND_BYTE	STX XBUF
		LDA	SD_CMD,X	; get one command byte
		JSR	SPI_OUT		; and send it
		LDX XBUF
		INX
		CPX	#$06		; all 6 cmd bytes send?
		BNE	SEND_BYTE	; no, send more bytes
					; yes, fall through to sd_wait_resp

; *****************************************************************************
; Wait for a 8 Bit Command R1 Response from SD-Card
; Output: A = Response Byte
; *****************************************************************************

SD_WAIT_RESP	LDX	#$08			; wait for max 8 cycles
READ_RESP1	JSR	SPI_BIN		; receive data
		CMP	#$FF			; is it a $FF?
		BNE	RESPONSE		; no, card did respond
		DEX				; yes, try again
		BNE	READ_RESP1		; check for timeout
RESPONSE	TAX
		TXA				; set proper status flags for A
		RTS

; *****************************************************************************
; Wait for a Special Token Response from SD-Card
; Input:  A = Token Byte
; Output: A = Response Byte
; *****************************************************************************

SD_WAIT_TOKEN	STA	TEMP			; store token into TEMP variable
		LDY	#$FF			; load low byte of time out counter
		LDX	#$0A			; load high byte of time out counter
WAIT_RESP	JSR	SPI_BIN		; read byte from SPI
		DEY				; decrement wait counter
		BNE	WAIT_RESP0
		DEX
		BEQ	WAIT_RESP_END		; wait counter is 0 -> time out
WAIT_RESP0	CMP	TEMP			; did we read the token we are waiting for?
		BNE	WAIT_RESP		; no, read next byte
WAIT_RESP_END	RTS

; *****************************************************************************

SD_RESET JSR SD_CLEAR_CMD	;clear command parameters
	LDA #$95
	STA SD_CRC		; and set crc to $95 for CMD0
	JSR SPI_INIT		;CS=1; MOSI&CLK=1;MISO is input
	JSR SD_PREPARE		; send dummy sequence to SD-Card
	BNE RESET_SDC		; is MISO line high?
	LDA #CMD0		; no, send CMD0 (reset) to SD-Card
	JSR SD_SEND_CMD
	JSR SD_PREPARE		; send init dummy s=ence again
	BEQ END_SD_RESET	; MISO still low? Exit with A = $FF
RESET_SDC LDA #CMD0		; send CMD0 (reset) to SD-Card
	JMP SD_SEND_CMD		; response should be $01

END_SD_RESET
	LDA #$FF		; reset failed
	RTS


; **** Prepare SD-Card for Communication **************************************
;
; *****************************************************************************

SD_PREPARE	LDX	#10		; first send 80 clocks to SD-Card
SEND_CLOCK	JSR	SPI_BIN		; send 8 clock cycles
		DEX
		BNE	SEND_CLOCK	; send more clock cycles
		TAX
		TXA			; set proper status flags
SD_END		RTS


; ********************************************
; Initialize Block Buffer Pointer
; ********************************************

INIT_BLKBUF	LDA	#BLKB/256
		STA	BLKBUF+1
		LDA	#BLKB
		STA	BLKBUF
		RTS

;--- Junior SPI routines --------------------

SPI_INIT LDA #$03	;init VIA for SPI
	STA DRB		;CS = 1, MOSI = 1
	LDA #%00000011	;Bit 0  MOSI; Bit 1  CS
	STA DDRB
	LDA #$04	;SR mode 010
	STA ACR
	LDA #5		;Timer2 = 14 us
	STA T2CL
	STA SR
	LDX #$10
SPI_IN1	JSR SPI_IN2	;delay 17 ms
	SBC #1
	BCS SPI_IN1
	DEX
	BNE SPI_IN1
SPI_IN2	RTS


SPI_BIN	LDA IFR		;read byte from SPI
	AND #$04	;test SR bit
	BEQ SPI_BIN
	STA IFR		;clear SR bit
	LDA SR
	RTS


SPI_OUT	PHA		;write A to SPI
SPIOUT1	LDA IFR		;read byte from SPI
	AND #$04	;test SR bit
	BEQ SPIOUT1
	STA IFR		;clear SR bit
	LDX #0
	LDY #%00000001	;byte to write a %1
	PLA
	SEC		;set marker bit
	ROL A		;Bit in C
	BCS SPIOUT7
	STX SR		;start timer2
	STX DRB		;set bit 7 = 0
	ASL A		;shift bit 6 in C
	BNE SPIOUT3

SPIOUT7	STX SR		;start timer2
	STY DRB		;set bit 7 = 1
	ASL A		;shift bit 6 in C
	BNE SPIOUT3

SPIOUT2	STX DRB		;output a %0
	ASL A
	BEQ SPIOUT4
SPIOUT3	BIT $00		;3 us delay
SPIOUT6	BCC SPIOUT2	;14 us per bit
	STY DRB		;output a %1
	ASL A
	BNE SPIOUT3	;marker bit in C
SPIOUT4	STY DRB		;CS = 0  MOSI = 1
	RTS

; ********************************************

ERROR	STA ERRNO		;ERROR ROUTINE
	JSR CRLF
	LDA #ERRTAB		;SET CCPV
	STA CCPV
	LDA #ERRTAB/256
	STA CCPV+1
	LDY #0			;CHECK ERROR CODE
	LDA (CCPV),Y
ERROR1	BEQ ERROR4		;END OF ERROR ROUTINE
	CMP ERRNO
	BEQ ERROR3
ERROR2	JSR INCCPV		;SKIP ERROR MESSAGE
	BNE ERROR2
	JSR INCCPV
	JMP ERROR1

ERROR3	JSR INCCPV
	PHA
	JSR ERRTYP		;PRINT ERROR TYPE
	JSR INCCPV
	LDA CCPV 		;PRINT ERROR MESSAGE
	LDY CCPV+1
	JSR PRTSTR
	PLA
	BPL ERROR5
	JSR ASKRTY		;ASK FOR RETRY
	LDA ERRNO
	RTS

ERROR4	JSR UETYP		;UNKNOWN ERROR
ERROR5	SEC
	RTS


ERRTYP	LDY #0			;PRINT ERROR TYPE
ERRTY1	LSR A			;SHIFT SOURCE BIT IN C
	BCS ERRTY2
	INY			;X= X+5
	INY
	INY
	INY
	INY
	BNE ERRTY1

ERRTY2	LDA ETYPTB,Y
	BEQ UETYP		;LAST CHR?
	JSR PRTCHR
	INY
	BNE ERRTY2

UETYP	LDA #ERRM1
	LDY #ERRM1/256
	JSR PRTSTR
	LDA ERRNO
	JSR PRTHEX

CRLF	LDA #CRLFM		;PRINTS A NEWLINE
	LDY #CRLFM/256

PRTSTR	LDX #STROUT
	JMP BDOS


PRTCHR	LDX #CONOUT		;PRINTS A CHR
	JMP BDOS


INCCPV	INC CCPV 		;INC CCPV
	BNE INCCP1
	INC CCPV+1
INCCP1	LDA (CCPV),Y
	RTS


ASKRTY	LDA #RTYMES		;RETRY?
	LDY #RTYMES/256
	JSR PRTSTR
	LDX #CONIN
	JSR BDOS
	JSR UPCASE
	CMP #'Y
	BNE ASKRT1
	CLC
	RTS

ASKRT1	SEC
	RTS


PRTHEX	PHA			;PRINTS A HEX NUMBER
	LSR A
	LSR A
	LSR A
	LSR A
	JSR PRTNIB		;PRINT NIBBLE
	PLA
	AND #$0F

PRTNIB	CMP #$0A
	BCC PRTNI1
	ADC #6
PRTNI1	ADC #$30
	JSR PRTCHR
	RTS


ERRM1	DB ' Error $',EOT
RTYMES	DB CR,LF,'Retry	(Y/N) ? ',EOT

ETYPTB	DB 'BIOS',$00,'BDOS',$00,'CCP',$00,$00
	DB 'RSX',$00,$00,'USER',$00

ERRTAB	DB $F9,$01,'Invalid Drive',EOT
	DB $DE,$82,'Directory full',EOT
	DB $DD,$82,'File not found',EOT
	DB $DC,$02,'File exists',EOT
	DB $D9,$02,'Invalid FCB',EOT
	DB $D8,$82,'Disk full',EOT
	DB $D6,$82,'File is R/O',EOT
	DB $8D,$10,'BIOS Version below 3.3',EOT
	DB $8C,$10,'no mounted drive found',EOT
	DB $8B,$10,'no image available',EOT
	DB $8A,$10,'too many IMG files',EOT
	DB $89,$10,'no partition found',EOT
	DB $88,$10,'no SD card found',EOT
	DB $87,$10,'SD card changed',EOT
	DB $86,$10,'SD card write protected',EOT
	DB $85,$10,'Invalid Drive',EOT
	DB $84,$10,'End of Line',EOT
	DB $83,$10,'Too Much Parameters',EOT
	DB $82,$10,'Illegal Parameter',EOT
	DB $81,$10,'Missing Parameter',EOT
	DB $80,$10,'Unknown Command',EOT

	DB $00

;--- Disk Image TABLE ---


SD_TYPE	DB 0		;SD Card Type

IMG_SIZ	DB 0,0,$10,0	;image size 1 MB
MINUS_2	DB $FE,$FF,$FF,$FF	;-2

;--- buffer for SD CARD

LBA	DS 4,0		;LSB ... MSB
LBA_B	DS 4,$FF
LBA_F	DB $00		;LBA FLAG BIT7 = dirty

IMGTAB	DS 4*4,$00	;Base Sector address of disk images mounted
IMGSTAT	DB $00,$00,$00,$00	;Flag $FF valid image, $00 no image

BASE	DS 4,0		;LBA of 1. dir sector/1. data sector - 2 cluster
NFAT	DB 0
SEC_CLUS DB 0
NDIR	DW 0

N32	DS 4,0		;32 bit argument

;--------------

STATM1	DB CR,LF,'SD-UTIL	V',VERSION/16+$30,'.',VERSION*$1000/$1000+$30
	DB '	SD-Drive ',EOT
STATM2	DB ':  Sector ',EOT
STATM3	DB '   LBA ',EOT

PROMPT	DB CR,LF,'Command >',EOT

CRLFM	DB CR,LF,EOT

HELPM	DB CLS,'   SD-UTIL V',VERSION/16+$30,'.',VERSION*$1000/$1000+$30
	DB '			(c) 18.7.2023',CR,LF
	DB 'Command Summary:',CR,LF
	DB ' D	dump current sector	'
	DB ' Snnnnnn	read sector nnnnnn',CR,LF
	DB ' +	read next sector	'
	DB ' -	read last sector',CR,LF
	DB ' R	read current sector	'
	DB ' W	write current sector',CR,LF
	DB ' I	read SD directory	'
	DB ' Md,m  mount on d image m',CR,LF
	DB ' P	push mounted d to disc  '
	DB ' Znn	wait for nn/10 s',CR,LF
	DB ' >	save sector to buffer   '
	DB ' <	get sector from buffer',CR,LF
	DB ' Baddr set buffer to addr	'
	DB ' /nn	repeat nn times',CR,LF
	DB ' CHad,by,by,...	change value at ad with by',CR,LF
	DB ' CAad,<string>	change at ad with string',CR,LF
	DB ' ?	prints this screen	'
	DB ' ',SEMI,'	command separator',CR,LF
	DB ' Lnnnnnn  read LBA nnnnnn	'
	DB ' X	exit to CP/M		',CR,LF
	DB EOT


DMABUF
BLKB = DMABUF+256		;BLKBUF
IMG_LBA	= BLKB+512		;Image LBAs

	END

