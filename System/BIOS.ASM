;FLOPPY DISK BIOS FUER DIE UDC-KARTE
;NACH ELEKTOR COMPUTING 6

;D. LAUSBERG	(C) 1987
;V1.2	11.11.88
;V1.3	13.12.89	PRINTER ROUTINES
;V1.4	14.06.90	NEW PARAMETERS
;V1.5	14.10.90	IMPROVEMENTS
;V1.6	10.11.91	ASM-65 EXTENDED VERSION
;V2.0	20.11.91	IRQ BASED VERSION
;V2.1	27.05.92	BOOT ROM VERSION
;V2.2	18.06.92	INCREASED SPEED, FORMAT TRACK
;V2.3	09.03.93	DOUBLE DRIVE
;V2.4	21.02.95	new memory map
;V2.5	27.03.95	SCSI support
;V2.6	09.12.95	2 logical SCSI discs
;V2.7	26.07.98	slightly modified memory map
;V2.8	30.06.00	change to sector bios
;V2.9	08.11.02	disk parameters corrected
;V2.A	25.03.21	4 Floppies + 2 HD Drives
;V2.B	30.05.21	SCSI2SD corrections interim fix, I2C commands
;V3.0	07.02.22	BOOTPROM v3.0 version
;V3.1	25.03.23	BOOTPROM v3.2 version
;V3.2	06.07.23	SCSI Write error correction
;V3.3	22.10.25	SD Card integration

VERSION	= $33		;VERSION NUMBER

;FORMAT	DS/DD	320 KBYTES/DISK, 4 Drives 0..3
;SCSI2HD	4x 16 MBytes on SCSI ID1  4..7
;SD		4x  1 MByte IMG on SD partition (FAT 12/16/32) 8..12
;		SD initialization and block transfer routines based on
;		work of Joerg Walke, JC-][
;=====================================

;PAGE 00 CELLS

DMAV	= $FE		;DMA-VECTOR
DIRBFV	= $FC		;DIRBF-VECTOR
BIOSP	= $FA		;BIOS-POINTER
DPHV	= $F8		;DPH-VECTOR
JPBDOS	= $F0

;SYSTEM CONSTANTS

CCP	= $D800
BEGIN	= $E400

ROM_CONST = $F806
ROM_CONIN = $F809
ROM_CONOUT = $F80C
ROM_LIST  = $F80F
ROM_LISTST = $F80A
ROM_MONITOR = $F830
ROM_SET_TIMER1 = $F833
ROM_GET_TIMER1 = $F836
ROM_SET_TIMER2 = $F839
ROM_GET_TIMER2 = $F83C
ROM_I2CCLR = $F83F		; 15
ROM_I2CSTOP = $F842		; 16
ROM_I2CADEV = $F845		; 17
ROM_I2CR16 = $F848		; 18
ROM_I2CRDEV = $F84B		; 19
ROM_I2CWByte = $F84E		; 1A
ROM_I2CRByte = $F851		; 1B
ROM_I2CRLB = $F854		; 1C

ROM_I2CGTime = $F85A		; 1E
ROM_I2CGDate = $F85D		; 1F

;UDC-CONTROLLER ADDRESSES

UDC	= $F500
SCSI_D	= UDC		;SCSI Data Port
SCSI_ST	= UDC+$10	;SCSI Status Port
SCSI_SE	= UDC+$20	;SCSI Select Register
SCSI_R	= UDC+$30	;SCSI Reset Port
FDSEL	= UDC+$40	;MOTOR/SELECT DRIVE LATCH
FDSCR	= UDC+$50	;READ STATUS, WRITE COMMAND
FDTRG	= UDC+$51	;R/W TRACK REGISTER
FDSRG	= UDC+$52	;R/W SECTOR REGISTER
FDDAR	= UDC+$53	;R/W DATA REGISTER
FLSTAT	= UDC+$60	;IRQ --> BIT 7, DRQ --> BIT 6

;VIA ADDRESSES
;SPI driver  Mode 0 uses VIA DRB
;BIT 1	CS
;Bit 0	MOSI
; CB 1	CLK
; CB 2	MISO

DRB	= $F400
DRA	= $F401
DDRB	= $F402
DDRA	= $F403
T2CL	= $F408
T2CH	= $F409
SR	= $F40A
ACR	= $F40B
PCR	= $F40C
IFR	= $F40D
IER	= $F40E

;6532 ADDRESSES

SCRATCH	= $F600

;--- buffer for SD CARD

SD_TYPE	= SCRATCH+$45		;SD Card Type
LBA	= SCRATCH+$46		;LSB ... MSB
SPEED	= SCRATCH+$4A		;SPEED flag b7=1 38400 Bd; b6=1 19200 Bd
I2CDEV	= SCRATCH+$4B		;I2C device nr 7 Bit
ID	= SCRATCH+$4C		;I2C memory data set ID
PNT16	= SCRATCH+$4E		;I2C 16 bit register
ADR	= SCRATCH+$50		;I2C adress pointer
ENDADR	= SCRATCH+$52		;I2C end adress
STPRATE	= SCRATCH+$54		;Steprate $00: 6ms, $01: 12 ms
BLK	= SCRATCH+$55		;BLK = SECS/2 (H/M/L)
BITTIME	= SCRATCH+$58		;bit delay for COM
HBTTIME	= SCRATCH+$59		;half bit delay for COM
COMBUF	= SCRATCH+$5A		;6 Byte SCSI Command Buffer  .. $5F
MODE	= SCRATCH+$60		;MODE SWITCH FOR HEX MONITOR
KEY	= SCRATCH+$61		;KEY BUFFER FOR HEX KEYBOARD
TIME	= SCRATCH+$62		;8 BIT TIME FOR ELEKTERMINAL
YBUF	= SCRATCH+$63		;Y buffer for CHRIN & CHROUT
CHARBUF	= SCRATCH+$64
CHR	= SCRATCH+$65
ABUF	= SCRATCH+$66		;ACCU BUFFER FOR IRQ ROUTINE
B_BLK	= SCRATCH+$67		;BLOCK BUFFER # (H/M/L)
BBLK_F	= SCRATCH+$6A		;BLK FLAG BIT7 = dirty
DRIVE	= SCRATCH+$6B
TRACK	= SCRATCH+$6C		;TRACK BUFFER FOR ROM-BIOS
SECTOR	= SCRATCH+$6D		;SECTOR BUFFER FOR ROM-BIOS
SIDEMS	= SCRATCH+$6E		;SIDE MASK	SIDE 1 =  $02
RETRY	= SCRATCH+$6F		;RETRY COUNTER
ERRCOD	= SCRATCH+$70		;ERROR CODE
TIMER0	= SCRATCH+$71		;SYSTEM TIMER
TIMER1	= SCRATCH+$72		;8 BIT USER TIMER
TIMER2	= SCRATCH+$73		;16 BIT USER TIMER
SECS	= SCRATCH+$75		;BIOS Sector Buffer $75 .. $77
UIRQV	= SCRATCH+$78		;&F678
NMIV 	= SCRATCH+$7A		;$F67A
BREAKV	= SCRATCH+$7C		;$F67C
IRQV	= SCRATCH+$7E		;$F67E

PAD	= $F680
PADD	= $F681
PBD	= $F682
PBDD	= $F683
RIOFLAG	= $F6D5		;RIOT FLAG REGISTER
TI1024K	= $F6FF		;TIMER 1024K IRQ ENABLE

;CONTROL CODES

BREAKC	= $03		;BREAK CODE
CLS	= $0C		;CLEAR SCREEN

;ERROR CODES

INVDRV	= $85
WRT_PROT = $86
SD_CHNG	= $87
NO_SD	= $88
SCSI_TIMEOUT = $F7
SCSI_ERROR = $F6
I2CError = $F5
NO_DEV	= $F4
ID_NFND	= $F3

;WD 2797 COMMANDS

RDSECC	= $88		;READ SECTOR
WRSECC	= $A8		;WRITE SECTOR
RSTORC	= $00		;RESTORE, steprate 6 ms
SEEKC	= $18		;SEEK WITHOUT VERIFY, steprate 6 ms

;SCSI COMMANDS

RD_COM	= $08		;SCSI READ BLOCK
WR_COM	= $0A		;SCSI WRITE BLOCK
SENSE_COM = $03		;SCSI SENSE STATUS

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

SD1ST	= 8		;I: is 1. drive on SD
SDMAX	= 12		;drives I..L on SD

;I2C adresses
RTC	= $68
24C32	= $57

;CONSTANTS

SELMSK	= $10		;MOTOR ON, DDEN ON
MAXTRK	= 80		;MAX TRACKS

SPT	= 16		;SECTORS PER TRACK
SPTDIV	= 4		;log(2) SPT
SPB	= 4		;SECTORS PER BLOCK
SPBDIV	= 2		;LOG(2) SPB
SYS	= 32		;SYSTEM sectors
BPD	= MAXTRK*SPT-SYS/SPB-1	;BLOCKS PER DISK
DIRSCS	= 16		;DIRECTORY SECTORS
DIRMSK	= %11111111	;DIR MASK FOR BAT
RPEXT	= 8*SPB*2 	;RECORD PER EXTENT
BATSIZ	= BPD/8		;SIZE OF BAT

DIRSCSC	= 32		;Directory sectors SCSI&SD

MAXTRY	= 10		;MAX TRIES ON ERROR

;==========================================

	ORG BEGIN

FBIOS	JMP BOOT		; 00
	JMP WBOOT		; 01
	JMP ROM_CONST		; 02
	JMP ROM_CONIN		; 03
	JMP ROM_CONOUT		; 04
	JMP ROM_LIST		; 05
	JMP AUXOUT		; 06
	JMP AUXIN		; 07
	JMP HOME		; 08
	JMP SELDSK		; 09
	JMP SETTRK		; 0A
	JMP SETSEC		; 0B
	JMP READ		; 0C
	JMP WRITE		; 0D
	JMP ROM_LISTST		; 0E
	JMP GET_VERSION		; 0F
	JMP ROM_MONITOR		; 10
	JMP ROM_SET_TIMER1	; 11
	JMP ROM_GET_TIMER1	; 12
	JMP ROM_SET_TIMER2	; 13
	JMP ROM_GET_TIMER2	; 14
	JMP ROM_I2CCLR		; 15
	JMP ROM_I2CSTOP		; 16
	JMP ROM_I2CADEV		; 17
	JMP ROM_I2CR16		; 18
	JMP ROM_I2CRDEV		; 19
	JMP ROM_I2CWByte	; 1A
	JMP ROM_I2CRByte	; 1B
	JMP ROM_I2CRLB		; 1C
	JMP I2CLOADID		; 1D
	JMP ROM_I2CGTime	; 1E
	JMP ROM_I2CGDate	; 1F
	JMP MOUNT		; 20

;========== DATA AREA =============

DPHA	DW DIRBF	;DISK PARAMETER HEADER
	DW DMA		;FOR DRIVE A to D
	DW BAT

	DB SPB-1	;DISK PARAMETER BLOCK FOR DISK A
	DB SPBDIV
	DW BPD
	DB DIRSCS
	DB DIRMSK
	DB RPEXT
	DB SYS,0,0
	DB BATSIZ

DPHE	DW DIRBF	;Disk Parameter Header for Drive E-H
	DW DMA		;FOR DRIVE E-H
	DW BAT

	DB 31		;SPB	DISK PARAMETER BLOCK
	DB  5		;SPBDIV
	DW $7FE		;BPD
	DB DIRSCSC	;DIRSCSC is used in SCSI_WR
	DB %10000000	;DIRMSK
	DB $80		;RPEXT
	DB SYS,0,0	;SYS
	DB $7FF/8	;BATSIZ

DPHI	DW DIRBF	;DISK PARAMETER HEADER
	DW DMA		;FOR DRIVE I-L
	DW BAT

	DB 8-1		;SPB DISK PARAMETER BLOCK FOR DISK I-L
	DB 3		;SPBDIV
	DW 128*32-32/8-1 ;BPD
	DB DIRSCSC	;DIRSCSC
	DB %11110000	;DIRMSK
	DB 8*8*2	;RPEXT
	DB SYS,0,0	;SYS
	DB 128*32-32/8-1/8 ;BATSIZ

;--- DRIVE TABLE Floppy Drives---

DRVTAB	DB $01,$02,$04,$08		;DRIVE Select TABLE
TRKTAB	DB 0,0,0,0

;--- Disk Image TABLE ---

IMGTAB	DS 4*4,0	;Base Sector address of disk images mounted
IMGSTAT	DS 4,0		;Flag $FF valid image, $00 no image

;--- BAT ----

BAT	DB %10000000	;Block Allocation Table Drive A-J
	DS $FF		;max. 1 page

;--- ERROR TABLES---

SETAB	DB $FE,$08,$FF,$10
	DB $FA,$04,$FE,$08,$FB,$10,$FC,$40,$FD,$80

;--- I2C load commands -----

SET_SRATE
	DB $57			;EEPROM adr
	DW $8001		;ID
	DW STPRATE		;steprate bytes memory location

;==================================

BOOT	LDA #$4C		;SET JMP BDOS
	STA JPBDOS
	LDA #CCP
	STA JPBDOS+1
	LDA #CCP/256
	STA JPBDOS+2
	LDA #%00001010		;SET VIA PORT A FOR CENTRONICS
	ORA PCR			;CA1 = /BUSY
	STA PCR
	LDA #%00000011
	STA IER
	STA IFR
	LDA #%11111111
	STA DDRA
	JSR SCSI_RST		;reset SCSI
	JSR SD_INIT		;init SD
	LDA PADD	
	ORA #%00000001		; PA0 is Output for I2C SCL
	STA PADD
	JSR ROM_I2CCLR		;reset I2C
	JSR SSRATE		;set steprates
	JSR HOME		;HOME BOOT DISK
	LDA DRIVE		;SELECT BOOT DRIVE
	LDX #$0E		;VIA BDOS
	JSR JPBDOS
	LDX #0 			;WARM BOOT
	JMP JPBDOS


WBOOT	LDA DRIVE		;SELECT DEFAULT DRIVE
	JSR SELDSK
	RTS


;---	CONST	JMP ROM_CONST


;---	CONIN	JMP ROM_CONIN


;---	CONOUT	JMP ROM_CONOUT


;---	LIST	JMP ROM_LIST	


AUXOUT	RTS

AUXIN	RTS


HOME	LDA DRIVE
	CMP #4			;Check for Drive > D
	BCS HOME2
	JSR DRVSEL		;HOMES TO TRACK 0
HOME0	JSR READY
	BCS HOMEX
	JSR RESTORE		;RESTORE DRIVE
	BCS HOMEX
	STA ERRCOD
	AND #%10011001		;MASK SEEK ERROR
	BNE HERR1
	STA TRACK		;SET TRACK=0
HOMEX	RTS

HOME2	CMP #SD1ST
	BCC HOMEX		;is SCSI?
	JMP SD_INIT		;else init SD

HERR1	JSR ERROR		;CHECK ERROR COUNT
	BCC HOME0		;RETRY
	JMP SERR		;CONT WITH SEEK ERROR


SELDSK	CMP #4			;TEST FOR DRIVE A to D
	BCS SELDS1
	LDX #DPHA		;SET DPHVEC
	LDY #DPHA/256
SELDS0	STA DRIVE		;SELECT DISK LOGICAL
	STX DPHV
	STY DPHV+1
	CLC
	RTS

SELDS1	LDY #$01		;select block device E - SDMAX
	STY BBLK_F		;mark BLK buffer as invalid
	CMP #8			;TEST FOR SCSI DRIVE E - H
	BCS SELDS3
	LDX #DPHE		;SET DPHVEC
	LDY #DPHE/256
	BNE SELDS0

SELDS3	CMP #SDMAX		;TEST FOR DRIVE I to SDMAX
	BCS SELDS4
	PHA
	AND #%00000011		;bits 0&1 are index
	TAX
	LDA IMGSTAT,X		;check if an image is mounted
	BEQ SELDS5
	PLA
	LDX #DPHI		;SET DPHVEC
	LDY #DPHI/256
	BNE SELDS0

SELDS5	PLA
SELDS4	LDA #$F9		;ILLEGAL DRIVE NR
	SEC
	RTS


SETSEC	TAX			;sector pointer for p.00 in Accu
	LDA 0,X
	STA SECS
	LDA 1,X
	STA SECS+1
	LDA 2,X
	STA SECS+2
	RTS


SETTRK	LDA #$FB		;Sector not found
	SEC
	RTS


READ	LDA DRIVE
	CMP #4			;CHECK FOR DRIVE > D
	BCS BLK_RD

READ4	JSR DRVSEL		;READ SECTOR TO DMA
	JSR FD_ST		;convert sec nr to sector/track
	BCS READX
READ0	JSR SEEKTR		;SELECT DRIVE AND SET TRACK
	BCS READX
	LDA SECTOR		;SET SECTOR
	STA FDSRG
	LDA #RDSECC
	ORA SIDEMS		;SET SIDE BIT
	SEI
	STA FDSCR		;READ SECTOR
	JSR DELAY
	LDY #0
READ1	BIT FLSTAT		;GET STATUS
	BMI READ2		;DONE
	BVC READ1		;NO DATA
	LDA FDDAR		;GET BYTE
	STA (DMAV),Y
	INY
	JMP READ1

READ2	JSR DELAY
	CLI
	LDA FDSCR
	STA ERRCOD
	AND #%11011111		;MASK ERROR
	BNE READ3
	LDA #0			;RESET RETRY COUNTER
	STA RETRY
	CLC
READX	RTS

READ3	JSR ERROR
	BCC READ0		;REREAD
	BCS WRITE4

BLK_RD	JSR SETBLK		;SET BLK=SECS/2
	JSR CHKBLK		;CHECK IF BLOCK VALID
	BCC BLKRD1
	JSR B_READ		;read BLK to buffer
	BCS BLKRDX
BLKRD1	LDA SECS
	LSR A
	LDY #0
	BCS BLKRD3
BLKRD2	LDA BLKBUF,Y
	STA (DMAV),Y
	INY
	BNE BLKRD2
	CLC
	RTS

BLKRD3	LDA BLKBUF+256,Y
	STA (DMAV),Y
	INY
	BNE BLKRD3
	CLC
BLKRDX	RTS


WRITE	LDA DRIVE
	CMP #4			;CHECK FOR DRIVE > D
	BCS BLK_WR

WRITE5	JSR DRVSEL		;WRITE SECTOR IN DMA
	JSR FD_ST		;convert sec nr to sector/track
	BCS WRITEX
WRITE0	JSR SEEKTR		;SET DRIVE AND TRACK
	BCS WRITEX
	LDA SECTOR
	STA FDSRG		;SET SECTOR
	LDA #WRSECC
	ORA SIDEMS		;SET SIDE BIT
	SEI
	STA FDSCR		;WRITE SECTOR
	JSR DELAY
	LDY #0
WRITE1	BIT FLSTAT		;GET STATUS
	BMI WRITE2		;DONE
	BVC WRITE1		;NO DATA
	LDA (DMAV),Y
	STA FDDAR
	INY
	JMP WRITE1

WRITE2	JSR DELAY
	CLI
	LDA FDSCR		;GET STATUS
	STA ERRCOD
	BNE WRITE3
	LDA #0			;RESET RETRY COUNTER
	STA RETRY
	CLC
WRITEX	RTS

WRITE3	JSR ERROR
	BCC WRITE0		;REWRITE
WRITE4	LDY #13
	JMP FDERR

BLK_WR	JSR SETBLK		;WRITE BLK
	JSR CHKBLK
	BCC BLKWR1
	JSR B_READ		;write implicitely
	BCS BLKWRX		;when BLK buffer dirty
BLKWR1	LDA SECS
	LSR A
	LDY #0
	BCS BLKWR2
BLKWR3	LDA (DMAV),Y
	STA BLKBUF,Y
	INY
	BNE BLKWR3
	BEQ BLKWR4

BLKWR2	LDA (DMAV),Y
	STA BLKBUF+256,Y
	INY
	BNE BLKWR2	
BLKWR4	LDA #$80
	STA BBLK_F		;BLKBUF dirty
;	LDA SECS+2
;	BNE BLKWR5
	LDA SECS+1
	BNE BLKWR5
	LDA SECS
	CMP #SYS+DIRSCSC	;is it a DIR block?		
	BCS BLKWR5
	JSR BB_SAVE		;write BLK block immediate
BLKWRX	RTS

BLKWR5	CLC
	RTS

;---	JMP ROM_LISTST


GET_VERSION			;GET VERSION NUMBER

	LDA #VERSION
	RTS

;---	JMP ROM_SET_TIMER1


;---	JMP ROM_GET_TIMER1


;---	JMP ROM_SET_TIMER2


;---	JMP ROM_GET_TIMER2


;--------- I2C Commands ------------------------------

;---	JMP ROM_I2CCLR	;clear I2C-Bus


;---	JMP ROM_I2CADEV	;send 7 bit dev adr to write


;---	JMP ROM_I2CR16	;send word L/H in ZP,A to dev


;---	JMP ROM_I2CRDEV	;send 7 bit dev adr to read


;---	JMP ROM_I2CWByte ;write byte in A to dev


;---	JMP ROM_I2CRBYTE ;read byte from dev


;---	JMP ROM_I2CRLB	;read last byte from dev


I2CLOADID		;load ID data to memory
	TAX
	LDA 0,X		;set parameters
	STA I2CDEV
	LDA 1,X
	STA ID
	LDA 2,X
	STA ID+1
	LDA 3,X
	STA ADR
	LDA 4,X
	STA ADR+1
	LDY #0		;start with dev adr = 0
	STY PNT16
	STY PNT16+1
LDID3	JSR RESDMA
	CLC		;ENDADR = DMA + 4
	ADC #4		;read 4 bytes header
	STA ENDADR
	BCC LDID4
	INX
LDID4	STX ENDADR+1
	JSR RDBLK	;read header to DMA
	BCS LDIDX
	LDA PNT16	;dev adr = dev adr + 4
	ADC #4
	STA PNT16
	BCC LDID5
	INC PNT16+1
LDID5	JSR RESDMA	;DMA = ADR
	LDY #1
	LDA (DMAV),Y
	TAX
	DEY
	LDA (DMAV),Y
	CMP #$FF
	BNE LDID1
	CPX #$FF	;ID = $FFFF ?
	BNE LDID1
	LDA #ID_NFND	;ID not found
	SEC
LDIDX	RTS

LDID1	CMP ID		;ID match?
	BNE LDID2
	CPX ID+1
	BNE LDID2
	CLC
	LDY #2		;ENDADR = ADR + LENGTH
	LDA (DMAV),Y
	ADC ADR
	STA ENDADR
	INY
	LDA (DMAV),Y
	ADC ADR+1
	STA ENDADR+1
	JSR RDBLK	;read data block
	RTS

LDID2	CLC
	LDY #2		;dev adr = dev adr + LENGTH
	LDA (DMAV),Y
	ADC PNT16
	STA PNT16
	INY
	LDA (DMAV),Y
	ADC PNT16+1
	STA PNT16+1
	JMP LDID3


;---	JMP ROM_I2CGTIME ;get time to p.00 cells


;---	JMP ROM_I2CGDATE ;get date to p.00 cells

MOUNT	TAX			;mount disk image on SD
	LDA 0,X			;load drive to be mounted
	AND #$3F
	SEC
	SBC #SD1ST
	BCC MOU_ERR
	CMP #SDMAX-SD1ST			;max I: - L:
	BCS MOU_ERR
	TAY
	LDA 0,X
	CMP #$40		;bit 6 or 7 set?
	BCS MOUNT2
	LDA #$FF		;mount disc image
	STA IMGSTAT,Y		;set drive byte as valid
	TYA
	ASL A			;*4
	ASL A
	TAY
	LDA #3			;4 bytes LBA
	SEC
MOUNT1	PHA
	INX
	LDA 0,X
	STA IMGTAB,Y		;copy LBA block 0
	INY
	PLA
	SBC #1
	BCS MOUNT1
	RTS

MOUNT2	CMP #$80		;check for bit 7
	BCS MOUNT3
	LDA IMGSTAT,Y		;LBA invalid
	BEQ MOU_ERR
	TYA			;report LBA mounted
	ASL A			;*4
	ASL A
	TAY
	LDA #4			;4 bytes LBA
MOUNT4	SEC
	PHA
	INX
	LDA IMGTAB,Y		;copy LBA block 0
	STA 0,X
	INY
	PLA
	SBC #1
	BNE MOUNT4
	CLC
	RTS

MOUNT3	LDA #$00		;dismount drive
	STA IMGSTAT,Y		;set drive byte as invalid
	CLC
	RTS

MOU_ERR	LDA #INVDRV		;invalid drive #
	SEC
	RTS

; I2C subroutines

RDBLK	LDA I2CDEV		;Read Block to Buffer
	JSR ROM_I2CADEV		;select dev for Write
	BCS RDBLKX
	LDA PNT16+1		;set 16 bit reg
	JSR ROM_I2CWByte
	BCS RDBLKY
	LDA PNT16
	JSR ROM_I2CWByte
	BCS RDBLKY
	LDA I2CDEV
	JSR ROM_I2CRDEV
	BCS RDBLKY
RDBLK1	LDY DMAV+1		;CHKEND
	CPY ENDADR+1
	BCC RDBLK2
	LDY DMAV
	CPY ENDADR
	BCS RDBLK2
	JSR ROM_I2CRBYTE	;read next byte
	BCS RDBLKX
	LDY #0
	STA (DMAV),Y
	INC DMAV
	BNE RDBLK1
	INC DMAV+1
	BNE RDBLK1

RDBLK2	JSR ROM_I2CRLB		;dummy read last byte
RDBLKX	RTS

RDBLKY	LDY #I2CERROR
	RTS

RESDMA	LDA ADR			;set DMA = ADR A/X
	STA DMAV
	LDX ADR+1
	STX DMAV+1
	RTS

; basic I2C states

;---	JMP ROM_I2CStop	; I2C stop condition

;====== SUBROUTINES =======

SSRATE	LDY #4		;try to load from EEPROM
SSR1	LDA SET_SRATE,Y
	STA 0,Y
	DEY
	BPL SSR1
	LDA #0		;page $00 vector = 0
	JSR I2CLOADID
	BCC SSRX	;if not succesful
	LDY #3		;set steprates to defaults
	LDA #$01
SSR2	STA STPRATE,Y
	DEY
	BPL SSR2
SSRX	RTS

DRVSEL	LDA DRIVE		;SELECT DRIVE
	AND #$03		;MAX. 4 DRIVES
	TAY
	LDA DRVTAB,Y		;GET DRIVE BIT
	ORA #SELMSK
	STA FDSEL		;MOTOR ON
	LDA #50/2		;5 SECONDS MOTOR ON
	STA TIMER0
	RTS


FD_ST	LDA SECS		;calc sector/track from sector nr
	STA MODE
	AND #SPT-1
	STA SECTOR		;always 2^n sectors/track
	LDA SECS+2
	BNE FD_ST1		;definitely not more than $7FF Tracks!
	LDX #SPTDIV
	LDA SECS+1
FD_ST2	LSR A			;TRACK = SECS/16
	ROR MODE
	DEX
	BNE FD_ST2
	CMP #0
	BNE FD_ST1		;max. $FF TRACK's
	LDA MODE
	CMP #MAXTRK
	BCS FD_ST1
	LSR A			;set track
	STA TRACK
	LDA #0
	ROL A
	ROL A
	STA SIDEMS		;set side bit
FD_STX	RTS

FD_ST1	LDA #$FB		;sector not found error
	SEC
	RTS


SEEKTR	JSR READY		;SEEK TRACK
	BCS SEEKX
	LDY DRIVE		;SET CONTROLLER TRACK REGISTER
	LDA TRKTAB,Y		;FOR THIS DRIVE
	STA FDTRG
	LDA TRACK		;GET NEW TRACK
	STA TRKTAB,Y		;SAVE IT TO TABLE
	STA FDDAR
	LDA #SEEKC
	ORA STPRATE,Y		;set Steprate
	STA FDSCR		;AND SEEK
SEEK1	JSR READY
	BCS SEEKX
	STA ERRCOD
	AND #%10011001		;MASK SEEK ERROR
	BNE SKERR1
	CLC
SEEKX	RTS

SKERR1	JSR ERROR		;CHECK ERROR COUNT
	BCC SEEKTR

SERR	LDY #3			;SET ERROR CONDITION

FDERR	LDA ERRCOD
	AND SETAB,Y
	BNE FDERR1
	DEY
	DEY
	BPL FDERR
	SEC			;C= 1 AND A= 0
	RTS			;UNKNOWN ERROR

FDERR1	DEY
	LDA SETAB,Y		;LOAD ERROR NR
	SEC
	RTS


RESTORE	LDA #0			;RESTORE DRIVE
	LDY DRIVE
	STA TRKTAB,Y		;SET TRACK =  0
	LDA #RSTORC
	ORA STPRATE,Y		;set Steprate
	STA FDSCR		;RESTORE
	JSR READY
	RTS


ERROR	INC RETRY		;INC RETRY COUNTERS
	LDA RETRY		;NO MORE TRIES?
	CMP #MAXTRY
	BCS ERROR1		;RETRY
	LSR A
	BCC ERROR1
	JSR RESTORE
ERROR1	RTS


DELAY	JSR DELAY1		;DELAY 28 uS
DELAY1	NOP
	RTS


READY	JSR DELAY		;WAITS WHILE CONTROLLER
READY1	LDA TIMER0		;IS BUSY AND DRIVE NOT READY
	BEQ READY2
	LDA #%10000001		;MASK READY/BUSY BITS
	BIT FDSCR
	BNE READY1
	LDA FDSCR
	CLC
	RTS

READY2	LDA #$FD		;DRIVE NOT READY
	SEC
	RTS

; *****************************************************************************
; Block device driver SCSI & SD
; *****************************************************************************

B_READ	BIT BBLK_F		;Read Block to BLKBUF
	BPL B_READ1		;if BLKBUF dirty
	JSR BB_SAVE		;SAVE BLKBUF
	BCS B_READ4
B_READ1	LDX #2			;set B_BLK = BLK
BREAD3	LDA BLK,X
	STA B_BLK,X
	DEX
	BPL BREAD3
	INX			;X=0, UPDATE BBLK FLAG
	STX BBLK_F
	LDA #%00001000		;Read BLK to BLKBUF
	BIT DRIVE		;is it SCSI or SD?
	BNE BREAD1
	JSR SCSI_RD		;read SCSI to BLKBUF
	BCS BREADX
B_READ4	RTS

BREAD1	JSR SD_RD		;read SD to BLKBUF
	BCC B_READ4

BREADX	LDX #1
	STA BBLK_F		;mark BLKBUF as invalid
	RTS


BB_SAVE	LDA #%00001000		;SAVE BLKBUF
	BIT DRIVE		;is it SCSI or SD?
	BNE BBSAV1
	JSR SCSI_WB		;write BLKBUF to SCSI at B_BUF
	BCC BBSAV2
BBSAVX	RTS

BBSAV1	JSR SD_WR
	BCS BBSAVX
BBSAV2	LDA #0			;mark BLKBUF as clean
	STA BBLK_F
	RTS


SETBLK	LDX #2			;BLK=SECS/2
	CLC
SETBLK1	LDA SECS,X
	ROR A
	STA BLK,X
	DEX
	BPL SETBLK1
	RTS


CHKBLK	LDX #2			;CHECK IF BLK IN BUFFER VALID
CHKBL2	LDA BLK,X		;C=1 invalid
	CMP B_BLK,X
	BNE CHKBL1
	DEX
	BPL CHKBL2
	LDA BBLK_F
	LSR A
	RTS

CHKBL1	SEC
	RTS


; *****************************************************************************
; SCSI routines
; *****************************************************************************

SCSI_RD JSR SET_SCSI_COMBUF	;SCSI READ
SCSIRD1	LDA #RD_COM
	STA COMBUF
	JSR S_COM		;DO SELECT AND EXEC COMMAND
	BCS SCSIRDX
	LDA #%01101111
	JSR S_WAIT
	BCS SCSIRDX
	LDY #0			;READ 512 BYTES
SCSIRD2	LDA SCSI_D
	STA BLKBUF,Y
	INY
	BNE SCSIRD2
SCSIRD3	LDA SCSI_D
	STA BLKBUF+256,Y
	INY
	BNE SCSIRD3
	JSR GET_SM		;GET SCSI STATUS
SCSIRDX	RTS


SCSI_WB	JSR SET_SCSI_COMBUF
	LDA #WR_COM		;Write SCSI block pysically
	STA COMBUF		;block nr in COMBUF
	JSR S_COM
	BCS SCSIWBX
	LDA #%01101110
	JSR S_WAIT
	BCS SCSIWBX
	LDY #0			;SAVE 512 BYTES
SCSIWB1	LDA BLKBUF,Y
	STA SCSI_D
	INY
	BNE SCSIWB1
SCSIWB2	LDA BLKBUF+256,Y
	STA SCSI_D
	INY
	BNE SCSIWB2
	JSR GET_SM		;GET SCSI STATUS
SCSIWBX	RTS


SET_SCSI_COMBUF			;set COMBUF with B_BLK
	LDA B_BLK
	STA COMBUF+3
	LDA B_BLK+1
	STA COMBUF+2
	LDA DRIVE
	AND #%00000011		;E:=$00 - H:$03 -> 0..3 16MB/DISK =64 MB
	ROL COMBUF+2		;add LSBit to COMBUF+2 via C
	LSR A
	ROR COMBUF+2
	STA COMBUF+1		;save MSBit in COMBUF+1
	LDX #1
	STX COMBUF+4		;1 sector
	DEX
	STX COMBUF+5		;0 control byte
	RTS


SCSI_RST			;do a SCSI bus reset
	LDA #1
	STA BBLK_F		;MARK SCSI BUFFER AS INVALID
	LDA #$80		;INITIALIZE SCSI DRIVE
	STA SCSI_R
	LDX #80			;wait 0.03 ms
S_RES1	DEX
	BNE S_RES1
	TXA			;reset pulse
	STA SCSI_R
S_RES2	DEX
	BNE S_RES2
	RTS


S_COM	LDA #20/2		;SET TIMEOUT TO 2 s
	STA TIMER0
	LDA #%00100110
	JSR S_WAIT
	BCS S_COMX
	LDA #1			;SELECT DRIVE 1
	STA SCSI_D
	STA SCSI_SE
	LDY #0-6		;to enable BNE S_COM1
S_COM1	LDA #%11101110
	JSR S_WAIT
	BCS S_COMX
	LDA COMBUF-256+6,Y
	STA SCSI_D
	INY
	BNE S_COM1
S_COMX	RTS


GET_SM	LDA #%11101111		;GET STATUS AND MESSAGE
	JSR S_WAIT
	BCS GET_SMX
	LDY SCSI_D
	LDA #%11111111
	JSR S_WAIT
	BCS GET_SMX
	LDA SCSI_D
	BNE GET_SM1
	TYA
	BNE GET_SM1
	CLC
GET_SMX	RTS

GET_SM1	JSR SCSI_RST		;reset SCSI bus
	RTS


GETBYT	LDA #%01101111		;get byte from SCSI
	JSR S_WAIT
	BCS GETBYTX
	LDA SCSI_D
GETBYTX	RTS


S_WAIT	LDX TIMER0
	BEQ S_WAIT1
	CMP SCSI_ST
	BNE S_WAIT
	CLC
	RTS

S_WAIT1	LDA #SCSI_TIMEOUT
	SEC
	RTS
	

; *****************************************************************************
; SD-Card routines
; *****************************************************************************

; *****************************************************************************
; Set LBA to buffered block
; LBA = Image Base + B_BLK
; Input:  B_BLK [LO:HI] is 24 Bit buffered block Address
; Output: LBA [LO:HI] is 32 Bit LBA adress in image
; *****************************************************************************

SETLBA	LDA DRIVE	;set LBA to B_BLK
	AND #%00000011
	ASL A		;Drive*4 = pnt for base LBA
	ASL A
	TAX
	LDA #0
	PHA		;push MSB=0
	LDY #2
SETLBA1	LDA B_BLK,Y	;push B_BLK to stack
	PHA
	DEY
	BPL SETLBA1
	LDY #0-4	;Y=$FC to enable BNE SETLBA0
	CLC
SETLBA0	PLA	 	;set LBA = B_BLK + IMGTAB
	ADC IMGTAB,X
	STA LBA-256+4,Y	;due to offset in Y
	INX
	INY
	BNE SETLBA0
	LDX #4		;load LBA to COMBUF
	LDY #$00
	LDA SD_TYPE
	BNE BLK_MODE	;depending on SD type
	CLC
	TYA
        STA COMBUF,X
	DEX
BIT_MODE LDA LBA,Y
	ROL A
        STA COMBUF,X
	INY
	DEX
	BNE BIT_MODE
	RTS

BLK_MODE LDA LBA,Y
	STA COMBUF,X
	INY
	DEX
	BNE BLK_MODE
	RTS

; *****************************************************************************
; Check Error
; *****************************************************************************

SD_RESP_ERR AND #$01		;is card in idle mode?
	BEQ SD_DISK_RO		;no, print error
	JSR SD_INIT		;yes, maybe card changed, reset
	BCC SD_DISK_CHNG
SD_NO_DISK RTS

SD_DISK_RO LDA #WRT_PROT
	SEC
	RTS

SD_DISK_CHNG LDA #SD_CHNG
	SEC
	RTS

; *****************************************************************************
; Read Single Data Block
; *****************************************************************************
SD_RD	JSR SETLBA		;read sector from SD to BLKBUF
	LDA #CMD17		;send CMD17 (blk read)
	JSR SD_SEND_CMD
	BNE SD_RESP_ERR		;response <> 0 check error type
	LDA #DATA_TOKEN	
	LDX #0
	STX T2CL		;set timer to 4 us/bit
	LDA #DATA_TOKEN
	JSR SD_WAIT_TOKEN	;wait for data token $FE
	CMP #$FE		;is card ready for block read?
	CLC
	BNE SD_RD_END		;did not receive data token, exit with C = 0
	LDY #$00		;initialize byte counter
SD_RD_BLK0 JSR	SPI_BIN		;read a byte
	STA BLKBUF,Y		;and store it into BLKBUF
	INY			;increment destination pointer
	BNE SD_RD_BLK0		;pointer overflow? No, read next byte
SD_RD_BLK1 JSR	SPI_BIN		;read a byte
	STA BLKBUF+256,Y	;and store it into BLKBUF+256
	INY			;increment destination pointer
	BNE SD_RD_BLK1		;pointer overflow? No, read next byte
	LDX #$02		;initialize counter
	JSR XSPI_BIN		;read 3 more bytes (CRC H, CRC L, dummy)
	LDX #$05
	STX T2CL		;set timer to 14 us/bit (X=5)
	CLC
	RTS			;yes, all data read, set C = 0

SD_RD_END SEC
	RTS

; *****************************************************************************
; Write Single Data Block
; *****************************************************************************
SD_WR	JSR SETLBA		;write sector from BLKBUF to SD
	LDA #CMD24		;send CMD24 (blk write)
	JSR SD_SEND_CMD
	BNE SD_RESP_ERR		;response <> 0 check error type
	LDA #DATA_TOKEN
	JSR SPI_OUT		;write data token
	LDY #$00		;initialize byte counter
SD_WR_BLK0 LDA BLKBUF,Y		;read next byte from buffer
	JSR SPI_OUT		;and write it to the card
	INY			;increment source pointer
	BNE SD_WR_BLK0		;pointer overflow? No, write next byte
SD_WR_BLK1 LDA BLKBUF+256,Y	;read next page from buffer
	JSR SPI_OUT		;and write it to the card
	INY			;increment source pointer
	BNE SD_WR_BLK1		;pointer overflow? No, write next byte
	LDX #3
	JSR XSPI_BIN		;read 4 more bytes (CRC H/L, dummy, response byte)
	PHA
SD_WR_BUSY JSR SPI_BIN		;read next byte
	BEQ SD_WR_BUSY		;check if busy ($00)
	PLA
	AND #$1F		;mask result bits
	CMP #$05		;data accepted?
	SEC
	BNE SD_WR_END		;no, exit with C = 1
	CLC			;yes, exit with C = 0
SD_WR_END RTS

; *****************************************************************************
; Initialize SD-Card
; Output: C = 0 Init OK, C = 1 Error
; *****************************************************************************

SD_INIT		LDA #$00
		STA SD_TYPE
		JSR SD_RESET	 ; reset SD-Card
		CMP #$01	 ; SD-Card present?
		BNE SDC_NOT_FOUND ; invalid response, no usable card found
		JSR SD_GET_VERS	 ; get SD-Card version
		CMP #$05	 ; seems to be a version 1 card
		BEQ INIT_SD0	 ; so just try to initialize it
		CMP #$AA	 ; version 2 cards should response with $(01)AA
		BNE SDC_NOT_FOUND ; invalid response, no usable card found
		LDA #$40	 ; try ACMD41($40000000) init (SD Ver. 2+)
		BNE INIT_SD1

INIT_SD0 	LDA #$00	 ; try ACMD41($00000000) init (SD Ver. 1)
INIT_SD1 	JSR SD_CLEAR_CMD ; prepare for new command
		STA COMBUF+1
INIT_SD2 	LDA #CMD55	 ; send prefix CMD55 (application cmd)
		JSR SD_SEND_CMD
		CMP #$01
		BNE SDC_NOT_FOUND ; invalid response, no usable card found
		LDA #ACMD41	 ; send ACMD41 (initialize)
		JSR SD_SEND_CMD
		BEQ INIT_SD3	 ; response = 0 means card waked up,
		CMP #$01	 ; card still idle?
		BEQ INIT_SD2	 ; yes, try again
		BNE SDC_NOT_FOUND ; no, invalid response, no usable card found
INIT_SD3 	LDA COMBUF+1	 ; Ver. 2+ Card?
		BEQ INIT_SD4	 ; no, just set block size
		JSR SD_CLEAR_CMD ; prepare for new command
		LDA #CMD58	 ; send CMD58 (get OCR)
		JSR SD_SEND_CMD
		BNE SDC_NOT_FOUND ; invalid response, no usable card found
		JSR SD_WAIT_RESP3 ; wait for OCR response
		LDA COMBUF+1	 ; Test Bit 30
		AND #$40	 ; 1 if SDHC/SDXC card, 0 else
		STA SD_TYPE	 ; set type $00 Byte mode, $40 LBA mode
INIT_SD4 	JSR SD_CLEAR_CMD ; prepare for new command
		LDA #$02	 ; set blocksize to 512 byte
		STA COMBUF+3
		LDA #CMD16	; send CMD16 (set block size)
		JSR SD_SEND_CMD
		BNE SDC_NOT_FOUND ; invalid response, no usable card found
		CLC		; everything gone well
		RTS

SDC_NOT_FOUND
		LDA #NO_SD
		SEC		; something went wrong
		RTS

; *****************************************************************************
; Get SD-Card Version
;  ****************************************************************************
SD_GET_VERS 
		LDA #$01	; set parameter byte 1
 		STA COMBUF+3
 		LDA #$AA	; set parameter byte 0
 		STA COMBUF+4
		LDA #$87	; set crc
		STA COMBUF+5
		LDA #CMD8	; send CMD8($000001AA) (get version)
		JSR SD_SEND_CMD	; response should be $01
		CMP #$01	; SD-Card present?
		BNE END_GET_VERS ; no, exit with result <> $01
				; yes, fall through to sd_wait_resp

; *****************************************************************************
; Wait for a 32 Bit Command R3 Response from SD-Card
; *****************************************************************************

SD_WAIT_RESP3	LDY #$00
READ_RESP3	JSR SD_WAIT_RESP ;yes, receive 4 response bytes
		STA COMBUF+1,Y	;store response bytes in COMBUF+1..4
		INY
		CPY #$04
		BNE READ_RESP3
END_GET_VERS	RTS

; *****************************************************************************
; Clear SD-Card Command Parameters
; *****************************************************************************

SD_CLEAR_CMD	LDA #$00
		LDY #$04	; 4 parameter bytes to clear
NEXT_PARAM	STA COMBUF,Y	; clear parameter byte
		DEY
		BNE NEXT_PARAM	; more to clear?
		LDA #$FF
		STA COMBUF+5	; no, finally set CRC byte to $FF
		RTS

; *****************************************************************************
; Send Command to SD-Card
; Input: A = Command Index
; *****************************************************************************

SD_SEND_CMD	STA COMBUF
		LDA #$01
		STA DRB		;CS = 0, MOSI = 1
		JSR SPI_BIN	; send one dummy
		LDY #$00
SEND_BYTE	LDA COMBUF,Y	; get one command byte
		JSR SPI_OUT	; and send it
		INY
		CPY #$06	; all 6 cmd bytes send?
		BNE SEND_BYTE	; no, send more bytes
				; yes, fall through to sd_wait_resp

; *****************************************************************************
; Wait for a 8 Bit Command R1 Response from SD-Card
; Output: A = Response Byte
; *****************************************************************************

SD_WAIT_RESP	LDX #$08	; wait for max 8 cycles
READ_RESP1	JSR SPI_BIN	; receive data
		CMP #$FF	; is it a $FF?
		BNE RESPONSE	; no, card did respond
		DEX		; yes, try again
		BNE READ_RESP1	; check for timeout
RESPONSE	CMP #0		;set Z flag
		RTS

; *****************************************************************************
; Wait for a Special Token Response from SD-Card
; Input:  A = Token Byte
; Output: A = Response Byte
; *****************************************************************************

SD_WAIT_TOKEN	STA MODE	; store token into TEMP variable
		LDY #$FF	; load low byte of time out counter
		LDX #$0A	; load high byte of time out counter
WAIT_RESP	JSR SPI_BIN	; read byte from SPI
		DEY		; decrement wait counter
		BNE WAIT_RESP0
		DEX
		BEQ WAIT_RESP_END ; wait counter is 0 -> time out
WAIT_RESP0	CMP MODE	; did we read the token we are waiting for?
		BNE WAIT_RESP	; no, read next byte
WAIT_RESP_END	RTS

; *****************************************************************************

SD_RESET	JSR SD_CLEAR_CMD ;clear command parameters
		LDA #$95
		STA COMBUF+5	; and set crc to $95 for CMD0
		JSR SPI_INIT	; CS=1; MOSI&CLK=1;MISO is input
		JSR SD_PREPARE	; send dummy sequence to SD-Card
		BNE RESET_SDC	; is MISO line high?
		LDA #CMD0	; no, send CMD0 (reset) to SD-Card
		JSR SD_SEND_CMD
		JSR SD_PREPARE	; send init dummy sequence again
		BEQ END_SD_RESET ; MISO still low? Exit with A = $FF
RESET_SDC 	LDA #CMD0	; send CMD0 (reset) to SD-Card
		JMP SD_SEND_CMD	; response should be $01

END_SD_RESET
		LDA #$FF	; reset failed
		RTS

; **** Prepare SD-Card for Communication **************************************
;
; *****************************************************************************

SD_PREPARE LDX #10		; first send 80 clocks to SD-Card
XSPI_BIN JSR SPI_BIN	; send 8 clock cycles
	DEX
	BPL XSPI_BIN	; send more clock cycles
	CMP #0		; set Z flag
SD_END	RTS

;--- SPI I/O routines ---

SPI_INIT LDX #%00000011	;init VIA for SPI
	STX DRB		;CS = 1, MOSI = 1
	STX DDRB	;Bit 0  MOSI; Bit 1  CS
	INX		;SR mode 010 = $04
	STX ACR
	LDA #0
	STA T2CH
	INX		;Timer2 = 14 us = $05
	STX T2CL
	STA SR
	LDX #24		;wait 100(500) ms for (very) slow SD cards
SPI_IN1	JSR SPI_IN2	;delay 4.3 ms
	SBC #1
	BCS SPI_IN1
	DEX
	BNE SPI_IN1
SPI_IN2	RTS


SPI_BIN	LDA #$04	;read byte from SPI
SPIBIN1	BIT IFR		;test SR bit
	BEQ SPIBIN1
	LDA SR		;read byte and clear SR bit
	RTS


SPI_OUT	TAX		;write A to SPI
	STY SPI_Y	;save Y
SPIOUT1	LDA IFR		;read byte from SPI
	AND #$04	;test SR bit
	BEQ SPIOUT1
	STA IFR
	TXA
	LDX #0
	LDY #%00000001	;byte to write a %1
	SEI		;no IRQs from here
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
	CLI
	LDY SPI_Y	;restore Y
	RTS

SPI_Y	DB 0		;buffer for Y

DMA	

DIRBF	=  DMA+256

BLKBUF	=  DIRBF+256


	END
