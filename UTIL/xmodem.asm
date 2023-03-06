;XMODEM - reads and writes XMODEM-Filetransfer
;code adapted from Daryl Rictor,www.6502.org/source/io/xmodem/xmodem.htm 

;(C) D. LAUSBERG	2021
;V1.0	25.01.21	receive only
;V1.1	08.05.21	8-Bit I/O
;V1.2	20.03.22	I/O more stable
;V1.3	23.12.22	correct file length
;V2.0	27.12.22	send & receive
;V2.1	02.01.23	some improvements & cleanup
;V2.2	22.01.23	Speed improvement, preset to 9600 Baud

VERSION	= $22		;VERSION NUMBER
;================================================

;KONSTANTEN

EOT	= $00
CLS	= $0C
SP	= $20
DEL	= $7F

; XMODEM Control Character Constants
SOH	= $01		; start block
XEOT	= $04		; end of text marker
ACK	= $06		; good block acknowledged
NAK	= $15		; bad block acknowledged
CAN	= $18		; cancel (not standard, not supported)
CR	= $0d		; carriage return
LF	= $0a		; line feed
ESC	= $1b		; ESC to exit

;ERROR CODES

EOF	= $D7
NOTFND	= $DD
XModem_terminated = $80
FName_miss = $83		;Filename missing
NO_ARG	= $84
FILE_TOO_LONG = $85
X_BLK	= $87		;wrong block nr
FILNW	= $88		;no File owerwrite

; BDOS COMMANDS

CONIN	= $01
CONOUT	= $02
STROUT	= $09
OPEN	= $0F
CLOSE	= $10
READ	= $14
WRITE	= $15
CREATE	= $16

;SYSTEMADRESSEN

;6532 ADDRESSES

SCRATCH	= $F600

PAD	= $F680
PBD	= $F682

TPA	= $0200

;Page 00 cells 
RBUFF	= 0 		;132 byte receive buffer (page aligned)
ZP	= $86		;start of zero page RAM cells
PAGES	= ZP+0
LENGTH	= ZP+1
ERRNO	= ZP+2		;ERROR NR
PNT	= ZP+3
SWITCH	= ZP+4
CRC	= ZP+5		; CRC lo byte  (two byte variable)
PTR	= ZP+7		; data pointer (two byte variable)
blkno	= ZP+9		; block number 
YBUF	= ZP+10		; Y buffer
;	= ZP+11		; 
STR	= ZP+12		; pointer for STR_OUT
errcnt	= ZP+14		; error counter for resend
lastblk	= ZP+15		;flag for last block to be sent
CHR	= ZP+16		;chr from CON
CONS	= ZP+17		;half bit time / bit time
SPEED	= ZP+19		;SPEED flag bit 7 1 38400 Baud
			;               6 1 19200 Baud

CCPV	= $DE
BDOS	= $F0
FCB1	= $F6
DMA	= $FE


	ORG TPA

	JMP START

SPEED_P	DB $00		;set to 9600 -1200 baud
CONS_P	DB $03		;default 9600 baud

START	LDA #STARTM
	LDY #STARTM/256
	JSR PRTSTR
	JSR INIT	;initialize vars, check for switch
	BCS STERR
	BIT SWITCH
	BPL XMODSEND
	JSR FOPEN	;receive file
	BCS STERR
	JSR XMODEM
	BCS STERR
	SEC
	LDA PTR
	SBC #LOMEM
	STA PTR	
	LDA PTR+1
	SBC #LOMEM/256
	STA LENGTH
	LDA PTR
	BEQ START2
	INC LENGTH
START2	LDA #0
	STA PAGES
	JSR SETDMA

START3	LDX #WRITE
	JSR BDOS
	BCS STERR
	JSR INCDMA
	BCC START3
	LDX #CLOSE
	JSR BDOS
	BCS STERR
START4	LDA #SUCCESS
	LDY #SUCCESS/256
	JSR STR_OUT
BOOT	LDA #$20	;deblock chr input
	STA SCRATCH+$64
	LDX #0
	JMP BDOS	

STERR1	LDA #FName_miss	;Filename missing
STERR	JSR ERROR	;XMODEM terminated
	LDA #BREAK
	LDY #BREAK/256
	JSR STR_OUT
	JMP BOOT


XMODSEND		;XMODEM send file
	LDX #OPEN
	JSR BDOS
	BCS STERR
	JSR SETDMA	;DMA = LOMEM
XMS1	LDX #READ	;read file to memory
	JSR BDOS
	BCS XMS2
	INC DMA+1
	CMP BDOS+2
	BCC XMS1
	LDA #FILE_TOO_LONG
	RTS

XMS2	CMP #EOF
	BNE STERR	;if EOF reached
	JSR XMSEND	;transmit file
	BCC START4
	BCS STERR

;=======================================

XModem	LDA #LOMEM	;set PTR to LOMEM for file storage
	STA PTR		; store file to PTR
	LDA #LOMEM/256
	STA PTR+1
	JSR INITCOM
	LDA #1		; block nr = 1
	STA BLKNO
	LDA #20		; max. 20 errors
	STA ERRCNT
	SEI		; no IRQ from here	
StartCrc
	LDA #'C		; C STArt transmission CRC option
	JSR D_CHROUT	; send it
	JSR D_CHRIN	; wait for input
	BCC GotByte	; byte received, process it
	BCS StartCrc	; resend 'C

StartBlk
	JSR D_CHROUT	; Send ACK or NAK
	JSR D_CHRIN	; get first byte of block
GotByte	CMP #SOH	; Start of block?
	BEQ BegBlk
	CMP #CAN	; quitting?
        BEQ BADBLK
	CMP #XEOT	; XMODEN EOT Byte
	BNE BadCrc	; Not SOH or EOT, so flush buffer & send NAK
	LDA #ACK	; last block, send ACK and exit.
	JSR PUT_CHR	; END OF CRITICAL PHASE
FLUSH	JSR D_CHRIN	; flush receive buffer
	BCC FLUSH
	CLC
	RTS

BegBlk	LDX #$00
GetBlk	JSR D_CHRIN	; get next character
			; if no chr -> save $00 -> CRC errror
	STA RBUFF,X	; good char, save it in the rcv buffer
	INX		; inc buffer pointer	
	CPX #$84	; <01> <FE> <128 bytes> <CRCH> <CRCL>
	BNE GetBlk	; get 130 characters
	LDX #$00
	LDA RBUFF,X	; get block # from buffer
	CMP BLKNO	; compare to expected block #	
	BNE BadBlk
GoodBlk1
	EOR #$FF	; 1's comp of block #
	INX
	CMP RBUFF,X	; compare with expected 1's comp of block #
	BEQ GoodBlk2 	; matched!
BadBlk	JSR FLUSH	; mismatched - flush buffer
	LDA #CAN	; end transfer
	JSR PUT_CHR	; END OF CRITICAL PHASE
	lda #X_BLK	; unexpected block # - fatal error
	SEC
	RTS
	
GoodBlk2
	JSR CalcCRC
	LDA RBUFF,X	; get hi CRC from buffer
	CMP CRC+1	; compare to calculated hi CRC
	BNE BadCrc	; bad CRC, send NAK
	INX
	LDA RBUFF,X	; get lo CRC from buffer
	CMP CRC		; compare to calculated lo CRC
	BEQ CopyBlk	; good CRC
BadCrc	JSR FLUSH	; flush the input port
	DEC ERRCNT
	BEQ BADBLK	;too manx errors
	LDA #NAK
	BNE StartBlk	; Start over, get the block again		
	
CopyBlk	LDX #$02	; data STARTS at pos $02
	LDY #$00	; set offset to zero
CopyBlk3
	LDA RBUFF,X	; get data byte from buffer
	STA (PTR),Y	; save to target
	INY
CopyBlk4
	INX		; point to next data byte
	CPX #$82	; is it the last byte
	BNE CopyBlk3	; no, get the next one
IncBlk	INC BLKNO	; done.  Inc the block #
	CLC
	LDA PTR		; PTR = PTR +$80
	ADC #$80
	STA PTR
	BCC INCBLK1
	INC PTR+1
INCBLK1	LDA #ACK	; send ACK
	JMP StartBlk	; get next block


;^^^^^^^^^^^^^^^^^^^^^^ Start of XModem send ^^^^^^^^^^^^^^^^^^^^^^

;
; Enter this routine with the ending address stored in DMA
;

XMSEND	LDX DMA+1
	LDY DMA		;set PTR = DMA - 1
	BNE XMSEND1
	DEX
XMSEND1	DEY
	STY PTR		;PTR is end of file
	STX PTR+1
	JSR SETDMA	;reset DMA = beginning adr
	JSR INITCOM
	JSR FLUSH
	SEI		;no IRQ from here
Wait4CRC
	LDA #'.		;while waiting
	JSR D_CHROUT	;print .
	jsr D_CHRIN
	BCS Wait4CRC	;wait for something to come in...
	cmp #'C		;is it the "C" to start a CRC xfer?
	beq LdBuff0	;yes
	cmp #CAN	;is it a cancel? <Esc> Key
	bne WAIT4CRC	; No, wait for another character
	jmp PrtAbort	; Print abort msg and exit

LdBuffer
	inc blkno	; inc block counter
	lda Lastblk	; Was the last block sent?
	beq LdBuff0	; no, send the next one	
	LDA #XEOT	; send EOT
	JSR PUT_CHR	; END OF CRITICAL PHASE
	CLC
	RTS		; yes, we're done

LdBuff0	ldx #$02	; init pointers
	ldy #$00
	lda Blkno
	sta RBUFF	; save in 1st byte of buffer
	eor #$FF
	sta RBUFF+1	; save 1's comp of blkno next
LdBuff1	lda (DMA),Y	; save 128 bytes of data
	sta RBUFF,X
LdBuff2	sec
	lda PTR
	sbc DMA		; Are we at the last address?
	bne LdBuff4	; no, inc pointer and continue
	lda PTR+1
	sbc DMA+1
	bne LdBuff4
	inc LastBlk	; Yes, Set last byte flag
LdBuff3	inx
	cpx #$82	; Are we at the end of the 128 byte block?
	beq SCalcCRC	; Yes, calc CRC
	lda #$00	; Fill rest of 128 bytes with $00
	sta RBUFF,X
	beq LdBuff3	; Branch always

LdBuff4	inc DMA		; Inc address pointer
	bne LdBuff5
	inc DMA+1
LdBuff5	inx
	cpx #$82	; last byte in block?
	bne LdBuff1	; no, get the next
SCalcCRC
	jsr CalcCRC
	lda crc+1	; save Hi byte of CRC to buffer
	sta RBUFF,X
	INX
	lda crc		; save lo byte of CRC to buffer
	sta RBUFF,X	
Resend	ldx #$00
	lda #SOH
	jsr D_CHROUT	; send SOH
SendBlk	lda RBUFF,X	; Send 132 bytes in buffer to the console
	jsr D_CHROUT
	inx
	cpx #$84	; last byte?
	bne SendBlk	; no, get next
	jsr D_CHRIN	; Wait for Ack/Nack
	BCS Seterror	; No chr received after 3 seconds, resend
	cmp #ACK		; Chr received... is it:
	beq LdBuffer	; ACK, send next block
	cmp #NAK
	beq Seterror	; NAK, inc errors and resend
	cmp #CAN
	beq PrtAbort	; Esc pressed to abort
			; fall through to error counter
Seterror
	DEC errcnt	; Inc error counter
	BNE Resend	; are there 10 errors? (Xmodem spec for failure)
			; yes, resend block
PrtAbort
	jsr Flush	; no, too many errors, flush buffer
	CLI		; END OF CRITICAL PHASE
	LDA #XModem_terminated
	SEC
	RTS

	;--- CRC subroutines ---

CalcCRC	LDA #$00	; yes, calculate the CRC for the 128 bytes
	STA CRC
	STA CRC+1
	LDX #$02
CalcCRC1
	LDA RBUFF,X
	EOR CRC+1 	; Quick CRC computation with lookup tables
       	TAY		; updates the two bytes at CRC & CRC+1
       	LDA CRC		; with the byte send in Accu
       	EOR CRCHI,Y
       	STA CRC+1
      	LDA CRCLO,Y
       	STA CRC
	INX
	CPX #$82	; done yet?
	BNE CalcCRC1	; no, get next
	RTS		; x=82 on exit

; non-zero page variables and buffers (page aligned)

MEM
	DS MEM/256+1*256-MEM,$FF		;page alignment

; The following tables are used to calculate the CRC for the 128 bytes
; in the xmodem data blocks. 

; low byte CRC lookup table (should be page aligned)

crclo
 DB $00,$21,$42,$63,$84,$A5,$C6,$E7,$08,$29,$4A,$6B,$8C,$AD,$CE,$EF
 DB $31,$10,$73,$52,$B5,$94,$F7,$D6,$39,$18,$7B,$5A,$BD,$9C,$FF,$DE
 DB $62,$43,$20,$01,$E6,$C7,$A4,$85,$6A,$4B,$28,$09,$EE,$CF,$AC,$8D
 DB $53,$72,$11,$30,$D7,$F6,$95,$B4,$5B,$7A,$19,$38,$DF,$FE,$9D,$BC
 DB $C4,$E5,$86,$A7,$40,$61,$02,$23,$CC,$ED,$8E,$AF,$48,$69,$0A,$2B
 DB $F5,$D4,$B7,$96,$71,$50,$33,$12,$FD,$DC,$BF,$9E,$79,$58,$3B,$1A
 DB $A6,$87,$E4,$C5,$22,$03,$60,$41,$AE,$8F,$EC,$CD,$2A,$0B,$68,$49
 DB $97,$B6,$D5,$F4,$13,$32,$51,$70,$9F,$BE,$DD,$FC,$1B,$3A,$59,$78
 DB $88,$A9,$CA,$EB,$0C,$2D,$4E,$6F,$80,$A1,$C2,$E3,$04,$25,$46,$67
 DB $B9,$98,$FB,$DA,$3D,$1C,$7F,$5E,$B1,$90,$F3,$D2,$35,$14,$77,$56
 DB $EA,$CB,$A8,$89,$6E,$4F,$2C,$0D,$E2,$C3,$A0,$81,$66,$47,$24,$05
 DB $DB,$FA,$99,$B8,$5F,$7E,$1D,$3C,$D3,$F2,$91,$B0,$57,$76,$15,$34
 DB $4C,$6D,$0E,$2F,$C8,$E9,$8A,$AB,$44,$65,$06,$27,$C0,$E1,$82,$A3
 DB $7D,$5C,$3F,$1E,$F9,$D8,$BB,$9A,$75,$54,$37,$16,$F1,$D0,$B3,$92
 DB $2E,$0F,$6C,$4D,$AA,$8B,$E8,$C9,$26,$07,$64,$45,$A2,$83,$E0,$C1
 DB $1F,$3E,$5D,$7C,$9B,$BA,$D9,$F8,$17,$36,$55,$74,$93,$B2,$D1,$F0 

; hi byte CRC lookup table (should be page aligned)
		
crchi
 DB $00,$10,$20,$30,$40,$50,$60,$70,$81,$91,$A1,$B1,$C1,$D1,$E1,$F1
 DB $12,$02,$32,$22,$52,$42,$72,$62,$93,$83,$B3,$A3,$D3,$C3,$F3,$E3
 DB $24,$34,$04,$14,$64,$74,$44,$54,$A5,$B5,$85,$95,$E5,$F5,$C5,$D5
 DB $36,$26,$16,$06,$76,$66,$56,$46,$B7,$A7,$97,$87,$F7,$E7,$D7,$C7
 DB $48,$58,$68,$78,$08,$18,$28,$38,$C9,$D9,$E9,$F9,$89,$99,$A9,$B9
 DB $5A,$4A,$7A,$6A,$1A,$0A,$3A,$2A,$DB,$CB,$FB,$EB,$9B,$8B,$BB,$AB
 DB $6C,$7C,$4C,$5C,$2C,$3C,$0C,$1C,$ED,$FD,$CD,$DD,$AD,$BD,$8D,$9D
 DB $7E,$6E,$5E,$4E,$3E,$2E,$1E,$0E,$FF,$EF,$DF,$CF,$BF,$AF,$9F,$8F
 DB $91,$81,$B1,$A1,$D1,$C1,$F1,$E1,$10,$00,$30,$20,$50,$40,$70,$60
 DB $83,$93,$A3,$B3,$C3,$D3,$E3,$F3,$02,$12,$22,$32,$42,$52,$62,$72
 DB $B5,$A5,$95,$85,$F5,$E5,$D5,$C5,$34,$24,$14,$04,$74,$64,$54,$44
 DB $A7,$B7,$87,$97,$E7,$F7,$C7,$D7,$26,$36,$06,$16,$66,$76,$46,$56
 DB $D9,$C9,$F9,$E9,$99,$89,$B9,$A9,$58,$48,$78,$68,$18,$08,$38,$28
 DB $CB,$DB,$EB,$FB,$8B,$9B,$AB,$BB,$4A,$5A,$6A,$7A,$0A,$1A,$2A,$3A
 DB $FD,$ED,$DD,$CD,$BD,$AD,$9D,$8D,$7C,$6C,$5C,$4C,$3C,$2C,$1C,$0C
 DB $EF,$FF,$CF,$DF,$AF,$BF,$8F,$9F,$6E,$7E,$4E,$5E,$2E,$3E,$0E,$1E

;======================================================================
;  I/O Device Specific Routines
;
;  Two routines are used to communicate with the I/O device.
;
; GET_Byte routine will scan the input port for a character.  It will
; return after timeout with the Carry flag SET ( no character is
; present) or return with the Carry flag CLEAR and the character in the "A"
; register.
;
; PUT_CHR routine will write one byte to the output port. Its assumed that the 
; character was sent upon return from this routine.
;
GET_Byte		;BIOS chr input with IRQ disabled
	SEI
	STY YBUF	;save Y
	JSR D_CHRIN
	LDY YBUF
	CLI
	RTS


D_CHRIN	LDY #$FF	;CHR input if C=0: CHR in A, 8 bit transparent
			;	      C=1  no CHR after 0,4s
	TYA		;setup delay approx. 0.4 sec
	SEC
CHRIN3	SBC #1
	BCC CHRIN4
CHRIN5	BIT PAD		;GET CHR FROM CON to CHR, destroys Y
	BMI CHRIN3	;WAIT FOR START BIT on PA7
	LDY #8		;SET FOR 8 BITS
	BIT SPEED	;DELAY 7 us
	BPL CHRINV	;38400 BAUD?
	NOP		;delay 2 us
CHRIN1	JSR CHRIN6	;DELAY 12 us
	LDA PAD		;GET ONE BIT
	ASL A		;shift BIT in C
	ROR CHR		;ROTATE BIT INTO CHR
	DEY
	BNE CHRIN1
	LDA CHR		;CHR received, stopbit ignored
	CLC
CHRIN6	RTS

CHRIN4	DEY
	SEC
	BNE CHRIN5
	SEC		;no chr received
	RTS

CHRINV	BIT SPEED
	BVC CHRIV0
	LDA (0),Y	; 10 us for 19200 Baud
	SEC
	BCS CHRIV1

CHRIV0	JSR DELHBT	;delay 1/2 bit
CHRIV1	JSR DELBIT	;DELAY 1 bit
	PHA		;+ 9 us
	PLA
	NOP
	LDA PAD		;GET ONE BIT
	ASL A		;shift BIT in C
	ROR CHR		;ROTATE BIT INTO CHR
	DEY
	BNE CHRIV1
	JSR DELBIT
	LDA CHR		;CHR received
	CLC
	RTS


				
PUT_CHR	SEI		;BIOS chr output with IRQ disabled
	STY YBUF	;save Y
	JSR D_CHROUT
	LDY YBUF
	CLI
	RTS


D_CHROUT		;PRINTS CHR in A TO CON, destroys Y
	STA CHR		;8 bit transparent
	LDA PBD
	ORA #$01	;PB0 = 1
	STA PBD
	BIT SPEED
	BPL CHROUTV	;alt speed
	AND #$FE	;START BIT
	STA PBD
	NOP		;DELAY 2 us
	LDY #8		;SEND 8 BITS
CHROUT1	LSR CHR		;SHIFT OUT 1 BIT
	LDA PBD
	BCS CHROUT3	;symmetric timing
	AND #$FE	;'0
	BCC CHROUT2
CHROUT3	ORA #$01	;'1
	BCC CHROUT2
CHROUT2	STA PBD
	NOP
	DEY
	BNE CHROUT1	
	LDA PBD		;SEND 1 STOP BIT
	ORA #$01	;NO PARITY
	STA PBD
	RTS

CHROUTV	AND #$FE	;START BIT
	STA PBD
	LDY #8		;SEND 8 BITS
CHROV1	JSR DELBIT
	LSR CHR		;SHIFT OUT 1 BIT
	LDA PBD
	BCS CHROV3	;symmetric timing
	AND #$FE	;'0
	BCC CHROV2
CHROV3	ORA #$01	;'1
	BCC CHROV2
CHROV2	STA PBD
	DEY
	BNE CHROV1
	JSR DELBIT	
	LDA PBD		;SEND 1 STOP BIT
	ORA #$01	;NO PARITY
	STA PBD
	JSR DELBIT
	RTS


STR_OUT	STA STR			;STRING OUTPUT 8 Bit
	STY STR+1		;SET POINTER
	LDY #0			;SET FOR FIRST CHR
STROUT1	LDA (STR),Y
	CMP #EOT		;LAST CHR?
	BEQ STROUTX
	JSR PUT_CHR		;PRINT CHR mode 8,n,1
	INY
	BNE STROUT1
	INC STR+1
	JMP STROUT1

STROUTX	CLC
	RTS

;-------  TERMINAL I/O-ROUTINES  -------
; separate timing loops for better accuracy

DELBIT	LDA CONS+1	;Set up for Bit Time
	BIT SPEED
	BVC DELBIT1
	PHA		; 7 us for 19200 Baud
	PLA
	RTS

DELHBT	LDA CONS	;SET UP FOR HALF BIT TIME
DELBIT1	SEC
DELBIT2	SBC #1
	BCS DELBIT2
	RTS


SETBTT	ASL A		;set bit times 0=1200 / 1=2400 / 2=4800
	TAX			       3=9600 / 4=14400 Baud
	LDA BITTIME,X
	STA CONS
	INX
	LDA BITTIME,X
	STA CONS+1
	RTS

BITTIME	DB $4E,$9D	; 1200 Baud
	DB $23,$49	; 2400 Baud
	DB $0E,$20	; 4800 Baud	
	DB $04,$0B	; 9600 Baud
	DB $01,$04	;14400 Baud


INITCOM	LDA SPEED_P	;set 38400 Baud
	STA SPEED
	LDA CONS_P	;default 9600 Baud
	JSR SETBTT
	RTS

;==========================================

INIT	ldy	#$00
	sty	errcnt		; error counter set to 0
	sty	lastblk		; set flag to false
	INY
	sty	blkno		; set block # to 1
	LDA (FCB1),Y
	CMP #SP			;check for filename
	BEQ INIT9
	LDY #0
	LDA (DMA),Y		;CHECK FOR SWITCH
	CMP #$01
	BEQ INIT8
	STA PNT
INIT3	INY
	CPY PNT
	BCS INIT8
	LDA (DMA),Y
	CMP #'/			;SWITCH?
	BNE INIT3
	INY
	CPY PNT
	BCS INIT8
	LDA (DMA),Y		;GET SWITCH
	LDX #0
INIT4	CMP SWTAB,X
	BEQ INIT5
	INX
	INX
	CPX #SWTABX-SWTAB
	BCC INIT4

INIT8	LDA #NO_ARG		;no valid argument
	SEC
	RTS

INIT5	INX
	LDA SWTAB,X
	STA SWITCH
	CLC
	RTS

INIT9	LDA #FName_miss		;no filename
	SEC
	RTS


FOPEN	LDX #OPEN		;open file to be transferred
	JSR BDOS
	BCS FOPEN1
	LDA #ASK		;if file exists, ask for overwrite
	LDY #ASK/256
	JSR PRTSTR
	LDX #CONIN
	JSR BDOS
	CMP #'y
	BEQ FOPENX
	CMP #'Y
	BEQ FOPENX
	LDA #FILNW
FOPEN2	SEC
	RTS

FOPEN1	CMP #NOTFND		;if file doesn't exist
	BNE FOPEN2
	LDX #CREATE		;create it
	JSR BDOS
	RTS

FOPENX	JSR CRLF
	CLC
	RTS


SETDMA	LDA #LOMEM		;DMA = LOMEM
	STA DMA
	LDA #LOMEM/256
	STA DMA+1
	RTS


INCDMA	INC DMA+1
	INC PAGES
	LDA PAGES
	CMP LENGTH
	RTS

;==========================================

ERROR	STA ERRNO	;ERROR ROUTINE
	JSR CRLF
	LDA #ERRTAB	;SET CCPV
	STA CCPV
	LDA #ERRTAB/256
	STA CCPV+1
	LDY #0		;CHECK ERROR CODE
	LDA (CCPV),Y
ERROR1	BEQ ERROR4	;END OF ERROR ROUTINE
	CMP ERRNO
	BEQ ERROR3
ERROR2	JSR INCCPV	;SKIP ERROR MESSAGE
	BNE ERROR2
	JSR INCCPV
	JMP ERROR1

ERROR3	JSR INCCPV
	PHA
	JSR ERRTYP	;PRINT ERROR TYPE
	JSR INCCPV
	LDA CCPV	;PRINT ERROR MESSAGE
	LDY CCPV+1
	JSR PRTSTR
	PLA
	BPL ERROR5
	JSR ASKRTY	;ASK FOR RETRY
	LDA ERRNO
	RTS

ERROR4	JSR UETYP	;UNKNOWN ERROR
ERROR5	SEC
	RTS


ERRTYP	LDY #0		;PRINT ERROR TYPE
ERRTY1	LSR A		;SHIFT SOURCE BIT IN C
	BCS ERRTY2
	INY		;X=X+5
	INY
	INY
	INY
	INY
	BNE ERRTY1

ERRTY2	LDA ETYPTB,Y
	BEQ UETYP	;LAST CHR?
	JSR PRTCHR
	INY
	BNE ERRTY2

UETYP	LDA #ERRM1
	LDY #ERRM1/256
	JSR PRTSTR
	LDA ERRNO
	JSR PRTHEX

CRLF	LDA #CRLFM	;PRINTS A NEWLINE
	LDY #CRLFM/256

PRTSTR	LDX #STROUT
	JMP BDOS


PRT2SP	JSR PRTSP

PRTSP	LDA #SP

PRTCHR	LDX #CONOUT	;PRINTS A CHR
	JMP BDOS

INCCPV	INC CCPV	;INC CCPV
	BNE INCCP1
	INC CCPV+1
INCCP1	LDA (CCPV),Y
	RTS


UPCASE	CMP #'a
	BCC UPCASX
	CMP #$7F
	BCS UPCASX
	AND #$5F
UPCASX	RTS


ASKRTY	LDA #RTYMES	;RETRY?
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



PRTHEX	PHA		;PRINTS A HEX NUMBER
	LSR A
	LSR A
	LSR A
	LSR A
	JSR PRTNIB	;PRINT NIBBLE
	PLA
	AND #$0F

PRTNIB	CMP #$0A
	BCC PRTNI1
	ADC #6
PRTNI1	ADC #$30
	JSR PRTCHR
	RTS

;-----	TABLES  --------------------------------

SWTAB	DB 'R',$80,'r',$80,'S',$00,'s',$00
SWTABX

;======	MESSAGES	======

ERRM1	DB ' Error $',EOT
RTYMES	DB CR,LF,'Retry	(Y/N) ? ',EOT

ETYPTB	DB 'BIOS',$00,'BDOS',$00,'CCP',$00,$00
	DB 'RSX',$00,$00,'USER',$00

ERRTAB	DB $FD,$81,'Drive not ready',EOT
	DB $FC,$81,'Disk write protected',EOT
	DB $FA,$81,'Lost data on writing track',EOT
	DB $F9,$01,'Invalid Drive',EOT
	DB $DE,$82,'Directory full',EOT
	DB $DD,$82,'File not found',EOT
	DB $DC,$02,'File exists',EOT
	DB $D9,$02,'Invalid FCB',EOT
	DB $D8,$82,'Disk full',EOT
	DB $D6,$82,'File is R/O',EOT

	DB $80,$10,'Transmission failed',EOT
	DB $81,$10,'Missing Parameter',EOT
	DB $82,$10,'Illegal Parameter',EOT
	DB FName_miss,$10,'Filename missing',EOT
	DB NO_ARG,$10,'no switch set',EOT
	DB FILE_TOO_LONG,$10,'File too long',EOT
	DB X_BLK,$10,'wrong blk nr',EOT
	DB FILNW,$10,'no file overwrite',EOT

	DB $00

STARTM	DB CLS
	DB 'XMODEM V',VERSION/16+$30,'.',VERSION*$1000/$1000+$30
	DB '			(c) D. Lausberg	2021'
	DB CR,LF,CR,LF,'Usage xmodem filename.ext /[r(eceive)/s(end)]
	DB CR,LF,CR,LF,EOT

ASK	DB '  File exists, overwrite? (Y/N) ',EOT

SUCCESS	DB CR,LF,'transfer successful',EOT

BREAK	DB CR,LF,'transfer terminated',EOT

CRLFM	DB CR,LF,EOT

LOMEM

	END