;XMODEM - liest ein XMODEM-File

;(C) D. LAUSBERG	2021
;V1.0	25.01.21	receive only
;V1.1	08.05.21	8-Bit I/O
;V1.2	20.03.22	I/O more stable
;V1.3	23.12.22	correct file length
;V2.0	27.12.22	send & receive

VERSION	= $20		;VERSION NUMBER
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
X_CAN	= $86
X_BLK	= $87		;wrong block nr
FILNW	= $88		;no File owerwrite

; BDOS COMMANDS

CONIN	= $01
CONOUT	= $02
CONIO	= $06
STROUT	= $09
OPEN	= $0F
CLOSE	= $10
READ	= $14
WRITE	= $15
CREATE	= $16

;SYSTEMADRESSEN

;6532 ADDRESSES

SCRATCH	= $F600

BITTIME	= SCRATCH+$58		;bit delay for COM
HBTTIME	= SCRATCH+$59		;half bit delay for COM

PAD	= $F680
PBD	= $F682

TPA	= $0200

;Page 00 cells 

PAGES	= 0
LENGTH	= 1
ERRNO	= 2		;ERROR NR
PNT	= 3
SWITCH	= 4
CRC	= 5		; CRC lo byte  (two byte variable)
PTR	= 7		; data pointer (two byte variable)
blkno	= 9		; block number 
retry	= 10		; retry counter 
retry2	= 11		; 2nd counter
STR	= 12		; pointer for STR_OUT
errcnt	= 14		; error counter for resend
lastblk	= 15		;flag for last block to be sent

CCPV	= $DE
BDOS	= $F0
FCB1	= $F6
DMA	= $FE

STATUS	= $F3		;STATUS-FLAG  ^P/^S/./. BAT/DIR/FCB/DEL


	ORG TPA

START	LDA #STARTM
	LDY #STARTM/256
	JSR PRTSTR
	JSR INIT		;initialize vars, check for switch
	BCS STERR
	BIT SWITCH
	BPL XMODSEND
	JSR FOPEN		;send file
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
BOOT	LDA #$20		;deblock chr input
	STA SCRATCH+$64
	LDX #0
	JMP BDOS	

STERR1	LDA #FName_miss		;Filename missing
STERR	JSR ERROR		;XMODEM terminated
	LDA #BREAK
	LDY #BREAK/256
	JSR STR_OUT
	JMP BOOT


XMODSEND			;XMODEM send file
	LDX #OPEN
	JSR BDOS
	BCS STERR
	LDA #0
	STA LENGTH
	JSR SETDMA
XMS1	LDX #READ
	JSR BDOS
	BCS XMS2
	INC LENGTH
	INC DMA+1
	CMP BDOS+2
	BCC XMS1
	LDA #FILE_TOO_LONG
	RTS

XMS2	CMP #EOF
	BNE STERR
	JSR XMSEND
	BCC START4
	BCS STERR

;=======================================
INIT	LDY #1
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
	SEC
	RTS

FOPENX	JSR CRLF
	CLC
	RTS

FOPEN1	CMP #NOTFND
	BNE FOPEN2
	LDX #CREATE
	JSR BDOS
	RTS

FOPEN2	SEC
	RTS

SETDMA	LDA #LOMEM
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

XModem		lda	#$01		;receive file
		sta	blkno		; set block # to 1
		LDA #LOMEM		;set PTR to LOMEM for file storage
		STA PTR
		LDA #LOMEM/256
		STA PTR+1
		JSR INITCOM
StartCrc	ldy	#$FF	
		sty	retry2		; set loop counter for ~60 sec delay
		iny
               	sty	crc
		sty	crc+1		; init CRC value to $0000	
		lda	#'C		; C start transmission CRC option
		jsr	PUT_CHR		; send it
		jsr	GET_BYTE	; wait for input
               	bcc	GotByte		; byte received, process it
		bcs	StartCrc	; resend 'C

StartBlk	lda	#$90		; 
		sta	retry2		; set loop counter for ~10 sec delay
		lda	#$00		;
		sta	crc		;
		sta	crc+1		; init CRC value	
		jsr	GET_BYTE	; get first byte of block
		bcs	StartBlk	; timed out, keep waiting...

GotByte	       	cmp	#SOH		; start of block?
		beq	BegBlk		; yes

		cmp	#CAN		; quitting?
                bne	GotByte1	; no
		lda	#X_CAN		; Error code
                SEC
		RTS

GotByte1 	cmp	#XEOT		; XMODEN EOT Byte
		bne	BadCrc		; Not SOH or EOT, so flush buffer & send NAK
		jmp	Done		; EOT - all done!


BegBlk		ldx	#$00
GetBlk		lda	#90		; 10 sec window to receive characters
		sta 	retry2		;
GetBlk1		jsr	GET_BYTE	; get next character
		bcs	BadCrc		; chr rcv error, flush and send NAK
GetBlk2		sta	Rbuff,X		; good char, save it in the rcv buffer
		inx			; inc buffer pointer	
		cpx	#$84		; <01> <FE> <128 bytes> <CRCH> <CRCL>
		bne	GetBlk		; get 132 characters
		ldx	#$00		;
		lda	Rbuff,X		; get block # from buffer
		cmp	blkno		; compare to expected block #	
		beq	GoodBlk1	; matched!
		jsr	Flush		; mismatched - flush buffer
		lda	#X_BLK		; 
                SEC			; unexpected block # - fatal error
		RTS

GoodBlk1	eor	#$ff		; 1's comp of block #
		inx			;
		cmp	Rbuff,X		; compare with expected 1's comp of block #
		beq	GoodBlk2 	; matched!
		jsr 	Flush		; mismatched - flush buffer
		lda	#X_BLK		; 
                SEC			; unexpected block # - fatal error
		RTS
	
GoodBlk2	JSR CalcCRC
		lda	Rbuff,Y		; get hi CRC from buffer
		cmp	crc+1		; compare to calculated hi CRC
		bne	BadCrc		; bad crc, send NAK
		iny			;
		lda	Rbuff,Y		; get lo CRC from buffer
		cmp	crc		; compare to calculated lo CRC
		beq	CopyBlk		; good CRC
BadCrc		jsr	Flush		; flush the input port
		lda	#NAK		;
		jsr	Put_Chr		; send NAK to resend block
		jmp	StartBlk	; start over, get the block again			
	
CopyBlk		LDX 	#$02		;data starts at pos $02
		ldy	#$00		; set offset to zero
CopyBlk3	lda	Rbuff,X		; get data byte from buffer
		sta	(ptr),Y		; save to target
		inc	ptr		; point to next address
		bne	CopyBlk4	; did it step over page boundary?
		inc	ptr+1		; adjust high address for page crossing
CopyBlk4	inx			; point to next data byte
		cpx	#$82		; is it the last byte
		bne	CopyBlk3	; no, get the next one
IncBlk		inc	blkno		; done.  Inc the block #
		LDX PTR+1		;check for EOM
		INX
		CPX BDOS+2
		BCS FTL
		lda	#ACK		; send ACK
		jsr	PUT_CHR		;
		jmp	StartBlk	; get next block

FTL		LDA #File_too_long
		SEC
		RTS

Done		lda	#ACK		; last block, send ACK and exit.
		jsr	PUT_CHR		;
		jsr	Flush		; get leftover characters, if any
		CLC
		rts


;^^^^^^^^^^^^^^^^^^^^^^ Start of XModem send ^^^^^^^^^^^^^^^^^^^^^^

;
; Enter this routine with the beginning address stored in the zero page address
; pointed to by ptr & ptrh and the ending address stored in the zero page address
; pointed to by eofp & eofph.

XMSEND		lda	#$00		;
		sta	errcnt		; error counter set to 0
		sta	lastblk		; set flag to false
		LDX DMA+1
		LDY DMA			;set PTR = DMA - 1
		BNE XMSEND1
		DEX
XMSEND1		DEY
		STY PTR			;PTR is end of file
		STX PTR+1
		JSR SETDMA		;reset DMA = beginning adr
		JSR INITCOM
		JSR FLUSH
		lda	#$01		;
		sta	blkno		; set block # to 1
Wait4CRC	LDA #'.			;while waiting
		JSR PUT_CHR		;print .
		lda	#$ff		; 3 seconds
		sta	retry2
		jsr	GET_BYTE
		BCS	Wait4CRC	; wait for something to come in...
		cmp	#'C		; is it the "C" to start a CRC xfer?
		beq	SetstAddr	; yes
		cmp	#CAN		; is it a cancel? <Esc> Key
		bne	WAIT4CRC	; No, wait for another character
		jmp	PrtAbort	; Print abort msg and exit


SetstAddr	ldy	#$00		; init data block offset to 0
		ldx	#$02		; preload X to Receive buffer
		lda	#$01		; manually load blk number	
		sta	Rbuff		; into 1st byte
		lda	#$FE		; load 1's comp of block #	
		sta	Rbuff+1		; into 2nd byte
		JMP	Ldbuff1		; jump into buffer load routine

LdBuffer	lda	Lastblk		; Was the last block sent?
		beq	LdBuff0		; no, send the next one	
		LDA #XEOT		;send EOT
		JSR PUT_CHR
		CLC
		RTS			; yes, we're done

LdBuff0		ldx	#$02		; init pointers
		ldy	#$00		;
		inc	Blkno		; inc block counter
		lda	Blkno		; 
		sta	Rbuff		; save in 1st byte of buffer
		eor	#$FF		; 
		sta	Rbuff+1		; save 1's comp of blkno next

LdBuff1		lda	(DMA),y		; save 128 bytes of data
		sta	Rbuff,x		;
LdBuff2		sec			; 
		lda	PTR		;
		sbc	DMA		; Are we at the last address?
		bne	LdBuff4		; no, inc pointer and continue
		lda	PTR+1		;
		sbc	DMA+1		;
		bne	LdBuff4		; 
		inc	LastBlk		; Yes, Set last byte flag
LdBuff3		inx			;
		cpx	#$82		; Are we at the end of the 128 byte block?
		beq	SCalcCRC	; Yes, calc CRC
		lda	#$00		; Fill rest of 128 bytes with $00
		sta	Rbuff,x		;
		beq	LdBuff3		; Branch always

LdBuff4		inc	DMA		; Inc address pointer
		bne	LdBuff5		;
		inc	DMA+1		;
LdBuff5		inx			;
		cpx	#$82		; last byte in block?
		bne	LdBuff1		; no, get the next
SCalcCRC	jsr 	CalcCRC
		lda	crc+1		; save Hi byte of CRC to buffer
		sta	Rbuff,y		;
		iny			;
		lda	crc		; save lo byte of CRC to buffer
		sta	Rbuff,y		;
Resend		ldx	#$00		;
		lda	#SOH
		jsr	PUT_CHR		; send SOH
SendBlk		lda	Rbuff,x		; Send 132 bytes in buffer to the console
		jsr	PUT_CHR		;
		inx			;
		cpx	#$84		; last byte?
		bne	SendBlk		; no, get next
		lda	#$FF		; yes, set 3 second delay 
		sta	retry2		; and
		jsr	GET_BYTE	; Wait for Ack/Nack
		BCS	Seterror	; No chr received after 3 seconds, resend
		cmp	#ACK		; Chr received... is it:
		beq	LdBuffer	; ACK, send next block
		cmp	#NAK		; 
		beq	Seterror	; NAK, inc errors and resend
		cmp	#CAN		;
		beq	PrtAbort	; Esc pressed to abort
					; fall through to error counter
Seterror	inc	errcnt		; Inc error counter
		lda	errcnt		; 
		cmp	#$0A		; are there 10 errors? (Xmodem spec for failure)
		bne	Resend		; no, resend block
PrtAbort	jsr	Flush		; yes, too many errors, flush buffer
		LDA	#XModem_terminated
		SEC
		RTS

;
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
;
; subroutines
;
;
Flush				; flush receive buffer
		JSR GET_BYTE
		BCC FLUSH
		CLC
		rts			; else done

;=========================================================================
;
;
;  CRC subroutines 
;
;
CalcCRC		lda	#$00		; yes, calculate the CRC for the 128 bytes
		sta	crc		;
		sta	crc+1		;
		ldy	#$02		;
CalcCRC1	lda	Rbuff,y		;
		eor 	crc+1 		; Quick CRC computation with lookup tables
       		tax		 	; updates the two bytes at crc & crc+1
       		lda 	crc		; with the byte send in the "A" register
       		eor 	CRCHI,X
       		sta 	crc+1
      	 	lda 	CRCLO,X
       		sta 	crc
		iny			;
		cpy	#$82		; done yet?
		bne	CalcCRC1	; no, get next
		rts			; y=82 on exit
;
;
;======================================================================
;  I/O Device Specific Routines
;
;  Two routines are used to communicate with the I/O device.
;
; "Get_Byte" routine will scan the input port for a character.  It will
; return without waiting with the Carry flag SET if no character is
; present or return with the Carry flag CLEAR and the character in the "A"
; register if one was present.
;
; "Put_Chr" routine will write one byte to the output port.  Its alright
; if this routine waits for the port to be ready.  its assumed that the 
; character was send upon return from this routine.
;
GET_Byte
	SEI			;NO IRQ FROM HERE!
	STX X_BUF
CHRIN3	dec	retry		; no character received, so dec counter
	BEQ	CHRIN4		;
CHRIN	BIT PAD			;THEN GET ONE FROM THE KEYBOARD
	BMI CHRIN3		;WAIT FOR START BIT
	LDX #8			;SET FOR 8 BITS
	JSR DELHBT		;DELAY 1/2 BIT
CHRIN1	JSR DELBIT		;DELAY 1 BIT
	BIT PAD			;GET ONE BIT
	CLC			;'0
	BPL CHRIN2
	SEC			;'1
CHRIN2	ROR CHR_BUF		;ROTATE BIT INTO CHR
	DEX
	BNE CHRIN1
	JSR DELBIT		;Parity Bit
	CLI			;END OF CRITICAL PHASE
	LDA CHR_BUF
	LDX X_BUF
	CLC
	RTS

CHRIN4	dec	retry2		; dec hi byte of counter
	bne	CHRIN		; look for character again
	CLI
	LDX X_BUF
	SEC			; time out	
	RTS

				;SAVE'S X,Y
PUT_CHR	STX X_BUF		;PRINTS CHR TO SENDER
	STA CHR_
	LDA PBD
	ORA #$01		;PB0 = 1
	STA PBD
	SEI			;NO IRQ FROM HERE
	AND #$FE		;START BIT
	STA PBD
	JSR DELBIT		
	LDX #8			;SEND 8 BITS

CHROUT1	LSR CHR_		;SHIFT OUT 1 BIT
	LDA PBD
	ORA #$01		;'1
	BCS CHROUT2
	AND #$FE		;'0
CHROUT2	STA PBD
	JSR DELBIT
	DEX
	BNE CHROUT1
	LDA PBD			;SEND parity + 1 STOP BIT
	ORA #$01		;NO PARITY
	STA PBD
	JSR DELBIT		; 1 stop bit
	CLI			;END OF CRITICAL PHASE
CHROUT5	LDX X_BUF
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

DELHBT	LDA HBTIME		;SET UP FOR HALF BIT TIME
	STA TIME_
DELHBT1	DEC TIME_
	BNE DELHBT1
	RTS

DELBIT	LDA BTIME		;Set up for Bit Time
	STA TIME_
DELBIT1	DEC TIME_
	BNE DELBIT1
	RTS

INITCOM	LDA HBTTIME			;set serial param's
	STA HBTIME
	LDA BITTIME
	STA BTIME
	RTS

HBTIME	DB 0
BTIME	DB 0
TIME_	DB 0
X_BUF	DB 0
CHR_BUF	DB 0
CHR_	DB 0


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
	DB X_CAN,$10,'<CAN>',EOT
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


; non-zero page variables and buffers (page aligned)
;
;
MEM
	DS MEM/256+1*256-MEM		;page alignment

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
;

Rbuff	DS 132,$00  	; temp 132 byte receive buffer (page aligned)

LOMEM

	END