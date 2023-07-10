;CP/M-65 Version   Apple II Version
;Version 2.08-S
;released:	10 October 1982
;last revision:
;	28 December 1985
;		added param for width
;	30 March 2008
;		reformatted for TASM & ASM210
;		eliminated page zero 0 & 1
;		added total block count to end msg
;	3 April 2008
;		corrected errors
;	17 December 2009
;		added pzstrt variable
;		change page zero to use pzstrt
;	28 January 2010
;		added opening banner
;		added MSGOUT routine
;	31 January 2015 (2.06)
;		added screen page control
;		added line counter
;	15 February 2015 (2.06)
;		changed message sequence
;	16 February 2015 (2.08)
;		moved notasp
;	30 November 2022 (2.09)
;		nr of free blocks corrected
;Syntax is ALLOC x where x is the drive (A to H)
;If x is blank then the default drive is used.
;users may want to change the value of width
;to a smaller value if their console is less than 80
;characters wide.
;external references

PEM	=	$F0		;pem entry
FCB1	= $F6			;default fcb vector
DPHV	= $F8			;DPH-VECTOR
TPA	=	$200		;tpa start
;fixed parameters
EOT	= $00
CR	=	$D		;return
LF	=	$A		;linefeed
YBAT	= 4			;index BAT vector in DPH
;width is a somewhat obscure way of controlling display width
;it can be set to 1, 3, 7, etc. - and when so set the number of
;8 block wide bit maps becomes 2, 4, 8, etc.
;width can only be set to (2 to the n power) - 1
width	=	3		;(width+1)*8-->chars per line
;maxline defines the number of map line to print
;before pausing.
maxlin	=	40
;page zero variables
pzstrt	=	$2		;start of free page zero ram
dcbpnt	= $4			;dcb pointer
bitcnt	= $6			;count of total blocks
VECPNT	= $8			;vector pointer
DFLDRV	= $9			;default drive
TMPDRV	= $0A			;temporary drive
FREE	= $B			;free blocks
BITNUM	= $D			;bit (0 to 7)
bytnum	= $F			;byte (0 to 255)
LEADZ	= $10			;leading zero flag
blkscd	= $11			;block size code
total	= $12			;total number of block
line	= $14			;line count
scrlin	= $15			;screen line

	;main program
	ORG	TPA
;send opening message including version followed by cr & lf
	lda	#opnmsg
	ldy	#opnmsg/256
	jsr	msgout

;now process command line
	LDX	#$19		;get and
	JSR	PEM		;save
	STA	DFLDRV		;default drive
	LDY	# 0
	LDA	(FCB1),Y	;get object
	STA	TMPDRV		;and save
	LDX	#$0D		;initialize
	JSR	PEM		;system
	LDY	TMPDRV		;get temp
	BNE	havdrv		;use default
	LDA	DFLDRV		;get default
	BPL	SETDRV		;and go
havdrv	DEY
	TYA
SETDRV	LDX	#$0E		;then
	sta	tmpdrv		;save drive for dcb vector
	JSR	PEM		;select

	JSR SETBAT		;force BAT to be set
				;SET dcbpnt = DPH
	LDA DPHV
	sta	dcbpnt		;save pointer
	LDA DPHV+1
	sta	dcbpnt+1
	ldy	#8		;get max block number
	lda	(dcbpnt),y
	clc			;bump to get number
	adc	#1
	sta	free		;set free count
	sta	total		;and total
	sta	bitcnt		;and bit counter
	iny			;now get high
	lda	(dcbpnt),y	;and add with carry
	adc	#0
	sta	free+1
	sta	total+1
	sta	bitcnt+1
	ldy	#7		;get size code
	lda	(dcbpnt),y
	sta	blkscd		;and save
	LDY #YBAT
	LDA (dcbpnt),Y
	STA	VECPNT		;save
	INY
	LDA (dcbpnt),Y
	STA	VECPNT+1	;in memory
	LDA	#0		;then clear
	STA	BITNUM		;bit
	STA	BYTNUM		;byte
	STA	LEADZ		;and leading zero flag
	sta	line		;clear line number
	sta	scrlin		;and screen line
	jsr	linnum		;start numbering lines at 1
OUTLPE	LDY	BYTNUM		;get byte number
	LDX	BITNUM		;and bit number
	LDA	(VECPNT),Y	;get map byte
	AND	BITMSK,X	;overlay mask
	BNE	ISFULL		;branch if allocated
	LDA	#'0'		;else send
	bne	next		;a zero
isfull	lda	free		;get low of free count
	bne	frenz		;jump if not zero
	dec	free+1		;else drop high then do low
frenz	dec	free
	LDA	#'1'		;send a
next	jsr	chrout		;one
	lda	bitcnt		;get low of counter
	bne	bitnz		;jump if not zero
	dec	bitcnt+1	;else drop high then do low
bitnz	dec	bitcnt
	lda	bitcnt		;test for zero
	ora	bitcnt+1
	beq	done		;done if zero
	INC	BITNUM		;bump bit number
	LDA	BITNUM		;get it
	cmp	#8		;see if done with byte
	BNE	OUTLPE		;then loop
	lda	#0
	STA	BITNUM		;else clear bit number
	INC	BYTNUM		;bump byte number
	bne	bitnz1		;skip if not zero
	inc	vecpnt+1	;else bump high pointer
bitnz1	LDA	BYTNUM		;get it
	AND	#width		;if not mod-(width+1)
	BNE	OUTLPE		;then loop
	inc	scrlin		;bump line count
	lda	scrlin		;get it
	cmp	#maxlin		;see if at limit
	bne	notmax		;not so continue
	lda	#0		;clear count
	sta	scrlin
	lda	#nxtmsg		;point to 'next' message
	ldy	#nxtmsg/256
	jsr	msgout		;send it
notasp	ldx	#6		;get space without echo
	LDA #$FF
	jsr	pem
	cmp	#' '		;see if space
	bne	notasp		;loop if not
notmax	JSR	CRLF		;else do cr and lf
	jsr	linnum		;number new line
	JMP	OUTLPE		;and loop
DONE	JSR	CRLF		;send two
	JSR	CRLF		;cr and lf pairs
;now send free count to screen
	LDA #3SP
	LDY #3SP/256
	JSR msgout
	INC FREE		;correction for dummy block created
	BNE done1
	INC FREE+1
DONE1	jsr	sndfre
	lda	#' '		;send a space
	jsr	chrout
	;now display block size
	lda	blkscd		;get code
	asl	a		;mult by two
	tax			;make index
	lda	blktbl,x	;get address
	ldy	blktbl+1,x
	jsr	msgout		;and send to console
	LDA	#CLSMSG		;send
	LDY	#CLSMSG/256	;size
	jsr	msgout		;to console
;now show total block count
	lda	#0		;first clear leading zero flag
	sta	leadz
	lda	total
	ldy	total+1
	sta	free
	sty	free+1
	jsr	sndfre
	lda	#ttlmsg
	ldy	#ttlmsg/256
	jsr	msgout
;now exit
	LDA	DFLDRV		;select
	LDX	#14		;default
	JMP	PEM		;and return

;send 16 bit value in free to screen
sndfre	ldx	#0		;clear 10000's
	sec
f10000	lda	free		;drop by 10000
	sbc	#10000
	sta	free
	lda	free+1
	sbc	#10000/256
	sta	free+1
	inx			;bump digit
	bcs	f10000		;loop if no borrow
	lda	free		;else add 10000 back in
	adc	#10000
	sta	free
	lda	free+1
	adc	#10000/256
	sta	free+1
	dex			;drop digit
	beq	n10000		;skip if zero
	txa			;else set flag
	sta	leadz
	jsr	snddig		;send
n10000	ldx	#0		;clear 1000's
	sec
f1000	lda	free		;drop by 1000
	sbc	#1000
	sta	free
	lda	free+1
	sbc	#1000/256
	sta	free+1
	inx			;bump digit
	bcs	f1000		;loop if no borrow
	lda	free		;else add back in
	adc	#1000
	sta	free
	lda	free+1
	adc	#1000/256
	sta	free+1
	jsr	digout		;send digit
	LDX	#0		;clear 100's count
	SEC			;now
f100	lda	free		;now do 100's
	sbc	#100
	sta	free
	lda	free+1
	sbc	#0
	sta	free+1
	INX			;bump count
	BCS	F100		;until a borrow
	lda	free
	ADC	#100		;then add
	STA	FREE		;100 back and save
	jsr	digout		;send digit
	LDX	#0		;clear 10's
	SEC			;now
	LDA	FREE		;drop
F10	SBC	#10		;free by 10
	INX			;and bump count
	BCS	F10		;until a borrow
	ADC	#10		;add 10 back in
	STA	FREE		;and save
	jsr	digout		;send digit
NO10	LDA	FREE		;always
	jmp	snddig		;do 1's
;output digit
digout	dex			;drop count
	beq	chkldz		;if zero check flag
	stx	leadz		;else set
	bne	mkedig		;then make and send
chkldz	lda	leadz		;get flag
	beq	extdig		;done if zero
mkedig	txa			;move to a
snddig	ora	#'0'
	jsr	chrout		;send
extdig	rts			;quit


;output a cr and lf
CRLF	LDA	#CR		;send
	JSR	CHROUT		;a cr
	LDA	#LF		;then a lf
;character output
CHROUT	LDX	#2		;send char
	JMP	PEM		;to console
;message output
msgout	ldx	#9		;send string
	jmp	pem		;to console
;line number counting and output
linnum	sed			;use line as decimal counter
	clc
	lda	line
	adc	#1
	sta	line
	cld
	lsr	a		;shift high digit to low
	lsr	a
	lsr	a
	lsr	a
	ora	#'0'		;make a digit
	jsr	chrout		;send it
	lda	line		;get line again
	and	#%00001111	;look at low
	ora	#'0'		;make it a digit
	jsr	chrout		;send it
	lda	#' '		;get space
	jsr	chrout		;send it
	rts

SETBAT	LDX #0		;forces BAT
	LDY #1		;set FCB1 to $120957$.$$$
SETBT1	LDA DUMMY,X
	STA (FCB1),Y
	INX
	INY
	CPY #12
	BCC SETBT1
	LDX #$16
	JSR PEM		;create dummy file
	LDX #$15
	JSR PEM		;Write dummy block
	LDX #$13
	JSR PEM		;Kill dummy file
	RTS


DUMMY	DB '$120957$$$$'
;bit mask table
BITMSK	DB	128,64,32,16,8,4,2,1
;block size messages
BLKMS025 DB	'0.25',EOT
BLKMS05	DB	'0.5',EOT
blkms0	DB	'1',EOT
blkms1	DB	'2',EOT
blkms2	DB	'4',EOT
blkms3	DB	'8',EOT
blkms4	DB	'16',EOT
;block size message pointers
blktbl	DW	BLKMS025,BLKMS05,blkms0,blkms1,blkms2,blkms3,blkms4
;opening message
opnmsg	DB	CR,LF,'CPM-65 DISK ALLOCATION MAP V2.09',CR,LF,CR,LF
	DB '   01234567890123456789012345678901'
	DB	CR,LF,EOT
nxtmsg	DB	cr,lf,'PRESS SPACE BAR TO SEE NEXT PAGE',EOT
;closing message
clsmsg	DB	'K BLOCKS FREE OF ',EOT
3SP	DB	'   ',EOT
ttlmsg	DB	' TOTAL',CR,LF,EOT

	END
