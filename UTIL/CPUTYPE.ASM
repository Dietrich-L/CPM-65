;********************************************************
;*							*
;*	    CPU TYPE TESTER				*
;*							*
;********************************************************
;
;released:	14 August 2014 (1.00)
;last revision:	12 February 2015 (1.00X)
;			native assembler
;
;DOS/65 definitions
PEM	=	$F0		;CPM ENTRY POINT
tea	=	$200		;TEA start
;DOS/65 PEM commands
WBOOT	=	$00		;warm boot
STRING	=	9		;STRING PRINT FUNCTION
;character definitions
EOT	= $00
ALF	=	$a		;ASCII LINE FEED
ACR	=	$d		;ASCII CARRIAGE RETURN
;
;code section
	ORG	tea
	ldx	#$ff		;set stack so must
	txs			;exit with warm boot
	lda	#opnmsg	;point to welcome message
	ldy	#opnmsg/256
	jsr	message		;send it
;now run the test
	lda	#$99
	clc
	sed
	adc	#$01
	cld
	beq	cmos
;we have an NMOS CPU
	lda	#NMOSMsg	;point to message
	ldy	#NMOSMsg/256
	jsr	message		;send it
	LDX #WBOOT
	jmp	PEM
;we have a CMOS CPU
cmos	lda	#CMOSMsg	;point to message
	ldy	#CMOSMsg/256
	jsr	message		;send it
	LDX #WBOOT
	jmp	PEM
;normal message output for message at ay
MESSAGE	ldx	#STRING		;DISPLAY THE MESSAGE
	jmp	PEM		;and return
;
;messages
opnmsg	DB	'CPM-65 CPU TYPE TESTER V1.00',acr,alf,EOT
NMOSMsg	DB	acr,alf,'**** CPU is NMOS - 6502 ****',ACR,ALF,EOT
CMOSMsg	DB	acr,alf,'**** CPU is CMOS - 65C02 ****',ACR,ALF,EOT
;
	END
