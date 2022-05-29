;DRUCKER.ASM

;Menueprogramm zur Einstellung eines RITEMAN II
;V1.0	30.1.91

;--- STEUERZEICHEN ---

EOT	= $00
BREAK	= $03
CR	= $0D
LF	= $0A
SO	= $0F
ESC	= $1B

;--- SYSTEMAUFRUFE ---

BOOT	= $00
CONOUT	= $02
LPRINT	= $05
STROUT	= $09
BIOS_CALL	= $1A

;--- SYSTEMVEKTOREN ---

POINTER	= $00
LENGTH	= $02
BDOS	= $F0


START	LDA #HEADER		;HEADER AUSGEBEN
	LDY #HEADER/256
	JSR PRTSTR

MAINLP	LDA #BEFTAB		;POINTER AUF BEFEHLSTABELLE
	STA POINTER
	LDA #BEFTAB/256
	STA POINTER+1
	LDA #MENUE		;MENUE AUSGEBEN
	LDY #MENUE/256
	JSR PRTSTR
LOOP1	JSR RECCHA		;CHR EINLESEN
	CMP #BREAK
	BEQ ENDE

LOOP2	LDY #0
	CMP (POINTER),Y		;CODE VERGLEICHEN
	BEQ EXEC
	TAX
	LDA (POINTER),Y		;ENDE DER TABELLE?
	BEQ LOOP1
	INY
	LDA (POINTER),Y		;LAENGENBYTE LADEN
	CLC
	ADC #2			;LAENGE+2
	ADC POINTER		;ZU POINTER ADDIEREN
	STA POINTER
	LDA POINTER+1
	ADC #0
	STA POINTER+1
	TXA
	JMP LOOP2


EXEC	JSR PRTCHR		;CHR AUSGEBEN
	JSR CRLF
	JSR CRLF
	LDY #1
	LDA (POINTER),Y		;LAENGE LADEN
	STA LENGTH
LOOP4	INY
	LDA (POINTER),Y
	LDX #LPRINT
	JSR BDOS
	DEC LENGTH
	BNE LOOP4
	BEQ MAINLP


ENDE	LDA #ENDMES
	LDY #ENDMES/256
	JSR PRTSTR
	RTS

;---  SUBROUTINEN  ---

RECCHA	LDY #3		;CHR INPUT VIA BIOS
	LDX #BIOS_CALL
	JSR BDOS
	CMP #$60
	BCC RECCHA_X
	AND #$5F
RECCHA_X
	RTS


CRLF	LDA #CRLFM	;PRINTS A NEWLINE
	LDY #CRLFM/256

PRTSTR  LDX #STROUT
	JMP BDOS


PRTCHR  LDX #CONOUT	;PRINTS A CHR
	JMP BDOS


;---------------------------------------------------

HEADER	DB CR,LF,'                 D R U C K E R - U T I L I T Y'
	DB CR,LF,'                 V1.0   30.1.91    RITEMAN II'
	DB CR,LF,'                 (c) D. Lausberg',CR,LF,EOT

ENDMES	DB CR,LF,CR,LF,' ... Druckereinstellung beendet',CR,LF,EOT

CRLFM	DB CR,LF,EOT

MENUE	DB CR,LF
	DB '                 O P T I O N E N',CR,LF,CR,LF
	DB ' 1 > 10 Zeichen/Inch           '
	DB ' 4 >  5 Zeichen linker Rand    ',CR,LF
	DB ' 2 > 12 Zeichen/Inch           '
	DB ' 5 >  7 Zeichen linker Rand    ',CR,LF
	DB ' 3 > 17 Zeichen/Inch           '
	DB ' 6 >  9 Zeichen linker Rand    ',CR,LF
	DB '                               '
	DB ' 7 >  kein linker Rand         ',CR,LF
	DB ' 8 >  Perforationssprung       '
	DB ' 9 >  Drucker-Reset            ',CR,LF
	DB ' L >  Letter Quality           '
	DB ' H >  High Speed Draft         ',CR,LF
	DB ' S >  Standard-Voreinstellung  '
	DB ' D >  Datendruck               ',CR,LF

	DB CR,LF,' Bitte Option eingeben   (^C = Ende)'
	DB CR,LF,' Option : ',EOT

BEFTAB	DB '1',3,ESC,'!',0	;10 Zeichen/Inch
	DB '2',3,ESC,'!',1	;12 Zeichen/Inch
	DB '3',3,ESC,'!',4	;17 Zeichen/Inch
	DB '4',3,ESC,'l',5	;5 Zeichen linker Rand
	DB '5',3,ESC,'l',7	;7 Zeichen linker Rand
	DB '6',3,ESC,'l',9	;9 Zeichen linker Rand
	DB '7',3,ESC,'l',0	;ohne Rand
	DB '8',3,ESC,'N',6	;Perforationssprung
	DB '9',2,ESC,$40	;Reset
	DB 'L',3,ESC,'x',1	;LQ
	DB 'H',3,ESC,'x',0	;Draft
	DB 'S',11,ESC,$40,ESC,'!',0,ESC,'l',0,ESC,'N',0	;Standard
	DB 'D',9,ESC,'!',4,ESC,'l',9,ESC,'N',6		;Datendruck

	DB $00,$00,$00

	END
