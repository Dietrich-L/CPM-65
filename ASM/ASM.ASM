;ASM.COM -- CP/M-65 ASSEMBLER
;BY D. LAUSBERG	 (C)1990
;V1.0	15.08.90
;V1.1	10.11.90
;V1.2	30.12.90	ASM-65 Version
;V1.3	20.02.91	improved error handling
;V1.4	16.03.95	some improvements
;V1.5	14.04.95	improved DB handling
;V2.0	04.06.95	APP command
;V2.1	24.06.95	LDX ABS,Y error corrected
;V2.2	26.07.98	error handling more stable
;V2.3	24.04.02	some corrections, DS length,$fillbyte
;V2.4	11.02.21	DS error correction
;V2.5	03.06.21	/L Switch
;V2.6	11.11.22	Some optimizations and error corrections
;V2.7	21.06.23	more error corrections
;V2.8	29.10.23	error in RSTNOD which could lead to spurious error messages
VERSION	= $28

;===	SYSTEM CONSTANTS
DMA	= $FE
FCB1	= $F6
BDOS	= $F0
INBUF	= $EC
CCPV	= $E0

;===	PAGE 00 CELLS
CODE	= $00	;pointer to the assembled code
PC	= $02	;actual program counter
LINCNT	= $04	;line number
LINE	= $06	;pointer to current source code line
LINPNT	= $08	;pointer to chr in LINE
BUFPNT	= $09	;pointer to chr in DMA
SNOD	= $0A	;buffer for NOD
PNOD	= $0C	
NOD	= $0E	;adr of node in label table
ERRCNT	= $10
OPCOD	= $11
OPLEN	= $12
OPLENA	= $13
MASK	= $14
MAP	= $15
TEMP	= $16
XTEMP	= $17
OPRT	= $18
OPERA	= $19	;operand A
OPERB	= $1B	;operand B
HEAP	= $1D	;lower end of lable table
ROOT	= $1F	;1. label in label table
NODSTR	= $21
LBLLEN	= $23
NODLEN	= $24
LFLAG	= $25
DFLAG	= $26
SWITCH	= $27  ; /P Printer on  /C  Check only  /L List to Screen
ERRNO	= $28
NODPNT	= $29
HEXCNT	= $2A
OPERX	= $2B
YTEMP	= $2D
YBUF	= $2E
BLK_CNT	= $2F
FCBBUF	= $30
HIMEM	= $3C	;adress of highes available memory byte +1
OUTDEV	= $3E	;Output Device

LBLSTR	= $40

TPA	= $0200

;BDOS COMMANDS
BOOTC	= $00
CONIN	= $01
CONOUT	= $02
LIST	= $05
STROUT	= $09
DSKSEL	= $0E
OPEN	= $0F
CLOSE	= $10
RDSEQ	= $14
WRITE	= $15
CREATE	= $16


;CONTROL CODES

EOT	= $00
TAB	= $09
CR	= $0D
EOTM	= $1A
LF	= $0A
CTRLP	= $10
SO	= $0F
SP	= $20

;===	PARAMETERS

MAX_ERR	= 25
MAX_LBL	= 32		;max label lenght
ABS_HIMEM = $F400/256	;absolute HIMEM
NOPC	= $EA
TAB1	= 11

;===	ERROR CODES

NOTFND	= $DD
DOUBLE	= $DC
EOF	= $D7
EOL	= $84
ZWARN	= $87
OUTRNG	= $88
LABMIS	= $89
MISWRD	= $8A
ARGMIS	= $8B
OFLOW	= $8C
ILLOPC	= $8D
SYNTAX	= $8E
ILLADM	= $8F
NOVAL	= $90
JINDW	= $91
WRDOVL	= $92
ILLDEC	= $93
DUPLBL	= $94
DUPVAL	= $95
DIV0	= $98	;DIVISION BY 0

NOCOM	= $B0
EXTERR	= $B1
ERR_OVL	= $B2
LINOVL	= $B3
LCOVL	= $B4
MEMOVL	= $B5
ILLFN	= $B6


	ORG TPA

ASM	JSR ASMINI	;INITIALIZE
	LDY #9
	LDA (FCB1),Y
	CMP #SP
	BEQ ASM1
	LDA #EXTERR
ASM6	JSR ERROR
BOOT	LDX #BOOTC
	JMP BDOS
ASM1	LDX #0
	JSR SETEXT	;.EXT = .ASM
ASM2	LDX #OPEN
	JSR BDOS
	BCC ASM3
	JSR ERROR
	BCC ASM2
	BCS BOOT
ASM3	LDA #PASS1M
	LDY #PASS1M/256
	JSR PRTSTR
ASM4	JSR GLINE
	BCS ASM7
	JSR DECOD
	BCC ASM4
ASM7	CMP #EOF
	BEQ ASM20
	JSR ERROR
	BCC ASM4
	BCS BOOT

ASM20	LDA #PASS2M
	LDY #PASS2M/256
	JSR PRTSTR
	JSR RESPNT
	LDX #OPEN
	JSR BDOS
	BCS ASM6
ASM22	JSR GLINE
	BCS ASM23
	JSR DECOP
	BCC ASM22
ASM23	CMP #EOF
	BEQ ASM30
	JSR ERROR
	BCC ASM22
ASM24	BCS BOOT

ASM30	BIT SWITCH
	BVS ASM31
	JSR R_FCB1		;RESTORE ORIGINAL FCB
	JSR SAVCOM		;SAVE *.COM FILE
	BCC ASM32
	JSR ERROR
	BCC ASM30
	BCS ASM24
ASM32	LDA #SCOMM
	LDY #SCOMM/256
	JSR PRTSTR
ASM31	BIT SWITCH
	BVS ASM33
	JSR SAVLBL
	BCC ASM34
	JSR ERROR
	BCC ASM32
	BCS ASM24
ASM34	LDA #SLBLM
	LDY #SLBLM/256
	JSR PRTSTR
ASM33	JSR PRSTAT
	JMP BOOT


ASMINI	LDA #0		;INITIALIZE VAR'S
	STA ERRCNT
	STA SWITCH
	STA LFLAG
	STA DFLAG
	LDA FCB1	;SAVE FCB1 VECTOR
	STA FCBBUF
	LDA FCB1+1
	STA FCBBUF+1
	JSR RESPNT1
	JSR LBLRST
	LDA #LBEG
	STA LINE
	LDA #LBEG/256
	STA LINE+1
	JSR SETSW
	BIT SWITCH
	BPL ASMINX
	LDX #LIST	;Printer on
	STX OUTDEV
	JSR ASMHEAD
ASMINX	RTS

ASMHEAD	LDY #0
ASMHD1	LDA PRTAB,Y
	BEQ ASMHDX
	LDX OUTDEV
	JSR BDOS
	INY
	BNE ASMHD1
ASMHDX	RTS


R_FCB1	LDA FCBBUF	;RESET FCB1 TO ORIGINAL
	STA FCB1
	LDA FCBBUF+1
	STA FCB1+1
	RTS


RESPNT	JSR R_FCB1	;RESET POINTERS FOR PASS 2
RESPNT1 LDA #0
	STA LINCNT	;RESET POINTERS
	STA LINCNT+1
	STA LINPNT
	STA PC
	STA BLK_CNT
	LDA #$FF
	STA BUFPNT
	LDA #TPA/256	;PC=TPA
	STA PC+1
	LDA #CODBEG
	STA CODE
	LDA #CODBEG/256
	STA CODE+1
	RTS


GLINE	LDY #0
	STY LINPNT
GLINE9	INC BUFPNT	;GET LINE FROM DMA
	BNE GLINE1
	LDX #RDSEQ
	JSR BDOS
	BCS GLINE2
	INC BLK_CNT
	LDA BLK_CNT
	AND #%00000111	;EVERY 8 BLOCKS
	BNE GLINE1
	STA BLK_CNT
	LDA #'.		;PRINT 1 POINT
	JSR PRTCHR
GLINE1	LDY BUFPNT	;GET NEXT CHR
	LDA (DMA),Y
	BEQ GLINE8	;EOT?
	CMP #CR
	BEQ GLINE3
	CMP #LF
	BEQ GLINE9
	CMP #EOTM
	BEQ GLINE8
	CMP #TAB
	BEQ GLINE4
	LDY LINPNT
	STA (LINE),Y
	INC LINPNT
	BPL GLINE9
GLINE7	LDA #LINOVL	;LINE OVERFLOW
	SEC
GLINE2	RTS

GLINE3	LDA #0		;END OF LINE
	LDY LINPNT
	STA (LINE),Y	;ASCIIZ
	STA LINPNT
	INC LINCNT
	BNE GLINE5
	INC LINCNT+1
	BNE GLINE5
	LDA #LCOVL	;LINE COUNTER OVERFLOW
	SEC
	RTS

GLINE5	CLC
	RTS

GLINE4	LDY LINPNT	;EXPAND TABS
GLINE6	LDA #SP
	STA (LINE),Y
	INY
	BMI GLINE7
	TYA
	AND #$07	;8 CHRS
	BNE GLINE6
	STY LINPNT
	BEQ GLINE9

GLINE8	LDA #EOF	;END OF FILE
	SEC
	RTS


DECOD	LDY #0		;DECODE LINE
	STY LINPNT
	JSR RDFLAG
	LDA (LINE),Y
	CMP #SP
	BEQ DECOD1	;LABEL?
	JSR GLBL	;GET LABEL
	BCS DECOD2
	JSR LBLPUT
	BCS DECOD3
	LDA PC
	LDX PC+1
	JSR LBLSET
	BCS DECOD3
	JSR SDFLAG
	JSR SAVNOD
DECOD1	JSR GLBL	;GET OPCODE
	BCS DECOD2
	JSR DOPC	;DECODE OPCODE
DECOD3	RTS

DECOD2	CMP #EOL
	BNE DECODX
	CLC
	RTS

DECODX	SEC
	RTS


DOPC	JSR GETMNE	;DECODE OPCODE
	BCS DOPC1
	ASL A
	TAX
	LDA XTAB,X
	PHA
	INX
	LDA XTAB,X
	PHA
DOPC1	RTS


DECOP	JSR PLINNR	;PRINT LINE NR + PC
	JSR RDFLAG
	LDY #0
	STY LINPNT
	LDA (LINE),Y
	CMP #SP
	BEQ DECOP1
	JSR GLBL
	BCS DECOP2
	JSR SDFLAG
DECOP1	JSR GLBL
	BCS DECOP2
	JSR DTWO	;DECODE PASS2
	BCS DECOP3
DECOP4	JSR PLINE
DECOP3	RTS

DECOP2	CMP #EOL
	BNE DECOPX
	LDX #TAB1
	JSR PRXSP
	CLC
	BCC DECOP4

DECOPX	SEC
	RTS


DTWO	JSR GETMNE
	BCS DTWO1
	ASL A
	TAX
	LDA X2TAB,X
	PHA
	INX
	LDA X2TAB,X
	PHA
DTWO1	RTS


PRSTAT	BIT SWITCH
	BPL PRSTA1
	LDA #CTRLP
	JSR PRTCHR
PRSTA1	LDA #PRST1M
	LDY #PRST1M/256
	JSR PRTSTR
	LDA HEAP
	STA OPERA
	LDA HEAP+1
	STA OPERA+1
	LDA CODE
	STA OPERB
	LDA CODE+1
	STA OPERB+1
	JSR MINUS
	LDA OPERA+1
	JSR PRTHEX
	LDA OPERA
	JSR PRTHEX
	JSR CRLF
	BIT SWITCH
	BPL PRSTA2
	LDA #CTRLP
	JSR PRTCHR
PRSTA2	RTS


;--- PASS 1 ---


NORMA	JSR GADRM
	BCS NORMA1
NORMA4	AND MAP
	BEQ NORMA2
	JSR PUTOPC
	JSR ICODPC
	RTS

NORMA1	CMP #$81	;ERROR?
	BCC NORMA4
	RTS

NORMA2	LDA #ILLADM
	SEC
	RTS


ACCUA	JSR GADRM
	BCS ACCUA1
ACCUA3	JMP NORMA4

ACCUA1	JSR ACCU
	BCS ACCUA4
	JMP ICODPC

ACCUA4	CMP #$10	;NO VALUE
	BEQ ACCUA3
	SEC
	RTS


ONEA	LDA MASK
	JSR ACCU5
	JMP ICODPC


REGA	JSR GADRM
	BCS REGA1
REGA5	JSR RUMC
REGA6	JMP NORMA4

REGA1	CMP #SYNTAX
	BNE REGA4
	SEC
REGA7	RTS

REGA4	CMP #$10
	BNE REGA5
	LDA MASK
	AND #$0F
	CMP #$80
	BNE REGA3
	LDY LINPNT
	DEY
	LDA (LINE),Y
	JSR GADRM4
	BCS REGA7
	PHA
	LDA OPCOD
	AND #$F7
	STA OPCOD
	DEC OPLEN
	PLA
	LSR A
	LSR A
	LSR A
	JMP NORMA4

REGA3	LDA #$10
	BNE REGA6


RUMC	CMP #$01	;IMM
	BNE RUMC1
	LDX MASK
	STX OPCOD
RUMC2	RTS

RUMC1	CMP #$40
	BNE RUMC2
	LDA OPERA+1
	BNE RUMC3
	DEC OPLEN
	LDA MASK
	ORA #$14
	STA OPCOD
	LDA #$08	;Z,Y
	RTS

RUMC3	LDA MASK
	ORA #$1C
	STA OPCOD
	LDA #$40	;ABS,Y
	RTS


BRAA	LDA #2
BRAA1	STA OPLEN
	JSR ICODPC
	RTS


JUMPA	LDA #3
	BNE BRAA1


EQUA	BIT DFLAG
	BPL EQUA2
	JSR EVAL
	JSR RSTNOD	;GET (NOD)
	BCS EQUA1
	LDA OPERA
	LDX OPERA+1
	JSR LBLSET
	CLC
	RTS

EQUA2	LDA #LABMIS
	SEC
	RTS


EQUA1	CMP #NOVAL
	BEQ EQUA4
	SEC
	RTS


EQUA4	LDY #6		;MARK NOD AS EMPTY
	LDA (NOD),Y
	ORA #$80
	STA (NOD),Y
	CLC
	RTS


EQUB	JSR EVAL
	BCS EQUB1
	BIT DFLAG
	BPL EQUA2
	LDA #0
	STA LINPNT
	JSR GLBL
	JSR SEARCH
	LDA OPERA
	LDX OPERA+1
	JSR LBLSET
	JSR ORGB3
	CLC
EQUB1	RTS


DBA	LDA #1
	STA OPLEN
DBA1	LDY LINPNT
DBA5	JSR GNCHR
	BCS DWA1	;ARGUMENT MISSING
	CMP #',
	BEQ DWA1	;ARGUMENT MISSING
	CMP #''
	BEQ DBA3	;STRING
DBA2	JSR GNCHR
	BCS DWA3
	CMP #',
	BNE DBA2
	STY LINPNT
	JSR ICODPC
	BCC DBA1
DBA6	RTS

DBA3	INY
	LDA (LINE),Y
	CMP #''			;check for 'C'
	BEQ DBA2
	DEY
	JSR STO_STR
	BCC DBA5
	CMP #EOL
	BNE DBA4
	CLC
	RTS

DBA4	SEC
	RTS


DWA	LDA #2
	STA OPLEN	;ALLWAYS 2 BYTES
DWA4	LDY LINPNT
	JSR GNCHR
	BCS DWA1	;ARGUMENT MISSING
	CMP #',
	BEQ DWA1	;ARGUMENT MISSING
DWA2	JSR GNCHR
	BCS DWA3	;ALLOCATE 2 BYTES AND EXIT
	CMP #',
	BNE DWA2
	STY LINPNT
	JSR ICODPC
	BCC DWA4
	RTS

DWA1	LDA #ARGMIS
	SEC
	RTS

DWA3	JMP ICODPC


DSA	JSR EVAL
	BCS DSA1
DSA4	JSR OPPC
	BCS DSA2
	LDA OPERA
	ADC CODE
	STA CODE
	TAY
	LDA OPERA+1
	ADC CODE+1
	STA CODE+1
	BCS DSA2
	JMP ICODP6

DSA2	LDA #OFLOW
	SEC
	RTS

DSA1	CMP #',
	BNE DWA1		;argument missing
	LDY LINPNT		;argument dummy read
DSA3	JSR GNCHR
	BCC DSA3
	STY LINPNT
	BCS DSA4


END	LDA #EOF
	SEC
	RTS


ORG	JSR EVAL
	BCS ORG1
	LDA OPERA
	STA PC
	LDA OPERA+1
	STA PC+1
ORG1	RTS


APP	JSR SETFCB		;APPEND FROM FILE
	BCS APPX
APP2	LDX #OPEN
	JSR BDOS
	BCS APP1
	LDA #$FF		;SET BUFFER POINTER
	STA BUFPNT		;TO END OF RECORD
APPX	RTS

APP1	JSR ERROR
	BCC APP2
	RTS

;--- PASS 2 ---


NORMB	JSR CHKAA
	BCC NORMB1
	JSR GADRM
	BCS NORMB2
NORMB5	AND MAP
	BEQ NORMB6
	LDA OPLEN
	CMP OPLENA
	BNE NORMB3
NORMB4	JSR PUTOPC
NORMB1	JSR I2CDPC
	RTS

NORMB3	LDA #NOPC
	STA OPERA+1
	JSR NORMB4
	BCS NORMB7
	LDA #ZWARN
	SEC
NORMB7	RTS

NORMB2	JMP SYNERR

NORMB6	LDA #ILLADM
	SEC
	RTS


BRAB	JSR EVAL
	BCS BRAB1
	LDA PC
	STA OPERB
	LDA PC+1
	STA OPERB+1
	JSR MINUS
	LDA #2
	STA OPERB
	LDA #0
	STA OPERB+1
	JSR MINUS
	JSR RNGCHK
	BCS BRAB2
	LDX #18
	JSR SETADM
	JSR PUTOPC
	JSR I2CDPC
BRAB1	RTS

BRAB2	LDA #OUTRNG
	SEC
	RTS


ACCUB	JSR CHKAA	;ACCU INSTRUCTION
	BCC ACCUB3
	JSR GADRM
	BCC NORMB5
ACCUB1	JSR ACCU	;CHECK FOR 'A
	BCS ACCUB2
ACCUB3	JSR I2CDPC
	RTS

ACCUB2	JMP SYNERR	;SYNTAX ERROR


ACCU	CMP #$10	;ABS?
	BNE ACCUB2
	LDX LBLSTR
	CPX #1		;1 CHR?
	BNE ACCU3
ACCU4	LDA LBLSTR,X
	CMP #'A		;=A?
	BNE ACCU3
	LDA #$08
	ORA MASK
ACCU5	LDY #0
	STA (CODE),Y
	LDA #1
	STA OPLEN
	CLC
	RTS

ACCU3	LDA #$10
	SEC
	RTS


ONEB	LDA #1		;ONE BYTE INSTRUCTIONS
	STA OPLEN
	JSR I2CDPC
	RTS


JUMPB	JSR EVAL	;JUMP INSTRUCTIONS
	BCS JUMPB1
	LDA MASK
JUMPB6	STA OPCOD
	LDA #3
	STA OPLEN
	JSR PUTOPC
	JSR I2CDPC
JUMPB7	RTS

JUMPB1	CMP #'(
	BNE JUMPB4
	JSR EVAL
	BCC JUMPB4
	CMP #')
	BNE JUMPB4
	LDY LINPNT
	LDA (LINE),Y
	BEQ JUMPB5
	CMP #SP
	BNE JUMPB4
JUMPB5	LDA MASK
	CMP #$4C
	BNE JUMPB4
	ORA #$20
	JSR JUMPB6
	BCS JUMPB7
	LDA #$FF
	CMP OPERA
	BNE JUMPB8
	LDA #JINDW
	RTS

JUMPB8	CLC
	RTS

JUMPB4	JMP SYNERR


REGB	JSR CHKAA
	BCC REGB1
	JSR GADRM
	BCS JUMPB4
	JSR RUMC
	JMP NORMB5

REGB1	JMP NORMB1


DSB	JSR EVAL		;DS
	BCS DSB1
	LDA #0			;Dummy fill byte = $00
	STA OPRT
DSB2	JSR OPPC		;PC=PC+OPERA
DSB3	LDX OPERA
	BNE DSB4
	LDA OPERA+1		;OPERA = 0 ?
	BEQ DSB5
	DEC OPERA+1
DSB4	DEC OPERA		;DEC OPERA
	LDA OPRT
	LDY #0
	STA (CODE),Y		;store fillbyte
	INY
	JSR ICODP7
	BCC DSB3
	JMP SYNERR

DSB5	JMP ORGB3		;print code

DSB1	CMP #',
	BNE DWB2		;argument missing
	LDA OPERA
	PHA			;save OPERA
	LDA OPERA+1
	PHA
	JSR EVAL		;get fillbyte
	BCS DSB6
	LDA OPERA
	STA OPRT
	PLA
	STA OPERA+1		;restore OPERA
	PLA
	STA OPERA
	JMP DSB2

DSB6	PLA
	PLA
	RTS


DWB	JSR EVAL		;DW
	BCS DWB1
	LDA OPERA
	JSR PHEX
	LDA OPERA+1
	JSR ORGB2
	JMP DWB3

DWB1	CMP #',
	BNE DWB2
	JSR DWB3
	BCC DWB
DWB2	JMP SYNERR

DWB3	LDA OPERA
	LDX OPERA+1
DWB4	JSR DSTO
	JSR ICODPC
	RTS


DDB	JSR EVAL
	BCS DDB1
	JSR ORGB3
DDB2	LDA OPERA+1
	LDX OPERA
	JMP DWB4

DDB1	CMP #',
	BNE DWB2
	JSR DDB2
	BCC DDB
	BCS DWB2


DBB	JSR DB
	BCS DWB2
	LDA OPERA
	JSR PHEX
	LDX #TAB1-2
	JSR PRXSP
	RTS


ORGB	JSR ORG
	BCS ORGB1
ORGB3	LDA OPERA+1
	JSR PHEX
	LDA OPERA
ORGB2	JSR PHEX
	LDX #TAB1-4
	JSR PRXSP
ORGB1	RTS


ENDB	LDX #TAB1
	JSR PRXSP
	JSR PLINE
	LDA #EOF
	SEC
	RTS


DB	LDA #1
	STA OPLEN
	LDY LINPNT
DB6	JSR GNCHR
	BCS DB9
	CMP #''
	BEQ DB7
DB8	JSR EVAL
	BCS DB1
DB3	LDA OPERA
	LDY #0
	STA (CODE),Y
	JSR ICODPC
	RTS

DB1	CMP #',
	BNE DB4
	CPX #0
	BEQ DB
	JSR DB3
	BCC DB
	RTS

DB7	INY
	LDA (LINE),Y		;check for 'C'
	CMP #''
	BEQ DB8
	DEY
	JSR STO_STR
	STY LINPNT
	BCC DB6
	CMP #EOL
	BNE DB4
	CLC
	RTS

DB5	CLC
	RTS

DB9	LDA #ARGMIS
DB4	SEC
	RTS


STO_STR	STY LINPNT
STO_STR4
	LDY LINPNT
	LDA (LINE),Y
	BEQ STO_STR2
	INC LINPNT
	CMP #''
	BEQ STO_STR2
	LDY #0
	STA (CODE),Y
	JSR ICODPC
	BCC STO_STR4
	RTS

STO_STR2
	LDY LINPNT
	JSR GNCHR
	BCS STO_STR1
	CMP #',
	BEQ STO_STR5
	LDA #SYNTAX
	SEC
STO_STR3
	RTS

STO_STR1
	LDA #EOL
	RTS

STO_STR5
	CLC
	RTS


OPPC	LDA OPERA		;PC=PC+OPERA
	CLC
	ADC PC
	STA PC
	LDA OPERA+1
	ADC PC+1
	STA PC+1
	RTS


I2CDPC	LDY #0
	LDA (CODE),Y
	JSR PHEX
	LDX #1
	JSR PRXSP
I2CDP2	INY
	CPY OPLEN
	BCS I2CDP1
	CPY #4
	BCS I2CDP1
	LDA (CODE),Y
	JSR PHEX
	JMP I2CDP2

I2CDP1	LDX #2
	JSR PRXSP
	INY
	CPY #5
	BCC I2CDP1
ICODPC	LDA OPLEN
	TAY		;SAVE OPLEN
	CLC
	ADC PC
	STA PC
	BCC ICODP5
	INC PC+1
ICODP7	CLC
ICODP5	TYA
	ADC CODE
	STA CODE
	TAY
	BCC ICODP6
	INC CODE+1
ICODP6	LDA CODE+1
	CMP HEAP+1
	BCC ICODP2
	BEQ ICODP3
ICODP4	LDA #MEMOVL
	RTS

ICODP3	CPY HEAP
	BCS ICODP4
ICODP2	RTS


SETFCB	LDY #0
	LDA (FCB1),Y
	TAX
	LDA #FCBA	;SET FCB1 = FCBA
	STA FCB1
	LDA #FCBA/256
	STA FCB1+1
	TXA
	STA (FCB1),Y	;SET DRIVE TO DEFAULT
	JSR RESFCB	;RESET FCB1
SETFCB3	JSR GNXTCHR	;GET NEXT CHR, SKIP SPACES
	BCC SETFCB4
	CMP #SP
	BEQ SETFCB3
	BNE SETFCB5

SETFCB4	TAX
	LDY LINPNT
	LDA (LINE),Y	;CHECK FOR ':' AS 2. CHR
	BEQ SETFCB7
	CMP #':
	BEQ SETFCB2
SETFCB7	DEC LINPNT	;ADJUST CMD VECTOR
SETFCB6	LDX #8		;MAX. 8 BYTES FILENAME
	LDY #1
	JSR FILFCB
	BCC SETFCB8	;end of filename
	CMP #'.
	BNE SETFCB5
	LDY #9
	LDX #3		;MAX. 3 BYTES EXTENSION
	JSR FILFCB
	BCC SETFCB8
SETFCB5	JSR RESPNT
	LDA #ILLFN	;ILLEGAL OR NO FILENAME
	SEC
SETFCB8	RTS

SETFCB2	CPX #'A		;CHECK FOR VALID DRIVE CHR
	BCC SETFCB5
	CPX #'I
	BCS SETFCB5
	LDY #0
	TXA
	AND #$0F
	STA (FCB1),Y	;SET DRIVE BYTE
	INC LINPNT
	BNE SETFCB6


FILFCB	JSR GNXTCHR	;fill FCB with filename or ext
	BCS FILFCB7
FILFCB1	STA (FCB1),Y
	INY
	DEX
	BNE FILFCB
FILFCB6	JSR GNXTCHR	;get next chr
FILFCB8	CMP #EOL	;EOL or SPACE only
	BEQ FILFCB5
	CMP #SP
	BEQ FILFCB5
FILFCB7	SEC		;must be a '. or an illegal chr
	RTS

FILFCB5	CLC		;end of FCB reached
	RTS


GNXTCHR	STY YBUF	;GET NEXT CHR FROM INBUF
	LDY LINPNT
	LDA (LINE),Y
	BEQ GNXTCH1
	INC LINPNT
	CMP #'.
	BEQ GNXTCH2
	JSR CHRCHK
GNXTCH2	LDY YBUF
	RTS

GNXTCH1	LDA #EOL
	RTS


RESFCB	LDY #11 	;RESET FCB
	LDA #SP
RESFC1	STA (FCB1),Y
	DEY
	BNE RESFC1
	RTS


GLBL	JSR GWORD
	BCS GLBL1
	RTS

GLBL1	CMP #MISWRD
	BEQ GLBL3
	CPX #0
	BNE GLBL2
	CMP #';
	BNE GLBL2
GLBL3	LDA #EOL
	SEC
	RTS

GLBL2	JMP SYNERR


SAVNOD	LDA NOD
	STA SNOD
	LDA NOD+1
	STA SNOD+1
	RTS


RSTNOD	LDX SNOD	;don't use A!
	STX NOD
	LDX SNOD+1
	STX NOD+1
	RTS


SDFLAG	LDA DFLAG
	ORA #$80
	STA DFLAG
	RTS


RDFLAG	LDA DFLAG
	AND #$7F
	STA DFLAG
	RTS


RNGCHK	LDA OPERA
	LDX OPERA+1
	BEQ RNGCH1
	INX 
	BEQ RNGCH2
RNGCH3	SEC
	RTS

RNGCH1	CMP #$80
	RTS

RNGCH2	CMP #$80
	BCC RNGCH3
	CLC
	RTS


DSTO	LDY #0
	STA (CODE),Y
	INY
	TXA
	STA (CODE),Y
	LDA #2
	STA OPLEN
	RTS


PUTOPC	LDY #0
	LDA OPCOD
	STA (CODE),Y
	INY
	LDA OPERA
	STA (CODE),Y
	INY
	CPY OPLEN
	BEQ PUTOP1
	LDA OPERA+1
	STA (CODE),Y
PUTOP1	RTS


CHKAA	LDY #0
	LDA (CODE),Y
	JSR LENOP
	STA OPLENA
	INY
	LDA (CODE),Y
	CMP #$AA
	BEQ CHKAA1
	LDA OPLENA
	STA OPLEN
	CLC
CHKAA1	RTS


LENOP	LDX #1
	CMP #0
	BEQ LENOPX
	CMP #$40
	BEQ LENOPX
	CMP #$60
	BEQ LENOPX
	LDX #3
	CMP #$20
	BEQ LENOPX
	AND #$1F
	CMP #$19
	BEQ LENOPX
	AND #$0F
	TAX
	LDA LENTAB,X
	RTS

LENOPX	TXA
	RTS


;------------------------------------

SETSW	LDY #0		;SET SWITCHES
	LDA (DMA),Y
	STA TEMP
SETSW2	INY		;SEARCH /
	CPY TEMP
	BCS SETSW1
	LDA (DMA),Y
	CMP #'/
	BNE SETSW2
SETSW5	INY
	CPY TEMP
	BCS SETSW1
	LDA (DMA),Y
	LDX #0
SETSW4	CMP SWTAB,X	;CHECK SWITCH KEY
	BEQ SETSW3
	INX
	INX
	CPX #OPTAB-SWTAB
	BCC SETSW4
SETSW1	RTS

SETSW3	INX		;SET SWITCH
	LDA SWTAB,X
	ORA SWITCH
	STA SWITCH
	INX
	BNE SETSW5


SAVCOM	LDX #3		;SAVE .COM-FILE
	JSR SETEXT	;.EXT = .COM
	JSR FOPEN	;OPEN FILE
	BCS SAVCO2
SAVCO5	JSR SETDMA
SAVCO3	LDX #WRITE
	JSR BDOS
	BCS SAVCO2
	INC DMA+1
	SEC
	LDA DMA
	SBC CODE
	LDA DMA+1
	SBC CODE+1
	BCC SAVCO3
	LDX #CLOSE
	JSR BDOS
SAVCO2	RTS


SAVLBL	LDX #6		;SAVE *.LBL-FILE
	JSR SETEXT	;.EXT = .LBL
	JSR SETDMA
	JSR FOPEN
	BCS SAVLB2
SAVLB7	JSR G1LBL	;GET 1. LABEL
	BCS SAVLB4
	LDY #0
	STY BUFPNT
SAVLB5	JSR WRLBL	;WRITE LABEL
	BCS SAVLB2
	JSR GNLBL	;GET NEXT LABEL
	BCS SAVLB4
	JSR WRLBCR	;WRITE LABEL + CR
	BCS SAVLB2
	JSR GNLBL
	BCC SAVLB5
SAVLB4	LDA #EOTM
	JSR WRLCH
	BCS SAVLB2
	LDA BUFPNT
	BEQ SAVLB3
	LDX #WRITE
	JSR BDOS
	BCS SAVLB2
SAVLB3	LDX #CLOSE
	JSR BDOS
SAVLB2	RTS


FOPEN	LDX #OPEN	;OPEN FILE
	JSR BDOS
	BCC FOPENX
	CMP #NOTFND	;IF NOT FOUND
	BNE FOPEN1
	LDX #CREATE	;THEN CREATE FILE
	JSR BDOS
FOPENX	RTS

FOPEN1	SEC
	RTS


WRLBCR	JSR WRLBL
	BCS WRLBCX
	LDA #CR
	JSR WRLCH
	BCS WRLBCX
	LDA #LF
	JSR WRLCH
WRLBCX	RTS


WRLBL	LDY #0
	LDA (NODSTR),Y
	BPL WRLBL4
	LDA LFLAG
	ORA #$40
	STA LFLAG
	LDA (NODSTR),Y
	AND #$1F
WRLBL4	STA NODLEN
	INC NODLEN
	INY
WRLBL2	STY NODPNT
	LDA (NODSTR),Y
	JSR WRLCH
	BCS WRLBLX
	LDY NODPNT
	INY
	CPY NODLEN
	BCC WRLBL2
WRLBL5	CPY #34
	BCS WRLBL3
	INY
	STY NODPNT
	JSR WRLSP
	BCS WRLBLX
	LDY NODPNT
	BPL WRLBL5

WRLBL3	LDA #'$
	JSR WRLCH
	BCS WRLBLX
	BIT LFLAG
	BVC WRLBL6
	LDX #3
WRLBL8	LDA #'*
	JSR WRLCH
	BCS WRLBLX
	DEX
	BPL WRLBL8
	LDA LFLAG
	AND #$BF
	STA LFLAG
	JMP WRLBL7

WRLBL6	LDY #5
	LDA (NOD),Y
	JSR WRLBY
	BCS WRLBLX
	LDY #4
	LDA (NOD),Y
	JSR WRLBY
	BCS WRLBLX
WRLBL7	JSR WRLSP
	BCS WRLBLX
WRLSP	LDA #SP
	JSR WRLCH
WRLBLX	RTS


WRLBY	PHA
	LSR A
	LSR A
	LSR A
	LSR A
	JSR WRLNI
	PLA
	BCS WRLBYX
	AND #$0F
WRLNI	CMP #$0A
	BCC WRLBY1
	ADC #6
WRLBY1	ADC #$30
	JSR WRLCH
WRLBYX	RTS


WRLCH	LDY BUFPNT
	STA (DMA),Y
	INC BUFPNT
	BEQ WRLCH1
	CLC
	RTS

WRLCH1	LDX #WRITE
	JSR BDOS
	RTS


GADRM	JSR EVAL
	BCS GADRM1
	LDA OPERA+1
	BEQ GADRM2
GADRM5	LDX #0		;ABS
	JMP SETADM

GADRM2	LDX #3		;Z
	JMP SETADM

GADRM1	CMP #NOVAL
	BNE GADRM3
	JSR GADRM5
	SEC
	RTS

GADRM3	CPX #0
	BNE GADRM4
	CMP #'#
	BNE GADRM6
	JSR EVAL
	BCS IMM1
IMM3	LDX #15 	;IMM
	JMP SETADM

IMM1	CMP #NOVAL
	BNE SYNERR
	JSR IMM3
	SEC
	RTS

GADRM6	CMP #'(
	BNE SYNERR
	JMP IND

SYNERR	CMP #$81
	BCS SYNERR1
	LDA #SYNTAX
	SEC
SYNERR1	RTS

GADRM4	CMP #',
	BNE SYNERR
	JSR GXYSP
	BCS SYNERR
	CMP #'Y
	BNE GADRM8
	LDX #6		;ABS,Y
	JMP SETADM

GADRM8	LDA OPERA+1
	BEQ GADRM9
	LDX #9		;ABS,X
	JMP SETADM

GADRM9	LDX #12 	;Z,X
	JMP SETADM


IND	JSR EVAL
	BCC SYNERR
	CMP #NOVAL
	BEQ IND2
	CMP #')
	BEQ IND3
	CMP #',
	BNE SYNERR
	JSR GWORD
	BCC SYNERR
	CMP #')
	BNE SYNERR
	CPX #1
	BNE SYNERR
	LDA LBLSTR,X
	CMP #'X
	BNE SYNERR
	LDY LINPNT
	LDA (LINE),Y
	BEQ IND6
	CMP #SP
	BNE SYNERR
IND6	LDX #21 	;(IND,X)
	JMP SETADM

IND2	JSR IND5
	SEC
	RTS

IND3	LDY LINPNT
	LDA (LINE),Y
	CMP #',
	BNE SYNERR
	INC LINPNT
	JSR GXYSP
	BCS SYNERR
	CMP #'Y
	BNE SYNERR
IND5	LDX #18 	;(IND),Y
SETADM	LDA ADMTAB,X
	ORA MASK
	STA OPCOD
	INX
	LDA ADMTAB,X
	STA OPLEN
	INX
	LDA ADMTAB,X
	CLC
	RTS


GXYSP	JSR GWORD
	BCS GXYSP1
	LDX LBLSTR
	CPX #1
	BNE GXYSP2
	LDA LBLSTR,X
	CMP #'X
	BEQ GXYSP3
	CMP #'Y
	BEQ GXYSP3
GXYSP2	SEC
GXYSP1	RTS

GXYSP3	CLC
	RTS


PLINNR	BIT SWITCH	;PRINT BEGINNING OF LINE
	BPL PLINN1	;CHECK SWITCH
	LDA #LIST
	STA OUTDEV
	JSR PLNR
PLINN1	LDA SWITCH
	AND #$20
	BEQ PLINNX
	LDA #CONOUT
	STA OUTDEV
	JSR PLNR
PLINNX	RTS


PLNR	LDA #CR		;PRINT NEWLINE
	JSR LSTCHR
	LDA #LF
	JSR LSTCHR
	LDY LINCNT+1	;PRINT LINE NR
	LDA LINCNT
	JSR HEXDEC	;IN DECIMAL
	JSR LSTLBL
	JSR LSTSP
	LDA PC+1	;PRINT PC
	JSR LSTHEX
	LDA PC
	JSR LSTHEX
	LDX #2
	JSR PRXSP
	RTS


PLINE	BIT SWITCH
	BPL PLINE1
	LDA #LIST
	STA OUTDEV
	JSR PRLINE
PLINE1	LDA SWITCH
	AND #$20
	BEQ PLINEX
	LDA #CONOUT
	STA OUTDEV
	JSR PRLINE
PLINEX	RTS

PRLINE	LDY #0
PRLIN2	LDA (LINE),Y
	BEQ PRLINX
	JSR LSTCHR
	INY
	BNE PRLIN2
PRLINX	RTS


PHEX	BIT SWITCH
	BPL PHEX1
	LDX #LIST
	STX OUTDEV
	PHA
	JSR LSTHEX
	PLA
PHEX1	TAX
	LDA SWITCH
	AND #$20
	BEQ PHEXX
	TXA
	LDX #CONOUT
	STX OUTDEV	
	JSR LSTHEX
PHEXX	RTS


PRXSP	BIT SWITCH
	BPL PRXSP1
	LDA #LIST
	STA OUTDEV
	JSR LSTXSP
PRXSP1	LDA SWITCH
	AND #$20
	BEQ PRXSPX
	LDA #CONOUT
	STA OUTDEV
	JSR LSTXSP
PRXSPX	RTS


LSTLBL	LDY #0
LSTLBL1	LDA LBLSTR,Y
	JSR LSTCHR
	INY
	CPY #5
	BCC LSTLBL1
	RTS

LSTXSP	STX XTEMP
	TXA
	PHA
LSTXSP2	DEC XTEMP
	BMI LSTXSPX
	JSR LSTSP
	JMP LSTXSP2

LSTXSPX	PLA
	TAX
	RTS

LSTSP	LDA #SP

LSTCHR	LDX OUTDEV
	JMP BDOS


LSTHEX	PHA
	LSR A
	LSR A
	LSR A
	LSR A
	JSR LSTNIB
	PLA
	AND #$0F
LSTNIB	CMP #10
	BCC LSTHE1
	ADC #6
LSTHE1	ADC #$30
	JSR LSTCHR
	RTS


GETMNE	LDX LBLSTR
GETMN3	CPX #3
	BEQ GETMN1
	BCS GETMN2
	INX
	LDA #SP
	STA LBLSTR,X
	BNE GETMN3

GETMN2	LDA #ILLOPC
	SEC
	RTS

GETMN1	LDX #MNR-OPCT+2
GETMN5	JSR OPCHK
	BCC GETMN4
	LDA OPIT,X
	TAX
	BNE GETMN5
	BEQ GETMN2

GETMN4	LDA OPPT,X
	STA MASK
	DEX
	LDA OPPT,X
	STA MAP
	DEX
	LDA OPPT,X
	RTS


OPCHK	STX XTEMP
	LDY #1
	DEX
	DEX
OPCHK3	LDA OPCT,X
	CMP LBLSTR,Y
	BEQ OPCHK1
	BCC OPCHK2
	LDX XTEMP
	DEX
	SEC
	RTS

OPCHK2	LDX XTEMP
	SEC
	RTS

OPCHK1	INX
	INY
	CPY #4
	BCC OPCHK3
	DEX
	CLC
	RTS


EVAL	JSR GOPND	;EVALUATE EXPRESSION
	BCS EVAL2	;GET 1. OPERAND
	STA OPERA
	STX OPERA+1
EVAL4	JSR GOPRT	;GET OPERATOR
	BCS EVAL1
	JSR GOPND	;GET 2. OPERAND
	BCS EVAL2
	STA OPERB
	STX OPERB+1
	LDA OPRT
	CMP #NOVAL
	BEQ EVAL3
	JSR CALC	;DO CALCULATION
	BCC EVAL4
EVAL2	RTS

EVAL1	CMP #EOL
	BNE EVAL5
	CLC
	RTS

EVAL3	STX OPERA	;SET OPERA = $AAAA
	STX OPERA+1
EVAL5	SEC
	RTS


GOPND	JSR GWORD
	STA OPRT
	BCS GOPND1
GOPND3	JSR GDEC
	BCS GOPND2
	RTS

GOPND1	CPX #0
	BNE GOPND3
	CMP #'$
	BNE GOPND4
	JMP GHEX
GOPND4	CMP #'%
	BNE GOPND5
	JMP GBIN
GOPND5	CMP #''
	BNE GOPND6
	JMP GASC
GOPND6	SEC
	RTS

GOPND2	CMP #ILLDEC
	BNE GOPND6
	JSR SEARCH
	BCS GOPND7
	JSR LBLOUT
	BCC GOPND8
GOPND7	LDA #NOVAL
	STA OPRT
	LDA #$AA
	TAX
	CLC
GOPND8	RTS


GOPRT	LDA OPRT
	BEQ GOPRT1
	CMP #NOVAL
	BEQ GOPRT2
	CMP #SP
	BEQ GOPRT3
GOPRT8	LDX #KEYTAB-OPTAB-1
GOPRT5	CMP OPTAB,X
	BEQ GOPRT4
	DEX
	BPL GOPRT5
	SEC
	RTS

GOPRT4	STX XTEMP
	CLC
	RTS

GOPRT1	LDA #EOL
	SEC
GOPRT2	RTS

GOPRT3	JSR GWORD
	BCS GOPRT6
GOPRT7	LDA #SYNTAX
	SEC
	RTS

GOPRT6	CMP #MISWRD
	BEQ GOPRT1
	CPX #0
	BNE GOPRT7
	CMP #';
	BEQ GOPRT1
	BNE GOPRT8


CALC	LDA XTEMP
	ASL A
	TAX
	LDA CTAB,X
	PHA
	INX
	LDA CTAB,X
	PHA
	RTS


GWORD	LDX #0
	LDY LINPNT
GWORD2	LDA (LINE),Y
	BEQ GWORD1
	INY
	CMP #SP
	BEQ GWORD2
GWORD6	JSR CHRCHK
	BCS GWORD3
	INX
	CPX #MAX_LBL		;check for max. length
	BCS GWORD4
	STA LBLSTR,X
	LDA (LINE),Y
	BEQ GWORD5
	INY
	BNE GWORD6
GWORD1	LDA #MISWRD
GWORD7	STX LBLSTR
	STY LINPNT
	SEC
	RTS

GWORD3	CMP #SP
	BNE GWORD7
GWORD5	STX LBLSTR
	STY LINPNT
	CLC
	RTS

GWORD4	LDA #WRDOVL
	BNE GWORD7


GNCHR	LDA (LINE),Y	;GET NEXT CHR
	BEQ GNCHR2	;END OF LINE?
	CMP #';		;COMMENT?
	BEQ GNCHR1
	INY
	CMP #SP		;IF SPACE THEN GET NEXT CHR
	BEQ GNCHR
	CLC
GNCHR1	RTS

GNCHR2	SEC
	RTS
	

SETDMA	LDA #CODBEG	;SET DMA TO CODEBUFFER
	STA DMA
	LDA #CODBEG/256
	STA DMA+1
	RTS


SETEXT	LDY #9		;SET EXTENSION
	JSR SETEXB	;X=EXT-POINTER
	JSR SETEXB
SETEXB	LDA EXTTAB,X	;SET 1 EXTENSION BYTE
	STA (FCB1),Y
	INY
	INX
	RTS


GASC	LDY LINPNT
	LDA (LINE),Y
	TAX
	INY
	LDA (LINE),Y
	BEQ GASC1
	CMP #SP
	BEQ GASC1
	CMP #''
	BEQ GASC2
GASC3	LDA #SYNTAX
	STY LINPNT
	SEC
	RTS

GASC2	INY
	LDA (LINE),Y
	INY
GASC1	STA OPRT
	TXA
	LDX #0
	STY LINPNT
	CLC
	RTS


GHEX	LDA #4
	STA HEXCNT
	JSR RESOPB
	LDA #SP
	STA OPRT
	LDY LINPNT
GHEX4	LDA (LINE),Y
	BEQ GHEX1
	INY
	JSR ASCHEX
	BCS GHEX1
	LDX #4
GHEX3	ASL OPERB
	ROL OPERB+1
	DEX
	BNE GHEX3
	ORA OPERB
	STA OPERB
	DEC HEXCNT
	BNE GHEX4
GHEX5	LDA OPERB
	LDX OPERB+1
	STY LINPNT
	CLC
	RTS

GHEX1	STA OPRT
	LDA HEXCNT
	BNE GHEX5
	STY LINCNT
	JMP SYNERR


RESOPB	LDA #0
	STA OPERB
	STA OPERB+1
	RTS


ASCHEX	TAX
	JSR CHRCHK
	BCS ASCHE1
	CMP #'G
	BCS ASCHE1
	CMP #'A
	BCS ASCHE2
ASCDEC	CMP #$3A
	BCS ASCHE1
	CMP #'0
	BCC ASCHE1
ASCHE3	AND #$0F
	CLC
	RTS

ASCHE1	TXA
	SEC
	RTS

ASCHE2	SBC #7
	BNE ASCHE3


GBIN	LDA #16
	STA HEXCNT
	LDA #SP
	STA OPRT
	JSR RESOPB
	LDY LINPNT
GBIN2	LDA (LINE),Y
	BEQ GBIN1
	INY
	JSR ASCBIT
	BCS GBIN1
	ASL OPERB
	ROL OPERB+1
	ORA OPERB
	STA OPERB
	DEC HEXCNT
	BNE GBIN2
GBIN3	LDA OPERB
	LDX OPERB+1
	STY LINPNT
	CLC
	RTS

GBIN1	STA OPRT
	LDA HEXCNT
	BNE GBIN3
	STY LINPNT
	JMP SYNERR


ASCBIT	CMP #'0
	BEQ ASCBI1
	CMP #'1
	BEQ ASCBI1
	SEC
	RTS

ASCBI1	AND #$01
	CLC
	RTS


GDEC	JSR RESOPB
	LDX #0
GDEC4	INX
	LDA LBLSTR,X
	JSR ASCDEC
	BCS GDEC2
	STA HEXCNT
	ASL OPERB
	ROL OPERB+1
	BCS GDEC3
	LDA OPERB
	LDY OPERB+1
	ASL OPERB
	ROL OPERB+1
	BCS GDEC3
	ASL OPERB
	ROL OPERB+1
	BCS GDEC3
	ADC OPERB
	STA OPERB
	TYA
	ADC OPERB+1
	STA OPERB+1
	BCS GDEC3
	LDA HEXCNT
	ADC OPERB
	STA OPERB
	BCC GDEC1
	INC OPERB+1
	BEQ GDEC3
GDEC1	CPX LBLSTR
	BCC GDEC4
	LDA OPERB
	LDX OPERB+1
	CLC
	RTS

GDEC3	LDA #OFLOW
	SEC
	RTS

GDEC2	LDA #ILLDEC
	SEC
	RTS


CHRCHK	CMP #'A 	;CHECK FOR VALID CHR
	BCC CHRCH1
	CMP #'[
	BCS CHRCH3
	RTS

CHRCH3	CMP #'a
	BCC CHRCH1
	CMP #'{
	BCS CHRCH1
	AND #$5F	;LOWER TO UPPER CASE
	RTS

CHRCH1	STY YTEMP
	LDY #ADMTAB-KEYTAB-1
CHRCH2	CMP KEYTAB,Y
	BEQ CHRCH4
	DEY
	BPL CHRCH2
	LDY YTEMP
	SEC
	RTS

CHRCH4	LDY YTEMP
	CLC
	RTS

;------------------------------------

LBLPUT	JMP LBLINI	;VECTOR FOR LBLPUT

LBLRST	LDA #LBLINI	;PATCH JMP LBLPUT
	STA LBLPUT+1
	LDA #LBLINI/256
	STA LBLPUT+2
	LDA BDOS+1	;HEAP=BDOS-1
	LDY BDOS+2
	STA HEAP
	STA HIMEM
	STY HEAP+1
	STY HIMEM+1
	SEC		;place NIL entry in ROOT
	SBC #8
	STA ROOT
	DEY
	STY ROOT+1
	LDA #0
	LDY #7
LBLRS1	STA (ROOT),Y
	DEY
	BPL LBLRS1
	LDY #6
	LDA #$81
	STA (ROOT),Y
	RTS


LBLINI	JSR LBLSTO	;put 1. label to label tree
	BCS LBLINX
	LDA NOD		;set ROOT to 1. label
	STA ROOT
	LDA NOD+1
	STA ROOT+1
	LDA #LBLPU	;PATCH JMP LBLPUT
	STA LBLPUT+1
	LDA #LBLPU/256
	STA LBLPUT+2
	CLC
LBLINX	RTS


LBLPU	JSR SEARCH	;FIND LEAF
	BCC LBLPU1
	STY NODLEN	;SAVE NOD
	LDA NOD
	STA PNOD
	LDA NOD+1
	STA PNOD+1
	JSR LBLSTO
	BCS LBLPU2
	LDY NODLEN	;SET PNOD
	LDA NOD
	STA (PNOD),Y
	INY
	LDA NOD+1
	STA (PNOD),Y
	CLC
LBLPU2	RTS

LBLPU1	LDA #DUPLBL	;DUPLICATE LABEL
	SEC
	RTS


LBLSET	LDY #4		;SET VALUE
	STA (NOD),Y
	INY
	TXA
	STA (NOD),Y
	INY
	LDA (NOD),Y	;RESET EMPTY FLAG
	BPL LBLSE1
	AND #$7F
	STA (NOD),Y
	CLC
	RTS

LBLSE1	LDA #DUPVAL
	SEC
	RTS


LBLOUT	LDY #6
	LDA (NOD),Y
	BMI LBLOU1
	DEY
	LDA (NOD),Y
	TAX
	DEY
	LDA (NOD),Y
	CLC
	RTS

LBLOU1	LDA #NOVAL
	SEC
	RTS


LBLSTO	CLC		;store new label
	LDA HEAP	;calculate room
	LDY HEAP+1
	SBC LBLSTR	;NODSTR = HEAP-LBLSTR-1
	BCS LBLST5
	DEY
LBLST5	STA NODSTR
	STY NODSTR+1
	SEC		;NOD = A/Y-6
	SBC #6
	BCS LBLST6
	DEY
LBLST6	STA NOD
	STA HEAP	;HEAP=NOD
	STY NOD+1
	STY HEAP+1
	CPY CODE+1	;HEAP > CODE ?
	BNE LBLST1
	CMP CODE
	BCC LBLST2
	BEQ LBLST2
LBLST1	LDA LBLSTR
	TAY
	ORA #$80	;SET EMPTY FLAG
	PHA
LBLST3	LDA LBLSTR,Y	;SET LABEL
	STA (NODSTR),Y
	DEY
	BNE LBLST3
	PLA
	STA (NODSTR),Y
	TYA		;RESET VECTORS
	LDY #5
LBLST4	STA (NOD),Y
	DEY
	BPL LBLST4
	CLC
	RTS

LBLST2	LDA #MEMOVL	;MEMORY FULL
	SEC
	RTS


SEARCH	LDA ROOT	;SEARCH LABEL IN LBLSTR
	LDX ROOT+1	;NOD=ROOT
SEARC4	JSR SETNOD
	JSR LBLCHK	;C=0 found, C=1 Y=index to next NOD vector
	BCC SEARC5	;FOUND
SEARC1	LDA (NOD),Y
	TAX
	BNE SEARC2
	DEY
	LDA (NOD),Y
	BNE SEARC4	;if (NOD <> $0000 then next NOD
	SEC		;NOT FOUND
SEARC5	RTS

SEARC2	DEY
	LDA (NOD),Y	;SET FOR NEXT NOD
	JMP SEARC4


LBLCHK	LDY #0		;CHECK LBLSTR VS. NODSTR
	LDA (NODSTR),Y	;SET NODLEN
	AND #$1F	;mask flag bits
	STA NODLEN
	LDA LBLSTR
	STA LBLLEN
LBLCH4	INY		;COMPARE
	LDA LBLSTR,Y
	CMP (NODSTR),Y
	BEQ LBLCH1
	BCS LBLCH2
LBLCH6	LDY #1		;<
	SEC
	RTS

LBLCH1	DEC LBLLEN
	BEQ LBLCH3
	DEC NODLEN
	BNE LBLCH4	;next chr
LBLCH2	LDY #3		;>
	SEC
	RTS

LBLCH3	DEC NODLEN
	BNE LBLCH6	;<
	CLC		;=
	RTS


G1LBL	LDA HEAP	;GET 1. LABEL
	LDX HEAP+1
	JMP GNLBL2


GNLBL	LDY #0		;GET NEXT LABEL
	LDA (NODSTR),Y
	AND #$1F	;MASK FLAGS
	SEC
	ADC NODSTR
	STA NOD
	LDA NODSTR+1
	ADC #0
	TAX
	LDA NOD 	;A/X=NEXT NOD
GNLBL2	JSR SETNOD
	LDA NOD+1
	CMP HIMEM+1
	BCC GNLBL1
	LDA NOD
	CMP HIMEM
GNLBL1	RTS


PLUS	CLC		;OPERA+OPERB
	LDA OPERA
	ADC OPERB
	STA OPERA
	LDA OPERA+1
	ADC OPERB+1
	STA OPERA+1
	CLC		;NO OVERFLOW
	RTS


MINUS	SEC		;OPERA-OPERB
	LDA OPERA
	SBC OPERB
	STA OPERA
	LDA OPERA+1
	SBC OPERB+1
	STA OPERA+1
	CLC		;NO UNDERFLOW
	RTS


MULT	JSR COPATX	;OPERA*OPERB
	LDX #16 	;16 BIT MULT
MULT1	DEX
	BEQ MULTX
	LSR OPERX+1	;SHR X
	ROR OPERX
	BCS MULT2
MULT3	ASL OPERB	;SHL B
	ROL OPERB+1
	JMP MULT1

MULT2	JSR PLUS	;A+B
	JMP MULT3

MULTX	CLC		;MOD 16 BIT
	RTS


DIV	JSR COPATX	;A/B 16 BIT DIV
	LDX #16 	;16 BIT
	LDY #0
	LDA OPERX+1	;NORMALIZE X
	BMI DIV3
DIV1	DEY
	DEX
	BEQ DIV2	;A=0
	ASL OPERX	;SHL X
	ROL OPERX+1	;NORMALIZE X
	BPL DIV1
	LDX #16
DIV3	LDA OPERB+1
	BMI DIV6	;NORMALIZE B
DIV8	INY
	DEX
	BEQ DIV5	;DIVISION BY 0
	ASL OPERB	;SHL B
	ROL OPERB+1
	BPL DIV8
DIV6	TYA		;B>X?
	BMI DIV2
	BNE DIV4
	JSR DIVSUB	;X-A
	BCC DIV2
	BCS DIV7
DIV4	JSR DIVSUB	;X-A
DIV7	ROL OPERA	;SHIFT BIT IN A
	ROL OPERA+1
	LSR OPERB+1	;SHR B
	ROR OPERB
	DEY		;SHIFT COUNT
	BPL DIV4
DIV2	CLC
	RTS

DIV5	LDA #$AA
	STA OPERA
	STA OPERA+1
	LDA #DIV0
	SEC
	RTS


DIVSUB	SEC		;X-B
	LDA OPERX
	SBC OPERB
	TAX
	LDA OPERX+1
	SBC OPERB+1
	BCC DIVSUB1
	STA OPERX+1
	STX OPERX
DIVSUB1	RTS


SETNOD	STA NOD 	;SET NOD
	STX NOD+1
	CLC		;SET NODSTR
	ADC #6
	STA NODSTR
	TXA
	ADC #0
	STA NODSTR+1
	RTS


COPATX	LDA OPERA	
	STA OPERX	;X=A
	LDA OPERA+1
	STA OPERX+1
	LDA #0		;A=0
	STA OPERA
	STA OPERA+1
	RTS


HEXDEC	STA OPERX		;HEX TO DECIMAL CONVERSION
	STY OPERX+1
	LDX #4
HEXDEC1	JSR DIV10		;/10 RESIDUAL IN A
	ORA #$30
	STA LBLSTR,X
	DEX
	BPL HEXDEC1
	RTS


DIV10	LDA #0			;OPERX/10
	LDY #15
	CLC
DIV10A	ROL OPERX
	ROL OPERX+1
	ROL A
	SBC #9			;C=0 !!
	BCS DIV10B
	ADC #10			;RESTORE A
	CLC
DIV10B	DEY
	BPL DIV10A
	ROL OPERX
	ROL OPERX+1
	RTS


ERROR	STA ERRNO
	CMP #$B0
	BCS STDERR
	LDA #ERR1M
	LDY #ERR1M/256
	JSR PRTSTR
	LDY LINCNT+1
	LDA LINCNT
	JSR HEXDEC		;CONVERT TO DECIMAL
	LDY #0
ERROR9	LDA LBLSTR,Y		;AND PRINT IT
	JSR PRTCHR
	INY
	CPY #5
	BCC ERROR9
	LDA #ERR2M
	LDY #ERR2M/256
	JSR PRTSTR
	LDA LINPNT
	JSR PRTHEX
	JSR PRT2SP	;2 SPACES
	JSR STDERR1	;PRINT ERROR MESSAGE
	JSR CRLF
	LDA LINE	;PRINT LINE
	LDY LINE+1
	JSR PRTSTR
	JSR ICODPC	;inc code and pc by oplen (usually 3!)
	INC ERRCNT
	LDA ERRCNT
	CMP #MAX_ERR	;MAX NR OF ERRORS REACHED?
	BCS ERRORX
	RTS

ERRORX	LDA #ERR_OVL	;TOO MUCH ERRORS
	STA ERRNO	;ERROR ROUTINE

STDERR	JSR CRLF
STDERR1	LDA #ERRTAB	;SET CCPV
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
	JSR PRT2SP
	RTS


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


ASKRTY	LDA #RTYMES	;RETRY?
	LDY #RTYMES/256
	JSR PRTSTR
	LDX #CONIN
	JSR BDOS
	CMP #'y
	BEQ ASKRT2
	CMP #'Y
	BNE ASKRT1
ASKRT2	CLC
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


;=== OPCODE TABLES ===

OPCT	DB '=  '
	DB 'ADC'
	DB 'AND'
	DB 'ASL'
	DB 'BCC'
	DB 'BCS'
	DB 'BEQ'
	DB 'BIT'
	DB 'BMI'
	DB 'BNE'
	DB 'BPL'
	DB 'BRK'
	DB 'BVC'
	DB 'BVS'
	DB 'CLC'
	DB 'CLD'
	DB 'CLI'
	DB 'CLV'
	DB 'CMP'
	DB 'CPX'
	DB 'CPY'
	DB 'DB '
	DB 'DD '
	DB 'DEC'
	DB 'DEX'
	DB 'DEY'
	DB 'DS '
	DB 'DW '
	DB 'END'
	DB 'EOR'
	DB 'INC'
MNR	DB 'INX'	;32
	DB 'INY'
	DB 'JMP'
	DB 'JSR'
	DB 'LDA'
	DB 'LDX'
	DB 'LDY'
	DB 'LSR'
	DB 'NOP'
	DB 'ORA'
	DB 'ORG'
	DB 'PHA'
	DB 'PHP'
	DB 'PLA'
	DB 'PLP'
	DB 'ROL'
	DB 'ROR'
	DB 'RTI'
	DB 'RTS'
	DB 'SBC'
	DB 'SEC'
	DB 'SED'
	DB 'SEI'
	DB 'STA'
	DB 'STX'
	DB 'STY'
	DB 'TAX'
	DB 'TAY'
	DB 'TSX'
	DB 'TXA'
	DB 'TXS'
	DB 'TYA'
	DB 'APP'

OPIT	DB 0,0,0	;= 
	DB 0,2,8	;ADC 2
	DB 0,0,191	;AND
	DB 0,5,17	;ASL	4
	DB 0,0,0	;BCC
	DB 0,14,20	;BCS 6
	DB 0,0,0	;BEQ
	DB 0,11,35	;BIT	8
	DB 0,0,0	;BMI
	DB 0,26,32	;BNE 10
	DB 0,0,0	;BPL
	DB 0,29,41	;BRK	12
	DB 0,0,0	;BVC
	DB 0,38,44	;BVS 14
	DB 0,0,0	;CLC
	DB 0,23,71	;CLD	16
	DB 0,0,0	;CLI
	DB 0,50,56	;CLV 18
	DB 0,0,0	;CMP
	DB 0,53,65	;CPX	20
	DB 0,0,0	;CPY
	DB 0,62,68	;DB	22
	DB 0,0,0	;DD
	DB 0,59,83	;DEC	24
	DB 0,0,0	;DEX
	DB 0,74,80	;DEY 26
	DB 0,0,0	;DS
	DB 0,77,89	;DW	28
	DB 0,0,0	;END
	DB 0,86,92	;EOR 30
	DB 0,0,0	;INC
	DB 0,47,143	;INX	32
	DB 0,0,0	;INY
	DB 0,98,104	;JMP 34
	DB 0,0,0	;JSR
	DB 0,101,113	;LDA	36
	DB 0,0,0	;LDX
	DB 0,110,116	;LDY 38
	DB 0,0,0	;LSR
	DB 0,107,131	;NOP	40
	DB 0,0,0	;ORA
	DB 0,122,128	;ORG 42
	DB 0,0,0	;PHA
	DB 0,125,137	;PHP	44
	DB 0,0,0	;PLA
	DB 0,134,140	;PLP 46
	DB 0,0,0	;ROL
	DB 0,119,167	;ROR	48
	DB 0,0,0	;RTI
	DB 0,146,152	;RTS 50
	DB 0,0,0	;SBC
	DB 0,149,161	;SEC	52
	DB 0,0,0	;SED
	DB 0,158,164	;SEI 54
	DB 0,0,0	;STA
	DB 0,155,179	;STX	56
	DB 0,0,0	;STY
	DB 0,170,176	;TAX 58
	DB 0,0,0	;TAY
	DB 0,173,185	;TSX	60
	DB 0,0,0	;TXA
	DB 0,182,188	;TXS 62
	DB 0,0,0	;TYA
	DB 0,0,0	;APP 64

OPPT	DB $0B,$00,$00	;= 
	DB $00,$FF,$61	;ADC
	DB $00,$FF,$21	;AND
	DB $03,$37,$02	;ASL
	DB $01,$00,$90	;BCC
	DB $01,$00,$B0	;BCS
	DB $01,$00,$F0	;BEQ
	DB $00,$12,$20	;BIT
	DB $01,$00,$30	;BMI
	DB $01,$00,$D0	;BNE
	DB $01,$00,$10	;BPL
	DB $02,$00,$00	;BRK
	DB $01,$00,$50	;BVC
	DB $01,$00,$70	;BVS
	DB $02,$00,$18	;CLC
	DB $02,$00,$D8	;CLD
	DB $02,$00,$58	;CLI
	DB $02,$00,$B8	;CLV
	DB $00,$FF,$C1	;CMP
	DB $05,$13,$E0	;CPX
	DB $05,$13,$C0	;CPY
	DB $07,$00,$00	;DB
	DB $09,$00,$00	;DD
	DB $00,$36,$C2	;DEC
	DB $02,$00,$CA	;DEX
	DB $02,$00,$88	;DEY
	DB $0A,$00,$00	;DS
	DB $08,$00,$00	;DW
	DB $0C,$00,$00	;END
	DB $00,$FF,$41	;EOR
	DB $00,$36,$E2	;INC
	DB $02,$00,$E8	;INX
	DB $02,$00,$C8	;INY
	DB $04,$00,$4C	;JMP
	DB $04,$00,$20	;JSR
	DB $00,$FF,$A1	;LDA
	DB $05,$5B,$A2	;LDX
	DB $05,$37,$A0	;LDY
	DB $03,$37,$42	;LSR
	DB $02,$00,$EA	;NOP
	DB $00,$FF,$01	;ORA
	DB $06,$00,$00	;ORG
	DB $02,$00,$48	;PHA
	DB $02,$00,$08	;PHP
	DB $02,$00,$68	;PLA
	DB $02,$00,$28	;PLP
	DB $03,$37,$22	;ROL
	DB $03,$37,$62	;ROR
	DB $02,$00,$40	;RTI
	DB $02,$00,$60	;RTS
	DB $00,$FF,$E1	;SBC
	DB $02,$00,$38	;SEC
	DB $02,$00,$F8	;SED
	DB $02,$00,$78	;SEI
	DB $00,$FE,$81	;STA
	DB $05,$1A,$82	;STX
	DB $05,$16,$80	;STY
	DB $02,$00,$AA	;TAX
	DB $02,$00,$A8	;TAY
	DB $02,$00,$BA	;TSX
	DB $02,$00,$8A	;TXA
	DB $02,$00,$9A	;TXS
	DB $02,$00,$98	;TYA
	DB $0D,$00,$00	;APP

;======	TABLES ======

XTAB	DD NORMA-1,BRAA-1,ONEA-1,ACCUA-1,JUMPA-1,REGA-1
	DD ORG-1,DBA-1,DWA-1,DWA-1,DSA-1,EQUA-1,END-1
	DD APP-1
X2TAB	DD NORMB-1,BRAB-1,ONEB-1,ACCUB-1,JUMPB-1,REGB-1
	DD ORGB-1,DBB-1,DWB-1,DDB-1,DSB-1,EQUB-1,ENDB-1
	DD APP-1
CTAB	DD PLUS-1,MINUS-1,MULT-1,DIV-1

LENTAB	DB 2,2,2,4,2,2,2,4,1,2,1,4,3,3,3,4
SWTAB	DB 'P',$80,'p',$80,'C',$40,'c',$40
	DB 'L',$20,'l',$20,' ',0,'/',0
OPTAB	DB '+-*/'
KEYTAB	DB '!&.@^_=:<>'
	DB '0123456789'
ADMTAB	DB $0C,3,$10,$04,2,$02,$18,3,$40
	DB $1C,3,$20,$14,2,$04,$08,2,$01
	DB $10,2,$08,$00,2,$80
EXTTAB	DB 'ASM','COM','LBL'

;======	MESSAGES ======

ERR1M	DB CR,LF,'Line ',EOT
ERR2M	DB '	Pos $',EOT
ERRM1	DB ' Error $',EOT
RTYMES	DB CR,LF,'Retry	(Y/N) ? ',EOT

ETYPTB	DB 'BIOS',$00,'BDOS',$00,'CCP',$00,$00
	DB 'RSX',$00,$00,'ASM',$00,$00

ERRTAB	DB $F9,$01,'Invalid drive',EOT
	DB $DE,$82,'Directory full',EOT
	DB $DD,$82,'File not found',EOT
	DB $DC,$02,'File exists',EOT
	DB $D9,$02,'Invalid FCB',EOT
	DB $D8,$82,'Disk full',EOT
	DB $D6,$82,'File is R/O',EOT
	DB $B6,$10,'Illegal filename',EOT
	DB $B5,$10,'Out of memory',EOT
	DB $B4,$10,'Line counter overflow',EOT
	DB $B3,$10,'Line too long',EOT
	DB $B2,$10,'Too many errors',EOT
	DB $B1,$10,'Extension must be omitted',EOT
	DB $87,$10,'WARNING: Zero page adressing conflict',EOT
	DB $88,$10,'Branch out of range',EOT
	DB $89,$10,'Label missing',EOT
	DB $8A,$10,'Word missing',EOT
	DB $8B,$10,'Argument missing',EOT
	DB $8C,$10,'Numerical overflow',EOT
	DB $8D,$10,'Illegal opcode',EOT
	DB $8E,$10,'Syntax error',EOT
	DB $8F,$10,'Illegal adressing mode',EOT
	DB $90,$10,'No value assigned to label',EOT
	DB $91,$10,'JMP (ind) WARNING',EOT
	DB $92,$10,'Word too long',EOT
	DB $93,$10,'Illegal decimal',EOT
	DB $94,$10,'Duplicate label',EOT
	DB $95,$10,'Label previsiously assigned',EOT
	DB $98,$10,'Division by 0',EOT
	DB $00

CRLFM	DB CR,LF,EOT
PASS1M	DB CR,LF,'ASM-65   
	DB ' V',VERSION/16+$30,'.',VERSION*$1000/$1000+$30
	DB '    (c) D. Lausberg'
	DB CR,LF,CR,LF,'... PASS 1 ...',CR,LF,EOT
PASS2M	DB CR,LF,CR,LF,'... PASS 2 ...',CR,LF,EOT
PRTAB	DB $1B,'!',4,$1B,$4E,$06,$1B,'l',10,CR,LF,'ASM-65   '
	DB ' V',VERSION/16+$30,'.',VERSION*$1000/$1000+$30
	DB '      (c) D. Lausberg  1991-2021',CR,LF,CR,LF,EOT
SCOMM	DB CR,LF,'*.COM-File saved',EOT
SLBLM	DB CR,LF,'*.LBL-File saved',EOT
PRST1M	DB CR,LF,CR,LF,'end of assembly'
	DB CR,LF,'bytes free $',EOT

FCBA			;FCB FOR APPEND
LBEG	= FCBA+32	;line Buffer
CODBEG	= LBEG+128	;code begins here

	END
