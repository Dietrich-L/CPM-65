;TEST.ASM - Testprogramm f}r den ASM-65
;
;(c) D. Lausberg    21.11.90
;
;
;Kommentare

;Leerzeile
	;Tabulator ';' unter dem l

;Deklarationen

TPA	= $200
CR	= $0D
LF	= $0A

;Pseudo Opcodes

	ORG $1000
	ORG TPA
	DB 1,%0111,'@ABC',CR	;$01,$07...
	DB $40,$41,$42		;@AB
	DB BRANCH-1
	DW $1234,$56
	DW BRANCH-1
	DD $1234,$56
	DD BRANCH-1,BRANCH-2	;VORWAERTSREFERENZ!
FLAG	DS $10
FLAG1	DS $10,$FF
FLAG2

;Normale Opcodes

	ADC #1	;$69
	AND #1	;$29
	BIT 1	;$24
	CMP #1	;$C9
	DEC 1	;$C6
	EOR #1	;$49
	INC 1	;$E6
	LDA #1	;$A9
	ORA #1	;$09
	SBC #1	;$E9
	STA 1	;$85

;Branch Instruktionen

BRANCH	BCC BRANCH	;$90
	BCS BRANCH	;$B0
	BEQ BRANCH	;$F0
	BMI BRANCH	;$30
	BNE BRANCH	;$D0
	BPL BRANCH	;$10
	BVC BRANCH	;$50
	BVS 1_BYTE_OPCODE	;$70

1_BYTE_OPCODE	;1-Byte Opcodes

	BRK	;$00
	CLC	;$18
	CLD	;$D8
	CLI	;$58
	CLV	;$B8
	DEX	;$CA
	DEY	;$88
	INX	;$E8
	INY	;$C8
	NOP	;$EA
	PHA	;$48
	PHP	;$08
	PLA	;$68
	PLP	;$28
	RTI	;$40
	RTS	;$60
	SEC	;$38
	SED	;$F8
	SEI	;$78
	TAX	;$AA
	TAY	;$A8
	TSX	;$BA
	TXA	;$8A
	TXS	;$9A
	TYA	;$98

;ACCU-INSTRUKTIONEN

	ASL $2	;$06
	ASL A	;$0A
	LSR A	;$4A
	ROL A	;$2A
	ROR A	;$6A

;JUMP-INSTRUKTIONEN

	JMP $200	;$4C
	JSR $200	;$20
	JMP ($200)	;$6C
	JMP APPLABEL

;XY-Register Instruktionen

	CPX #10		;$E0
	CPY #$BC	;$C0
	LDX TPA		;$AE
	LDY #'A		;$A0
	STX TPA		;$8E
	STX $40,Y	;$96
	STY $10		;$84
	STY $10,X	;$94

;ARGUMENTE

	LDA #0		;$00
	LDA #10		;$0A
	LDA #255	;$FF

	LDA #$1		;$01
	LDA #$08	;$08
	LDA #$AA	;$AA

	LDA #'A		;$41
	LDA #''		;$27
	LDA #';		;$3B
	LDA #' '	;$20
	LDA #' 		;$20
	LDA #'		;$20

	LDA #%1		;$01
	LDA #%010101	;$15
	LDA #%10001111	;$8F

;Calcuations

	LDA #1+1		;$02
	LDA #$55-$10		;$45
	LDA #11*8		;$58
	LDA $1234/256		;$12
	LDA $6543/256+2		;$67
	LDA $FFFF/1		;$FFFF
	LDA $FEDC/256		;$FE
	LDA #$FFFF/$FFFF	;$1
	LDA #1/1		;$1
	LDA #1/2		;$0
	LDA #10000/100		;$64
	LDA #$1234/256+1*256	;$1300
	LDA #$1234/256+1*256-$1234	;
	LDA #$100-$34
;Adressierungsarten

	LDA #1		;$A9
	LDA $1111	;$AD
	LDA $5A		;$A5
	LDA (10,X)	;$A1
	LDA ($02),Y	;$B1
	LDA $50,X	;$B5
	LDA $1234,X	;$BD
	LDA $5678,Y	;$B9

;APP

	APP Test.app

;post app
