;RESTART.ASM - Junior Computer SD Version
;(c) by D. Lausberg

;restart CPM-65 von SD

;V1.0	28.07.23
Version = $10

;---- System Constants
RESET	= $F800
TPA	= $200

BDOS	= $F0

;CONSTANTS

CR	= $D
LF	= $A
EOT	= $0

;---- BDOS COMMANDS

CONIN	= $01		;CONSOLE INPUT
CONOUT	= $02		;CONSOLE OUTPUT
STROUT	= $09		;PRINT STRING


	ORG TPA

RESTART	LDA #BOOTM	;GREETING MESSAGE
	LDY #BOOTM/256
	LDX #STROUT
	JSR BDOS

	JMP RESET



;---  MESSAGES  -----------------------------

BOOTM	DB CR,LF,'Reset to CP/M-65 on Junior Computer   V'
	DB VERSION/16+$30,'.',VERSION*$1000/$1000+$30
	DB CR,LF,'Press <RUBOUT>   ',EOT

;--------------------------------------------

	END