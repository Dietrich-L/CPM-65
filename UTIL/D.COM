�1� Y ��� � � ���)�  y�� ��� ��  ����� � ����� ��𩵠 Y�� � ` ���
�Р YLC � ��T� Y � �L| ���	 ����� E��� Y  ���� YLC� ���� � �� �`� � �iA��� ���$Hi@���E��ܠ Y� �  Uh8�� � �����` !� ���ȱ� 8�� ��	ȱ�
 QL$0�`���
��i ��i���`8�����������`� ȱ�	�0�	��
�`�0�`�� ������`�� �ȥ��e���`� ��ȱ�`���` � ����L EL�� � � � � � a �� 4�. c� 4 ^��� ��� ��J&����8i e���`��He���hL����	��&��Ff����`�� � c ����`� �� ��� ��)HFfFfFfh������ ���	� ^ ^`� U`���� �����&�e��e��e����ؠ�  ���`
H�0��1 ch`���	�����`���� �	�?������`� U�ޅ��� ���)�� h�� hL hH 2 h��� Yh	 q�` I8`� J����������� c������ Y� ��.��	L�  a� �L� ������`��� Y� � �y��Y�8``HJJJJ �h)$0� ����
�ii0 c` Error $ 
Retry  (Y/N) ?  BIOS BDOS CCP  RSX  USER ��Drive not ready �Invalid Drive ݂File not found �File exists �Invalid FCB  
 
D.COM V2.2      (c) D.Lausberg
 
Attr filename.ext   rec    kB    Attr filename.ext   rec    kB
 

total diskspace used   kB 

Directory finished

 
no file
 
Insert disc to drive B:
and press any key 
 Y #FCB_EXT+1		;compare FN&ext
	BMI C_DIRL
	RTS

SWDIR	LDY #3			;switch dir entries
SWDIRL	LDA FN1,Y
	STA (BUFINDP),Y
	DEY
	BPL SWDIRL
	LDA #$FF
	STA EX_FLAG		;switch occurred
	RTS


SavBDP	LDA BUFPNT		;save BUFPNT to BUFDIR & INC 