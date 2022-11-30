CPM-65
===========
Dietrich Lausberg <lausbergd@gmail.com>
https://github.com/dietrich-l

This repository contains CPM-65, a CP/M-80 analogue operating system for 6502 based microcomputers

System Requirements
--------------------------
6502 processor
12 kByte RAM (not tested)
6532 I/O chip for the system timer (could be replaced by a 6522 or other)
min 1 Floppy drive
Serial-I/O

My system is a heavily modified and expanded Elektor Junior Computer with 57 kB RAM. For system components and memory map see separate docs

System Structure
--------------------
CPM-65 consists of 3 layers:
- BIOS Basic I/O system - currently 3 kB, could be reduced to 2kB by removing SCSI and I2C drivers. Drives can be A-H non consecutive. I/O console is serial TTY 1200 - 9600 Baud
- BDOS Basic disc operating system - this is the CPM-65 kernal. Size 2 kB
- CCP Console command program - a simple console which only allows to invoke CPM-65 programs. No resident commands. Size 1 kB

File & Disc Format
----------------------
Filenames are CP/M-style d:filename.ext with d <Drive A-H>
Programs must have .COM as extension and are loaded to $0200 and started there.

The directory structure is CP/M-compatible and can be read with appropriate  tools like CPMTOOLS

The Disc format is typically 40 tracks/ 8 sectors/ 256 byte/sector. It is defined in the BIOS. The BDOS operates on sector numbers. Maximum sector number is $800 blocks with $20 sectors = 65536 sectors á 256 bytes = 16 MBytes

Software List
---------------------
Name		Version
ALLOC		2.9
ASM		2.5
BASIC		1.5
BDOS		2.2
BIOS		3.0
BOOT		1.7
BOOTPROM	3.0
BROWSE		1.0
CCP		1.5
COPY		1.4
D		2.0
DATETIME	1.1
DEBUG		1.5
DRUCKER		1.0
DUTIL		1.5
EPROMMER	1.2
ERASE		1.5
FORMAT		2.4
FORTH		1.6a
HEXMON		1.1
I2C-UTIL	1.2
RENAME		1.1
RFILE		1.1
SCSIMGR		1.1
STEP		1.0a
SUPERTAP	1.4
SYS 		1.5
SYSGEN		1.0
TYPE		1.6
XMODEM		1.2

All software is supplied as assembler files to be assembled with the CPM-65 assembler. In case you wish to use a different assembler, the syntax has to be adapted accordingly.

Documentation
--------------------
Currently the documentation of CPM-65 is sparse and only for my personal needs. I plan to write appropriate docs over time. If there are any whishes, please open a DISCUSSION

Errors
--------------------
The CPM-65 system has now seen more than 30 years of service. Currently there are no known errors. However, since an error free software does not exist, please report any errors in the ISSUE section

Other related systems
---------------------
When I started the development of cpm-65, I was blissfully unaware of any other aproaches. However there are some, most notably:
- DOS/65 by Richard Leary. There is a limited compatibility
- OUP/M  by Jiang - Xiong Shao. Published 1983, no further development
- CPM65 by David Given, published 2022


Redistribution
--------------
Source code, and all documents, are freely redistributable in
any form. Please see the the COPYRIGHT file included in this
Repository.