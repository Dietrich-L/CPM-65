<html>
  <head>
    <meta http-equiv="content-type" content="text/html; charset=windows-1252">
    <link rel="alternate stylesheet" type="text/css" href="resource://gre-resources/plaintext.css">
  </head>
  <body>
    <pre>CPM-65 APPLE II Port
===========
Dietrich Lausberg &lt;lausbergd@gmail.com&gt;
https://github.com/dietrich-l

This repository contains CPM-65, <br>a CP/M-80 analogue operating system for 6502 based microcomputers

![Junior Computer](https://github.com/Dietrich-L/CPM-65/assets/83355183/81120437-620d-40d7-9cd0-a68b2e8ef978)<br>Elektor Junior Computer - system setup

![CPM-65 Screen System Disc](https://github.com/Dietrich-L/CPM-65/assets/83355183/3056852b-30cd-47ca-b2da-cb22a5808fff)<br>Screenshot system disc

&nbsp;System Requirements
--------------------------
6502 processor
12 kByte RAM (not tested)
6532 I/O chip for the system timer (could be replaced by a 6522 or other)
min 1 Floppy drive
Serial-I/O

My system is a heavily modified and expanded Elektor Junior Computer with 57 kB RAM. <br>For system components and memory map see separate docs

&nbsp;System Structure
--------------------
CPM-65 consists of 3 layers:
- BIOS Basic I/O system - currently 3 kB, could be reduced to 2kB <br>  by removing SCSI and I2C drivers. Drives can be A-H non consecutive. <br>  I/O console is serial TTY 1200 - 9600 Baud
- BDOS Basic disc operating system - this is the CPM-65 kernal. Size 2 kB
- CCP Console command program - a simple console which only allows <br>  to invoke CPM-65 programs. No resident commands. Size 1 kB
<br><br>&nbsp;File &amp; Disc Format
----------------------
Filenames are CP/M-style d:filename.ext with d &lt;Drive A-H&gt;
Programs must have .COM as extension and are loaded to $0200 and started there.

The directory structure is nearly CP/M-compatible. Disk images can be read with appropriate  tools <br>like CPMTOOLS or CpmtoolsGUI. A disdefs file is in the IMAGES section. However due to an inconsistency <br>in the file system implementation of these tools max. 8 kB files can be written this way. <br>For the transfer of larger files, see separate documentation.


The Disc format is typically 40 tracks/ 16 sectors/ 256 byte/sector. It is defined in the BIOS. <br>The BDOS operates on sector numbers. <br>Maximum sector number is $800 blocks with $20 sectors = 65536 sectors á 256 bytes = 16 MBytes

Software List
</pre>
    <table style="width: 809px;" border="1">
      <tbody>
        <tr>
          <td style="width: 140.383px;"><span style="font-family: Courier New,Courier,monospace;">Program<br>
            </span> </td>
          <td style="margin-left: 90px; width: 83.65px;"><span style="font-family: Courier New,Courier,monospace;">Version<br>
            </span> </td>
          <td style="width: 575px; margin-left: -100px;"><span style="font-family: Courier New,Courier,monospace;">Description<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">ALLOC<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">2.9<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">shows
              disc allocation map<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">ASM<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">2.7<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">native
              6502 Assembler<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">BASIC<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.5<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">Microsoft
              BASIC interpreter<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">BDOS<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">2.3<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">CPM-65
              BDOS<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">BIOS<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">0.7<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">APPLE II
              CPM-65 BIOS</span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">BOOT<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.7<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">APPLE II
              CPM-65 BOOT program in track 0, sector 0<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">BOOTPROM<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">2.3<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">Junior
              Computer BOOT EPROM<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">BROWSE<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.0<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">text
              file browser<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">CCP<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.5<br>
            </span> </td>
          <td style="height: 18.7px;"><span style="font-family: Courier New,Courier,monospace;">CPM-65
              CCP<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">COPY<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.4<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">multi
              file copy utility<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">D<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">2.0<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">show
              directory alphabetically sorted<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">DEBUG<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.7<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">Debugger
              with breakpoint capability<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">DRUCKER<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.0<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">Drucker
              Utility to control EPSON FX-80 printer<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">DUTIL<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.5<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">disc
              sector editor<br>
            </span> </td>
        </tr>
        <tr>
          <td style="height: 28.8167px;"><span style="font-family: Courier New,Courier,monospace;">EDIT<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.1<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">simple
              screen editor for text files and FORTH screens<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">EPROMMER<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.2<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">Utility
              to program 2716&amp;2732 EPROMS with the ELEKTOR Eprommer<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">ERASE<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.5<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">erase
              files<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">FORMAT<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">2.4<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">Disc
              formatter<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">FORTH<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.6a<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">FIG
              FORTH including mudule for standalone applications<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">HEXMON<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.1<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">simple
              HEX monitor Elektor style<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">I2C-UTIL<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.3<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">Utility
              to handle I2C devices, esp. DS3231 RTC and AT24C32 EEPROM<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">RENAME<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.1<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">rename
              files<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">RFILE<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.1<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">reads a
              file through serial I/O<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">SCSIMGR<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.1<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">Utility
              to control and test SCSI devices<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">STEP<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.0a<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">Utility
              to set the steprate of ral floppy drives<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">SUPERTAP<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.4<br>
            </span></td>
          <td><span style="font-family: Courier New,Courier,monospace;">Cassette
              I/O with 3600&amp;7200 baud<br>
            </span></td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">SYS<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.6<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">puts
              code for BOOT, BIOS, BDOS, CCP into the system tracks<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">SYSGEN<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.0<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">copy
              operating system to another disc<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">TYPE<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">1.6<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">prints
              text file to screen<br>
            </span> </td>
        </tr>
        <tr>
          <td><span style="font-family: Courier New,Courier,monospace;">XMODEM<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">2.2<br>
            </span> </td>
          <td><span style="font-family: Courier New,Courier,monospace;">File
              transfer through serial I/O, max. 38400 Baud<br>
            </span> </td>
        </tr>
      </tbody>
    </table>
    <pre>*available

All software is supplied as assembler files to be assembled with the CPM-65 assembler. <br>In case you wish to use a different assembler, the syntax has to be adapted accordingly.

Documentation
--------------------
Currently the documentation of CPM-65 is sparse and only for my personal needs. <br>I plan to write appropriate docs over time. If there are any whishes, please open a DISCUSSION

Errors
--------------------
The CPM-65 system has now seen more than 30 years of service. Currently there are no known errors. 

However, since an error free software does not exist, please report any errors in the ISSUE section

Other related systems
---------------------
When I started the development of cpm-65, I was blissfully unaware of any other aproaches. <br>However there are some, most notably:
- DOS/65 by Richard Leary. There is a limited compatibility
- OUP/M  by Jiang - Xiong Shao. Published 1983, no further development
- CPM65 by David Given, published 2022


Redistribution
--------------
Source code, and all documents, are freely redistributable in
any form. Please see the the COPYRIGHT file included in this
Repository.</pre>
  </body>
</html>
