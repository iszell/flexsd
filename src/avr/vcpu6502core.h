/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2017  Ingo Korb <ingo@akana.de>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN, see tff.c|h.

   Virtual 6502 CPU (VCPU) emulation by balagesz, (C) 2021+

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   vcpu6502core.h: 6502 emulation core, AVR assembly definitions

*/

#if CONFIG_VCPUOPTIMLEVEL != 3
#error "Please NOT compile CONFIG_VCPUOPTIMLEVEL != 3, only for testing!"
#endif

#if   CONFIG_VCPUOPTIMLEVEL == 0
#elif CONFIG_VCPUOPTIMLEVEL == 1
  #define INSTRTABLEALIGNED
#elif CONFIG_VCPUOPTIMLEVEL == 2
  #define CPUMEMORYALIGNED
#elif CONFIG_VCPUOPTIMLEVEL == 3
  #define INSTRTABLEALIGNED
  #define CPUMEMORYALIGNED
#else
  #error "Wrong CONFIG_VCPUOPTIMLEVEL value! (0..3)"
#endif

/* Assign of registers
   WARNING: please don't reorder the assign of registers,
   only if you really know what are you doing! */
#define R_ZERO r1
#define R_USER1L r2
#define R_USER1H r3
#define R_USER1 R_USER1L
#define R_USER2L r4
#define R_USER2H r5
#define R_USER2 R_USER2L
#define R_USERRETL r6
#define R_USERRETH r7
#define R_USERRET R_USERRETL
#define R_RAML r8
#define R_RAMH r9
#define R_RAM R_RAML
#define R_SAVEADDRL r10
#define R_SAVEADDRH r11
#define R_SAVEADDR R_SAVEADDRL
#ifndef CPUMEMORYALIGNED
  #define R_ZPL r12
#endif
#define R_ZPH r13
#define R_S_Z r14
#define R_S_N r15
#define R_A r16
#define R_X r17
#define R_Y r18
#define R_SR r19
#ifndef CPUMEMORYALIGNED
#  define R_SPL r20
#else
#  define R_SP r20
#endif
#define R_SPH r21
#ifndef CPUMEMORYALIGNED
#  define R_SP r22
#endif
#define R_TEMP r23
#define R_TEMP2 r24
#define R_TEMP3 r25
#define R_PCL xl
#define R_PCH xh
#define R_PC x
#define R_CADDRL yl
#define R_CADDRH yh
#define R_CADDR y
#define R_IR zl
#define R_IRH zh
#define R_IRADDR z

/* RR / U1R / U2R registers addresses in SRAM address space: */
#define userreg_addr 0x0002

;=============================================
;===	Set Instruction table macro:
.macro	setinstr
#ifdef INSTRTABLEALIGNED
		ldi	R_IRH,pm_hi8(instrtable)	;1 => all: 1 clk (aligned instr. table), 0 clk (not needed in unaligned)
#endif
.endm

;===	Opcode fetch and call macro:
.macro	opcodefetchcall
#ifndef INSTRTABLEALIGNED
		ld	R_IRH,R_PC+			;2
		ldi	R_IR,pm_lo8(instrtable)		;1
		add	R_IR,R_IRH			;1
		ldi	R_IRH,pm_hi8(instrtable)	;1
		adc	R_IRH,R_ZERO			;1
#else
		ld	R_IR,R_PC+			;2
#endif
		ijmp					;2 => all: 4 clk (aligned instr. table) or 8 clk (unaligned)
.endm

;=============================================
;===	Address decoding macros:

;	Address Mode 1: ($zp,X)
;	Return address: 6502 address
.macro  calcvaddr_am1
		ld	R_CADDRL,R_PC+			;2	Fetch parameter (ZP address BYTE)
		add	R_CADDRL,R_X			;1
#ifdef CPUMEMORYALIGNED
		mov	R_CADDRH,R_ZPH			;1
		ld	R_TEMP,R_CADDR			;2	Get addr LO from ZP
		inc	R_CADDRL			;1
		ld	R_CADDRH,R_CADDR		;2	Get addr HI from ZP
#else
		clr	R_CADDRH			;1
		add	R_CADDRL,R_ZPL			;1
		adc	R_CADDRH,R_ZPH			;1
		ld	R_TEMP,R_CADDR+			;2	Get addr LO from ZP
		ld	R_CADDRH,R_CADDR		;2	Get addr HI from ZP
#endif
		mov	R_CADDRL,R_TEMP			;1	10/11 CADDR = 6502 memory address
.endm

;	Address Mode 2: $zp
;	Return address: AVR SRAM address
.macro	calcraddr_am2
		ld	R_CADDRL,R_PC+			;2	Fetch parameter (ZP address BYTE)
#ifdef CPUMEMORYALIGNED
		mov	R_CADDRH,R_ZPH			;1	3/5
#else
		clr	R_CADDRH			;1
		add	R_CADDRL,R_ZPL			;1
		adc	R_CADDRH,R_ZPH			;1	3/5
#endif
.endm

;	Address Mode 4: $ghjk
;	Return address: 6502 address
.macro	calcvaddr_am4
		ld	R_CADDRL,R_PC+			;2
		ld	R_CADDRH,R_PC+			;2
.endm

;	Address mode 5: ($zp),Y
;	Return address: 6502 address
.macro	calcvaddr_am5
		ld	R_CADDRL,R_PC+			;2	Fetch parameter (ZP address BYTE)
#ifdef CPUMEMORYALIGNED
		mov	R_CADDRH,R_ZPH			;1
		ld	R_TEMP,R_CADDR			;2	Get addr LO from ZP
		inc	R_CADDRL			;1
		ld	R_CADDRH,R_CADDR		;2	Get addr HI from ZP
		mov	R_CADDRL,R_TEMP			;1	CADDR = 6502 memory address
		add	R_CADDRL,R_Y			;1
		adc	R_CADDRH,R_ZERO			;1	Add offset	11/12
#else
		clr	R_CADDRH			;1
		add	R_CADDRL,R_ZPL			;1
		adc	R_CADDRH,R_ZPH			;1
		ld	R_TEMP,R_CADDR+			;2	Get addr LO from ZP
		ld	R_CADDRH,R_CADDR		;2	Get addr HI from ZP
		mov	R_CADDRL,R_TEMP			;1	CADDR = 6502 memory address
		add	R_CADDRL,R_Y			;1
		adc	R_CADDRH,R_ZERO			;1	Add offset	11/12
#endif
.endm

;	Address mode 6: $zp,X
;	Return address: AVR SRAM address
.macro	calcraddr_am6
		ld	R_CADDRL,R_PC+			;2	Fetch parameter (ZP address BYTE)
		add	R_CADDRL,R_X			;1
#ifdef CPUMEMORYALIGNED
		mov	R_CADDRH,R_ZPH			;1	4/6
#else
		clr	R_CADDRH			;1
		add	R_CADDRL,R_ZPL			;1
		adc	R_CADDRH,R_ZPH			;1	4/6
#endif
.endm

;	Address mode 7: $ghjk,Y
;	Return address: 6502 address
.macro	calcvaddr_am7
		ld	R_CADDRL,R_PC+			;2
		ld	R_CADDRH,R_PC+			;2
		add	R_CADDRL,R_Y			;1
		adc	R_CADDRH,R_ZERO			;1	6
.endm

;	Address mode 8: $ghjk,X
;	Return address: 6502 address
.macro	calcvaddr_am8
		ld	R_CADDRL,R_PC+			;2
		ld	R_CADDRH,R_PC+			;2
		add	R_CADDRL,R_X			;1
		adc	R_CADDRH,R_ZERO			;1	6
.endm

;	Address mode 9: $zp,Y
;	Return address: AVR SRAM address
.macro	calcmaddr_am9
		ld	R_CADDRL,R_PC+			;2	Fetch parameter (ZP address BYTE)
		add	R_CADDRL,R_Y			;1
#ifdef CPUMEMORYALIGNED
		mov	R_CADDRH,R_ZPH			;1
#else
		clr	R_CADDRH			;1
		add	R_CADDRL,R_ZPL			;1
		adc	R_CADDRH,R_ZPH			;1
#endif
.endm

;=============================================
;	Stack address calculation
;	Return address: AVR SRAM address
.macro	calcmaddr_stack
#ifdef CPUMEMORYALIGNED
		movw	R_CADDR,R_SP			;1	1/3
#else
		movw	R_CADDR,R_SPL			;1
		add	R_CADDRL,R_SP			;1
		adc	R_CADDRH,R_ZERO			;1	1/3
#endif
.endm

;=============================================
;	Macros for serial bus INPUT handling:

.macro si_readbusdirect  reg
		in	\reg, _SFR_IO_ADDR(IEC_INPUT)
.endm
#ifndef IEC_INPUTS_INVERTED
  .macro si_skipiflinelow  bitno
		sbic	_SFR_IO_ADDR(IEC_INPUT), \bitno
  .endm
  .macro si_skipiflinehigh  bitno
		sbis	_SFR_IO_ADDR(IEC_INPUT), \bitno
  .endm
;  .macro si_skipifregblow  reg, bitno
;		sbrc	\reg, \bitno
;  .endm
  .macro si_skipifregbhigh  reg, bitno
		sbrs	\reg, \bitno
  .endm
  .macro si_invertregbits  reg
		com	\reg
  .endm
#else
  .macro si_skipiflinelow  bitno
		sbis	_SFR_IO_ADDR(IEC_INPUT), \bitno
  .endm
  .macro si_skipiflinehigh  bitno
		sbic	_SFR_IO_ADDR(IEC_INPUT), \bitno
  .endm
;  .macro si_skipifregblow  reg, bitno
;		sbrs	\reg, \bitno
;  .endm
  .macro si_skipifregbhigh  reg, bitno
		sbrc	\reg, \bitno
  .endm
  .macro si_invertregbits  reg
		nop
  .endm
#endif

;=============================================
