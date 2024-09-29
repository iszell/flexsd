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


   vcpu6502core.S: 6502 emulation core in AVR assembly

*/

#include "asmconfig.h"
#include "vcpu6502emu.h"
#include "vcpu6502core.h"

;=============================================
#ifdef INSTRTABLEALIGNED
  .section .vcpu6502early
#endif

;=============================================
;===	6502 emulation entry:

;   From avr-gcc wiki:
;	R18–R27, R30, R31
;	  These GPRs are call clobbered. An ordinary function may use them
;	  without restoring the contents.
;	R0, T-Flag
;	  The temporary register and the T-flag in SREG are also call-clobbered,
;	  but this knowledge is not exposed explicitly to the compiler (R0 is a fixed register). 
;
;	R2–R17, R28, R29
;	  The remaining GPRs are call-saved, i.e. a function that uses such a registers
;	  must restore its original content. This applies even if the register is used
;	  to pass a function argument. 
;	R1
;	  The zero-register is implicity call-saved (implicit because R1 is a fixed register). 

;===
;	For now, it remains to save/restore all registers. (Compiler ABI change does not
;	  affect the run.) This may change in the future due to optimization.
;===

  .global vcpu6502core

vcpu6502core:	push	R_USER1L			;R2
		push	R_USER1H			;R3
		push	R_USER2L			;R4
		push	R_USER2H			;R5
		push	R_USERRETL			;R6
		push	R_USERRETH			;R7
		push	R_RAML				;R8
		push	R_RAMH				;R9
		push	R_SAVEADDRL			;R10
		push	R_SAVEADDRH			;R11
#ifndef CPUMEMORYALIGNED
		push	R_ZPL				;R12
#endif
		push	R_ZPH				;R13
		push	R_S_Z				;R14
		push	R_S_N				;R15
		push	R_A				;R16
		push	R_X				;R17
		push	R_Y				;R18		|
		push	R_SR				;R19		|
#ifndef CPUMEMORYALIGNED
		push	R_SPL				;R20		|
#endif
		push	R_SPH				;R21		|
		push	R_SP				;R22 / R20	|
		push	R_TEMP				;R23		|
		push	R_TEMP2				;R24		|
		push	R_TEMP3				;R25		|
		push	R_PCL				;R26 (XL)	|
		push	R_PCH				;R27 (XH)	|
		push	R_CADDRL			;R28 (YL)
		push	R_CADDRH			;R29 (YH)
		push	R_IR				;R30 (ZL)	|
		push	R_IRH				;R31 (ZH)	|

#ifdef CONFIG_UART_DEBUG
		lds	R_TEMP,UCSRB			;USART control register B
		push	R_TEMP
		andi	R_TEMP,~((1<<RXCIE0)|(1<<TXCIE0)|(1<<UDRIE0))
		sts	UCSRB,R_TEMP			;Disable USART interrupts
#endif
		rcall	cpustateload			;Load 6502 state
		rcall	movesreg_to_zn

		ldi	R_TEMP,VCPU_ERROR_ALIGN		;"Align Error" flag
;#if CONFIG_VCPUOPTIMLEVEL == 1
;		ldi	R_TEMP2,pm_lo8(instrtable)
;		tst	R_TEMP2
;		brne	vcpu6502core_exit
;#elif CONFIG_VCPUOPTIMLEVEL == 2
;		tst	R_RAML
;		brne	vcpu6502core_exit
;#elif CONFIG_VCPUOPTIMLEVEL == 3
		ldi	R_TEMP2,pm_lo8(instrtable)
		or	R_TEMP2,R_RAML
		brne	vcpu6502core_exit
;#endif
		rcall	1f
		subi	R_TEMP3,-2			;AVR with 16 bit PC: xCALL push 2 BYTEs to STACK
		in	R_TEMP2,_SFR_IO_ADDR(SPL)
		cp	R_TEMP3,R_TEMP2
		brne	vcpu6502core_exit
		rjmp	calc_pc				;Calculate PC, fetch next instruction
1:		in	R_TEMP3,_SFR_IO_ADDR(SPL)
		ret



common_readmemtoa:

		rcall	read_memory			;12/13 (RAM)
		mov	R_A,R_TEMP			;1

decode_setnz_a:	mov	R_S_Z,R_A			;1
decode_setn:	mov	R_S_N,R_S_Z			;1
decode_fetch:	opcodefetchcall				;4/7

decode_setnz_x:	mov	R_S_Z,R_X			;1
		mov	R_S_N,R_S_Z			;1
#ifdef INSTRTABLEALIGNED
		opcodefetchcall				;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_fetch			;2
#endif

decode_setnz_y:	mov	R_S_Z,R_Y			;1
		mov	R_S_N,R_S_Z			;1
#ifdef INSTRTABLEALIGNED
		opcodefetchcall				;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_fetch			;2
#endif



ninstr_hangup:	ldi	R_TEMP,VCPU_ERROR_HANGUP	;Set "hangup" flag
		rjmp	vcpu6502core_exit

vcpu6502core_exit_sfs:
		ldi	R_TEMP,0x00			;No NEW flag

vcpu6502core_exit:
		rcall	movezn_to_sreg			;Copy Z/N bits to SREG BYTE
		rcall	cpustatesave			;Save 6502 state, convert PC to 6502 PC

		cpi	R_PCH,CONFIG_BUFFER_COUNT	;PC is higher than valid memory address?
		brcs	1f				;If no, OK
		ori	R_TEMP,VCPU_ERROR_ILLEGALFC	;If yes, set FLAG
1:		cli
		lds	R_TEMP2,vcpureg_interrupt
		or	R_TEMP2,R_TEMP
		lds	R_TEMP,interruptcode		;Requested / early saved interrupt flag
		or	R_TEMP2,R_TEMP
		sts	vcpureg_interrupt,R_TEMP2
		sei

#ifdef CONFIG_UART_DEBUG
		pop	R_TEMP
		sts	UCSRB,R_TEMP			;Restore USART interrupts
#endif
		pop	R_IRH				;R31 (ZH)	|
		pop	R_IR				;R30 (ZL)	|
		pop	R_CADDRH			;R29 (YH)
		pop	R_CADDRL			;R28 (YL)
		pop	R_PCH				;R27 (XH)	|
		pop	R_PCL				;R26 (XL)	|
		pop	R_TEMP3				;R25		|
		pop	R_TEMP2				;R24		|
		pop	R_TEMP				;R23		|

		pop	R_SP				;R22 / R20	|
		pop	R_SPH				;R21		|
#ifndef CPUMEMORYALIGNED
		pop	R_SPL				;R20		|
#endif
		pop	R_SR				;R19		|
		pop	R_Y				;R18		|
		pop	R_X				;R17
		pop	R_A				;R16
		pop	R_S_N				;R15
		pop	R_S_Z				;R14
		pop	R_ZPH				;R13
#ifndef CPUMEMORYALIGNED
		pop	R_ZPL				;R12
#endif
		pop	R_SAVEADDRH			;R11
		pop	R_SAVEADDRL			;R10
		pop	R_RAMH				;R9
		pop	R_RAML				;R8
		pop	R_USERRETH			;R7
		pop	R_USERRETL			;R6
		pop	R_USER2H			;R5
		pop	R_USER2L			;R4
		pop	R_USER1H			;R3
		pop	R_USER1L			;R2
		ret
;=============================================
;	Save 6502 state to SRAM:
cpustatesave:
#ifndef CPUMEMORYALIGNED
		sub	R_PCL,R_RAML
		sbc	R_PCH,R_RAMH
#else
		sub	R_PCH,R_RAMH
#endif
		sub	R_SPH,R_RAMH
		sub	R_ZPH,R_RAMH
#ifndef INSTRTABLEALIGNED
		subi	R_IR,pm_lo8(instrtable)
#endif
		sts	vcpureg_lastopcode,R_IR
		ldi	R_CADDRL,lo8(vcpuregs)
		ldi	R_CADDRH,hi8(vcpuregs)
		st	R_CADDR+,R_PCL
		st	R_CADDR+,R_PCH
		st	R_CADDR+,R_A
		st	R_CADDR+,R_X
		st	R_CADDR+,R_Y
		st	R_CADDR+,R_SR
		st	R_CADDR+,R_SP
		st	R_CADDR+,R_SPH
		st	R_CADDR+,R_ZPH

#if VCPU_VERSION >= 2
		adiw	R_CADDR,3		;Skip Interrupt, ReqFunct, LastOp
  #ifndef CPUMEMORYALIGNED
		sub	R_USERRETL,R_RAML
		sbc	R_USERRETH,R_RAMH
		sub	R_USER1L,R_RAML
		sbc	R_USER1H,R_RAMH
		sub	R_USER2L,R_RAML
		sbc	R_USER2H,R_RAMH
  #else
		sub	R_USERRETH,R_RAMH
		sub	R_USER1H,R_RAMH
		sub	R_USER2H,R_RAMH
  #endif
		st	R_CADDR+,R_USERRETL
		st	R_CADDR+,R_USERRETH
		st	R_CADDR+,R_USER1L
		st	R_CADDR+,R_USER1H
		st	R_CADDR+,R_USER2L
		st	R_CADDR+,R_USER2H
#endif
		ret

;	Load 6502 state from SRAM:
cpustateload:	clr	R_ZERO

		ldi	R_CADDRL,lo8(bufferdata)
		ldi	R_CADDRH,hi8(bufferdata)
		movw	R_RAM,R_CADDR

		ldi	R_CADDRL,lo8(vcpuregs)
		ldi	R_CADDRH,hi8(vcpuregs)

		ld	R_PCL,R_CADDR+
		ld	R_PCH,R_CADDR+
		ld	R_A,R_CADDR+
		ld	R_X,R_CADDR+
		ld	R_Y,R_CADDR+
		ld	R_SR,R_CADDR+
		ld	R_SP,R_CADDR+

		clr	R_IR
		rcall	cpustateld_rdhca		;Read SPH
#ifndef CPUMEMORYALIGNED
		movw	R_SPL,R_IR
#else
		mov	R_SPH,R_IRH
#endif

		clr	R_IR
		rcall	cpustateld_rdhca		;Read ZPH
#ifndef CPUMEMORYALIGNED
		movw	R_ZPL,R_IR
#else
		mov	R_ZPH,R_IRH
#endif

#if VCPU_VERSION >= 2
		adiw	R_CADDR,3		;Skip Interrupt, ReqFunct, LastOp
		rcall	cpustateld_rdwca	;Get new word, and check address
		movw	R_USERRET,R_IRADDR
		rcall	cpustateld_rdwca	;Get new word, and check address
		movw	R_USER1,R_IRADDR
		rcall	cpustateld_rdwca	;Get new word, and check address
		movw	R_USER2,R_IRADDR
#endif
		setinstr
		ret

cpustateld_rdwca:
		ld	R_IR,R_CADDR+			;Read lo BYTE from CPU state
cpustateld_rdhca:
		ld	R_IRH,R_CADDR+			;Read (hi) BYTE from CPU state
		cpi	R_IRH,CONFIG_BUFFER_COUNT	;addr Hi > SRAM?
		brcs	1f
		clr	R_IRH
		clr	R_IR
1:
#ifndef CPUMEMORYALIGNED
		add	R_IR,R_RAML
		adc	R_IRH,R_RAMH
#else
		add	R_IRH,R_RAMH
#endif
		ret
;=============================================
;=== If VCPU version lower than 2, these opcodes not defined:
#if VCPU_VERSION < 2
  #define einstr_userr ninstr_hangup
  #define einstr_user1 ninstr_hangup
  #define einstr_user2 ninstr_hangup
  #define einstr_confg ninstr_hangup
#endif

;=== If no any fast serial support, these opcodes not defined:
#if CONFIG_FASTSERIAL_MODE < 1
  #define uinstr_op_0x8f ninstr_hangup
  #define uinstr_op_0x9f ninstr_hangup
  #define uinstr_op_0xaf ninstr_hangup
#endif

;=== If no parallel port, these opcodes not defined:
#if (!defined(HAVE_PARALLEL) || (VCPU_VERSION < 2))
  #define uinstr_op_0x07 ninstr_hangup
  #define uinstr_op_0x17 ninstr_hangup
  #define uinstr_op_0x27 ninstr_hangup
  #define uinstr_op_0x37 ninstr_hangup
  #define uinstr_op_0x47 ninstr_hangup
#endif



;=== If bus type not CBMSER, these opcodes not defined:
#if VCPU_BUSTYPE != VCPU_INTERFACE_CBMSER
  #define uinstr_op_0x03 ninstr_hangup
  #define uinstr_op_0x13 ninstr_hangup
  #define uinstr_op_0x23 ninstr_hangup
  #define uinstr_op_0x33 ninstr_hangup
  #define uinstr_op_0x43 ninstr_hangup
  #define uinstr_op_0x53 ninstr_hangup
  #define uinstr_op_0x63 ninstr_hangup
  #define uinstr_op_0x73 ninstr_hangup
  #define uinstr_op_0x83 ninstr_hangup
  #define uinstr_op_0x93 ninstr_hangup
  #define uinstr_op_0xa3 ninstr_hangup
  #define uinstr_op_0xb3 ninstr_hangup
  #define uinstr_op_0xc3 ninstr_hangup
  #define uinstr_op_0xd3 ninstr_hangup
  #define uinstr_op_0xe3 ninstr_hangup
  #define uinstr_op_0xf3 ninstr_hangup
  #define uinstr_op_0x0b ninstr_hangup
  #define uinstr_op_0x1b ninstr_hangup
  #define uinstr_op_0x2b ninstr_hangup
  #define uinstr_op_0x3b ninstr_hangup
  #define uinstr_op_0x4b ninstr_hangup
  #define uinstr_op_0x5b ninstr_hangup
  #define uinstr_op_0x6b ninstr_hangup
  #define uinstr_op_0x7b ninstr_hangup
  #define uinstr_op_0x8b ninstr_hangup
  #define uinstr_op_0x9b ninstr_hangup
  #define uinstr_op_0xdb ninstr_hangup
#endif

;=== If UART Debug not enabled:
#ifndef CONFIG_UART_DEBUG
  #define writeio_dbg_uarttx writeio_na
#endif



#ifdef INSTRTABLEALIGNED
  .section .vcpu6502core
#endif

;===	Instructions decoder table:
instrtable:	rjmp	instr_brk		;$00: BRK
		rjmp	instr_ora_am1		;$01: ORA ($zp,X)
		rjmp	einstr_ldzph		;$02: (IOp) LDZPH
		rjmp	uinstr_op_0x03		;$03: (IOp) UWDTL
		rjmp	ninstr_hangup		;$04: IOp
		rjmp	instr_ora_am2		;$05: ORA $zp
		rjmp	instr_asl_am2		;$06: ASL $zp
		rjmp	uinstr_op_0x07		;$07: (IOp) PPACK
		rjmp	instr_php		;$08: PHP
		rjmp	instr_ora_am3		;$09: ORA #$gh
		rjmp	instr_asl		;$0A: ASL
		rjmp	uinstr_op_0x0b		;$0B: (IOp) UCLDL
		rjmp	ninstr_hangup		;$0C: IOp
		rjmp	instr_ora_am4		;$0D: ORA $ghjk
		rjmp	instr_asl_am4		;$0E: ASL $ghjk
		rjmp	einstr_userr		;$0F: (IOp) (W/R65C02: BBR0 $zp,offs) USERR (R2)

		rjmp	instr_bpl		;$10: BPL $rtyu
		rjmp	instr_ora_am5		;$11: ORA ($zp),Y
		rjmp	ninstr_hangup		;$12: IOp
		rjmp	uinstr_op_0x13		;$13: (IOp) UWDTH
		rjmp	ninstr_hangup		;$14: IOp
		rjmp	instr_ora_am6		;$15: ORA $zp,X
		rjmp	instr_asl_am6		;$16: ASL $zp,X
		rjmp	uinstr_op_0x17		;$17: (IOp) PPDRD
		rjmp	instr_clc		;$18: CLC
		rjmp	instr_ora_am7		;$19: ORA $ghjk,Y
		rjmp	ninstr_hangup		;$1A: IOp
		rjmp	uinstr_op_0x1b		;$1B: (IOp) UCLDH
		rjmp	ninstr_hangup		;$1C: IOp
		rjmp	instr_ora_am8		;$1D: ORA $ghjk,X
		rjmp	instr_asl_am8		;$1E: ASL $ghjk,X
		rjmp	einstr_user1		;$1F: (IOp) (W/R65C02: BBR1 $zp,offs) USER1 (R2)

		rjmp	instr_jsr_am4		;$20: JSR $ghjk
		rjmp	instr_and_am1		;$21: AND ($zp,X)
		rjmp	einstr_ldsph		;$22: (IOp) LDSPH
		rjmp	uinstr_op_0x23		;$23: (IOp) UWCKL
		rjmp	instr_bit_am2		;$24: BIT $zp
		rjmp	instr_and_am2		;$25: AND $zp
		rjmp	instr_rol_am2		;$26: ROL $zp
		rjmp	uinstr_op_0x27		;$27: (IOp) PPDWR
		rjmp	instr_plp		;$28: PLP
		rjmp	instr_and_am3		;$29: AND #$gh
		rjmp	instr_rol		;$2A: ROL
		rjmp	uinstr_op_0x2b		;$2B: (IOp) UCHDL
		rjmp	instr_bit_am4		;$2C: BIT $ghjk
		rjmp	instr_and_am4		;$2D: AND $ghjk
		rjmp	instr_rol_am4		;$2E: ROL $ghjk
		rjmp	einstr_user2		;$2F: (IOp) (W/R65C02: BBR2 $zp,offs) USER2 (R2)

		rjmp	instr_bmi		;$30: BMI $rtyu
		rjmp	instr_and_am5		;$31: AND ($zp),Y
		rjmp	ninstr_hangup		;$32: IOp
		rjmp	uinstr_op_0x33		;$33: (IOp) UWCKH
		rjmp	ninstr_hangup		;$34: IOp
		rjmp	instr_and_am6		;$35: AND $zp,X
		rjmp	instr_rol_am6		;$36: ROL $zp,X
		rjmp	uinstr_op_0x37		;$37: (IOp) PPWAI
		rjmp	instr_sec		;$38: SEC
		rjmp	instr_and_am7		;$39: AND $ghjk,Y
		rjmp	ninstr_hangup		;$3A: IOp
		rjmp	uinstr_op_0x3b		;$3B: (IOp) UCHDH
		rjmp	ninstr_hangup		;$3C: IOp
		rjmp	instr_and_am8		;$3D: AND $ghjk,X
		rjmp	instr_rol_am8		;$3E: ROL $ghjk,X
		rjmp	ninstr_hangup		;$3F: IOp

		rjmp	instr_rti		;$40: RTI
		rjmp	instr_eor_am1		;$41: EOR ($zp,X)
		rjmp	ninstr_hangup		;$42: IOp
		rjmp	uinstr_op_0x43		;$43: (IOp) UWATL
		rjmp	einstr_tyzph		;$44: (IOp) TYZPH
		rjmp	instr_eor_am2		;$45: EOR $zp
		rjmp	instr_lsr_am2		;$46: LSR $zp
		rjmp	uinstr_op_0x47		;$47: (IOp) PPWDW
		rjmp	instr_pha		;$48: PHA
		rjmp	instr_eor_am3		;$49: EOR #$gh
		rjmp	instr_lsr		;$4A: LSR
		rjmp	uinstr_op_0x4b		;$4B: (IOp) UATDT
		rjmp	instr_jmp_am4		;$4C: JMP $ghjk
		rjmp	instr_eor_am4		;$4D: EOR $ghjk
		rjmp	instr_lsr_am4		;$4E: LSR $ghjk
		rjmp	ninstr_hangup		;$4F: IOp

		rjmp	instr_bvc		;$50: BVC $rtyu
		rjmp	instr_eor_am5		;$51: EOR ($zp),Y
		rjmp	ninstr_hangup		;$52: IOp
		rjmp	uinstr_op_0x53		;$53: (IOp) UWATH
		rjmp	einstr_tzphy		;$54: (IOp) TZPHY
		rjmp	instr_eor_am6		;$55: EOR $zp,X
		rjmp	instr_lsr_am6		;$56: LSR $zp,X
		rjmp	ninstr_hangup		;$57: IOp
		rjmp	ninstr_hangup		;$58: CLI: NOT implemented!
		rjmp	instr_eor_am7		;$59: EOR $ghjk,Y
		rjmp	ninstr_hangup		;$5A: IOp
		rjmp	uinstr_op_0x5b		;$5B: (IOp) UDTTA
		rjmp	ninstr_hangup		;$5C: IOp
		rjmp	instr_eor_am8		;$5D: EOR $ghjk,X
		rjmp	instr_lsr_am8		;$5E: LSR $ghjk,X
		rjmp	ninstr_hangup		;$5F: IOp

		rjmp	instr_rts		;$60: RTS
		rjmp	instr_adc_am1		;$61: ADC ($zp,X)
		rjmp	ninstr_hangup		;$62: IOp
		rjmp	uinstr_op_0x63		;$63: (IOp) USND1
		rjmp	ninstr_hangup		;$64: IOp
		rjmp	instr_adc_am2		;$65: ADC $zp
		rjmp	instr_ror_am2		;$66: ROR $zp
		rjmp	ninstr_hangup		;$67: IOp
		rjmp	instr_pla		;$68: PLA
		rjmp	instr_adc_am3		;$69: ADC #$gh
		rjmp	instr_ror		;$6A: ROR
		rjmp	uinstr_op_0x6b		;$6B: (IOp) UATCK
		rjmp	instr_jmp_am10		;$6C: JMP ($ghjk)
		rjmp	instr_adc_am4		;$6D: ADC $ghjk
		rjmp	instr_ror_am4		;$6E: ROR $ghjk
		rjmp	ninstr_hangup		;$6F: IOp

		rjmp	instr_bvs		;$70: BVS $rtyu
		rjmp	instr_adc_am5		;$71: ADC ($zp),Y
		rjmp	ninstr_hangup		;$72: IOp
		rjmp	uinstr_op_0x73		;$73: (IOp) URCV1
		rjmp	ninstr_hangup		;$74: IOp
		rjmp	instr_adc_am6		;$75: ADC $zp,X
		rjmp	instr_ror_am6		;$76: ROR $zp,X
		rjmp	ninstr_hangup		;$77: IOp
		rjmp	decode_fetch		;$78: SEI
		rjmp	instr_adc_am7		;$79: ADC $ghjk,Y
		rjmp	ninstr_hangup		;$7A: IOp
		rjmp	uinstr_op_0x7b		;$7B: (IOp) UCKTA
		rjmp	ninstr_hangup		;$7C: IOp
		rjmp	instr_adc_am8		;$7D: ADC $ghjk,X
		rjmp	instr_ror_am8		;$7E: ROR $ghjk,X
		rjmp	ninstr_hangup		;$7F: IOp

		rjmp	ninstr_hangup		;$80: IOp
		rjmp	instr_sta_am1		;$81: STA ($zp,X)
		rjmp	ninstr_hangup		;$82: IOp
		rjmp	uinstr_op_0x83		;$83: (IOp) USDTL
		rjmp	instr_sty_am2		;$84: STY $zp
		rjmp	instr_sta_am2		;$85: STA $zp
		rjmp	instr_stx_am2		;$86: STX $zp
		rjmp	ninstr_hangup		;$87: IOp
		rjmp	instr_dey		;$88: DEY
		rjmp	ninstr_hangup		;$89: IOp
		rjmp	instr_txa		;$8A: TXA
		rjmp	uinstr_op_0x8b		;$8B: (IOp) UATCD
		rjmp	instr_sty_am4		;$8C: STY $ghjk
		rjmp	instr_sta_am4		;$8D: STA $ghjk
		rjmp	instr_stx_am4		;$8E: STX $ghjk
		rjmp	uinstr_op_0x8f		;$8F: (IOp) (W/R65C02: BBS0 $zp,offs) FSTXB

		rjmp	instr_bcc		;$90: BCC $rtyu
		rjmp	instr_sta_am5		;$91: STA ($zp),Y
		rjmp	ninstr_hangup		;$92: IOp
		rjmp	uinstr_op_0x93		;$93: (IOp) USDTH
		rjmp	instr_sty_am6		;$94: STY $zp,X
		rjmp	instr_sta_am6		;$95: STA $zp,X
		rjmp	instr_stx_am9		;$96: STX $zp,Y
		rjmp	ninstr_hangup		;$97: IOp
		rjmp	instr_tya		;$98: TYA
		rjmp	instr_sta_am7		;$99: STA $ghjk,Y
		rjmp	instr_txs		;$9A: TXS
		rjmp	uinstr_op_0x9b		;$9B: (IOp) UCDTA
		rjmp	ninstr_hangup		;$9C: IOp
		rjmp	instr_sta_am8		;$9D: STA $ghjk,X
		rjmp	ninstr_hangup		;$9E: IOp
		rjmp	uinstr_op_0x9f		;$9F: (IOp) (W/R65C02: BBS1 $zp,offs) FSRXB

		rjmp	instr_ldy_am3		;$A0: LDY #$gh
		rjmp	instr_lda_am1		;$A1: LDA ($zp,X)
		rjmp	instr_ldx_am3		;$A2: LDX #$gh
		rjmp	uinstr_op_0xa3		;$A3: (IOp) USCKL
		rjmp	instr_ldy_am2		;$A4: LDY $zp
		rjmp	instr_lda_am2		;$A5: LDA $zp
		rjmp	instr_ldx_am2		;$A6: LDX $zp
		rjmp	ninstr_hangup		;$A7: IOp
		rjmp	instr_tay		;$A8: TAY
		rjmp	instr_lda_am3		;$A9: LDA #$gh
		rjmp	instr_tax		;$AA: TAX
		rjmp	uinstr_op_0xab		;$AB: (IOp) UINDB
		rjmp	instr_ldy_am4		;$AC: LDY $ghjk
		rjmp	instr_lda_am4		;$AD: LDA $ghjk
		rjmp	instr_ldx_am4		;$AE: LDX $ghjk
		rjmp	uinstr_op_0xaf		;$AF: (IOp) (W/R65C02: BBS2 $zp,offs) FSRXC

		rjmp	instr_bcs		;$B0: BCS $rtyu
		rjmp	instr_lda_am5		;$B1: LDA ($zp),Y
		rjmp	ninstr_hangup		;$B2: IOp
		rjmp	uinstr_op_0xb3		;$B3: (IOp) USCKH
		rjmp	instr_ldy_am6		;$B4: LDY $zp,X
		rjmp	instr_lda_am6		;$B5: LDA $zp,X
		rjmp	instr_ldx_am9		;$B6: LDX $zp,Y
		rjmp	ninstr_hangup		;$B7: IOp
		rjmp	instr_clv		;$B8: CLV
		rjmp	instr_lda_am7		;$B9: LDA $ghjk,Y
		rjmp	instr_tsx		;$BA: TSX
		rjmp	uinstr_op_0xbb		;$BB: (IOp) UDEDB
		rjmp	instr_ldy_am8		;$BC: LDY $ghjk,X
		rjmp	instr_lda_am8		;$BD: LDA $ghjk,X
		rjmp	instr_ldx_am7		;$BE: LDX $ghjk,Y
		rjmp	ninstr_hangup		;$BF: IOp

		rjmp	instr_cpy_am3		;$C0: CPY #$gh
		rjmp	instr_cmp_am1		;$C1: CMP ($zp,X)
		rjmp	ninstr_hangup		;$C2: IOp
		rjmp	uinstr_op_0xc3		;$C3: (IOp) USATL
		rjmp	instr_cpy_am2		;$C4: CPY $zp
		rjmp	instr_cmp_am2		;$C5: CMP $zp
		rjmp	instr_dec_am2		;$C6: DEC $zp
		rjmp	ninstr_hangup		;$C7: IOp
		rjmp	instr_iny		;$C8: INY
		rjmp	instr_cmp_am3		;$C9: CMP #$gh
		rjmp	instr_dex		;$CA: DEX
		rjmp	einstr_confg		;$CB: (IOp) (65C02: WAI) CONFG (VCPU configuration, 2+ BYTEs OpCode) (R2)
		rjmp	instr_cpy_am4		;$CC: CPY $ghjk
		rjmp	instr_cmp_am4		;$CD: CMP $ghjk
		rjmp	instr_dec_am4		;$CE: DEC $ghjk
		rjmp	ninstr_hangup		;$CF: IOp

		rjmp	instr_bne		;$D0: BNE $rtyu
		rjmp	instr_cmp_am5		;$D1: CMP ($zp),Y
		rjmp	ninstr_hangup		;$D2: IOp
		rjmp	uinstr_op_0xd3		;$D3: (IOp) USATH
		rjmp	einstr_tysph		;$D4: (IOp) TYSPH
		rjmp	instr_cmp_am6		;$D5: CMP $zp,X
		rjmp	instr_dec_am6		;$D6: DEC $zp,X
		rjmp	ninstr_hangup		;$D7: IOp
		rjmp	decode_fetch		;$D8: CLD
		rjmp	instr_cmp_am7		;$D9: CMP $ghjk,Y
		rjmp	ninstr_hangup		;$DA: IOp
		rjmp	uinstr_op_0xdb		;$DB: (IOp) (65C02: STP) ULBIT
		rjmp	ninstr_hangup		;$DC: IOp
		rjmp	instr_cmp_am8		;$DD: CMP $ghjk,X
		rjmp	instr_dec_am8		;$DE: DEC $ghjk,X
		rjmp	ninstr_hangup		;$DF: IOp

		rjmp	instr_cpx_am3		;$E0: CPX #$gh
		rjmp	instr_sbc_am1		;$E1: SBC ($zp,X)
		rjmp	ninstr_hangup		;$E2: (IOp) (deprecated BTASC)
		rjmp	uinstr_op_0xe3		;$E3: (IOp) USND2
		rjmp	instr_cpx_am2		;$E4: CPX $zp
		rjmp	instr_sbc_am2		;$E5: SBC $zp
		rjmp	instr_inc_am2		;$E6: INC $zp
		rjmp	ninstr_hangup		;$E7: IOp
		rjmp	instr_inx		;$E8: INX
		rjmp	instr_sbc_am3		;$E9: SBC #$gh
		rjmp	instr_nop		;$EA: NOP
		rjmp	ninstr_hangup		;$EB: (IOp) (deprecated UTEST)
		rjmp	instr_cpx_am4		;$EC: CPX $ghjk
		rjmp	instr_sbc_am4		;$ED: SBC $ghjk
		rjmp	instr_inc_am4		;$EE: INC $ghjk
		rjmp	ninstr_hangup		;$EF: IOp

		rjmp	instr_beq		;$F0: BEQ $rtyu
		rjmp	instr_sbc_am5		;$F1: SBC ($zp),Y
		rjmp	ninstr_hangup		;$F2: IOp
		rjmp	uinstr_op_0xf3		;$F3: (IOp) URCV2
		rjmp	einstr_tsphy		;$F4: (IOp) TSPHY
		rjmp	instr_sbc_am6		;$F5: SBC $zp,X
		rjmp	instr_inc_am6		;$F6: INC $zp,X
		rjmp	ninstr_hangup		;$F7: IOp
		rjmp	ninstr_hangup		;$F8: SED: NOT implemented!
		rjmp	instr_sbc_am7		;$F9: SBC $ghjk,Y
		rjmp	ninstr_hangup		;$FA: IOp
		rjmp	uinstr_op_0xfb		;$FB: (IOp) UDELY
		rjmp	ninstr_hangup		;$FC: IOp
		rjmp	instr_sbc_am8		;$FD: SBC $ghjk,X
		rjmp	instr_inc_am8		;$FE: INC $ghjk,X
		rjmp	ninstr_hangup		;$FF: IOp



;=============================================
;	I/O decoder tables, followed the instructions decoder table:
;	I/O Write routines: (Aligned table: 0xXX00)
writeiotable:	rjmp	writeio_na		;$00:
#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
		rjmp	writeio_linediag	;$01: Linediag
#else
		rjmp	writeio_na		;$02:
#endif
		rjmp	writeio_hidbits		;$02: LEDs
		rjmp	writeio_na		;$03:
		rjmp	writeio_dbg_uarttx	;$04: (UART debug: Transmit BYTE)
		rjmp	writeio_na		;$05:
		rjmp	writeio_na		;$06:
		rjmp	writeio_convbin2dec	;$07: Binary to decimal converter: convert to decimal
		rjmp	writeio_convbin2asc	;$08: Binary to decimal converter: convert to ascii
		rjmp	writeio_na		;$09:
#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
		rjmp	writeio_cbmseraso	;$0A: ATN/SRQ Lines Output register write (B7=ATN, B6=SRQ)
		rjmp	writeio_na		;$0B:
		rjmp	writeio_cbmserdco	;$0C: DAT/CLK Lines output register write (B1=DAT, B0=CLK)
		rjmp	writeio_na		;$0D:
		rjmp	writeio_cbmserco	;$0E: CLK Line write (CLK=B0)
		rjmp	writeio_cbmserdo	;$0F: DAT Line write (DAT=B0)
  #if (defined(HAVE_PARALLEL) && (VCPU_VERSION >= 2))
		rjmp	writeio_parlines	;$10: Parallel port lines
		rjmp	writeio_paraux		;$11: Parallel port auxilary
  #endif
#else
		rjmp	writeio_na		;$0A:
		rjmp	writeio_na		;$0B:
		rjmp	writeio_na		;$0C:
		rjmp	writeio_na		;$0D:
		rjmp	writeio_na		;$0E:
		rjmp	writeio_na		;$0F:
#endif

;	I/O Read routines:
readiotable:	rjmp	readio_version		;$00: VCPU version + bus type
		rjmp	readio_memiosize	;$01: MEM + I/O area size
		rjmp	readio_hidbits		;$02: HID + LEDs
		rjmp	readio_deviceaddress	;$03: Device address (8, 9, 10, 11)
		rjmp	readio_hostspd		;$04: Host CPU speed (UART debug: Receive BYTE)
		rjmp	readio_checkfw		;$05:
		rjmp	readio_globalflags	;$06: Firmware "globalflags" variable
		rjmp	readio_decimallow	;$07: Binary to decimal converter: result Low digit
		rjmp	readio_decimalmid	;$08: Binary to decimal converter: result Mid digit
		rjmp	readio_decimalhigh	;$09: Binary to decimal converter: result High digit
#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
		rjmp	readio_cbmseraso	;$0A: ATN/SRQ Lines Output register read (B7=ATN, B6=SRQ)
		rjmp	readio_cbmserasi	;$0B: ATN/SRQ Lines read back (B7=ATN, B6=SRQ)
		rjmp	readio_cbmserdco	;$0C: DAT/CLK Lines output register read (B1=DAT, B0=CLK)
		rjmp	readio_cbmserin		;$0D: DAT/CLK Lines read back (B7=DAT, B6=CLK)
		rjmp	readio_cbmserci		;$0E: CLK Line read back (B7=CLK)
		rjmp	readio_cbmserdi		;$0F: DAT Line read back (B7=DAT)
  #if (defined(HAVE_PARALLEL) && (VCPU_VERSION >= 2))
		rjmp	readio_parlines		;$10: Parallel port lines
		rjmp	readio_paraux		;$11: Parallel port auxilary
  #endif
#else
		rjmp	readio_na		;$0A:
		rjmp	readio_na		;$0B:
		rjmp	readio_na		;$0C:
		rjmp	readio_na		;$0D:
		rjmp	readio_na		;$0E:
		rjmp	readio_na		;$0F:
#endif
;=============================================
;===	Commands:

;	BRK:
instr_brk:	ld	R_TEMP,R_PC+			;Fetch function BYTE
		sts	vcpureg_reqfunction,R_TEMP	;Store requested function number
		ldi	R_TEMP,VCPU_FUNCTIONCALL	;Function Call
		rjmp	vcpu6502core_exit		;Exit

;=== LDA/X/Y:
;	LDA ($zp,X)
instr_lda_am1:	rcall	getaddr_am1			;17/18
		rjmp	common_readmemtoa		;18+n
;	LDA $zp
instr_lda_am2:	;rcall	getaddr_am2			;10/12
		calcraddr_am2				;3/5
		ld	R_A,R_CADDR			;2
		rjmp	decode_setnz_a			;4+n
;	LDA #$gh
instr_lda_am3:	ld	R_A,R_PC+			;2	Get data
		rjmp	decode_setnz_a			;4+n
;	LDA $ghjk
instr_lda_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		rjmp	common_readmemtoa		;18+n
;	LDA ($zp),Y
instr_lda_am5:	rcall	getaddr_am5			;18/19
		rjmp	common_readmemtoa		;18+n
;	LDA $zp,X
instr_lda_am6:	;rcall	getaddr_am6			;11/13
		calcraddr_am6				;4/6
		ld	R_A,R_CADDR			;2
		rjmp	decode_setnz_a			;4+n	16 Clk: "2T"
;	LDA $ghjk,Y
instr_lda_am7:	;rcall	getaddr_am7			;13
		calcvaddr_am7				;6
		rjmp	common_readmemtoa		;18+n
;	LDA $ghjk,X
instr_lda_am8:	;rcall	getaddr_am8			;13
		calcvaddr_am8				;6
		rjmp	common_readmemtoa		;18+n

;	LDX #$gh
instr_ldx_am3:	ld	R_X,R_PC+			;2	Get data
		rjmp	decode_setnz_x			;4+n
;	LDX $zp
instr_ldx_am2:	;rcall	getaddr_am2			;10/12
		calcraddr_am2				;3/5
		ld	R_X,R_CADDR			;2
		rjmp	decode_setnz_x			;4+n
;	LDX $ghjk
instr_ldx_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		rcall	read_memory			;12/13 (RAM)
		mov	R_X,R_TEMP			;1
		rjmp	decode_setnz_x			;4+n
;	LDX $zp,Y
instr_ldx_am9:	;rcall	getaddr_am9			;3+8(10)
		calcmaddr_am9				;4/6
		ld	R_X,R_CADDR			;2
		rjmp	decode_setnz_x			;4+n
;	LDX $ghjk,Y
instr_ldx_am7:	;rcall	getaddr_am7			;13
		calcvaddr_am7				;6
		rcall	read_memory			;12/13 (RAM)
		mov	R_X,R_TEMP			;1
		rjmp	decode_setnz_x			;4+n

;	LDY #$gh
instr_ldy_am3:	ld	R_Y,R_PC+			;2	Get data
		rjmp	decode_setnz_y			;4+n
;	LDY $zp
instr_ldy_am2:	;rcall	getaddr_am2			;10/12
		calcraddr_am2				;3/5
		ld	R_Y,R_CADDR			;1
		rjmp	decode_setnz_y			;4+n
;	LDY $ghjk
instr_ldy_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		rcall	read_memory			;12/13 (RAM)
		mov	R_Y,R_TEMP			;1
		rjmp	decode_setnz_y			;4+n
;	LDY $zp,X
instr_ldy_am6:	;rcall	getaddr_am6			;11/13
		calcraddr_am6				;4/6
		ld	R_Y,R_CADDR			;2
		rjmp	decode_setnz_y			;4+n
;	LDY $ghjk,X
instr_ldy_am8:	;rcall	getaddr_am8			;13
		calcvaddr_am8				;6
		rcall	read_memory			;12/13 (RAM)
		mov	R_Y,R_TEMP			;1
		rjmp	decode_setnz_y			;4+n

;=== STA/X/Y:
;	STA ($zp,X)
instr_sta_am1:	rcall	getaddr_am1			;17/18
		mov	R_TEMP,R_A			;1
		rjmp	write_memory			;9+n (RAM)
;	STA $zp
instr_sta_am2:	;rcall	getaddr_am2			;10/12
		calcraddr_am2				;3/5
		st	R_CADDR,R_A			;2
1:		rjmp	decode_fetch			;2
;	STA $ghjk
instr_sta_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		mov	R_TEMP,R_A			;1
		rjmp	write_memory			;9+n (RAM)
;	STA ($zp),Y
instr_sta_am5:	rcall	getaddr_am5			;18/19
		mov	R_TEMP,R_A			;1
		rjmp	write_memory			;9+n (RAM)
;	STA $zp,X
instr_sta_am6:	;rcall	getaddr_am6			;11/13
		calcraddr_am6				;4/6
		st	R_CADDR,R_A			;2
		rjmp	1b				;4+n	16 Clk: "2T"
;	STA $ghjk,Y
instr_sta_am7:	;rcall	getaddr_am7			;13
		calcvaddr_am7				;6
		mov	R_TEMP,R_A			;1
		rjmp	write_memory			;9+n (RAM)
;	STA $ghjk,X
instr_sta_am8:	;rcall	getaddr_am8			;13
		calcvaddr_am8				;6
		mov	R_TEMP,R_A			;1
		rjmp	write_memory			;9+n (RAM)

;	STX $zp
instr_stx_am2:	;rcall	getaddr_am2			;10/12
		calcraddr_am2				;3/5
		st	R_CADDR,R_X			;2
		rjmp	decode_fetch			;2
;	STX $ghjk
instr_stx_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		mov	R_TEMP,R_X			;1
		rjmp	write_memory			;9+n (RAM)
;	STX $zp,Y
instr_stx_am9:	;rcall	getaddr_am9			;3+8(10)
		calcmaddr_am9				;4/6
		st	R_CADDR,R_X			;2
		rjmp	decode_fetch			;2

;	STY $zp
instr_sty_am2:	;rcall	getaddr_am2			;10/12
		calcraddr_am2				;3/5
		st	R_CADDR,R_Y			;2
		rjmp	decode_fetch			;2
;	STY $ghjk
instr_sty_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		mov	R_TEMP,R_Y			;1
		rjmp	write_memory			;9+n (RAM)
;	STY $zp,X
instr_sty_am6:	;rcall	getaddr_am6			;11/13
		calcraddr_am6				;4/6
		st	R_CADDR,R_Y			;2
		rjmp	decode_fetch			;2

;=== AND:
;	AND ($zp,X)
instr_and_am1:	rcall	getaddr_am1			;17/18
		rjmp	instr_and_comm			;19+n
;	AND $zp
instr_and_am2:	rcall	getaddr_am2			;10/12
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_and_comn			;7+n
;	AND #$gh
instr_and_am3:	ld	R_TEMP,R_PC+			;2	Get data
		and	R_A,R_TEMP			;1
		rjmp	decode_setnz_a			;4+n
;	AND $ghjk
instr_and_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_and_comm			;2
instr_and_comm:	rcall	read_memory			;12/13 (RAM)
instr_and_comn:	and	R_A,R_TEMP			;1
		rjmp	decode_setnz_a			;4+n
;	AND ($zp),Y
instr_and_am5:	rcall	getaddr_am5			;18/19
		rjmp	instr_and_comm			;19+n
;	AND $zp,X
instr_and_am6:	rcall	getaddr_am6			;11/13
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_and_comn			;7+n
;	AND $ghjk,Y
instr_and_am7:	rcall	getaddr_am7			;13
		rjmp	instr_and_comm			;19+n
;	AND $ghjk,X
instr_and_am8:	rcall	getaddr_am8			;13
		rjmp	instr_and_comm			;19+n

;=== ORA:
;	ORA ($zp,X)
instr_ora_am1:	rcall	getaddr_am1			;17/18
		rjmp	instr_ora_comm			;19+n
;	ORA $zp
instr_ora_am2:	rcall	getaddr_am2			;10/12
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_ora_comn			;7+n
;	ORA #$gh
instr_ora_am3:	ld	R_TEMP,R_PC+			;2	Get data
		or	R_A,R_TEMP			;1
		rjmp	decode_setnz_a			;4+n
;	ORA $ghjk
instr_ora_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_ora_comm			;2
instr_ora_comm:	rcall	read_memory			;12/13 (RAM)
instr_ora_comn:	or	R_A,R_TEMP			;1
		rjmp	decode_setnz_a			;4+n
;	ORA ($zp),Y
instr_ora_am5:	rcall	getaddr_am5			;18/19
		rjmp	instr_ora_comm			;19+n
;	ORA $zp,X
instr_ora_am6:	rcall	getaddr_am6			;11/13
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_ora_comn			;7+n
;	ORA $ghjk,Y
instr_ora_am7:	rcall	getaddr_am7			;13
		rjmp	instr_ora_comm			;19+n
;	ORA $ghjk,X
instr_ora_am8:	rcall	getaddr_am8			;13
		rjmp	instr_ora_comm			;19+n

;=== EOR:
;	EOR ($zp,X)
instr_eor_am1:	rcall	getaddr_am1			;17/18
		rjmp	instr_eor_comm			;19+n
;	EOR $zp
instr_eor_am2:	rcall	getaddr_am2			;10/12
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_eor_comn			;7+n
;	EOR #$gh
instr_eor_am3:	ld	R_TEMP,R_PC+			;2	Get data
		eor	R_A,R_TEMP			;1
		rjmp	decode_setnz_a			;4+n
;	EOR $ghjk
instr_eor_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_eor_comm			;2
instr_eor_comm:	rcall	read_memory			;12/13 (RAM)
instr_eor_comn:	eor	R_A,R_TEMP			;1
		rjmp	decode_setnz_a			;4+n
;	EOR ($zp),Y
instr_eor_am5:	rcall	getaddr_am5			;18/19
		rjmp	instr_eor_comm			;19+n
;	EOR $zp,X
instr_eor_am6:	rcall	getaddr_am6			;11/13
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_eor_comn			;7+n
;	EOR $ghjk,Y
instr_eor_am7:	rcall	getaddr_am7			;13
		rjmp	instr_eor_comm			;19+n
;	EOR $ghjk,X
instr_eor_am8:	rcall	getaddr_am8			;13
		rjmp	instr_eor_comm			;19+n

;=== ADC:
;	ADC ($zp,X)
instr_adc_am1:	rcall	getaddr_am1			;17/18
		rjmp	instr_adc_comm			;24+n
;	ADC $zp
instr_adc_am2:	rcall	getaddr_am2			;10/12
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_adc_addd			;12+n
;	ADC #$gh
instr_adc_am3:	ld	R_TEMP,R_PC+			;2	Get data
		rjmp	instr_adc_addd			;12+n
;	ADC $ghjk
instr_adc_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_adc_comm			;2
instr_adc_comm:	rcall	read_memory			;12/13 (RAM)
instr_adc_addd:	andi	R_SR,0xbf			;1	Clear 6502 "V" flag
		lsr	R_SR				;1	Shift-out 6502 "C" to AVR C
		adc	R_A,R_TEMP			;1
		brvc	1f				;1/2
		ori	R_SR,0x20			;1	Set 6502 "V" flag (B5 now), AVR C not changed
1:		rol	R_SR				;1	Shift-in new 6502 "C" from AVR C
		rjmp	decode_setnz_a			;4+n
;	ADC ($zp),Y
instr_adc_am5:	rcall	getaddr_am5			;18/19
		rjmp	instr_adc_comm			;24+n
;	ADC $zp,X
instr_adc_am6:	rcall	getaddr_am6			;11/13
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_adc_addd			;12+n
;	ADC $ghjk,Y
instr_adc_am7:	rcall	getaddr_am7			;13
		rjmp	instr_adc_comm			;24+n
;	ADC $ghjk,X
instr_adc_am8:	rcall	getaddr_am8			;13
		rjmp	instr_adc_comm			;24+n

;=== SBC:
;	SBC ($zp,X)
instr_sbc_am1:	rcall	getaddr_am1			;17/18
		rjmp	instr_sbc_comm			;27+n
;	SBC $zp
instr_sbc_am2:	rcall	getaddr_am2			;10/12
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_sbc_subd			;15+n
;	SBC #$gh
instr_sbc_am3:	ld	R_TEMP,R_PC+			;2	Get data
		rjmp	instr_sbc_subd			;15+n
;	SBC $ghjk
instr_sbc_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_sbc_comm			;2
instr_sbc_comm:	rcall	read_memory			;12/13 (RAM)
instr_sbc_subd:	clc					;1
		sbrs	R_SR,0				;1/2	SKIP, if 6502 "C" = 1
		sec					;1
		andi	R_SR,0xbe			;1	Clear 6502 "V"+"C" flag (AVR N/Z/V/S flags modified, C not)
		sbc	R_A,R_TEMP			;1
		brvc	1f				;1/2
		ori	R_SR,0x40			;1	Set 6502 "V" flag
1:		brcs	2f				;1/2
		ori	R_SR,0x01			;1
2:		rjmp	decode_setnz_a			;4+n
;	SBC ($zp),Y
instr_sbc_am5:	rcall	getaddr_am5			;18/19
		rjmp	instr_sbc_comm			;27+n
;	SBC $zp,X
instr_sbc_am6:	rcall	getaddr_am6			;11/13
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_sbc_subd			;15+n
;	SBC $ghjk,Y
instr_sbc_am7:	rcall	getaddr_am7			;13
		rjmp	instr_sbc_comm			;27+n
;	SBC $ghjk,X
instr_sbc_am8:	rcall	getaddr_am8			;13
		rjmp	instr_sbc_comm			;27+n

;=== CMP/X/Y:
;	CMP ($zp,X)
instr_cmp_am1:	rcall	getaddr_am1			;17/18
		rjmp	instr_cmp_comm			;23+n
;	CMP $zp
instr_cmp_am2:	rcall	getaddr_am2			;10/12
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_cmp_cmpd			;11+n
;	CMP #$gh
instr_cmp_am3:	ld	R_TEMP,R_PC+			;2	Get data
		rjmp	instr_cmp_cmpd			;11+n
;	CMP $ghjk
instr_cmp_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_cmp_comm			;2
instr_cmp_comm:	rcall	read_memory			;12/13 (RAM)
instr_cmp_cmpd:	mov	R_TEMP2,R_A			;1
instr_cmp_cmpa:	andi	R_SR,0xfe			;1	Clear 6502 Cy
		sub	R_TEMP2,R_TEMP			;1	"CMP"
		brcs	1f				;1/2
		ori	R_SR,0x01			;1	AVR Cy inv. copy to 6502 Cy
1:		mov	R_S_Z,R_TEMP2			;1
		rjmp	decode_setn			;3+n
;	CMP ($zp),Y
instr_cmp_am5:	rcall	getaddr_am5			;18/19
		rjmp	instr_cmp_comm			;23+n
;	CMP $zp,X
instr_cmp_am6:	rcall	getaddr_am6			;11/13
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_cmp_cmpd			;11+n
;	CMP $ghjk,Y
instr_cmp_am7:	rcall	getaddr_am7			;13
		rjmp	instr_cmp_comm			;23+n
;	CMP $ghjk,X
instr_cmp_am8:	rcall	getaddr_am8			;13
		rjmp	instr_cmp_comm			;23+n

;	CPX #$gh
instr_cpx_am3:	ld	R_TEMP,R_PC+			;2	Get data
		mov	R_TEMP2,R_X			;1
		rjmp	instr_cmp_cmpa			;10+n
;	CPX $zp
instr_cpx_am2:	rcall	getaddr_am2			;10/12
		ld	R_TEMP,R_CADDR			;2
		mov	R_TEMP2,R_X			;1
		rjmp	instr_cmp_cmpa			;10+n
;	CPX $ghjk
instr_cpx_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		rcall	read_memory			;12/13 (RAM)
		mov	R_TEMP2,R_X			;1
		rjmp	instr_cmp_cmpa			;10+n

;	CPY #$gh
instr_cpy_am3:	ld	R_TEMP,R_PC+			;2	Get data
		mov	R_TEMP2,R_Y			;1
		rjmp	instr_cmp_cmpa			;10+n
;	CPY $zp
instr_cpy_am2:	rcall	getaddr_am2			;10/12
		ld	R_TEMP,R_CADDR			;2
		mov	R_TEMP2,R_Y			;1
		rjmp	instr_cmp_cmpa			;10+n
;	CPY $ghjk
instr_cpy_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		rcall	read_memory			;12/13 (RAM)
		mov	R_TEMP2,R_Y			;1
		rjmp	instr_cmp_cmpa			;10+n

;=== ASL:
;	ASL $zp
instr_asl_am2:	rcall	getaddr_am2			;10/12
instr_asl_cmzp:	ld	R_TEMP,R_CADDR			;2
		bst	R_TEMP,7			;1
		bld	R_SR,0				;1
		lsl	R_TEMP				;1
		st	R_CADDR,R_TEMP			;2
		mov	R_S_Z,R_TEMP			;1
		rjmp	decode_setn			;3+n
;	ASL
instr_asl:	bst	R_A,7				;1
		bld	R_SR,0				;1
		lsl	R_A				;1
		rjmp	decode_setnz_a			;4+n
;	ASL $ghjk
instr_asl_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_asl_comm
instr_asl_comm:	movw	R_SAVEADDR,R_CADDR		;1
		rcall	read_memory			;12/13 (RAM)
		bst	R_TEMP,7			;1
		bld	R_SR,0				;1
		lsl	R_TEMP				;1
		rjmp	writeback_memory		;10+n (RAM)
;	ASL $zp,X
instr_asl_am6:	rcall	getaddr_am6			;11/13
		rjmp	instr_asl_cmzp			;13+n
;	ASL $ghjk,X
instr_asl_am8:	rcall	getaddr_am8			;13
		rjmp	instr_asl_comm			;30+n

;=== LSR:
;	LSR $zp
instr_lsr_am2:	rcall	getaddr_am2			;10/12
instr_lsr_cmzp:	ld	R_TEMP,R_CADDR			;2
		bst	R_TEMP,0			;1
		bld	R_SR,0				;1
		lsr	R_TEMP				;1
		st	R_CADDR,R_TEMP			;2
		mov	R_S_Z,R_TEMP			;1
		rjmp	decode_setn			;3+n
;	LSR
instr_lsr:	bst	R_A,0				;1
		bld	R_SR,0				;1
		lsr	R_A				;1
		rjmp	decode_setnz_a			;4+n
;	LSR $ghjk
instr_lsr_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_lsr_comm
instr_lsr_comm:	movw	R_SAVEADDR,R_CADDR		;1
		rcall	read_memory			;12/13 (RAM)
		bst	R_TEMP,0			;1
		bld	R_SR,0				;1
		lsr	R_TEMP				;1
		rjmp	writeback_memory		;10+n (RAM)
;	LSR $zp,X
instr_lsr_am6:	rcall	getaddr_am6			;11/13
		rjmp	instr_lsr_cmzp			;13+n
;	LSR $ghjk,X
instr_lsr_am8:	rcall	getaddr_am8			;13
		rjmp	instr_lsr_comm			;30+n

;=== ROL:
;	ROL $zp
instr_rol_am2:	rcall	getaddr_am2			;10/12
instr_rol_cmzp:	ld	R_TEMP,R_CADDR			;2
		lsr	R_SR				;1	6502 "C" -> AVR C
		rol	R_TEMP				;1
		rol	R_SR				;1	AVR C -> 6502 "C"
		st	R_CADDR,R_TEMP			;2
		mov	R_S_Z,R_TEMP			;1
		rjmp	decode_setn			;3+n
;	ROL
instr_rol:	lsr	R_SR				;1	6502 "C" -> AVR C
		rol	R_A				;1
		rol	R_SR				;1	AVR C -> 6502 "C"
		rjmp	decode_setnz_a			;4+n
;	ROL $ghjk
instr_rol_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_rol_comm
instr_rol_comm:	movw	R_SAVEADDR,R_CADDR		;1
		rcall	read_memory			;12/13 (RAM)
		lsr	R_SR				;1	6502 "C" -> AVR C
		rol	R_TEMP				;1
		rol	R_SR				;1	AVR C -> 6502 "C"
		rjmp	writeback_memory		;10+n (RAM)
;	ROL $zp,X
instr_rol_am6:	rcall	getaddr_am6			;11/13
		rjmp	instr_rol_cmzp			;13+n
;	ROL $ghjk,X
instr_rol_am8:	rcall	getaddr_am8			;13
		rjmp	instr_rol_comm			;30+n

;=== ROR:
;	ROR $zp
instr_ror_am2:	rcall	getaddr_am2			;10/12
instr_ror_cmzp:	ld	R_TEMP,R_CADDR			;2
		lsr	R_SR				;1	6502 "C" -> AVR C
		ror	R_TEMP				;1
		rol	R_SR				;1	AVR C -> 6502 "C"
		st	R_CADDR,R_TEMP			;2
		mov	R_S_Z,R_TEMP			;1
		rjmp	decode_setn			;3+n
;	ROR
instr_ror:	lsr	R_SR				;1	6502 "C" -> AVR C
		ror	R_A				;1
		rol	R_SR				;1	AVR C -> 6502 "C"
		rjmp	decode_setnz_a			;4+n
;	ROR $ghjk
instr_ror_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_ror_comm
instr_ror_comm:	movw	R_SAVEADDR,R_CADDR		;1
		rcall	read_memory			;12/13 (RAM)
		lsr	R_SR				;1	6502 "C" -> AVR C
		ror	R_TEMP				;1
		rol	R_SR				;1	AVR C -> 6502 "C"
		rjmp	writeback_memory		;10+n (RAM)
;	ROR $zp,X
instr_ror_am6:	rcall	getaddr_am6			;11/13
		rjmp	instr_ror_cmzp			;13+n
;	ROR $ghjk,X
instr_ror_am8:	rcall	getaddr_am8			;13
		rjmp	instr_ror_comm			;30+n

;=== INC:
;	INC $zp
instr_inc_am2:	rcall	getaddr_am2			;10/12
instr_inc_cmzp:	ld	R_TEMP,R_CADDR			;2
		inc	R_TEMP				;1
		st	R_CADDR,R_TEMP			;2
		mov	R_S_Z,R_TEMP			;1
		rjmp	decode_setn			;3+n
;	INC $ghjk
instr_inc_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_inc_comm
instr_inc_comm:	movw	R_SAVEADDR,R_CADDR		;1
		rcall	read_memory			;12/13 (RAM)
		inc	R_TEMP				;1
		rjmp	writeback_memory		;10+n (RAM)
;	INC $zp,X
instr_inc_am6:	rcall	getaddr_am6			;11/13
		rjmp	instr_inc_cmzp			;11+n
;	INC $ghjk,X
instr_inc_am8:	rcall	getaddr_am8			;13
		rjmp	instr_inc_comm			;28+n

;=== DEC:
;	DEC $zp
instr_dec_am2:	rcall	getaddr_am2			;10/12
instr_dec_cmzp:	ld	R_TEMP,R_CADDR			;2
		dec	R_TEMP				;1
		st	R_CADDR,R_TEMP			;2
		mov	R_S_Z,R_TEMP			;1
		rjmp	decode_setn			;3+n
;	DEC $ghjk
instr_dec_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_dec_comm
instr_dec_comm:	movw	R_SAVEADDR,R_CADDR		;1
		rcall	read_memory			;12/13 (RAM)
		dec	R_TEMP				;1
		rjmp	writeback_memory		;10+n (RAM)
;	DEC $zp,X
instr_dec_am6:	rcall	getaddr_am6			;11/13
		rjmp	instr_dec_cmzp			;11+n
;	DEC $ghjk,X
instr_dec_am8:	rcall	getaddr_am8			;13
		rjmp	instr_dec_comm			;28+n

;=== INX/Y/DEX/Y
;	INX:
instr_inx:	inc	R_X				;1
		rjmp	decode_setnz_x			;4+n
;	DEX:
instr_dex:	dec	R_X				;1
		rjmp	decode_setnz_x			;4+n

;	INY:
instr_iny:	inc	R_Y				;1
		rjmp	decode_setnz_y			;4+n
;	DEY
instr_dey:	dec	R_Y				;1
		rjmp	decode_setnz_y			;4+n

;=== BIT:
;	BIT $zp
instr_bit_am2:	rcall	getaddr_am2			;10/12
		ld	R_TEMP,R_CADDR			;2
		rjmp	instr_bit_comn			;9+n
;	BIT $ghjk
instr_bit_am4:	;rcall	getaddr_am4			;11
		calcvaddr_am4				;4
		;rjmp	instr_bit_comm
instr_bit_comm:	rcall	read_memory			;12/13 (RAM)
instr_bit_comn:	bst	R_TEMP,6			;1
		bld	R_SR,6				;1	B6 to 6502 "V"
		mov	R_S_N,R_TEMP			;1	B7 to "N"
		and	R_TEMP,R_A			;1
		mov	R_S_Z,R_TEMP			;1	Mem & A to "Z"
		rjmp	decode_fetch			;2+n

;=== JMP/JSR/RTS/RTI:
;	JMP ($ghjk)
instr_jmp_am10:	ld	R_TEMP,R_PC+			;2
		ld	R_PCH,R_PC			;2
		mov	R_PCL,R_TEMP			;1
		cpi	R_PCH,CONFIG_BUFFER_COUNT	;1
		brcc	instr_branch_e			;1/2
#ifdef CPUMEMORYALIGNED
		add	R_PCH,R_RAMH			;1
#else
		add	R_PCL,R_RAML			;1
		adc	R_PCH,R_RAMH			;1
#endif
;	JMP $ghjk
instr_jmp_am4:	ld	R_TEMP,R_PC+			;2	Read new PC LO
		ld	R_PCH,R_PC			;2	Read new PC HI
		mov	R_PCL,R_TEMP			;1
		rjmp	calc_pc				;8+n
;	JSR $ghjk
instr_jsr_am4:
#ifndef CPUMEMORYALIGNED
		rcall	calculate_stack_addr		;8/10
#else
		calcmaddr_stack				;1
#endif
		ld	R_TEMP,R_PC+			;2	Subroutine address LO byte read
		movw	R_SAVEADDR,R_PC			;1
#ifdef CPUMEMORYALIGNED
		sub	R_SAVEADDRH,R_RAMH		;1
#else
		sub	R_SAVEADDRL,R_RAML		;1
		sbc	R_SAVEADDRH,R_RAMH		;1
#endif
		st	R_CADDR,R_SAVEADDRH		;2	Addr HI to stack
#ifdef CPUMEMORYALIGNED
		dec	R_CADDRL			;1
#else
		sbiw	R_CADDR,0x01			;2
#endif
		st	R_CADDR,R_SAVEADDRL		;2	Addr LO to stack
		subi	R_SP,0x02			;1
		ld	R_PCH,R_PC			;2	Subroutine address HI byte read
		mov	R_PCL,R_TEMP			;1

calc_pc:	cpi	R_PCH,CONFIG_BUFFER_COUNT	;1	### Entry point ###
		brcc	instr_branch_e			;1/2
#ifdef CPUMEMORYALIGNED
		add	R_PCH,R_RAMH			;1
#else
		add	R_PCL,R_RAML			;1
		adc	R_PCH,R_RAMH			;1
#endif
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	decode_fetch			;2+n
		rjmp	vcpu6502core_exit_sfs		;	Exit emulation
;	RTS
instr_rts:	subi	R_SP,-2				;1	SP + 2
#ifndef CPUMEMORYALIGNED
		rcall	calculate_stack_addr		;8/10
		ld	R_PCH,R_CADDR			;2
		ld	R_PCL,-R_CADDR			;2	Restore 6502 address
#else
		calcmaddr_stack				;1
		ld	R_PCH,R_CADDR			;2
		dec	R_CADDRL			;1
		ld	R_PCL,R_CADDR			;2	Restore 6502 address
#endif
		adiw	R_PC,1				;2
		rjmp	calc_pc				;8+n
;	RTI
instr_rti:	subi	R_SP,-3				;1	SP + 3
#ifndef CPUMEMORYALIGNED
		rcall	calculate_stack_addr		;8/10
		ld	R_PCH,R_CADDR			;2
		ld	R_PCL,-R_CADDR			;2
		ld	R_SR,-R_CADDR			;2
#else
		calcmaddr_stack				;1
		ld	R_PCH,R_CADDR			;2
		dec	R_CADDRL			;1
		ld	R_PCL,R_CADDR			;2
		dec	R_CADDRL			;1
		ld	R_SR,R_CADDR			;2
#endif
		rcall	movesreg_to_zn			;11
		rjmp	calc_pc				;8+n

;	"Jump address error", exit:
instr_branch_e:	ldi	R_TEMP,VCPU_ERROR_ADDRESS	;	Set "AddrError" flag
		rjmp	vcpu6502core_exit		;

;=== Bcc:
;	BPL $rtyu
instr_bpl:	ld	R_TEMP,R_PC+			;2
		sbrs	R_S_N,7				;1/2/3
		rjmp	instr_bxx_go			;1/9+n
		rjmp	decode_fetch			;2+n
;	BMI $rtyu
instr_bmi:	ld	R_TEMP,R_PC+			;2
		sbrc	R_S_N,7				;1/2/3
		rjmp	instr_bxx_go			;1/9+n
		rjmp	decode_fetch			;2+n
;	BVC $rtyu
instr_bvc:	ld	R_TEMP,R_PC+			;2
		sbrs	R_SR,6				;1/2/3
		rjmp	instr_bxx_go			;1/9+n
		rjmp	decode_fetch			;2+n
;	BVS $rtyu
instr_bvs:	ld	R_TEMP,R_PC+			;2
		sbrc	R_SR,6				;1/2/3
		rjmp	instr_bxx_go			;1/9+n
		rjmp	decode_fetch			;2+n
;	BCC $rtyu
instr_bcc:	ld	R_TEMP,R_PC+			;2
		sbrs	R_SR,0				;1/2/3
		rjmp	instr_bxx_go			;1/9+n
		rjmp	decode_fetch			;2+n
;	BCS $rtyu
instr_bcs:	ld	R_TEMP,R_PC+			;2
		sbrc	R_SR,0				;1/2/3
		rjmp	instr_bxx_go			;1/9+n
		rjmp	decode_fetch			;2+n
;	BNE $rtyu
instr_bne:	ld	R_TEMP,R_PC+			;2
		tst	R_S_Z				;1
		brne	instr_bxx_go			;1/9+n
		rjmp	decode_fetch			;2+n
;	BEQ $rtyu
instr_beq:	ld	R_TEMP,R_PC+			;2
		tst	R_S_Z				;1
		breq	instr_bxx_go			;1/9+n
		rjmp	decode_fetch			;2+n
instr_bxx_go:	;ldi	R_TEMP2,0x00			;1
		;sbrc	R_TEMP,7			;1/2
		;ldi	R_TEMP2,0xff			;1
		;add	R_PCL,R_TEMP			;1
		;adc	R_PCH,R_TEMP2			;1
		add	R_PCL,R_TEMP			;1
		adc	R_PCH,R_ZERO			;1
		sbrc	R_TEMP,7			;1/2/3
		dec	R_PCH				;1
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	decode_fetch			;2+n
		rjmp	vcpu6502core_exit_sfs		;	Exit emulation

;=== Txx:
;	TAX
instr_tax:	mov	R_X,R_A				;1
		rjmp	decode_setnz_a			;4+n
;	TXA
instr_txa:	mov	R_A,R_X				;1
		rjmp	decode_setnz_a			;4+n
;	TAY
instr_tay:	mov	R_Y,R_A				;1
		rjmp	decode_setnz_a			;4+n
;	TYA
instr_tya:	mov	R_A,R_Y				;1
		rjmp	decode_setnz_a			;4+n
;	TSX
instr_tsx:	mov	R_X,R_SP			;1
		rjmp	decode_setnz_x			;4+n
;	TXS
instr_txs:	mov	R_SP,R_X			;1
		rjmp	decode_fetch			;2+n

;=== SEx/CLx:
;	CLC:
instr_clc:	andi	R_SR,0xfe			;1	Clear "C"
		rjmp	decode_fetch			;2+n
;	SEC:
instr_sec:	ori	R_SR,0x01			;1	Set "C"
		rjmp	decode_fetch			;2+n
;	CLV:
instr_clv:	andi	R_SR,0xbf			;1	Clear "V"

;	NOP:
instr_nop:	rjmp	decode_fetch			;2+n	NOP: 6+2 = 8 Clk: 1T (UDLY1)

;=== PHx/PLx:
;	PHA:
instr_pha:
#ifndef CPUMEMORYALIGNED
		rcall	calculate_stack_addr		;8/10
#else
		calcmaddr_stack				;1
#endif
		st	R_CADDR,R_A			;2
		dec	R_SP				;1
		rjmp	decode_fetch			;2+n	12 Clk "1.5T"
;	PLA:
instr_pla:	inc	R_SP				;1
#ifndef CPUMEMORYALIGNED
		rcall	calculate_stack_addr		;8/10
#else
		calcmaddr_stack				;1
#endif
		ld	R_A,R_CADDR			;2
#ifdef INSTRTABLEALIGNED
		mov	R_S_Z,R_A			;1
		mov	R_S_N,R_S_Z			;1	12 Clk "1.5T"
		opcodefetchcall				;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_setnz_a			;4+n
#endif

;	PHP:
instr_php:	rcall	movezn_to_sreg			;3+11
#ifndef CPUMEMORYALIGNED
		rcall	calculate_stack_addr		;8/10
#else
		calcmaddr_stack				;1
#endif
		st	R_CADDR,R_SR			;2
		dec	R_SP				;1
		rjmp	decode_fetch			;2+n
;	PLP:
instr_plp:	inc	R_SP				;1
#ifndef CPUMEMORYALIGNED
		rcall	calculate_stack_addr		;8/10
#else
		calcmaddr_stack				;1
#endif
		ld	R_SR,R_CADDR			;2
		rcall	movesreg_to_zn			;11
		rjmp	decode_fetch			;2+n

;=============================================
;===	Extended instructions:

;	TZPHY:
einstr_tzphy:	mov	R_Y,R_ZPH			;1
1:		sub	R_Y,R_RAMH			;1
		rjmp	decode_setnz_y			;4+n
;	TYZPH:
einstr_tyzph:	mov	R_TEMP,R_Y			;1
		rjmp	einstr_zph			;2
;	LDZPH #$xx:
einstr_ldzph:	ld	R_TEMP,R_PC+			;2	Get data
einstr_zph:	cpi	R_TEMP,CONFIG_BUFFER_COUNT	;1
		brcc	einstr_error			;1/2
#ifndef CPUMEMORYALIGNED
		movw	R_ZPL,R_RAM			;1
#else
		mov	R_ZPH,R_RAMH			;1
#endif
		add	R_ZPH,R_TEMP			;1
		rjmp	decode_fetch			;2+n
;	TSPHY:
einstr_tsphy:	mov	R_Y,R_SPH			;1
		rjmp	1b				;2
;	TYSPH:
einstr_tysph:	mov	R_TEMP,R_Y			;1
		rjmp	einstr_sph			;2
;	LDSPH #$xx:
einstr_ldsph:	ld	R_TEMP,R_PC+			;2	Get data
einstr_sph:	cpi	R_TEMP,CONFIG_BUFFER_COUNT	;1
		brcc	einstr_error			;1/2
#ifndef CPUMEMORYALIGNED
		movw	R_SPL,R_RAM			;1
#else
		mov	R_SPH,R_RAMH			;1
#endif
		add	R_SPH,R_TEMP			;1
		rjmp	decode_fetch			;2+n

;	Address error:
einstr_error:	rjmp	instr_branch_e



;=============================================
;===	VCPU R2 instructions:
#if VCPU_VERSION >= 2

;	USERR: Return:
einstr_userr:	movw	R_PC,R_USERRET			;1	Restore PC
  #ifdef INSTRTABLEALIGNED
		ld	R_IR,R_PC+			;2
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		ijmp					;2	8+4 Clk: "1T"
		sbiw	R_PC,1				;	PC -1
		rjmp	vcpu6502core_exit_sfs		;	Exit emulation
  #else
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	decode_fetch			;2+n	9? Clk
		rjmp	vcpu6502core_exit_sfs		;	Exit emulation
  #endif

;	USER1: Call 1:
einstr_user1:	movw	R_USERRET,R_PC			;1
		movw	R_PC,R_USER1			;1
		sbis	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	vcpu6502core_exit_sfs		;	Exit emulation
		rjmp	decode_fetch			;2+n	12 Clk: "1.5T"

;	USER2: Call 2:
einstr_user2:	movw	R_USERRET,R_PC			;1
		movw	R_PC,R_USER2			;1
		sbis	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	vcpu6502core_exit_sfs		;	Exit emulation
		rjmp	decode_fetch			;2+n	12 Clk: "1.5T"

;	CONFG: VCPU configuration:
einstr_confg:	ld	R_TEMP,R_PC+			;2	Get parameter
		mov	R_TEMP2,R_TEMP			;1
		andi	R_TEMP2,0b11111000		;1	$00..$07: handle USER vectors
		brne	1f				;1/2

		ldi	R_CADDRH,(userreg_addr >> 8)	;1
		clt					;1
		mov	R_CADDRL,R_TEMP			;1
		andi	R_CADDRL,0b00000110		;1	only vector no remain ($00/$02/$04/$06)
		brne	6f				;1/2	Valid vector nos in 1/2/3 ($02/$04/$06)
		ldi	R_CADDRL,0b00000110		;1	Src/Dest: RR
		set					;1	from/to stack
6:
  #if userreg_addr != 0x0002
		subi	R_CADDRL,(256-(userreg_addr&0xff)) ;1
  #endif
		sbrc	R_TEMP,0			;1/2	SKIP if vector write
		rjmp	2f				;2
;	Write vector:
		brtc	4f				;1/2	Jump, if source is Y:X
		movw	R_SAVEADDR,R_CADDR		;1
		subi	R_SP,-2				;1	SP + 2
  #ifndef CPUMEMORYALIGNED
		rcall	calculate_stack_addr		;8/10
		ld	R_TEMP3,R_CADDR			;2
		ld	R_TEMP2,-R_CADDR		;2	Read 6502 address from stack
  #else
		calcmaddr_stack				;1
		ld	R_TEMP3,R_CADDR			;2
		dec	R_CADDRL			;1
		ld	R_TEMP2,R_CADDR			;2	Read 6502 address from stack
  #endif
		movw	R_CADDR,R_SAVEADDR		;1
		rjmp	5f				;2

4:		mov	R_TEMP2,R_X			;1
		mov	R_TEMP3,R_Y			;1

5:		cpi	R_TEMP3,CONFIG_BUFFER_COUNT	;1	R_TEMP3:R_TEMP2 = address, point to valid RAM address?
		brcc	einstr_error			;1/2	If no, illegal address error
  #ifdef CPUMEMORYALIGNED
		add	R_TEMP3,R_RAMH			;1
  #else
		add	R_TEMP2,R_RAML			;1
		adc	R_TEMP3,R_RAMH			;1
  #endif
		st	R_CADDR+,R_TEMP2		;2	New address Lo store to requested register Lo
		st	R_CADDR,R_TEMP3			;2	New address Hi store to requested register Hi
		rjmp	decode_fetch			;2+n

1:
  #if CONFIG_FASTSERIAL_MODE >= 1
		cpi	R_TEMP2,0b00010000		;1	$1x: Fast Serial config?
		brne	7f				;1/2
		sbrs	R_TEMP,0			;1/2/3
		call	fastser_recvdis			;	Disable receiver (not RCALL, a routine can be anywhere)
		sbrc	R_TEMP,0			;1/2/3
		call	fastser_recven			;	Enable receiver (not RCALL, a routine can be anywhere)
		rjmp	decode_fetch			;2+n
7:
  #endif
		rjmp	ninstr_hangup			;	Illegal OpCode, error

;	Read vector:
2:		ld	R_TEMP2,R_CADDR+		;2	Read address Lo from requested register Lo
		ld	R_TEMP3,R_CADDR			;2	Read address Hi from requested register Hi
  #ifdef CPUMEMORYALIGNED
		sub	R_TEMP3,R_RAMH			;1
  #else
		sub	R_TEMP2,R_RAML			;1
		sbc	R_TEMP3,R_RAMH			;1
  #endif
		brts	3f				;1/2	Jump, if destination is stack
		mov	R_X,R_TEMP2			;1
		mov	R_Y,R_TEMP3			;1
		rjmp	decode_fetch			;2+n
3:
  #ifndef CPUMEMORYALIGNED
		rcall	calculate_stack_addr		;8/10
  #else
		calcmaddr_stack				;1
  #endif
		st	R_CADDR,R_TEMP3			;2	Addr HI to stack
  #ifdef CPUMEMORYALIGNED
		dec	R_CADDRL			;1
  #else
		sbiw	R_CADDR,0x01			;2
  #endif
		st	R_CADDR,R_TEMP2			;2	Addr LO to stack
		subi	R_SP,0x02			;1
		rjmp	decode_fetch			;2+n

#endif	// VCPU_VERSION >= 2
;=============================================
;===	µOp instructions:

#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER

#if CONFIG_MCU_FREQ != 8000000
  #error "MCU running frequency is not 8MHz!"
#endif

;=== Waits:
;	UWDTL: Wait for serial DAT Low:
uinstr_op_0x03:	sbis	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	vcpu6502core_exit_sfs				;	Exit emulation
		si_skipiflinelow IEC_PIN_DATA				;1/2/3
		rjmp	uinstr_op_0x03					;2
		rjmp	decode_fetch					;2+n	12 Clk "1.5T", if no wait
;	UWCKL: Wait for serial CLK Low:
uinstr_op_0x23:	sbis	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	vcpu6502core_exit_sfs				;	Exit emulation
		si_skipiflinelow IEC_PIN_CLOCK				;1/2/3
		rjmp	uinstr_op_0x23					;2
		rjmp	decode_fetch					;2+n	12 Clk "1.5T", if no wait
;	UWATL: Wait for serial ATN Low:
uinstr_op_0x43:	sbis	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	vcpu6502core_exit_sfs				;	Exit emulation
		si_skipiflinelow IEC_PIN_ATN				;1/2/3
		rjmp	uinstr_op_0x43					;2
		rjmp	decode_fetch					;2+n	12 Clk "1.5T", if no wait

;	UWDTH: Wait for serial DAT high:
uinstr_op_0x13:	sbis	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	vcpu6502core_exit_sfs				;	Exit emulation
		si_skipiflinehigh IEC_PIN_DATA				;1/2/3
		rjmp	uinstr_op_0x13					;2
		rjmp	decode_fetch					;2+n	12 Clk "1.5T", if no wait
;	UWCKH: Wait for serial CLK high:
uinstr_op_0x33:	sbis	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	vcpu6502core_exit_sfs				;	Exit emulation
		si_skipiflinehigh IEC_PIN_CLOCK				;1/2/3
		rjmp	uinstr_op_0x33					;2
		rjmp	decode_fetch					;2+n	12 Clk "1.5T", if no wait
;	UWATH: Wait for serial ATN high:
uinstr_op_0x53:	sbis	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	vcpu6502core_exit_sfs				;	Exit emulation
		si_skipiflinehigh IEC_PIN_ATN				;1/2/3
		rjmp	uinstr_op_0x53					;2
		rjmp	decode_fetch					;2+n	12 Clk "1.5T", if no wait


;=== One line set/clear:
;	USDTL: Set DAT line to Low:
uinstr_op_0x83:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		ori	R_TEMP,(1 << IEC_OPIN_DATA)		;1
#else
		andi	R_TEMP,~(1 << IEC_OPIN_DATA)		;1
#endif
		rjmp	uinstr_commout_d1			;2+1+1	12 Clk. "1.5T"
;	USDTH: Set DAT line to HighZ:
uinstr_op_0x93:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~(1 << IEC_OPIN_DATA)		;1
#else
		ori	R_TEMP,(1 << IEC_OPIN_DATA)		;1
#endif
		rjmp	uinstr_commout_d1			;2+1+1	12 Clk. "1.5T"
;	USCKL: Set CLK line to Low:
uinstr_op_0xa3:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		ori	R_TEMP,(1 << IEC_OPIN_CLOCK)		;1
#else
		andi	R_TEMP,~(1 << IEC_OPIN_CLOCK)		;1
#endif
		rjmp	uinstr_commout_d1			;2+1+1	12 Clk. "1.5T"
;	USCKH: Set CLK line to HighZ:
uinstr_op_0xb3:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~(1 << IEC_OPIN_CLOCK)		;1
#else
		ori	R_TEMP,(1 << IEC_OPIN_CLOCK)		;1
#endif
		rjmp	uinstr_commout_d1			;2+1+1	12 Clk. "1.5T"
;	USATL: Set ATN line to Low:
uinstr_op_0xc3:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		ori	R_TEMP,(1 << IEC_OPIN_ATN)		;1
#else
		andi	R_TEMP,~(1 << IEC_OPIN_ATN)		;1
#endif
		rjmp	uinstr_commout_d1			;2+1+1	12 Clk. "1.5T"
;	USATH: Set ATN line to HighZ:
uinstr_op_0xd3:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~(1 << IEC_OPIN_ATN)		;1
#else
		ori	R_TEMP,(1 << IEC_OPIN_ATN)		;1
#endif
		rjmp	uinstr_commout_d1			;2+1+1	12 Clk. "1.5T"


;=== Two lines set/clear:
;	UCLDL: CLK line to Low / DAT line to Low:
uinstr_op_0x0b:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		ori	R_TEMP,((1 << IEC_OPIN_CLOCK) | (1 << IEC_OPIN_DATA))	;1
#else
		andi	R_TEMP,~((1 << IEC_OPIN_CLOCK) | (1 << IEC_OPIN_DATA))	;1
#endif
		rjmp	uinstr_commout_d1			;2+1+1	12 Clk. "1.5T"
;	UCLDH: CLK line to Low / DAT line to HighZ:
uinstr_op_0x1b:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~(1 << IEC_OPIN_DATA)		;1
		ori	R_TEMP,(1 << IEC_OPIN_CLOCK)		;1
#else
		ori	R_TEMP,(1 << IEC_OPIN_DATA)		;1
		andi	R_TEMP,~(1 << IEC_OPIN_CLOCK)		;1
#endif
		rjmp	uinstr_commout				;2+1	12 Clk. "1.5T"
;	UCHDL: CLK line to HighZ / DAT line to Low:
uinstr_op_0x2b:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~(1 << IEC_OPIN_CLOCK)		;1
		ori	R_TEMP,(1 << IEC_OPIN_DATA)		;1
#else
		ori	R_TEMP,(1 << IEC_OPIN_CLOCK)		;1
		andi	R_TEMP,~(1 << IEC_OPIN_DATA)		;1
#endif
		rjmp	uinstr_commout				;2+1	12 Clk. "1.5T"
;	UCHDH: CLK line to HighZ / DAT line to HighZ:
uinstr_op_0x3b:	in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~((1 << IEC_OPIN_CLOCK) | (1 << IEC_OPIN_DATA))	;1
#else
		ori	R_TEMP,((1 << IEC_OPIN_CLOCK) | (1 << IEC_OPIN_DATA))	;1
#endif
		rjmp	uinstr_commout_d1			;2+1+1	12 Clk. "1.5T"


;=== Line <-> "A" register moves:
;	UATDT #$xx: "A" register bit to DAT line:
uinstr_op_0x4b:	ld	R_TEMP2,R_PC+				;2	Read bitmask
		in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
		andi	R_TEMP,~(1 << IEC_OPIN_DATA)		;1
		and	R_TEMP2,R_A				;1
#ifdef IEC_OUTPUTS_INVERTED
		brne	1f					;1/2
#else
		breq	1f					;1/2
#endif		
		ori	R_TEMP,(1 << IEC_OPIN_DATA)		;1
1:		rjmp	uinstr_commout				;2+1	16 Clk. "2T"

;	UDTTA #$xx: DAT line to "A" register bit:
uinstr_op_0x5b:	ld	R_TEMP2,R_PC+				;2	Read bitmask
		si_readbusdirect R_TEMP				;1
		or	R_A,R_TEMP2				;1	Set requested bit in A register
		com	R_TEMP2					;1
		si_skipifregbhigh R_TEMP,IEC_PIN_DATA		;1/2/3
		and	R_A,R_TEMP2				;1	Clear requested bit in A register, if line low
		nop						;1
		rjmp	decode_fetch				;2+n	16 Clk "2T"

;	UATCK #$xx: "A" register bit to CLK line:
uinstr_op_0x6b:	ld	R_TEMP2,R_PC+				;2	Read bitmask
		in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
		andi	R_TEMP,~(1 << IEC_OPIN_CLOCK)		;1
		and	R_TEMP2,R_A				;1
#ifdef IEC_OUTPUTS_INVERTED
		brne	1f					;1/2
#else
		breq	1f					;1/2
#endif		
		ori	R_TEMP,(1 << IEC_OPIN_CLOCK)		;1
1:		rjmp	uinstr_commout				;2+1	16 Clk. "2T"

;	UCKTA #$xx: CLK line to "A" register bit:
uinstr_op_0x7b:	ld	R_TEMP2,R_PC+				;2	Read bitmask
		si_readbusdirect R_TEMP				;1
		or	R_A,R_TEMP2				;1	Set requested bit in A register
		com	R_TEMP2					;1
		si_skipifregbhigh R_TEMP,IEC_PIN_CLOCK		;1/2/3
		and	R_A,R_TEMP2				;1	Clear requested bit in A register, if line low
		nop						;1
		rjmp	decode_fetch				;2+n	16 Clk "2T"


;=== Lines <-> "A" register moves:
;	UATCD #$xx, #$xx: A register bits to CLK/DAT Lines:
uinstr_op_0x8b:	ld	R_TEMP2,R_PC+				;2	Read CLK bitmask
		ld	R_TEMP3,R_PC+				;2	Read DAT bitmask
		in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)		;1
		andi	R_TEMP,~((1 << IEC_OPIN_CLOCK) | (1 << IEC_OPIN_DATA))	;1	clear CLK/DAT bits
		and	R_TEMP2,R_A				;1	CLK?
#ifdef IEC_OUTPUTS_INVERTED
		brne	2f					;1/2
#else
		breq	2f					;1/2
#endif
		ori	R_TEMP,(1 << IEC_OPIN_CLOCK)		;1
2:		and	R_TEMP3,R_A				;1	DAT?
#ifdef IEC_OUTPUTS_INVERTED
		brne	3f					;1/2
#else
		breq	3f					;1/2
#endif
		ori	R_TEMP,(1 << IEC_OPIN_DATA)		;1
3:
uinstr_commout_d1:
		nop						;1
uinstr_commout:	out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP		;1	20 Clk "2.5T"
#ifdef INSTRTABLEALIGNED
		opcodefetchcall					;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_fetch				;2
#endif

;	UCDTA #$xx, #$xx: CLK/DAT lines to A register bits:
uinstr_op_0x9b:	ld	R_TEMP2,R_PC+				;2	Read CLK bitmask
		si_readbusdirect R_TEMP				;1
		ld	R_TEMP3,R_PC+				;2	Read DAT bitmask
		or	R_A,R_TEMP2				;1
		or	R_A,R_TEMP3				;1	Set bits to A register
		com	R_TEMP2					;1
		com	R_TEMP3					;1
		si_skipifregbhigh R_TEMP,IEC_PIN_CLOCK		;1/2/3
		and	R_A,R_TEMP2				;1	Clear requested bit in A register, if CLK line low
		si_skipifregbhigh R_TEMP,IEC_PIN_DATA		;1/2/3
		and	R_A,R_TEMP3				;1	Clear requested bit in A register, if DAT line low
		nop						;1	20 Clk "2.5T"
uinstr_defetch:
#ifdef INSTRTABLEALIGNED
		opcodefetchcall					;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_fetch				;2
#endif

;=== Serial Lines "BIT":
;	ULBIT:
uinstr_op_0xdb:	rjmp	1f					;2
1:		si_readbusdirect R_TEMP				;1	Read ATN/CLK/DAT (/SRQ)
		si_invertregbits R_TEMP				;1
		bst	R_TEMP,IEC_PIN_ATN			;1
		bld	R_SR,6					;1	ATN -> SR B6 (V flag)
		bst	R_TEMP,IEC_PIN_CLOCK			;1
		bld	R_S_N,7					;1	CLK -> SR B7 (N flag)
		andi	R_TEMP,(1 << IEC_PIN_DATA)		;1
		mov	R_S_Z,R_TEMP				;1	DAT -> SR B1 (Z flag) 16 Clk: "2T"
#ifdef INSTRTABLEALIGNED
		opcodefetchcall					;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_fetch				;2
#endif

;=== BYTE send / receive instructions:
;	USND1: Send A register bits to DAT:
uinstr_op_0x63:	mov	R_CADDRH,R_A
#ifdef IEC_OUTPUTS_INVERTED
		com	R_CADDRH				;For inverted outputs, flip data bits
#else
		nop						;Delay
#endif
		ldi	R_TEMP3,(1 << IEC_PIN_CLOCK)		;tick = CLK
#ifdef IEC_INPUTS_INVERTED
		ldi	R_TEMP2,(0 << IEC_PIN_CLOCK)		;Start phase: CLK =? High
#else
		ldi	R_TEMP2,(1 << IEC_PIN_CLOCK)		;Start phase: CLK =? High
#endif
		ldi	R_CADDRL,8				;8 bit
		in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~(1 << IEC_OPIN_CLOCK)
#else
		ori	R_TEMP,(1 << IEC_OPIN_CLOCK)
#endif
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP		;Release CLK
1:		bst	R_CADDRH,0				;Move next sended bit
		bld	R_TEMP,IEC_OPIN_DATA			;  to DAT line
		rcall	_uinst_wait_line			;Wait CLK for selected level
		dec	R_CADDRL
		brne	1b
		rjmp	decode_fetch				;2+n

;	URCV1: Receive bits from DAT to A register:
uinstr_op_0x73:	ldi	R_TEMP3,(1 << IEC_PIN_CLOCK)		;tick = CLK
#ifdef IEC_INPUTS_INVERTED
		ldi	R_TEMP2,(1 << IEC_PIN_CLOCK)		;Start phase: CLK =? Low
#else
		ldi	R_TEMP2,(0 << IEC_PIN_CLOCK)		;Start phase: CLK =? Low
#endif
		ldi	R_CADDRL,8				;8 bit
		in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~((1 << IEC_OPIN_CLOCK) | (1 << IEC_OPIN_DATA))
#else
		ori	R_TEMP,((1 << IEC_OPIN_CLOCK) | (1 << IEC_OPIN_DATA))
#endif
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP		;Release CLK/DAT
1:		rcall	_uinst_wait_line			;Wait CLK for selected level
		si_skipiflinelow IEC_PIN_DATA			;Skip, if next received bit (DAT line) 0
		ori	R_CADDRH,0x80
		dec	R_CADDRL
		brne	1b
		mov	R_A,R_CADDRH
		rjmp	decode_fetch				;2+n

;	USND2: Send A register (bits-pairs) to CLK/DAT:
uinstr_op_0xe3:	mov	R_CADDRH,R_A
#ifdef IEC_OUTPUTS_INVERTED
		com	R_CADDRH				;For inverted outputs, flip data bits
#else
		nop						;Delay
#endif
		ldi	R_TEMP3,(1 << IEC_PIN_ATN)		;tick = ATN
#ifdef IEC_INPUTS_INVERTED
		ldi	R_TEMP2,(0 << IEC_PIN_ATN)		;Start phase: ATN =? High
#else
		ldi	R_TEMP2,(1 << IEC_PIN_ATN)		;Start phase: ATN =? High
#endif
		ldi	R_CADDRL,4				;4×2 = 8 bit
		in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~(1 << IEC_OPIN_ATN)
#else
		ori	R_TEMP,(1 << IEC_OPIN_ATN)
#endif
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP		;Release ATN
1:		bst	R_CADDRH,1				;Move next sended bits
		bld	R_TEMP,IEC_OPIN_DATA			;  odd to DAT line
		bst	R_CADDRH,0
		bld	R_TEMP,IEC_OPIN_CLOCK			;  even to CLK line
		rcall	_uinst_wait_line			;Wait ATN Hi
		lsr	R_CADDRH
		dec	R_CADDRL
		brne	1b
		rjmp	decode_fetch				;2+n

;	URCV2: Receive (bit-pairs) from CLK/DAT to A register:
uinstr_op_0xf3:	ldi	R_TEMP3,(1 << IEC_PIN_ATN)		;tick = ATN
#ifdef IEC_INPUTS_INVERTED
		ldi	R_TEMP2,(1 << IEC_PIN_ATN)		;Start phase: ATN =? Low
#else
		ldi	R_TEMP2,(0 << IEC_PIN_ATN)		;Start phase: ATN =? Low
#endif
		ldi	R_CADDRL,4				;4×2 = 8 bit
		in	R_TEMP,_SFR_IO_ADDR(IEC_OUTPUT)
#ifdef IEC_OUTPUTS_INVERTED
		andi	R_TEMP,~((1 << IEC_OPIN_CLOCK) | (1 << IEC_OPIN_DATA) | (1 << IEC_OPIN_ATN))
#else
		ori	R_TEMP,((1 << IEC_OPIN_CLOCK) | (1 << IEC_OPIN_DATA) | (1 << IEC_OPIN_ATN))
#endif
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP		;Release CLK/DAT/ATN
1:		rcall	_uinst_wait_line			;Wait ATN for selected level
		si_skipiflinelow IEC_PIN_DATA			;Skip, if next received bit (DAT line) 0
		ori	R_CADDRH,0x80
		lsr	R_CADDRH
		si_skipiflinelow IEC_PIN_CLOCK			;Skip, if next received bit (CLK line) 0
		ori	R_CADDRH,0x80
		dec	R_CADDRL
		brne	1b
		mov	R_A,R_CADDRH
		rjmp	decode_fetch				;2+n

;	Wait any line for any level:
;	R_TEMP   <- serial output
;	R_TEMP2  <- Interesting bit start phase
;	R_TEMP3  <- Interesting bit
;	R_TEMP2  -> Interesting bit next phase
;	R_CADDRH -> Right Shift 1
_uinst_wait_line:
		nop
1:		sbis	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT
		rjmp	2f						;Exit emulation
		si_readbusdirect R_SAVEADDRL
		and	R_SAVEADDRL,R_TEMP3				;Only "tick" line remain
		cp	R_SAVEADDRL,R_TEMP2
		brne	1b						;Wait for requested level
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP
		lsr	R_CADDRH
		eor	R_TEMP2,R_TEMP3					;Change requested level
		nop
		ret
;	Exit
2:		pop	R_SAVEADDRL
		pop	R_SAVEADDRL					;Drop return address
		rjmp	vcpu6502core_exit_sfs				;Exit emulation

  #if CONFIG_FASTSERIAL_MODE >= 1
;=== Fast serial related comands
;	FSTXB: Fast Serial Transmit one BYTE (send to host):
uinstr_op_0x8f:	push	r24					;2
		push	r0					;2
		mov	r24, R_A				;1
		call	fastser_sendbyte			;307	Send BYTE (not RCALL, a routine can be anywhere)
		pop	r0					;2
		pop	r24					;2
		rjmp	0f					;2
0:		rjmp	decode_fetch				;2+n	all: 320 CLK, "40T"

;	FSRXB: Fast Serial Receive one BYTE (receive from host):
uinstr_op_0x9f:	sbic	_SFR_IO_ADDR(globalflags),FASTSER_RECVRDY_BIT	;1/2/3
		rjmp	1f						;2
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	uinstr_op_0x9f					;2
		rjmp	vcpu6502core_exit_sfs				;	Exit emulation
1:		in	R_A,_SFR_IO_ADDR(fastserrecvb)			;1	Read received BYTE
		cbi	_SFR_IO_ADDR(globalflags),FASTSER_RECVRDY_BIT	;2	12 Clk "1.5T" Clear Recv. flag
#ifdef INSTRTABLEALIGNED
		opcodefetchcall						;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_fetch					;2
#endif

;	FSRXC: Fast Serial: check recv. flag:
uinstr_op_0xaf:	in	R_TEMP,_SFR_IO_ADDR(globalflags)	;1
		bst	R_TEMP,FASTSER_RECVRDY_BIT		;1
		bld	R_SR,6					;1	FS Recv. flag -> SR B6 (V flag)
		nop						;1
		rjmp	decode_fetch				;2+n	12 Clk "1.5T"
  #endif    // SRQ handling

  #if (defined(HAVE_PARALLEL) && (VCPU_VERSION >= 2))
;=== Parallel bus related commands
;	PPACK: Parallel Port ACK: send 1 µSec "ACK" pulse ('DRWP') to host:
uinstr_op_0x07:	cbi	_SFR_IO_ADDR(PARALLEL_HPORT), PARALLEL_HSK_OUT_BIT ;2	PORT bit = 0 (pullup disable + prepare OC Low)
		sbi	_SFR_IO_ADDR(PARALLEL_HDDR), PARALLEL_HSK_OUT_BIT  ;2	DDR bit 1: switch to output and line Low
		push	R_TEMP						;2
		pop	R_TEMP						;2
		rjmp	1f						;2
1:		cbi	_SFR_IO_ADDR(PARALLEL_HDDR), PARALLEL_HSK_OUT_BIT  ;2	DDR bit 0: switch to input and line release
		sbi	_SFR_IO_ADDR(PARALLEL_HPORT), PARALLEL_HSK_OUT_BIT ;2	PORT bit = 1 (pullup enable)
#ifdef INSTRTABLEALIGNED
		opcodefetchcall						;	20 Clk "2.5T" Fetch next opcode, execute
#else
		rjmp	decode_fetch					;2
#endif

;	PPDRD: Parallel Port Data ReaD: read port lines to A register:
uinstr_op_0x17:	rjmp	1f					;2
1:		in	R_A,_SFR_IO_ADDR(PARALLEL_PPIN)		;1
		rjmp	3f					;2+n	16 Clk "2T"

;	PPWDW: Parallel Port Wait ACK pulse ('HRWP') from host, and Data Write:
uinstr_op_0x47:	sbic	_SFR_IO_ADDR(PARALLEL_IRQFLAG_REG),PARALLEL_IRQEN_BIT ;1/2/3
		rjmp	1f					;2
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	uinstr_op_0x47				;2
		rjmp	vcpu6502core_exit_sfs			;	
1:		nop						;2	+4 = 20 Clk "2.5T"

;	PPDWR: Parallel Port Data WRite: write port lines from A register:
uinstr_op_0x27:	mov	R_TEMP,R_A				;1
		com	R_TEMP					;1	Swap bits
		out	_SFR_IO_ADDR(PARALLEL_PDDR),R_ZERO	;1	All lines to input
		out	_SFR_IO_ADDR(PARALLEL_PPORT),R_A	;1	Enable pullups on '1' lines, prepare OC low on '0' lines
		out	_SFR_IO_ADDR(PARALLEL_PDDR),R_TEMP	;1	Set '0' lines to output (drive low this lines)
3:		rjmp	2f					;2
2:		nop						;1
		ldi	R_TEMP,(1 << PARALLEL_IRQEN_BIT)	   ;1
		out	_SFR_IO_ADDR(PARALLEL_IRQFLAG_REG),R_TEMP  ;1	Clear pending IRQ ('HRWFLAG')
#ifdef INSTRTABLEALIGNED
		opcodefetchcall						;	16 Clk "2T" Fetch next opcode, execute
#else
		rjmp	decode_fetch					;2
#endif

;	PPWAI: Parallel Port WAIt ACK pulse ('HRWP') from host:
uinstr_op_0x37:	nop							;1
1:		sbis	_SFR_IO_ADDR(PARALLEL_IRQFLAG_REG),PARALLEL_IRQEN_BIT ;1/2/3
		rjmp	2f						;2	Wait for 'HRWFLAG'
		sbic	_SFR_IO_ADDR(PARALLEL_HPIN),PARALLEL_HSK_IN_BIT	;1/2/3	SKIP, if ACK line is low
		rjmp	decode_fetch					;2+n	12 Clk, minimal delay "1.5T"
2:		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	1b						;2
		rjmp	vcpu6502core_exit_sfs				;	Exit emulation

  #endif    // Parallel bus handling
#endif    // Only CBM serial interface



;=== Cycle instructions:
;	UINDB: Increment index, Decrement counter, Branch if no underrun:
uinstr_op_0xab:	ld	R_TEMP,R_PC+				;2	Read offset
		inc	R_X					;1	Increment index
		subi	R_Y,0x01				;1	Decrement BYTE-counter
		brcs	1f					;1/2	12 Clk: "1.5T"
		add	R_PCL,R_TEMP				;1
		adc	R_PCH,R_ZERO				;1
		sbrc	R_TEMP,7				;1/2/3
		dec	R_PCH					;1
#ifdef INSTRTABLEALIGNED
		ld	R_IR,R_PC+				;2
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		ijmp						;2	16+4 Clk: "2T"
		rjmp	uinstr_jmpexit				;	Exit
#else
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	decode_fetch				;2+n
		rjmp	vcpu6502core_exit_sfs			;	Exit emulation
#endif

;	UDEDB: Decrement index, Decrement counter, Branch if no underrun:
uinstr_op_0xbb:	ld	R_TEMP,R_PC+				;2	Read offset
		dec	R_X					;1	Decrement index
		subi	R_Y,0x01				;1	Decrement BYTE-counter
		brcs	1f					;1/2	12 Clk: "1.5T"
		add	R_PCL,R_TEMP				;1
		adc	R_PCH,R_ZERO				;1
		sbrc	R_TEMP,7				;1/2/3
		dec	R_PCH					;1
#ifdef INSTRTABLEALIGNED
		ld	R_IR,R_PC+				;2
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		ijmp						;2	16+4 Clk: "2T"
uinstr_jmpexit:	sbiw	R_PC,0x03
		ld	R_IR,R_PC+				;	Read opcode
		ld	R_TEMP,R_PC+				;	read parameter
#else
		sbic	_SFR_IO_ADDR(VCPURUNFLAG_REG),VCPURUNFLAG_BIT	;1/2/3
		rjmp	decode_fetch				;2+n	18 Clk
#endif
		rjmp	vcpu6502core_exit_sfs			;	Exit emulation

1:
#ifdef INSTRTABLEALIGNED
		opcodefetchcall					;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_fetch				;2
#endif




;=== Delay:
;	UDELY #$xx: "xT" delay:
uinstr_op_0xfb:	ld	R_TEMP,R_PC+				;2	Read "time"
		tst	R_TEMP					;1
		brne	1f					;1/2
3:		rjmp	decode_fetch				;2+n	12 Clk: "1.5T" + x × "0.5T"
1:		dec	R_TEMP					;1
		breq	3b					;1/2	16 Clk: "2T"
		rjmp	1b					;2
;=============================================
;===	Address decoders for addressing modes:

;	Set parameter address: Address Mode 1: ($zp,X)
;	Return address: direct 6502 address
getaddr_am1:	calcvaddr_am1				;10/11
		ret					;4	14/15
;	Set parameter address: Address mode 2: $zp
;	Return address: AVR SRAM address!
getaddr_am2:	calcraddr_am2				;3/5
		ret					;4	7/9
;;	Set parameter address: Address mode 4: $qwer
;;	Return address: direct 6502 address
;getaddr_am4:	calcvaddr_am4				;4
;		ret					;4	8
;	Set parameter address: Address mode 5: ($zp),Y
;	Return address: direct 6502 address
getaddr_am5:	calcvaddr_am5				;11/12
		ret					;4	15/16
;	Set parameter address: Address mode 6: $zp,X
;	Return address: AVR SRAM address!
getaddr_am6:	calcraddr_am6				;4/6
		ret					;4	8/10
;	Set parameter address: Address mode 7: $ghjk,Y
;	Return address: direct 6502 address
getaddr_am7:	calcvaddr_am7				;6
		ret					;4	10
;	Set parameter address: Address mode 8: $ghjk,X
;	Return address: direct 6502 address
getaddr_am8:	calcvaddr_am8				;6
		ret					;4	10
;;	Set parameter address: Address mode 9: $zp,Y
;;	Return address: AVR SRAM address!
;getaddr_am9:	calcmaddr_am9				;4/6
;		ret					;4 	8/10

;	Calculate actual stack address:
;	Return address: AVR SRAM address!
#ifndef CPUMEMORYALIGNED
calculate_stack_addr:
		calcmaddr_stack				;3
		ret					;4	 5/7
#endif

;=============================================
;	Move Zero + Negative bits to SR:
movezn_to_sreg:	ori	R_SR,0x30				;1	not used + Break bit set / 6502 SR
		andi	R_SR,0x75				;1	Clear N+Z + D / 6502 SR
		tst	R_S_Z					;1
		brne	1f					;1/2
		ori	R_SR,0x02				;1
1:		bst	R_S_N,7					;1
		bld	R_SR,7					;1
		ret						;4	8
;	Move N+Z bits from SR to R_S_N + R_S_Z
movesreg_to_zn:	mov	R_S_N,R_SR				;1	Set B7
		clr	R_S_Z					;1
		sbrs	R_SR,1					;1/2/3
		dec	R_S_Z					;1	Set reg to nonzero
		;ori	R_SR,0x30				;1	not used + Break bit 1
		;andi	R_SR,0xf7				;1	Decimal bit 0
		ret						;4	8
;=============================================
;=== "Normal" memory R/W operations:

;	Read 6502 memory:
read_memory:	cpi	R_CADDRH,CONFIG_BUFFER_COUNT		;1	VCPU RAM?
		brcc	1f					;1/2
#ifdef CPUMEMORYALIGNED
		add	R_CADDRH,R_RAMH				;1
#else
		add	R_CADDRL,R_RAML				;1
		adc	R_CADDRH,R_RAMH				;1
#endif
2:		ld	R_TEMP,R_CADDR				;2	Read from AVR memory (any buffer)
		ret						;4	9/10 clk.

1:		cpi	R_CADDRH,(VCPU_ADDR_IO >> 8)		;1	VCPU I/O?
		brne	3f

read_io:	;rjmp	4f					;2	Delay 4Clk. RCALL + 5Clk. RET
4:		;andi	R_CADDRL,VCPU_MAXIO_SIZE-1		;1	Max. 16 Readable I/O address
		cpi	R_CADDRL,VCPU_MAXIO_SIZE		;1
		brcc	5f					;1/2
		ldi	R_IRH,pm_hi8(readiotable)		;1
		ldi	R_IR,pm_lo8(readiotable)		;1
		add	R_IR,R_CADDRL				;1
#ifndef INSTRTABLEALIGNED
		adc	R_IRH,R_ZERO				;1
#endif
		ijmp						;2

3:		rcall	checkaddr_bufs				;3+
		brcc	2b					;1/2	If address is good, read memory
5:		pop	R_TEMP
		pop	R_TEMP					;	If address error: drop return address
		rjmp	_rwaddr_error				;	Exit with error

;	Write back 6502 memory (end of RMW cycle), fetch next instruction
writeback_memory:
		mov	R_S_Z,R_TEMP				;1
		mov	R_S_N,R_S_Z				;1
		movw	R_CADDR,R_SAVEADDR			;1

;	Write 6502 memory, fetch next instruction
write_memory:	cpi	R_CADDRH,CONFIG_BUFFER_COUNT		;1	7+n
		brcc	1f					;1/2
#ifdef CPUMEMORYALIGNED
		add	R_CADDRH,R_RAMH				;1
#else
		add	R_CADDRL,R_RAML				;1
		adc	R_CADDRH,R_RAMH				;1
#endif
2:		st	R_CADDR,R_TEMP				;2
#ifdef INSTRTABLEALIGNED
		opcodefetchcall					;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_fetch				;2
#endif

1:		cpi	R_CADDRH,(VCPU_ADDR_IO >> 8)		;1	VCPU I/O?
		brne	3f

write_io:	;andi	R_CADDRL,VCPU_MAXIO_SIZE-1		;1	Max. 16 Writeable I/O address
		cpi	R_CADDRL,VCPU_MAXIO_SIZE		;1
		brcc	_rwaddr_error				;1/2
		ldi	R_IRH,pm_hi8(writeiotable)		;1
#ifndef INSTRTABLEALIGNED
		ldi	R_IR,pm_lo8(writeiotable)		;1
		add	R_IR,R_CADDRL				;1
		adc	R_IRH,R_ZERO				;1
#else
		mov	R_IR,R_CADDRL				;1	Write table jump table address is 0xXX00
#endif
		ijmp						;2

3:		rcall	checkaddr_bufs				;3+
		brcc	2b					;1/2	If address is good, write BYTE to memory (any buffer)
_rwaddr_error:	ldi	R_TEMP,VCPU_ERROR_RWADDR		;	RW Address error
		rjmp	vcpu6502core_exit			;	Exit emulation

;	Check and calculate buffer address:
;	R_CADDR <= 6502 address
;	R_CADDR => AVR memory address
;	Cy.     => 0: OK, 1: No valid AVR memory address
checkaddr_bufs:	cpi	R_CADDRH,(VCPU_ADDR_COMMANDBUFFER >> 8)	;1	Command buffer?
		brne	1f					;1/2
		cpi	R_CADDRL,CONFIG_COMMAND_BUFFER_SIZE	;1
		brcc	3f					;1/2
		ldi	R_TEMP2,lo8(command_buffer)		;1
		ldi	R_CADDRH,hi8(command_buffer)		;1
		rjmp	2f					;2	Go to address calculation
1:		cpi	R_CADDRH,(VCPU_ADDR_ERRORBUFFER >> 8)	;1	Error buffer?
		brne	3f					;1/2
		cpi	R_CADDRL,CONFIG_ERROR_BUFFER_SIZE	;1
		brcc	3f					;1/2
		ldi	R_TEMP2,lo8(error_buffer)		;1
		ldi	R_CADDRH,hi8(error_buffer)		;1
2:		add	R_CADDRL,R_TEMP2			;1
		adc	R_CADDRH,R_ZERO				;1
		clc						;1
		ret						;4
3:		sec						;1
		ret						;4

;=============================================
;===	I/O routines:
;===	Readable registers:
readio_version:	ldi	R_TEMP,(VCPU_BUSTYPE_REP<<5)|VCPU_VERSION
		rjmp	readio_common

readio_memiosize:
		ldi	R_TEMP,((VCPU_MAXIO_ADDRBITS-1)<<5)|CONFIG_BUFFER_COUNT	;I/O area ~size (B765) + RAM size (B43210)
		rjmp	readio_common

readio_hidbits:	clr	R_TEMP
#if LED_ACTIVE_LEVEL == 1
		sbic	_SFR_IO_ADDR(LED_BUSY_PORT),LED_BUSY_PIN
		ori	R_TEMP,0b00000001
  #ifndef SINGLE_LED
		sbic	_SFR_IO_ADDR(LED_DIRTY_PORT),LED_DIRTY_PIN
		ori	R_TEMP,0b00000010
  #endif
#else
		sbis	_SFR_IO_ADDR(LED_BUSY_PORT),LED_BUSY_PIN
		ori	R_TEMP,0b00000001
  #ifndef SINGLE_LED
		sbis	_SFR_IO_ADDR(LED_DIRTY_PORT),LED_DIRTY_PIN
		ori	R_TEMP,0b00000010
  #endif
#endif
#ifdef CONFIG_REMOTE_DISPLAY
		sbis	_SFR_IO_ADDR(SOFTI2C_PIN),SOFTI2C_BIT_INTRQ
		ori	R_TEMP,0b00100000
#endif

#ifndef BUTTONS_SPECIAL
		sbis	_SFR_IO_ADDR(BUTTONS_INPUT),BUTTONS_PREV_PIN
		ori	R_TEMP,0b01000000
		sbis	_SFR_IO_ADDR(BUTTONS_INPUT),BUTTONS_NEXT_PIN
		ori	R_TEMP,0b10000000
#else
		in	R_TEMP2,_SFR_IO_ADDR(BUTTONS_MUX_DDR)
		push	R_TEMP2
		andi	R_TEMP2,~(1<<BUTTONS_NEXT_PIN | 1<<BUTTONS_PREV_PIN)
		out	_SFR_IO_ADDR(BUTTONS_MUX_DDR),R_TEMP2
		in	R_TEMP2,_SFR_IO_ADDR(BUTTONS_MUX_PORT)
		push	R_TEMP2
		ori	R_TEMP2,(1<<BUTTONS_NEXT_PIN | 1<<BUTTONS_PREV_PIN)
		out	_SFR_IO_ADDR(BUTTONS_MUX_PORT),R_TEMP2
		pop	R_TEMP2
		in	R_TEMP3,_SFR_IO_ADDR(BUTTONS_INPUT)
		out	_SFR_IO_ADDR(BUTTONS_MUX_PORT),R_TEMP2
		pop	R_TEMP2
		out	_SFR_IO_ADDR(BUTTONS_MUX_DDR),R_TEMP2
		sbrs	R_TEMP3,BUTTONS_PREV_PIN
		ori	R_TEMP,0b01000000
		sbrs	R_TEMP3,BUTTONS_NEXT_PIN
		ori	R_TEMP,0b10000000
#endif
		rjmp	readio_common

readio_deviceaddress:
		lds	R_TEMP,device_address
#ifndef DEVICE_DEFAULT_UNITNO
  #if _SFR_IO_ADDR(DEVICE_HW_ADDR_PIN) < 0x20
		sbis	_SFR_IO_ADDR(DEVICE_HW_ADDR_PIN),DEVICE_HW_ADDR_B0
		ori	R_TEMP,0b01000000
		sbis	_SFR_IO_ADDR(DEVICE_HW_ADDR_PIN),DEVICE_HW_ADDR_B1
		ori	R_TEMP,0b10000000
  #else
		in	R_TEMP2,_SFR_IO_ADDR(DEVICE_HW_ADDR_PIN)
		sbrs	R_TEMP2,DEVICE_HW_ADDR_B0
		ori	R_TEMP,0b01000000
		sbrs	R_TEMP2,DEVICE_HW_ADDR_B1
		ori	R_TEMP,0b10000000
  #endif
#endif
		rjmp	readio_common

readio_hostspd:
#ifdef CONFIG_UART_DEBUG
		lds	R_TEMP,UCSRA
		sbrs	R_TEMP,RXC0			;SKIP, if received data present
		rjmp	1f
		lds	R_TEMP,UDR			;Read received BYTE from UART data register
		ori	R_SR,0x40			;Set 6502 "V" flag, new received BYTE
		rjmp	readio_common
1:
#endif
		ldi	R_TEMP,CONFIG_MCU_FREQ/1000000		;Host Speed in MHz
		rjmp	readio_common

readio_checkfw:	ldi	R_TEMP,pm_lo8(instrtable)
		or	R_TEMP,R_RAML
		rjmp	readio_common

;	Firmware "globalflags" variable
readio_globalflags:
		in	R_TEMP,_SFR_IO_ADDR(globalflags)
		rjmp	readio_common

;	Binary to Decimal converter, result registers:
;	Low digit:
readio_decimallow:
		lds	R_TEMP,b2decimal+0
		rjmp	readio_common
;	Mid digit:
readio_decimalmid:
		lds	R_TEMP,b2decimal+1
		rjmp	readio_common
;	High digit:
readio_decimalhigh:
		lds	R_TEMP,b2decimal+2
		rjmp	readio_common

#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
;	ATN/SRQ lines output register read:
readio_cbmseraso:
		clr	R_TEMP					;1
  #ifdef IEC_OUTPUTS_INVERTED
		sbis	_SFR_IO_ADDR(IEC_OUTPUT),IEC_OPIN_ATN	;1/2/3
  #else
		sbic	_SFR_IO_ADDR(IEC_OUTPUT),IEC_OPIN_ATN	;1/2/3
  #endif
		ori	R_TEMP,0x80				;1

  #ifdef SRQHANDLE
    #ifdef IEC_OUTPUTS_INVERTED
		sbis	_SFR_IO_ADDR(IEC_OUTPUT),IEC_OPIN_SRQ	;1/2/3
    #else
		sbic	_SFR_IO_ADDR(IEC_OUTPUT),IEC_OPIN_SRQ	;1/2/3
    #endif
  #else
		nop						;1
  #endif
		ori	R_TEMP,0x40				;1
		setinstr					;1(2)
		ret						;4
;	ATN/SRQ Lines read back:
readio_cbmserasi:
		clr	R_TEMP					;1
		si_skipiflinelow IEC_PIN_ATN			;1/2/3
		ori	R_TEMP,0x80				;1

  ;#ifdef SRQHANDLE
		si_skipiflinelow IEC_PIN_SRQ			;1/2/3
  ;#else
  ;		nop						;1
  ;#endif
		ori	R_TEMP,0x40
		setinstr					;1(2)
		ret						;4

;	DAT/CLK Lines output register read (B1=DAT, B0=CLK)
readio_cbmserdco:
		ldi	R_TEMP,0x03				;1
  #ifdef IEC_OUTPUTS_INVERTED
		sbic	_SFR_IO_ADDR(IEC_OUTPUT),IEC_OPIN_CLOCK	;1/2/3
  #else
		sbis	_SFR_IO_ADDR(IEC_OUTPUT),IEC_OPIN_CLOCK	;1/2/3
  #endif
		andi	R_TEMP,0xaa				;1
  #ifdef IEC_OUTPUTS_INVERTED
		sbic	_SFR_IO_ADDR(IEC_OUTPUT),IEC_OPIN_DATA	;1/2/3
  #else
		sbis	_SFR_IO_ADDR(IEC_OUTPUT),IEC_OPIN_DATA	;1/2/3
  #endif
		andi	R_TEMP,0x55				;1
		setinstr					;1(2)
		ret						;4

;	DAT/CLK Lines read back (B7=DAT, B6=CLK)
readio_cbmserin:
		ldi	R_TEMP,0xc0				;1
		si_readbusdirect R_TEMP2			;1
		si_skipifregbhigh R_TEMP2,IEC_PIN_CLOCK		;1/2/3
		andi	R_TEMP,0xaa				;1
		si_skipifregbhigh R_TEMP2,IEC_PIN_DATA		;1/2/3
		andi	R_TEMP,0x55				;1
		setinstr					;1(2)
		ret						;4

;	CLK Line read back (B7=CLK)
readio_cbmserci:
		ldi	R_TEMP,0x80				;1
		si_skipiflinehigh IEC_PIN_CLOCK			;1/2/3
		ldi	R_TEMP,0x00				;1	CLK line low
		setinstr					;1(2)
		ret						;4

;	DAT Line read back (B7=DAT)
readio_cbmserdi:
		ldi	R_TEMP,0x80				;1
		si_skipiflinehigh IEC_PIN_DATA			;1/2/3
		ldi	R_TEMP,0x00				;1	CLK line low
		setinstr					;1(2)
		ret						;4

;	Parallel port registers:
  #if (defined(HAVE_PARALLEL) && (VCPU_VERSION >= 2))
;	Parallel port lines read:
readio_parlines:
		in	R_TEMP,_SFR_IO_ADDR(PARALLEL_PPIN)	;1	Read port input register
		setinstr					;1(2)
		ret						;4
;	Parallel port auxilary read:
readio_paraux:	ldi	R_TEMP,0x02					;1	%00000010 B543 = 0, B1 = 1
		sbic	_SFR_IO_ADDR(PARALLEL_HPIN),PARALLEL_HSK_IN_BIT	;1/2/3	SKIP, if 'HRWP' line low
		ori	R_TEMP,0x80					;1	'HRWP' line is '1' -> B7
		sbic	_SFR_IO_ADDR(PARALLEL_IRQFLAG_REG),PARALLEL_IRQEN_BIT ;1/2/3
		ori	R_TEMP,0x40					;1	'HRWFLAG' is '1' -> B6
		sbis	_SFR_IO_ADDR(PARALLEL_HDDR),PARALLEL_HSK_OUT_BIT ;1/2/3	SKIP, if 'DRWP' drive to low
		ori	R_TEMP,0x04					;1	'DRWP' bit -> B2
		sbic	_SFR_IO_ADDR(PARALLEL_HPIN),PARALLEL_HSK_OUT_BIT ;1/2/3	SKIP, if 'DRWP' LINE is low
		ori	R_TEMP,0x01					;1	Line state -> B0
		setinstr					;1(2)
		ret						;4
  #endif    // Parallel port registers

#endif	// CBM Serial lines I/O

readio_na:	ldi	R_TEMP,0x00				;1	%00000000: not implemented (readable) register bits = 0
readio_common:	setinstr					;1(2)	Restore IRH/IR
		ret						;4



;===	Writeable registers:
writeio_hidbits:
		sbrs	R_TEMP,0
#if LED_ACTIVE_LEVEL == 1
		cbi	_SFR_IO_ADDR(LED_BUSY_PORT),LED_BUSY_PIN
#else
		sbi	_SFR_IO_ADDR(LED_BUSY_PORT),LED_BUSY_PIN
#endif
		sbrc	R_TEMP,0
#if LED_ACTIVE_LEVEL == 1
		sbi	_SFR_IO_ADDR(LED_BUSY_PORT),LED_BUSY_PIN
#else
		cbi	_SFR_IO_ADDR(LED_BUSY_PORT),LED_BUSY_PIN
#endif
#ifndef SINGLE_LED
		sbrs	R_TEMP,1
  #if LED_ACTIVE_LEVEL == 1
		cbi	_SFR_IO_ADDR(LED_DIRTY_PORT),LED_DIRTY_PIN
  #else
		sbi	_SFR_IO_ADDR(LED_DIRTY_PORT),LED_DIRTY_PIN
  #endif
		sbrc	R_TEMP,1
  #if LED_ACTIVE_LEVEL == 1
		sbi	_SFR_IO_ADDR(LED_DIRTY_PORT),LED_DIRTY_PIN
  #else
		cbi	_SFR_IO_ADDR(LED_DIRTY_PORT),LED_DIRTY_PIN
  #endif
#endif
		rjmp	writeio_na				;3+n


#ifdef CONFIG_UART_DEBUG
writeio_dbg_uarttx:
		lds	R_TEMP2,UCSRA
		sbrs	R_TEMP2,UDRE0		;SKIP, if USART Data Register Empty set
		rjmp	writeio_dbg_uarttx	;If not empty, wait...
		sts	UDR,R_TEMP		;Transmit data
		rjmp	writeio_na
#endif

;	Convert binary to decimal numbers: R_TEMP (0..$ff) => high:mid:low (000..255)
;	Bin to Dec:
writeio_convbin2dec:
		ldi	R_TEMP2,0x00
		rjmp	1f
;	Bin to Ascii:
writeio_convbin2asc:
		ldi	R_TEMP2,'0'
1:		mov	R_CADDRL,R_TEMP2
		mov	R_CADDRH,R_TEMP2

2:		cpi	R_TEMP,100
		brcs	3f
		inc	R_CADDRH
		subi	R_TEMP,100
		rjmp	2b
3:		cpi	R_TEMP,10
		brcs	4f
		inc	R_CADDRL
		subi	R_TEMP,10
		rjmp	3b
4:		or	R_TEMP2,R_TEMP
		sts	b2decimal+2,R_CADDRH
		sts	b2decimal+1,R_CADDRL
		sts	b2decimal+0,R_TEMP2
		rjmp	writeio_na

#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
;	ATN/SRQ lines output register write:
writeio_cbmseraso:
  #ifdef IEC_OUTPUTS_INVERTED
		com	R_TEMP					;1
  #endif
		in	R_TEMP2,_SFR_IO_ADDR(IEC_OUTPUT)	;1
		bst	R_TEMP,7				;1
		bld	R_TEMP2,IEC_OPIN_ATN			;1
  #ifdef SRQHANDLE
    #ifndef IEC_SEPARATE_OUT
		cbi	_SFR_IO_ADDR(IEC_PORT),IEC_PIN_SRQ	;2	SRQ pullup off (Don't drive SRQ to high!)
    #endif
		bst	R_TEMP,6				;1
		bld	R_TEMP2,IEC_OPIN_SRQ			;1
  #endif
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP2	;1
  #ifdef SRQHANDLE
    #ifndef IEC_SEPARATE_OUT
      #ifdef IEC_OUTPUTS_INVERTED
		sbrs	R_TEMP2,IEC_OPIN_SRQ			;1/2/3	SKIP, if SRQ pin drive Low
      #else
		sbrc	R_TEMP2,IEC_OPIN_SRQ			;1/2/3	SKIP, if SRQ pin drive Low
      #endif
		sbi	_SFR_IO_ADDR(IEC_PORT),IEC_PIN_SRQ	;2	If SRQ is HiZ, SRQ pullup on
    #endif
  #endif
		rjmp	writeio_na				;3+n

;	DAT/CLK Lines output register write (B1=DAT, B0=CLK)
writeio_cbmserdco:
  #ifdef IEC_OUTPUTS_INVERTED
		com	R_TEMP					;1
  #else
		nop						;1
  #endif
		in	R_TEMP2,_SFR_IO_ADDR(IEC_OUTPUT)	;1
		bst	R_TEMP,0				;1
		bld	R_TEMP2,IEC_OPIN_CLOCK			;1
		bst	R_TEMP,1				;1
		bld	R_TEMP2,IEC_OPIN_DATA			;1
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP2	;1
		rjmp	writeio_na				;3+n

;	CLK Line write (CLK=B0)
writeio_cbmserco:
  #ifdef IEC_OUTPUTS_INVERTED
		com	R_TEMP					;1
  #else
		nop						;1
  #endif
		bst	R_TEMP,0				;1
		in	R_TEMP2,_SFR_IO_ADDR(IEC_OUTPUT)	;1
		bld	R_TEMP2,IEC_OPIN_CLOCK			;1
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP2	;1
		rjmp	writeio_na				;3+n

;	DAT Line write (DAT=B0)
writeio_cbmserdo:
  #ifdef IEC_OUTPUTS_INVERTED
		com	R_TEMP					;1
  #else
		nop						;1
  #endif
		bst	R_TEMP,0				;1
		in	R_TEMP2,_SFR_IO_ADDR(IEC_OUTPUT)	;1
		bld	R_TEMP2,IEC_OPIN_DATA			;1
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP2	;1
		rjmp	writeio_na				;3+n

  #if (defined(HAVE_PARALLEL) && (VCPU_VERSION >= 2))
;	Parallel port lines write:
writeio_parlines:
		mov	R_TEMP2,R_TEMP				;1
		com	R_TEMP2					;1	Swap bits
		out	_SFR_IO_ADDR(PARALLEL_PDDR),R_ZERO	;1	All lines to input
		out	_SFR_IO_ADDR(PARALLEL_PPORT),R_TEMP	;1	Enable pullups on '1' lines, prepare OC low on '0' lines
		out	_SFR_IO_ADDR(PARALLEL_PDDR),R_TEMP2	;1	Set '0' lines to output (drive low this lines)
		rjmp	writeio_na				;3+n
;	Parallel port auxilary write:
writeio_paraux:	sbrc	R_TEMP,2				;1/2/3	SKIP if B2 ('DRWP' out) =0
		rjmp	1f
		cbi	_SFR_IO_ADDR(PARALLEL_HPORT), PARALLEL_HSK_OUT_BIT ;2	PORT bit = 0 (pullup disable + prepare OC Low)
		sbi	_SFR_IO_ADDR(PARALLEL_HDDR), PARALLEL_HSK_OUT_BIT  ;2	DDR bit 1: switch to output and line Low
		rjmp	2f
1:		cbi	_SFR_IO_ADDR(PARALLEL_HDDR), PARALLEL_HSK_OUT_BIT  ;2	DDR bit 0: switch to input and line release
		sbi	_SFR_IO_ADDR(PARALLEL_HPORT), PARALLEL_HSK_OUT_BIT ;2	PORT bit = 1 (pullup enable)
2:		ldi	R_TEMP2,(1 << PARALLEL_IRQEN_BIT)		;1
		sbrc	R_TEMP,4					;1/2/3	SKIP if B4 (clear 'HRWFLAG') is 0
		out	_SFR_IO_ADDR(PARALLEL_IRQFLAG_REG), R_TEMP2	;1	Clear 'HRWFLAG'
		rjmp	writeio_na				;3+n
  #endif    // Parallel port registers

;	Serial Linediag
writeio_linediag:
  #ifdef VCPU_LINEDIAG
    #ifdef IEC_OUTPUTS_INVERTED
		com	R_TEMP
    #endif
		in	R_TEMP2,_SFR_IO_ADDR(IEC_OUTPUT)
		bst	R_TEMP,7
		bld	R_TEMP2,IEC_OPIN_DATA
		bst	R_TEMP,6
		bld	R_TEMP2,IEC_OPIN_CLOCK
		bst	R_TEMP,5
		bld	R_TEMP2,IEC_OPIN_ATN
    #ifdef SRQHANDLE
      #ifndef IEC_SEPARATE_OUT
		cbi	_SFR_IO_ADDR(IEC_PORT),IEC_PIN_SRQ	;SRQ pullup off (Don't drive SRQ to high!)
      #endif
		bst	R_TEMP,4
		bld	R_TEMP2,IEC_OPIN_SRQ
    #endif
		out	_SFR_IO_ADDR(IEC_OUTPUT),R_TEMP2
		nop
		si_readbusdirect R_TEMP
		si_readbusdirect R_TEMP2
		si_readbusdirect R_TEMP3
		si_readbusdirect R_IRH
		ldi	R_CADDRL,lo8(error_buffer)
		ldi	R_CADDRH,hi8(error_buffer)
		st	R_CADDR+,R_TEMP
		st	R_CADDR+,R_TEMP2
		st	R_CADDR+,R_TEMP3
		st	R_CADDR+,R_IRH
    #ifdef SRQHANDLE
      #ifndef IEC_SEPARATE_OUT
		sbrs	_SFR_IO_ADDR(IEC_OUTPUT),IEC_OPIN_SRQ	;SKIP, if DDR bit switched to output (and line pulled low)
		sbi	_SFR_IO_ADDR(IEC_PORT),IEC_PIN_SRQ	;SRQ pullup on, if SRQ pin is input
      #endif
    #endif
		;rjmp	writeio_na				;	Next routine, jump not needed
  #endif  // VCPU_LINEDIAG
#endif	// CBM Serial lines I/O

;	Unimplemented register write, no function:
writeio_na:	setinstr					;1(2)
#ifdef INSTRTABLEALIGNED
		opcodefetchcall					;	Fetch next opcode, execute (4(7)+2)
#else
		rjmp	decode_fetch				;2
#endif

;=============================================
;=============================================
