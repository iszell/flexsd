/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2017  Ingo Korb <ingo@akana.de>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN and Jim Brain, see ff.c|h.

   Virtual 6502 CPU (VCPU) emulation by balagesz, (C) 2021

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


   vcpu6502emu.h: VCPU frame, definitions

*/

#include "config.h"

#ifdef CONFIG_VCPUSUPPORT

#define VCPU_ERROR_ALIGN 0x80
#define VCPU_ERROR_ADDRESS 0x40
#define VCPU_ERROR_ILLEGALFC 0x20
#define VCPU_ERROR_HANGUP 0x10
#define VCPU_ERROR_CARD 0x08
#define VCPU_ERROR_ATN 0x04
#define VCPU_ERROR_RWADDR 0x02
#define VCPU_FUNCTIONCALL 0x01

#define SYSCALL_EXIT_OK 0x00
#define SYSCALL_EXIT_SETERROR 0x01
#define SYSCALL_EXIT_FILLEDERROR 0x02
#define SYSCALL_EXIT_REMAIN 0x03
#define SYSCALL_DISABLEATNIRQ 0x11
#define SYSCALL_ENABLEATNIRQ 0x12
#define SYSCALL_SETFATPARAMS 0x13
#define SYSCALL_DIRECTCOMMAND 0x21
#define SYSCALL_DIRECTCOMMAND_MEM 0x22
#define SYSCALL_OPEN 0x23
#define SYSCALL_OPEN_MEM 0x24
#define SYSCALL_CLOSE 0x25
#define SYSCALL_CLOSEALL 0x26
#define SYSCALL_REFILLBUFFER 0x27
#define SYSCALL_GETCHANNELPARAMS 0x28
#define SYSCALL_CHANGEDISK 0x31

#define PARAMPOS 96

#define VCPU_VERSION 0x01
#define VCPU_INTERFACE_IEEE 1
#define VCPU_INTERFACE_CBMSER 2
#define VCPU_INTERFACE_CBMSER_FS 3
#define VCPU_INTERFACE_TCBM 4

#ifdef CONFIG_HAVE_IEEE
  #define VCPU_BUSTYPE VCPU_INTERFACE_IEEE
#endif
#ifdef CONFIG_HAVE_IEC
  #define VCPU_BUSTYPE VCPU_INTERFACE_CBMSER
#endif

// For enable SRQ handle, define this:
// WARNING: not ready!
//#define SRQHANDLE

// For enable debug functions, define this:
//#define VCPUDEBUG_EN

#ifndef __ASSEMBLER__       /* !ASSEMBLER */

typedef struct {
  uint16_t crc;
  uint16_t startaddr;
  uint16_t vcpustartaddr;
} Tploaderdatas;

/* Virtual 6502 CPU registers
   This struct is part of the "API", don't reorder! */
typedef struct {
  uint16_t pc;
  uint8_t a;
  uint8_t x;
  uint8_t y;
  uint8_t sr;
  uint8_t sp;
  uint8_t sph;
  uint8_t zph;
  uint8_t interrupt;
  volatile uint8_t reqfunction;
  uint8_t lastopcode;
} Tcpureg;

extern Tcpureg vcpuregs;
extern Tploaderdatas plparams;
extern volatile uint8_t emucalled;
extern volatile uint8_t interruptcode;

void vcpu6502emu(void);
void vcpu6502core(void);

#else         /* ASSEMBLER */

#define vcpureg_pc vcpuregs+0
#define vcpureg_a vcpuregs+2
#define vcpureg_x vcpuregs+3
#define vcpureg_y vcpuregs+4
#define vcpureg_sr vcpuregs+5
#define vcpureg_sp vcpuregs+6
#define vcpureg_sph vcpuregs+7
#define vcpureg_zph vcpuregs+8
#define vcpureg_interrupt vcpuregs+9
#define vcpureg_reqfunction vcpuregs+10
#define vcpureg_lastopcode vcpuregs+11

#define VCPU_ADDR_COMMANDBUFFER 0xfc00
#define VCPU_ADDR_ERRORBUFFER 0xfd00
#define VCPU_ADDR_IO 0xfe00
#define VCPU_MAXIO_MASK 0x0f

.extern device_address;
.extern bufferdata;
.extern command_buffer;
.extern error_buffer;
.extern vcpuregs;
.extern interruptcode;

#endif        /* ASSEMBLER */
#endif
