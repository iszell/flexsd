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


   vcpu6502emu.c: VCPU frame for calling VCPU core

*/

#include "config.h"

#ifdef CONFIG_VCPUSUPPORT

#include <string.h>
#include "errormsg.h"
#include "buffers.h"
#include "doscmd.h"
#include "timer.h"
#include "fileops.h"
#include "fatops.h"
#include "diskchange.h"
#include "iec-bus.h"
#include "vcpu6502emu.h"

Tcpureg vcpuregs;
Tploaderdatas plparams;
volatile uint8_t emucalled;
volatile uint8_t interruptcode;
uint8_t b2decimal[3];

#ifdef VCPUDEBUG_EN
extern uint8_t _end;
extern uint8_t __stack;

void StackPaint(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));

void StackPaint(void) {
#if 0
  uint8_t *p = &_end;
  while(p <= &__stack) {
    *p = 0xc5;
    p++;
  }
#else
  __asm volatile ("    ldi r30,lo8(_end)\n"
                  "    ldi r31,hi8(_end)\n"
                  "    ldi r24,lo8(0xc5)\n" /* STACK_CANARY = 0xc5 */
                  "    ldi r25,hi8(__stack)\n"
                  "    rjmp .cmp\n"
                  ".loop:\n"
                  "    st Z+,r24\n"
                  ".cmp:\n"
                  "    cpi r30,lo8(__stack)\n"
                  "    cpc r31,r25\n"
                  "    brlo .loop\n"
                  "    breq .loop"::);
#endif
}

#endif

/* Set "file" parameters for VCPU usage */
void return_parameters(uint8_t secondary) {
  buffer_t *buf;
  vcpuregs.a = current_error;
  vcpuregs.x = buffers[ERRORBUFFER_IDX].lastused + 1;
  if (secondary < 15) {
    buf = find_buffer(secondary);
    if (buf) {
      vcpuregs.y = (((uint8_t *)buf->data - (uint8_t *)&bufferdata) >> 8);
      error_buffer[PARAMPOS+0] = buf->position;
      error_buffer[PARAMPOS+1] = buf->lastused;
      error_buffer[PARAMPOS+2] = buf->sendeoi;
      return;
    }
  }
  vcpuregs.y = 0xff;
  error_buffer[PARAMPOS+0] = 0;
  error_buffer[PARAMPOS+1] = 0;
  error_buffer[PARAMPOS+2] = 0;
}

/* Copy command string from VCPU mem to command_buffer */
uint8_t copymemtocommandbuffer(void) {
  uint16_t addr;
  if (vcpuregs.x > CONFIG_COMMAND_BUFFER_SIZE) {
    vcpuregs.a = 0xfe;          //  Return parameter "ERROR" code
    return 1;
  }
  addr = (vcpuregs.zph << 8) | vcpuregs.y;
  memcpy((uint8_t *)&command_buffer, ((uint8_t *)&bufferdata + addr), vcpuregs.x);
  return 0;
}

/* VCPU cycle */
void vcpu6502emu(void) {
  buffer_t *buf;
  if (emucalled != 0) return;       // No recursive call accepted
  emucalled = 1;
  while (emucalled) {
    timer_disable();                // Disable timer interrupt
    set_vcpurunflag(1);             // RUN flag set, if any hardware interrupt clear, VCPU core exit
    interruptcode = 0;
    vcpuregs.interrupt = 0;
    vcpuregs.reqfunction = 0;
    vcpu6502core();                 // Call VCPU
    timer_enable();                 // Re-Enable timer interrupt
    if (vcpuregs.interrupt == VCPU_FUNCTIONCALL) {
      switch (vcpuregs.reqfunction) {
        case SYSCALL_DIRECTCOMMAND_MEM: {
          if (copymemtocommandbuffer()) break;    // If error in params, return, else continue
        }
        case SYSCALL_DIRECTCOMMAND: {
          command_length = vcpuregs.x;
          buffers[ERRORBUFFER_IDX].lastused = 0;
          current_error = 0xff;     // Error code = "not set"
          parse_doscommand();
          vcpuregs.x = buffers[ERRORBUFFER_IDX].lastused + 1;
          vcpuregs.a = current_error;
          break;
        }
        case SYSCALL_OPEN_MEM: {
          if (copymemtocommandbuffer()) break;      // If error in params, return, else continue
        }
        case SYSCALL_OPEN: {
          command_length = vcpuregs.x;
          current_error = 0xff;     // Error code = "not set"
          if (vcpuregs.a < 15) file_open(vcpuregs.a);   // File Open and continue to return params
        }
        case SYSCALL_GETCHANNELPARAMS: {
          return_parameters(vcpuregs.a);
          break;
        }
        case SYSCALL_CLOSE: {
          buf = find_buffer(vcpuregs.a);
          current_error = 0xff;     // Error code = "not set"
          if (buf) {
            cleanup_and_free_buffer(buf);
            vcpuregs.a = 0x00;
          } else {
            vcpuregs.a = 0xff;
          }
          break;
        }
        case SYSCALL_CLOSEALL: {
          current_error = 0xff;     // Error code = "not set"
          free_multiple_buffers(FMB_USER_CLEAN);
          vcpuregs.a = 0x00;
          break;
        }
        case SYSCALL_REFILLBUFFER: {
          buf = find_buffer(vcpuregs.a);
          current_error = 0xff;     // Error code = "not set"
          if (!buf) return_parameters(0xff);
          else {
            buf->position = vcpuregs.x;
            buf->lastused = vcpuregs.y;
            if (buf->refill(buf)) return_parameters(0xff);
            else return_parameters(vcpuregs.a);
          }
          break;
        }
        case SYSCALL_CHANGEDISK: {
          linenum = vcpuregs.x;
          if (mount_line()) vcpuregs.a = 0x00;
          else vcpuregs.a = 0xff;
          break;
        }
        case SYSCALL_DISABLEATNIRQ: {
          set_atn_irq(0);           // Disable ATN interrupt
          break;
        }
        case SYSCALL_ENABLEATNIRQ: {
          set_atn_irq(1);           // Enable ATN interrupt
          break;
        }
        case SYSCALL_SETFATPARAMS: {
          fat_setbufferparams(vcpuregs.x, vcpuregs.y);
          vcpuregs.x = fatfile_blockstart;
          vcpuregs.y = fatfile_blockbytesno & 0xff;
          break;
        }
        case SYSCALL_EXIT_OK: {
          set_error(ERROR_OK);       // "OK"
          break;
        }
        case SYSCALL_EXIT_SETERROR: {
          set_error_ts(vcpuregs.a, vcpuregs.x, vcpuregs.y);
          break;
        }
        case SYSCALL_EXIT_FILLEDERROR: {
          buffers[ERRORBUFFER_IDX].lastused = vcpuregs.x - 1;
          if (buffers[ERRORBUFFER_IDX].lastused >= CONFIG_ERROR_BUFFER_SIZE) {
            buffers[ERRORBUFFER_IDX].lastused = 0;
          }
          break;
        }
        case SYSCALL_EXIT_REMAIN: {
          break;                    // No new data in error buffer
        }
        default: {
          set_error_ts(ERROR_VCPU, 101, vcpuregs.reqfunction);
          emucalled = 0;          // Unknown function: exit
          break;
        }
      }
      if (vcpuregs.reqfunction < 0x10) {    // If required, exit
        emucalled = 0;    
      }
    } else {
      set_error_ts(ERROR_VCPU, 100, vcpuregs.interrupt);
      break;              // No syscall: exit
    }
  }
  set_atn(1);
  set_clock(1);
  set_data(1);
  set_srq(1);
  if (current_error == ERROR_VCPU) free_multiple_buffers(FMB_USER_CLEAN);
  set_vcpurunflag(0);
  emucalled = 0;
  fat_setbufferparams(2, 254);      // Restore FAT buffer-start + buffer length
  set_atn_irq(1);                   // Enable ATN interrupt
}

#endif
