/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN and Jim Brain, see ff.c|h.

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


   iec.h: Definitions for the IEC handling code

*/

#ifndef IEC_H
#define IEC_H

#include "bus.h"

/**
 * struct iecflags_t - Bitfield of various flags, mostly IEC-related
 * @eoi_recvd      : Received EOI with the last byte read
 * @command_recvd  : Command or filename received
 * @jiffy_active   : JiffyDOS-capable master detected
 * @jiffy_load     : JiffyDOS LOAD operation detected
 * @dolphin_active : DolphinDOS parallel mode active
 *
 * NOTE: This was converted from a struct with bitfields to
 *       a single variable with macros because the struct
 *       version created worse code on AVR.
 *
 * This is a bitfield for a number of boolean variables used
 * (FIXME: This seems to be truncated)
 */

#define EOI_RECVD       (1<<0)
#define COMMAND_RECVD   (1<<1)
#define JIFFY_ACTIVE    (1<<2)
#define JIFFY_LOAD      (1<<3)

#ifdef CONFIG_PARALLEL_DOLPHIN
#  define DOLPHIN_ACTIVE (1<<4)
#else
#  define DOLPHIN_ACTIVE 0
#endif


#if ((CONFIG_FASTSERIAL_MODE >= 2) || defined(HAVE_PARALLEL))
extern uint8_t xbusen_config;
extern uint8_t xbusenflags;
#endif
#if CONFIG_FASTSERIAL_MODE >= 2
#  define FASTSER_ACTIVE (1<<5)
extern iec_bus_t fastsr;
#else
#  define FASTSER_ACTIVE 0
#endif

typedef struct {
  uint8_t iecflags;
  enum { BUS_IDLE = 0, BUS_ATNACTIVE, BUS_FOUNDATN, BUS_FORME, BUS_NOTFORME, BUS_ATNFINISH, BUS_ATNPROCESS, BUS_CLEANUP, BUS_SLEEP } bus_state;
  enum { DEVICE_IDLE = 0, DEVICE_LISTEN, DEVICE_TALK } device_state;
  uint8_t secondary_address;
} iec_data_t;

extern iec_data_t iec_data;

#if CONFIG_FASTSERIAL_MODE >= 1
/* Fast serial stuffs, variables defined in iec.c */
/* AVR uses GPIOR1/2 for these */
#  ifndef __AVR__
extern uint8_t fastsershreg;
extern volatile uint8_t fastserrecvb;
#  endif
#endif

void fastser_configure(void);
extern iec_bus_t clklinesave;
static inline void burst_init(void) {
  clklinesave = IEC_BIT_CLOCK;      // $8056 set CLK save state to default
}
uint8_t burst_handsk(uint8_t value);
uint8_t burst_sendblock(uint8_t *ptr, uint8_t len);

uint8_t iec_check_atn(void);
void iec_init(void);

void  __attribute__ ((noreturn)) iec_mainloop(void);

#endif
