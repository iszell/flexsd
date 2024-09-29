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


   llfl-parallel.c: Low level handling of parallel port transfers

*/

#include "config.h"

#ifdef PARALLEL_ENABLED

#include "bitband.h"
#include "iec-bus.h"
#include "timer.h"
#include "fastloader-ll.h"


uint8_t parallel_read(void) {
  return parallel_data_read();
}

void parallel_write(uint8_t value) {
  parallel_data_write(value);
  delay_us(1);
}

void parallel_set_dir(parallel_dir_t direction) {
  if (direction == PARALLEL_DIR_IN) {
    /* set all lines high - FIODIR is not used in open drain mode */
    parallel_data_write(0xff);
  }
}

void parallel_send_handshake(void) {
  parallel_hskline_outset(0);
  delay_us(2);
  parallel_hskline_outset(1);
}

void parallel_sethiz_handshake(void) {
  parallel_hskline_outset(1);
}

#endif
