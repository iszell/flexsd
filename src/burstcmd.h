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


   burstcmd.h: Definitions for the U0 (burst) commands parser

*/

#ifndef BURSTCMD_H
#define BURSTCMD_H

#include <stdint.h>


/* Fast Serial status codes: */
#define FSST_OK 1
#define FSST_FILENOTFOUND 2
#define FSST_LOADERROR 3
#define FSST_NODRIVE 15
#define FSST_EOI 31

uint8_t parse_burstcommands(void);




#endif
