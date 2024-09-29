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


   burstcmd.c: U0... (burst) commands parser

*/

#include <string.h>
#include <stdbool.h>
#include "config.h"
#include "burstcmd.h"
#include "atomic.h"
#include "flags.h"
#include "dirent.h"
#include "bus.h"
#include "iec.h"
#include "display.h"
#include "doscmd.h"
#include "buffers.h"
#include "fileops.h"
#include "fastloader-ll.h"
#include "errormsg.h"

#if CONFIG_FASTSERIAL_MODE >= 2

/*
 * BURST CMD NINE - CHGUTL UTILITY
 *      BYTE BIT  7       6       5       4       3       2       1       0 
 * ============================================================================
 *        00      0       1       0       1       0       1       0       1
 * ----------------------------------------------------------------------------
 *        01      0       0       1       1       0       0       0       0
 * ----------------------------------------------------------------------------
 *        02      X       X       X       1       1       1       1       0
 * ----------------------------------------------------------------------------
 *        03         UTILITY COMMANDS: 'S', 'B', 'M', 'R', 'T', 'I', 'V',
 *                                     'MR', 'MW', #DEV
 * ----------------------------------------------------------------------------
 *        04         COMMAND PARAMETER 
 * ----------------------------------------------------------------------------
 * SWITCHES:
 *      X - DON'T CARE.
 * 
 * UTILITY COMMANDS:
 *        'S' - DOS SECTOR INTERLEAVE.
 *        'B' - FAST SERIAL.
 *        'M' - MODE SELECT.
 *        'R' - DOS RETRIES.
 *        'T' - ROM SIGNATURE ANALYSIS.
 *        'I' - IEEE CACHE TIMEOUT
 *        'MR/W' - BURST MEMORY READ/WRITE
 *        'V' - VERIFY SELECT.
 *        #DEV - DEVICE #.
 * 
 * NOTE: BYTE 02 IS EQUIVALENT TO A '>'
 * 	
 * EXAMPLES:
 *      "U0>S"+CHR$(SECTOR-INTERLEAVE)
 *      "U0>I"+CHR$(IEEE_TIMEOUT_VALUE)
 *      "U0>R"+CHR$(RETRIES)
 *      "U0>T"
 *      "U0>B0"=SLOW SERIAL, "U0>B1"=FAST SERIAL    (1581)
 *      "U0>M1"=1571 MODE, "U0>M0"=1541 MODE        (1571)
 *      "U0>V1"=VERIFY ON, "U0>V0"=VERIFY OFF 
 *      "U0>MR"=BURST MEM READ, "U0>MW"=BURST MEM WRITE 
 *      "U0>"+CHR$(#DEV), WHERE #DEV = 4-30
 */
static uint8_t burstcommand_chgutl(void) {
  if (command_buffer[3] >= 4 &&
      command_buffer[3] <= 30) {
    device_address = command_buffer[3];
    display_address(device_address);
    return 0;
  } else if ((command_buffer[3] == 'B') || (command_buffer[3] == 'M')) {    // 'B'/1581, 'M'/1571
    if ((command_buffer[4] >= 0x30) && (command_buffer[4] <= 0x32)) {       // '0'/'1'/'2' OK
      xbusen_config = ((xbusen_config & 0xfc) | (command_buffer[4] & 0x03));
      fastser_configure();
      return 0;
    }
    return 1;
  }
  return 1;
}

/*
 * BURST CMD TEN - FASTLOAD UTILITY
 *      BYTE  BIT 7       6       5       4       3       2       1       0
 * ============================================================================
 *        00      0       1       0       1       0       1       0       1
 * ----------------------------------------------------------------------------
 *        01      0       0       1       1       0       0       0       0
 * ----------------------------------------------------------------------------
 *        02      P       X       X       1       1       1       1       1
 * ----------------------------------------------------------------------------
 *        03                    FILE NAME
 * ----------------------------------------------------------------------------
 * SWITCHES:
 *        P - SEQUENTIAL FILE BIT (P=1, DOES NOT HAVE TO BE A PROGRAM 
 *                                      FILE)
 *        X - DON'T CARE.
 * 
 * PROTOCOL:
 *        BURST.
 * 
 * OUTPUT:
 *         BURST STATUS BYTE PRECEEDING EACH SECTOR TRANSFERED.
 * 
 * STATUS IS AS FOLLOWS:
 *        0000000X ............. OK
 *      * 00000010 ............. FILE NOT FOUND
 *     ** 00011111 ............. EOI
 * 
 * * VALUES BETWEEN THE RANGE 3-15 SHOULD BE CONSIDERED A FILE READ ERROR.
 * ** EOI STATUS BYTE, FOLLOWED BY NUMBER OF BYTES TO FOLLOW.
 */

static uint8_t burstcommand_fstload(void) {
  buffer_t *buf;
  uint8_t l;
  bool first, nolast;
  memmove(&command_buffer[0], &command_buffer[3], command_length);    // Move buffer content back
  command_length -= 3;                                                // Remove "U0x" from length
  datacrc = 0xffff;         // This is probably a good place for crc init (Normal, 'slow' load is also init here)
  file_open(0);             // Open file with secondary address = 0
  buf = find_buffer(0);     // Search buffer
  if (buf == NULL) {
    if (current_error == ERROR_FILE_NOT_FOUND)
      burst_handsk(FSST_FILENOTFOUND);
    else
      burst_handsk(FSST_NODRIVE);
    return 0;
  }
  first = true;           // First block
  nolast = true;          // No last block
  do {
    l = buf->lastused - 1;          // 0xFF = 254 BYTEs (2..255)
    if (buf->sendeoi) {
      nolast = false;     // Last block
      if (burst_handsk(FSST_EOI)) break;          // Send EOI
      if (burst_handsk(first ? l - 2 : l)) break; // Send BYTE-no (if first block == last block, load address not counted)
    } else {
      if (burst_handsk(FSST_OK)) break;
    }
    first = false;
    if (burst_sendblock(&buf->data[2], l)) break;     // Send block
    if (nolast && (buf->refill(buf))) {
      burst_handsk(FSST_LOADERROR);
      break;
    }
  } while (nolast);
  /* Close file */
  buf->cleanup(buf);
  /* Free the buffer */
  free_buffer(buf);
  return 0;
}


/* ---------------------------- */
/*  U0 (burst) commands parser  */
/* ---------------------------- */
uint8_t parse_burstcommands(void) {
  uint8_t bcmd = command_buffer[2] & 0x1f;
  burst_init();
  if (command_length < 3) {         // $8030
    return 1;     // ERROR
  }
  //if ((bcmd != 30) && !(iec_data.iecflags & FASTSER_ACTIVE)) {      // $804B
  if ((bcmd != 30) && (!(xbusenflags & 0x80))) {
    return 1;     // ERROR
  }
  
  switch (bcmd) {
    case 0x00:            // fstrd    ; fast read drv #0 - 0000
    case 0x10:            // fstrd    ; fast read drv #0 - 0000
      return 0;
    case 0x01:            // ndkrd 	  ; fast read drv #1 - 0001
    case 0x11:            // ndkrd 	  ; fast read drv #1 - 0001
      return 0;
    case 0x02:            // fstwrt   ; fast write drv #0 - 0010
    case 0x12:            // fstwrt   ; fast write drv #0 - 0010
      return 0;
    case 0x03:            // ndkwrt	  ; fast write drv #1 - 0011
    case 0x13:            // ndkwrt   ; fast write drv #1 - 0011
      return 0;
    case 0x04:            // fstsek   ; seek disk drv #0 - 0100
    case 0x14:            // fstsek   ; seek disk drv #0 - 0100
      return 0;
    case 0x05:            // ndkrd    ; seek disk drv #1 - 0101
    case 0x15:            // ndkrd    ; seek disk drv #1 - 0101
      return 0;
    case 0x06:            // fstfmt   ; format disk drv #0 - 0110
    case 0x16:            // fstfmt
    case 0x07:            // fstfmt   ; format disk drv #1 - 0111
    case 0x17:            // fstfmt
      return 0;
    case 0x08:            // cpmint	  ; interleave disk drv #0 - 1000
    case 0x09:            // cpmint	  ; interleave disk drv #1 - 1001
      return 0;
    case 0x0a:            // querdk	  ; query disk format - 1010
    case 0x1a:            // querdk   ; query disk format - 1010
      return 0;
    case 0x0b:            // ndkrd    ; seek disk drv #1 - 1011
    case 0x1b:            // ndkrd    ; seek disk drv #1 - 1011
      return 0;
    case 0x0c:            // inqst    ; return disk status - 1100
      return 0;
    case 0x0d:            // ndkrd    ; return disk status - 1101
      return 0;
    case 0x0e:            // duplc1	  ; backup drv0 to drv1 - 1110
    case 0x0f:            // duplc1	  ; backup drv1 to drv0 - 1111
      return 0;

    case 0x18:            // unused
    case 0x19:            // unused
    case 0x1c:            // unused
    case 0x1d:            // unused
      return 0;
    case 0x1e:            // chgutl
      return burstcommand_chgutl();
    case 0x1f:            // fstload
      return burstcommand_fstload();
    default:
      break;
  }
  return 1;
}

#endif        // CONFIG_FASTSERIAL_MODE >= 2
