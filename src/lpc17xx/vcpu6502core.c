/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2017  Ingo Korb <ingo@akana.de>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN and Jim Brain, see ff.c|h.

   Virtual 6502 CPU (VCPU) emulation by balagesz, (C) 2023+

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


   vcpu6502core.c: VCPU core, C implementation for "more-than-8-bit" targets

*/

#include <stdbool.h>
#include "config.h"
#include "buffers.h"
#include "vcpu6502emu.h"
#include "timer.h"
#include "bus.h"
#include "iec-bus.h"
#include "iec.h"
#include "errormsg.h"
#include "doscmd.h"
#include "uart.h"
#include "fastloader-ll.h"
#include "fastloader.h"
#include "llfl-common.h"



/* Pre-defined runtimes: */
enum { rtm05t = 0,
       rtm10t,
       rtm15t,
       rtm20t,
       rtm25t,
       rtm30t,
       rtm35t,
       rtm40t,
       rtm45t,
       rtm50t,
       rtm55t,
       rtm60t,
       rtm65t,
       rtm70t,
       rtm75t,
       rtm80t
};


/* (V)CPU flags:
   These flags are the same / in the same place as in the real 6502 CPU.
   Of course, they cannot be rearranged. In the program code, the exact
   bit position important later on, they are just named for clarity. */
#define f_n (1 << 7)
#define f_v (1 << 6)
#define f_x (1 << 5)
#define f_b (1 << 4)
#define f_d (1 << 3)
#define f_i (1 << 2)
#define f_z (1 << 1)
#define f_c (1 << 0)


#define memoryblockno CONFIG_BUFFER_COUNT
#define memorysize (memoryblockno << 8)
#define zpaddr (((uint16_t)vcpuregs.zph)<<8)
#define stackaddr (((uint16_t)vcpuregs.sph)<<8)

volatile uint8_t timedelay;
#ifdef HAVE_PARALLEL
uint8_t hskoutline;         // Parallel port handshake out line ("DRWP") saved state
#endif

/**
 * VCPU RUN flag for non-AVR targets
 * On ARM targets, "globalstatus" is handled in non-atomic mode.
 * It's easier if it is replaced by an external flag-byte.
 */
volatile uint8_t vcpurun;



/* Read BYTE from VCPU memory and increment PC: */
static inline uint8_t readbytepc(void) {
  return bufferdata[vcpuregs.pc++];
}

/* Push BYTE to STACK: */
static void pushtostack(uint8_t val) {
  bufferdata[stackaddr|vcpuregs.sp--] = val;
}

/* Pop BYTE from STACK: */
static uint8_t popfromstack(void) {
  return bufferdata[stackaddr|++vcpuregs.sp];
}



/* AM1: Address Mode 1: "($zp,X)", return: 6502 address: */
static uint16_t am1_getaddress(void) {
  uint8_t x = readbytepc()+vcpuregs.x;
  uint16_t a;
  a = bufferdata[zpaddr|x++];
  a |= bufferdata[zpaddr|x]<<8;
  return a;
}

/* AM2: Address Mode 2: "$zp", return: buffer (RAM) offset: */
static uint16_t am2_getbufferoffset(void) {
  return (zpaddr|readbytepc());
}

/* AM3: Address Mode 3: "#$xx", return: readed value: */
#define am3_read readbytepc

/* AM4: Address Mode 4: "$ghjk", return: 6502 address: */
static uint16_t am4_getaddress(void) {
  return (readbytepc()|(readbytepc() << 8));
}

/* AM5: Address Mode 5: "($zp),Y", return: 6502 address: */
static uint16_t am5_getaddress(void) {
  uint16_t a;
  uint8_t x = readbytepc();
  a = bufferdata[zpaddr|x++];
  a |= bufferdata[zpaddr|x]<<8;
  a += vcpuregs.y;
  return a;  
}

/* AM6: Address Mode 6: "$zp,X", return: buffer (RAM) offset: */
static uint16_t am6_getbufferoffset(void) {
  return (zpaddr|(readbytepc()+vcpuregs.x));
}

/* AM7: Address Mode 7: "$ghjk,Y", return: 6502 address: */
static uint16_t am7_getaddress(void) {
  return ((readbytepc()|(readbytepc() << 8))+vcpuregs.y);
}

/* AM8: Address mode 8: "$ghjk,X", return: 6502 address: */
static uint16_t am8_getaddress(void) {
  return ((readbytepc()|(readbytepc() << 8))+vcpuregs.x);
}

/* AM9: Address Mode 9: "$zp,Y", return: buffer (RA) offset: */
static uint16_t am9_getbufferoffset(void) {
  return (zpaddr|(readbytepc()+vcpuregs.y));
}



/* Set N/Z flags: */
static inline void setnzflags(uint8_t val) {
  vcpuregs.sr &= ~(f_n | f_z);        // N+Z flags clear
  if (val == 0) vcpuregs.sr |= f_z;   // If value == 0, Z flag set
  vcpuregs.sr |= (val & f_n);         // Copy B7 to N flag
}





void setrwaddrerror(void) {
  vcpuregs.interrupt |= VCPU_ERROR_RWADDR;      // Set "rwaddr" flag
  set_vcpurunflag(0);
}





// Unumplemented I/O read:
uint8_t readio_na(void) {
  return 0x00;
}

// VCPU version + bus type
uint8_t readio_version(void) {
  return (VCPU_BUSTYPE_REP<<5)|VCPU_VERSION;
}
// MEM + I/O area size
uint8_t readio_memiosize(void) {
  return ((VCPU_MAXIO_ADDRBITS-1)<<5)|memoryblockno;    // I/O area ~size (B765) + RAM size (B43210)
}
// HID + LEDs
uint8_t readio_hidbits(void) {
  rawbutton_t b = buttons_read();
  uint8_t v = 0x00;
  if (get_busy_led()) v |= 0x01;
  if (get_dirty_led()) v |= 0x02;
#if set_vcpu_led != set_test_led
  if (get_test_led()) v |= 0x04;
#endif
#ifdef BUTTON_SWAP
  if (!(b & BUTTON_SWAP)) v |= 0x10;
#endif
#ifdef BUTTON_MENU
  if (!(b & BUTTON_MENU)) v |= 0x20;
#endif
  if (!(b & BUTTON_PREV)) v |= 0x40;
  if (!(b & BUTTON_NEXT)) v |= 0x80;
  return v;
}
// Device address (8, 9, 10, 11, ...)
uint8_t readio_deviceaddress(void) {
  return (((device_hw_address()-8)<<6)&0xc0)|(device_address&0x1f);
}
// Host CPU speed (UART debug: Receive BYTE)
uint8_t readio_hostspd(void) {
#ifdef CONFIG_UART_DEBUG
  if (uart_gotc()) {
    vcpuregs.sr |= f_v;         // Set "V" flag, new char received
    return uart_getc();
  }
#endif
  return CONFIG_MCU_FREQ/1000000;
}
// Fw
uint8_t readio_checkfw(void) {
  return 0x00;
}

// Firmware "globalflags" variable
uint8_t readio_globalflags(void) {
#if CONFIG_FASTSERIAL_MODE < 1
  return globalflags|vcpurun;
#else
  return globalflags|vcpurun|fastserrecven_flag|fastserrecvrdy_flag;
#endif
}

// Binary to decimal converter: result Low digit
uint8_t readio_decimallow(void) {
  return b2decimal[0];
}
// Binary to decimal converter: result Mid digit
uint8_t readio_decimalmid(void) {
  return b2decimal[1];
}
// Binary to decimal converter: result High digit
uint8_t readio_decimalhigh(void) {
  return b2decimal[2];
}
#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
// ATN/SRQ Lines Output register read (B7=ATN, B6=SRQ)
uint8_t readio_cbmseraso(void) {
  int8_t l = 0x00;
  if (get_atndrive()) l |= 0x80;
  #ifdef SRQHANDLE
  if (get_srqdrive())
  #endif
    l |= 0x40;
  return l;
}
// ATN/SRQ Lines read back (B7=ATN, B6=SRQ)
uint8_t readio_cbmserasi(void) {
  uint8_t l = 0x00;
  iec_bus_t t = iec_bus_read();
  if (t&IEC_BIT_ATN) l |= 0x80;
  //#ifdef SRQHANDLE
  if (t&IEC_BIT_SRQ)
  //#endif
    l |= 0x40;
  return l;
}
// DAT/CLK Lines output register read (B1=DAT, B0=CLK)
uint8_t readio_cbmserdco(void) {
  int8_t l = 0x00;
  if (get_datadrive()) l |= 0x02;
  if (get_clockdrive()) l |= 0x01;
  return l;
}
// DAT/CLK Lines read back (B7=DAT, B6=CLK)
uint8_t readio_cbmserin(void) {
  uint8_t l = 0x00;
  iec_bus_t t = iec_bus_read();
  if (t&IEC_BIT_CLOCK) l |= 0x40;
  if (t&IEC_BIT_DATA) l |= 0x80;
  return l;
}
// CLK Line read back (B7=CLK)
uint8_t readio_cbmserci(void) {
  if (iec_bus_read()&IEC_BIT_CLOCK) return 0x80;
  return 0x00;
}
// DAT Line read back (B7=DAT)
uint8_t readio_cbmserdi(void) {
  if (iec_bus_read()&IEC_BIT_DATA) return 0x80;
  return 0x00;
}
#endif

#ifdef HAVE_PARALLEL
// $10: Parallel port lines read:
uint8_t readio_parlines(void) {
  return parallel_data_read();
}

// $11: Parallel port auxilary read:
uint8_t readio_paraux(void) {
  uint8_t r = 0x02;                                 // %00000010
  if (parallel_hskline_get()) r |= 0x80;            // If HRWP line is high, set B7
  if (parallel_rxflag) r |= 0x40;                   // If HRWFLAG set, set B6
  if (hskoutline) r |= 0x04;                        // If DRWP line not drived, set B2
  if (parallel_hskline_outget()) r |= 0x01;         // If DRWP line high, set B0
  return r;
}
#endif



/* I/O area readable registers: */
uint8_t (* const readiotable[VCPU_MAXIO_SIZE])(void) = {
  &readio_version,        // $00: VCPU version + bus type
  &readio_memiosize,      // $01: MEM + I/O area size
  &readio_hidbits,        // $02: HID + LEDs
  &readio_deviceaddress,  // $03: Device address (8, 9, 10, 11)
  &readio_hostspd,        // $04: Host CPU speed (UART debug: Receive BYTE)
  &readio_checkfw,        // $05:
  &readio_globalflags,    // $06: Firmware "globalflags" variable
  &readio_decimallow,     // $07: Binary to decimal converter: result Low digit
  &readio_decimalmid,     // $08: Binary to decimal converter: result Mid digit
  &readio_decimalhigh,    // $09: Binary to decimal converter: result High digit
#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
  &readio_cbmseraso,      // $0A: ATN/SRQ Lines Output register read (B7=ATN, B6=SRQ)
  &readio_cbmserasi,      // $0B: ATN/SRQ Lines read back (B7=ATN, B6=SRQ)
  &readio_cbmserdco,      // $0C: DAT/CLK Lines output register read (B1=DAT, B0=CLK)
  &readio_cbmserin,       // $0D: DAT/CLK Lines read back (B7=DAT, B6=CLK)
  &readio_cbmserci,       // $0E: CLK Line read back (B7=CLK)
  &readio_cbmserdi        // $0F: DAT Line read back (B7=DAT)
 #ifdef HAVE_PARALLEL
  ,
  &readio_parlines,       // $10: Parallel port lines
  &readio_paraux          // $11: Parallel port auxilary
 #endif
#else
  &readio_na,             // $0A:
  &readio_na,             // $0B:
  &readio_na,             // $0C:
  &readio_na,             // $0D:
  &readio_na,             // $0E:
  &readio_na              // $0F:
#endif
};




/* Memory read function, return readed BYTE: */
uint8_t readmemory(uint16_t addr) {
  uint8_t p;
  uint8_t (*funcptr)(void);
  if (addr < memorysize) return bufferdata[addr];
  p = addr&0x00ff;
  switch (addr&0xff00) {
    case VCPU_ADDR_IO:
      if (p < VCPU_MAXIO_SIZE) {
        funcptr = readiotable[p];
        return funcptr();
      }
      break;
    case VCPU_ADDR_COMMANDBUFFER:
      if (p < CONFIG_COMMAND_BUFFER_SIZE) return command_buffer[p];
      break;
    case VCPU_ADDR_ERRORBUFFER:
      if (p < CONFIG_ERROR_BUFFER_SIZE) return error_buffer[p];
      break;
  }
  setrwaddrerror();
  return 0xee;
}



// Unumplemented I/O write:
void writeio_na(uint8_t val) {
  (void)val;
}

// $02: LEDs
void writeio_hidbits(uint8_t val) {
  if (val & 0x01) set_busy_led(1); else set_busy_led(0);
  if (val & 0x02) set_dirty_led(1); else set_dirty_led(0);
#if set_vcpu_led != set_test_led
  if (val & 0x04) set_test_led(1); else set_test_led(0);
#endif
}
// $04: (UART debug: Transmit BYTE)
#ifndef CONFIG_UART_DEBUG
  #define writeio_dbg_uarttx writeio_na
#else
void writeio_dbg_uarttx(uint8_t val) {
  uart_putc_direct(val);
}
#endif

// Binary to decimal converter:
void wrio_convbinto(uint8_t hibits, uint8_t val) {
  b2decimal[2] = hibits|(val/100);
  val = val%100;
  b2decimal[1] = hibits|(val/10);
  val = val%10;
  b2decimal[0] = hibits|(val/1);
}
// $07: Binary to decimal converter: convert to decimal
void writeio_convbin2dec(uint8_t val) {
  wrio_convbinto(0x00, val);
}
// $08: Binary to decimal converter: convert to ascii
void writeio_convbin2asc(uint8_t val) {
  wrio_convbinto('0', val);
}

#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
// $01: Linediag
#ifndef VCPU_LINEDIAG
  #define writeio_linediag writeio_na
#else
static uint8_t wrio_ldiagcalclines(iec_bus_t lines) {
  uint8_t r = 0x00;
  if (lines&IEC_BIT_DATA) r |= 0x80;
  if (lines&IEC_BIT_CLOCK) r |= 0x40;
  if (lines&IEC_BIT_ATN) r |= 0x20;
  //#ifdef SRQHANDLE
  if (lines&IEC_BIT_SRQ)
  //#endif
    r |= 0x10;
  return r;
}
void writeio_linediag(uint8_t val) {
  iec_bus_t r1, r2, r3, r4;
  switch (val & 0xf0) {
    case 0xf0:
      r1 = iec_bus_read();
      if (!(r1&IEC_BIT_DATA)) {
        set_data(1);
        break;
      } else if (!(r1&IEC_BIT_CLOCK)) {
        set_clock(1);
        break;
      } else if (!(r1&IEC_BIT_ATN)) {
        set_atn(1);
        break;
  //#ifdef SRQHANDLE
      } else if (!(r1&IEC_BIT_SRQ)) {
        set_srq(1);
        break;
  //#endif
      }
      break;
    case 0x70:
      set_data(0);
      break;
    case 0xb0:
      set_clock(0);
      break;
    case 0xd0:
      set_atn(0);
      break;
  #ifdef SRQHANDLE
    case 0xe0:
      set_srq(0);
      break;
  #endif
    default:
      set_data(1);
      set_clock(1);
      set_atn(1);
  //#ifdef SRQHANDLE
      set_srq(1);
  //#endif
      break;
  }
  r1 = iec_bus_read();
  r2 = iec_bus_read();
  r3 = iec_bus_read();
  r4 = iec_bus_read();
  error_buffer[0] = wrio_ldiagcalclines(r1);
  error_buffer[1] = wrio_ldiagcalclines(r2);
  error_buffer[2] = wrio_ldiagcalclines(r3);
  error_buffer[3] = wrio_ldiagcalclines(r4);
  timedelay = 1;                // Add extra time
#endif
}
// $0A: ATN/SRQ Lines Output register write (B7=ATN, B6=SRQ)
void writeio_cbmseraso(uint8_t val) {
  if (val & 0x80) set_atn(1); else set_atn(0);
  #ifdef SRQHANDLE  
  if (val & 0x40) set_srq(1); else set_srq(0);
  #endif
}
// $0C: DAT/CLK Lines output register write (B1=DAT, B0=CLK)
void writeio_cbmserdco(uint8_t val) {
  if (val & 0x02) set_data(1); else set_data(0);
  if (val & 0x01) set_clock(1); else set_clock(0);
}
// $0E: CLK Line write (CLK=B0)
void writeio_cbmserco(uint8_t val) {
  if (val & 0x01) set_clock(1); else set_clock(0);
}
// $0F: DAT Line write (DAT=B0)
void writeio_cbmserdo(uint8_t val) {
  if (val & 0x01) set_data(1); else set_data(0);
}
#endif

#ifdef HAVE_PARALLEL
// $10: Parallel port lines write:
void writeio_parlines(uint8_t val) {
  parallel_data_write(val);
}
// $11: Parallel port auxilary wite:
void writeio_paraux(uint8_t val) {
  if (val & 0x04) {
    parallel_hskline_outset(1);
    hskoutline = 1;
  } else {
    parallel_hskline_outset(0);
    hskoutline = 0;
  }
  if (val & 0x10) parallel_rxflag = 0;
}
#endif


/* I/O area writeable registers: */
void (* const writeiotable[VCPU_MAXIO_SIZE])(uint8_t val) = {
  &writeio_na,            // $00:
#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
  &writeio_linediag,      // $01: Linediag
#else
  &writeio_na,            // $02:
#endif
  &writeio_hidbits,       // $02: LEDs
  &writeio_na,            // $03:
  &writeio_dbg_uarttx,    // $04: (UART debug: Transmit BYTE)
  &writeio_na,            // $05:
  &writeio_na,            // $06:
  &writeio_convbin2dec,   // $07: Binary to decimal converter: convert to decimal
  &writeio_convbin2asc,   // $08: Binary to decimal converter: convert to ascii
  &writeio_na,            // $09:
#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER
  &writeio_cbmseraso,     // $0A: ATN/SRQ Lines Output register write (B7=ATN, B6=SRQ)
  &writeio_na,            // $0B:
  &writeio_cbmserdco,     // $0C: DAT/CLK Lines output register write (B1=DAT, B0=CLK)
  &writeio_na,            // $0D:
  &writeio_cbmserco,      // $0E: CLK Line write (CLK=B0)
  &writeio_cbmserdo       // $0F: DAT Line write (DAT=B0)
 #ifdef HAVE_PARALLEL
  ,
  &writeio_parlines,      // $10: Parallel port lines
  &writeio_paraux         // $11: Parallel port auxilary
 #endif
#else
  &writeio_na,            // $0A:
  &writeio_na,            // $0B:
  &writeio_na,            // $0C:
  &writeio_na,            // $0D:
  &writeio_na,            // $0E:
  &writeio_na             // $0F:
#endif
};

/* Memory write function: */
void writememory(uint16_t addr, uint8_t val) {
  uint8_t p;
  void (*funcptr)(uint8_t val);
  if (addr < memorysize) {
    bufferdata[addr] = val;
    return;
  }
  p = addr&0x00ff;
  switch (addr&0xff00) {
    case VCPU_ADDR_IO:
      if (p < VCPU_MAXIO_SIZE) {
        funcptr = writeiotable[p];
        funcptr(val);
        return;
      }
      break;
    case VCPU_ADDR_COMMANDBUFFER:
      if (p < CONFIG_COMMAND_BUFFER_SIZE) {
        command_buffer[p] = val;
        return;
      }
      break;
    case VCPU_ADDR_ERRORBUFFER:
      if (p < CONFIG_ERROR_BUFFER_SIZE) {
        error_buffer[p] = val;
        return;
      }
      break;
  }
  setrwaddrerror();
}





/* Illegal instruction: */
vcputick_t ninstr_hangup(void) {
  vcpuregs.interrupt |= VCPU_ERROR_HANGUP;      // Set "hangup" flag
  set_vcpurunflag(0);
  return 0;
}

/* $00: BRK */
vcputick_t instr_brk(void) {
  vcpuregs.reqfunction = readbytepc();          // Read function code
  vcpuregs.interrupt |= VCPU_FUNCTIONCALL;      // Set "functioncall" flag
  set_vcpurunflag(0);
  return 0;
}



/* === LDA/X/Y: */
// $A1: LDA ($zp,X)
vcputick_t instr_lda_am1(void) {
  vcpuregs.a = readmemory(am1_getaddress());
  setnzflags(vcpuregs.a);
  return rtm50t;
}

// $A5: LDA $zp
vcputick_t instr_lda_am2(void) {
  vcpuregs.a = bufferdata[am2_getbufferoffset()];
  setnzflags(vcpuregs.a);
  return rtm25t;
}

// $A9: LDA #$gh
vcputick_t instr_lda_am3(void) {
  vcpuregs.a = am3_read();
  setnzflags(vcpuregs.a);
  return rtm20t;
}

// $AD: LDA $ghjk
vcputick_t instr_lda_am4(void) {
  vcpuregs.a = readmemory(am4_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}

// $B1: LDA ($zp),Y
vcputick_t instr_lda_am5(void) {
  vcpuregs.a = readmemory(am5_getaddress());
  setnzflags(vcpuregs.a);
  return rtm50t;
}

// $B5: LDA $zp,X
vcputick_t instr_lda_am6(void) {
  vcpuregs.a = bufferdata[am6_getbufferoffset()];
  setnzflags(vcpuregs.a);
  return rtm20t;
}

// $B9: LDA $ghjk,Y
vcputick_t instr_lda_am7(void) {
  vcpuregs.a = readmemory(am7_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}

// $BD: LDA $ghjk,X
vcputick_t instr_lda_am8(void) {
  vcpuregs.a = readmemory(am8_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}


// $A2: LDX #$gh
vcputick_t instr_ldx_am3(void) {
  vcpuregs.x = am3_read();
  setnzflags(vcpuregs.x);
  return rtm20t;
}

// $A6: LDX $zp
vcputick_t instr_ldx_am2(void) {
  vcpuregs.x = bufferdata[am2_getbufferoffset()];
  setnzflags(vcpuregs.x);
  return rtm25t;
}

// $AE: LDX $ghjk
vcputick_t instr_ldx_am4(void) {
  vcpuregs.x = readmemory(am4_getaddress());
  setnzflags(vcpuregs.x);
  return rtm40t;
}

// $B6: LDX $zp,Y
vcputick_t instr_ldx_am9(void) {
  vcpuregs.x = bufferdata[am9_getbufferoffset()];
  setnzflags(vcpuregs.x);
  return rtm20t;
}

// $BE: LDX $ghjk,Y
vcputick_t instr_ldx_am7(void) {
  vcpuregs.x = readmemory(am7_getaddress());
  setnzflags(vcpuregs.x);
  return rtm40t;
}


// $A0: LDY #$gh
vcputick_t instr_ldy_am3(void) {
  vcpuregs.y = am3_read();
  setnzflags(vcpuregs.y);
  return rtm20t;
}

// $A4: LDY $zp
vcputick_t instr_ldy_am2(void) {
  vcpuregs.y = bufferdata[am2_getbufferoffset()];
  setnzflags(vcpuregs.y);
  return rtm25t;
}

// $AC: LDY $ghjk
vcputick_t instr_ldy_am4(void) {
  vcpuregs.y = readmemory(am4_getaddress());
  setnzflags(vcpuregs.y);
  return rtm40t;
}

// $B4: LDY $zp,X
vcputick_t instr_ldy_am6(void) {
  vcpuregs.y = bufferdata[am6_getbufferoffset()];
  setnzflags(vcpuregs.y);
  return rtm20t;
}

// $BC: LDY $ghjk,X
vcputick_t instr_ldy_am8(void) {
  vcpuregs.y = readmemory(am8_getaddress());
  setnzflags(vcpuregs.y);
  return rtm40t;
}



/* === STA/X/Y: */
// $81: STA ($zp,X)
vcputick_t instr_sta_am1(void) {
  writememory(am1_getaddress(), vcpuregs.a);
  return rtm50t;
}

// $85: STA $zp
vcputick_t instr_sta_am2(void) {
  bufferdata[am2_getbufferoffset()] = vcpuregs.a;
  return rtm25t;
}

// $8D: STA $ghjk
vcputick_t instr_sta_am4(void) {
  writememory(am4_getaddress(), vcpuregs.a);
  return rtm40t;
}

// $91: STA ($zp),Y
vcputick_t instr_sta_am5(void) {
  writememory(am5_getaddress(), vcpuregs.a);
  return rtm50t;
}

// $95: STA $zp,X
vcputick_t instr_sta_am6(void) {
  bufferdata[am6_getbufferoffset()] = vcpuregs.a;
  return rtm20t;
}

// $99: STA $ghjk,Y
vcputick_t instr_sta_am7(void) {
  writememory(am7_getaddress(), vcpuregs.a);
  return rtm40t;
}

// $9D: STA $ghjk,X
vcputick_t instr_sta_am8(void) {
  writememory(am8_getaddress(), vcpuregs.a);
  return rtm40t;
}


// $86: STX $zp
vcputick_t instr_stx_am2(void) {
  bufferdata[am2_getbufferoffset()] = vcpuregs.x;
  return rtm25t;
}

// $8E: STX $ghjk
vcputick_t instr_stx_am4(void) {
  writememory(am4_getaddress(), vcpuregs.x);
  return rtm40t;
}

// $96: STX $zp,Y
vcputick_t instr_stx_am9(void) {
  bufferdata[am9_getbufferoffset()] = vcpuregs.x;
  return rtm20t;
}


// $84: STY $zp
vcputick_t instr_sty_am2(void) {
  bufferdata[am2_getbufferoffset()] = vcpuregs.y;
  return rtm25t;
}

// $8C: STY $ghjk
vcputick_t instr_sty_am4(void) {
  writememory(am4_getaddress(), vcpuregs.y);
  return rtm40t;
}

// $94: STY $zp,X
vcputick_t instr_sty_am6(void) {
  bufferdata[am6_getbufferoffset()] = vcpuregs.y;
  return rtm20t;
}



/* === AND: */
// $21: AND ($zp,X)
vcputick_t instr_and_am1(void) {
  vcpuregs.a &= readmemory(am1_getaddress());
  setnzflags(vcpuregs.a);
  return rtm50t;
}

// $25: AND $zp
vcputick_t instr_and_am2(void) {
  vcpuregs.a &= bufferdata[am2_getbufferoffset()];
  setnzflags(vcpuregs.a);
  return rtm25t;
}

// $29: AND #$gh
vcputick_t instr_and_am3(void) {
  vcpuregs.a &= am3_read();
  setnzflags(vcpuregs.a);
  return rtm20t;
}

// $2D: AND $ghjk
vcputick_t instr_and_am4(void) {
  vcpuregs.a &= readmemory(am4_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}

// $31: AND ($zp),Y
vcputick_t instr_and_am5(void) {
  vcpuregs.a &= readmemory(am5_getaddress());
  setnzflags(vcpuregs.a);
  return rtm50t;
}

// $35: AND $zp,X
vcputick_t instr_and_am6(void) {
  vcpuregs.a &= bufferdata[am6_getbufferoffset()];
  setnzflags(vcpuregs.a);
  return rtm30t;
}

// $39: AND $ghjk,Y
vcputick_t instr_and_am7(void) {
  vcpuregs.a &= readmemory(am7_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}

// $3D: AND $ghjk,X
vcputick_t instr_and_am8(void) {
  vcpuregs.a &= readmemory(am8_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}



/* === ORA: */
// $01: ORA ($zp,X)
vcputick_t instr_ora_am1(void) {
  vcpuregs.a |= readmemory(am1_getaddress());
  setnzflags(vcpuregs.a);
  return rtm50t;
}

// $05: ORA $zp
vcputick_t instr_ora_am2(void) {
  vcpuregs.a |= bufferdata[am2_getbufferoffset()];
  setnzflags(vcpuregs.a);
  return rtm25t;
}

// $09: ORA #$gh
vcputick_t instr_ora_am3(void) {
  vcpuregs.a |= am3_read();
  setnzflags(vcpuregs.a);
  return rtm20t;
}

// $0D: ORA $ghjk
vcputick_t instr_ora_am4(void) {
  vcpuregs.a |= readmemory(am4_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}

// $11: ORA ($zp),Y
vcputick_t instr_ora_am5(void) {
  vcpuregs.a |= readmemory(am5_getaddress());
  setnzflags(vcpuregs.a);
  return rtm50t;
}

// $15: ORA $zp,X
vcputick_t instr_ora_am6(void) {
  vcpuregs.a |= bufferdata[am6_getbufferoffset()];
  setnzflags(vcpuregs.a);
  return rtm30t;
}

// $19: ORA $ghjk,Y
vcputick_t instr_ora_am7(void) {
  vcpuregs.a |= readmemory(am7_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}

// $1D: ORA $ghjk,X
vcputick_t instr_ora_am8(void) {
  vcpuregs.a |= readmemory(am8_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}



/* === EOR: */
// $41: EOR ($zp,X)
vcputick_t instr_eor_am1(void) {
  vcpuregs.a ^= readmemory(am1_getaddress());
  setnzflags(vcpuregs.a);
  return rtm50t;
}

// $45: EOR $zp
vcputick_t instr_eor_am2(void) {
  vcpuregs.a ^= bufferdata[am2_getbufferoffset()];
  setnzflags(vcpuregs.a);
  return rtm25t;
}

// $49: EOR #$gh
vcputick_t instr_eor_am3(void) {
  vcpuregs.a ^= am3_read();
  setnzflags(vcpuregs.a);
  return rtm20t;
}

// $4D: EOR $ghjk
vcputick_t instr_eor_am4(void) {
  vcpuregs.a ^= readmemory(am4_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}

// $51: EOR ($zp),Y
vcputick_t instr_eor_am5(void) {
  vcpuregs.a ^= readmemory(am5_getaddress());
  setnzflags(vcpuregs.a);
  return rtm50t;
}

// $55: EOR $zp,X
vcputick_t instr_eor_am6(void) {
  vcpuregs.a ^= bufferdata[am6_getbufferoffset()];
  setnzflags(vcpuregs.a);
  return rtm30t;
}

// $59: EOR $ghjk,Y
vcputick_t instr_eor_am7(void) {
  vcpuregs.a ^= readmemory(am7_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}

// $5D: EOR $ghjk,X
vcputick_t instr_eor_am8(void) {
  vcpuregs.a ^= readmemory(am8_getaddress());
  setnzflags(vcpuregs.a);
  return rtm40t;
}



/* === ADC: */
static inline uint8_t instrall_adc(uint8_t val1, uint8_t val2) {
  uint8_t c6 = ((val1&0x7f)+(val2&0x7f)+(vcpuregs.sr&f_c))&0x80;
  uint16_t r = val1+val2+(vcpuregs.sr&f_c);
  vcpuregs.sr &= ~(f_n|f_v|f_z|f_c);                // N+V+Z+C flags clear
  if (r > 255) vcpuregs.sr |= f_c;                  // C flag
  if (r & 0x0080) vcpuregs.sr |= f_n;               // N flag
  if (!(r & 0x00ff)) vcpuregs.sr |= f_z;            // Z flag
  if (c6^((r>>1)&0x80)) vcpuregs.sr |= f_v;         // V flag
  return r & 0xff;
}

// $61: ADC ($zp,X)
vcputick_t instr_adc_am1(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, readmemory(am1_getaddress()));
  return rtm50t;
}

// $65: ADC $zp
vcputick_t instr_adc_am2(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, bufferdata[am2_getbufferoffset()]);
  return rtm25t;
}

// $69: ADC #$gh
vcputick_t instr_adc_am3(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, am3_read());
  return rtm20t;
}

// $6D: ADC $ghjk
vcputick_t instr_adc_am4(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, readmemory(am4_getaddress()));
  return rtm40t;
}

// $71: ADC ($zp),Y
vcputick_t instr_adc_am5(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, readmemory(am5_getaddress()));
  return rtm50t;
}

// $75: ADC $zp,X
vcputick_t instr_adc_am6(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, bufferdata[am6_getbufferoffset()]);
  return rtm30t;
}

// $79: ADC $ghjk,Y
vcputick_t instr_adc_am7(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, readmemory(am7_getaddress()));
  return rtm40t;
}

// $7D: ADC $ghjk,X
vcputick_t instr_adc_am8(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, readmemory(am8_getaddress()));
  return rtm40t;
}



/* === SBC: */
// $E1: SBC ($zp,X)
vcputick_t instr_sbc_am1(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, (readmemory(am1_getaddress())^0xff));
  return rtm50t;
}

// $E5: SBC $zp
vcputick_t instr_sbc_am2(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, (bufferdata[am2_getbufferoffset()]^0xff));
  return rtm25t;
}

// $E9: SBC #$gh
vcputick_t instr_sbc_am3(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, (am3_read()^0xff));
  return rtm20t;
}

// $ED: SBC $ghjk
vcputick_t instr_sbc_am4(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, (readmemory(am4_getaddress())^0xff));
  return rtm40t;
}

// $F1: SBC ($zp),Y
vcputick_t instr_sbc_am5(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, (readmemory(am5_getaddress())^0xff));
  return rtm50t;
}

// $F5: SBC $zp,X
vcputick_t instr_sbc_am6(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, (bufferdata[am6_getbufferoffset()]^0xff));
  return rtm30t;
}

// $F9: SBC $ghjk,Y
vcputick_t instr_sbc_am7(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, (readmemory(am7_getaddress())^0xff));
  return rtm40t;
}

// $FD: SBC $ghjk,X
vcputick_t instr_sbc_am8(void) {
  vcpuregs.a = instrall_adc(vcpuregs.a, (readmemory(am8_getaddress())^0xff));
  return rtm40t;
}



/* === CMP/X/Y: */
static inline void instrall_cmp(uint8_t val1, uint8_t val2) {
  uint16_t r = val1+val2+1;
  vcpuregs.sr &= ~(f_n|f_z|f_c);                    // N+Z+C flags clear
  if (r > 255) vcpuregs.sr |= f_c;                  // C flag
  if (r & 0x0080) vcpuregs.sr |= f_n;               // N flag
  if (!(r & 0x00ff)) vcpuregs.sr |= f_z;            // Z flag
}

// $C1: CMP ($zp,X)
vcputick_t instr_cmp_am1(void) {
  instrall_cmp(vcpuregs.a, (readmemory(am1_getaddress())^0xff));
  return rtm50t;
}

// $C5: CMP $zp
vcputick_t instr_cmp_am2(void) {
  instrall_cmp(vcpuregs.a, (bufferdata[am2_getbufferoffset()]^0xff));
  return rtm25t;
}

// $C9: CMP #$gh
vcputick_t instr_cmp_am3(void) {
  instrall_cmp(vcpuregs.a, (am3_read()^0xff));
  return rtm20t;
}

// $CD: CMP $ghjk
vcputick_t instr_cmp_am4(void) {
  instrall_cmp(vcpuregs.a, (readmemory(am4_getaddress())^0xff));
  return rtm40t;
}

// $D1: CMP ($zp),Y
vcputick_t instr_cmp_am5(void) {
  instrall_cmp(vcpuregs.a, (readmemory(am5_getaddress())^0xff));
  return rtm50t;
}

// $D5: CMP $zp,X
vcputick_t instr_cmp_am6(void) {
  instrall_cmp(vcpuregs.a, (bufferdata[am6_getbufferoffset()]^0xff));
  return rtm30t;
}

// $D9: CMP $ghjk,Y
vcputick_t instr_cmp_am7(void) {
  instrall_cmp(vcpuregs.a, (readmemory(am7_getaddress())^0xff));
  return rtm40t;
}

// $DD: CMP $ghjk,X
vcputick_t instr_cmp_am8(void) {
  instrall_cmp(vcpuregs.a, (readmemory(am8_getaddress())^0xff));
  return rtm40t;
}


// $E0: CPX #$gh
vcputick_t instr_cpx_am3(void) {
  instrall_cmp(vcpuregs.x, (am3_read()^0xff));
  return rtm20t;
}

// $E4: CPX $zp
vcputick_t instr_cpx_am2(void) {
  instrall_cmp(vcpuregs.x, (bufferdata[am2_getbufferoffset()]^0xff));
  return rtm25t;
}

// $EC: CPX $ghjk
vcputick_t instr_cpx_am4(void) {
  instrall_cmp(vcpuregs.x, (readmemory(am4_getaddress())^0xff));
  return rtm40t;
}


// $C0: CPY #$gh
vcputick_t instr_cpy_am3(void) {
  instrall_cmp(vcpuregs.y, (am3_read()^0xff));
  return rtm20t;
}

// $C4: CPY $zp
vcputick_t instr_cpy_am2(void) {
  instrall_cmp(vcpuregs.y, (bufferdata[am2_getbufferoffset()]^0xff));
  return rtm25t;
}

// $CC: CPY $ghjk
vcputick_t instr_cpy_am4(void) {
  instrall_cmp(vcpuregs.y, (readmemory(am4_getaddress())^0xff));
  return rtm40t;
}



/* === ASL: */
static inline uint8_t instrall_asl(uint8_t val) {
  vcpuregs.sr &= ~(f_c);
  if (val & 0x80) vcpuregs.sr |= f_c;
  val <<= 1;
  setnzflags(val);
  return val;
}

// $06: ASL $zp
vcputick_t instr_asl_am2(void) {
  uint16_t x = am2_getbufferoffset();
  bufferdata[x] = instrall_asl(bufferdata[x]);
  return rtm40t;
}

// $0A: ASL
vcputick_t instr_asl(void) {
  vcpuregs.a = instrall_asl(vcpuregs.a);
  return rtm15t;
}

// $0E: ASL $ghjk
vcputick_t instr_asl_am4(void) {
  uint8_t d;
  uint16_t a = am4_getaddress();
  d = readmemory(a);
  d = instrall_asl(d);
  writememory(a, d);
  return rtm50t;
}

// $16: ASL $zp,X
vcputick_t instr_asl_am6(void) {
  uint16_t x = am6_getbufferoffset();
  bufferdata[x] = instrall_asl(bufferdata[x]);
  return rtm40t;
}

// $1E: ASL $ghjk,X
vcputick_t instr_asl_am8(void) {
  uint8_t d;
  uint16_t a = am8_getaddress();
  d = readmemory(a);
  d = instrall_asl(d);
  writememory(a, d);
  return rtm50t;
}



/* === LSR: */
static inline uint8_t instrall_lsr(uint8_t val) {
  vcpuregs.sr &= ~(f_c);
  if (val & 0x01) vcpuregs.sr |= f_c;
  val >>= 1;
  setnzflags(val);
  return val;
}

// $46: LSR $zp
vcputick_t instr_lsr_am2(void) {
  uint16_t x = am2_getbufferoffset();
  bufferdata[x] = instrall_lsr(bufferdata[x]);
  return rtm40t;
}

// $4A: LSR
vcputick_t instr_lsr    (void) {
  vcpuregs.a = instrall_lsr(vcpuregs.a);
  return rtm15t;
}

// $4E: LSR $ghjk
vcputick_t instr_lsr_am4(void) {
  uint8_t d;
  uint16_t a = am4_getaddress();
  d = readmemory(a);
  d = instrall_lsr(d);
  writememory(a, d);
  return rtm50t;
}

// $56: LSR $zp,X
vcputick_t instr_lsr_am6(void) {
  uint16_t x = am6_getbufferoffset();
  bufferdata[x] = instrall_lsr(bufferdata[x]);
  return rtm40t;
}

// $5E: LSR $ghjk,X
vcputick_t instr_lsr_am8(void) {
  uint8_t d;
  uint16_t a = am8_getaddress();
  d = readmemory(a);
  d = instrall_lsr(d);
  writememory(a, d);
  return rtm50t;
}



/* === ROL: */
static inline uint8_t instrall_rol(uint8_t val) {
  uint8_t x = vcpuregs.sr&f_c;
  vcpuregs.sr &= ~(f_c);
  if (val & 0x80) vcpuregs.sr |= f_c;
  val = (val<<1)|x;
  setnzflags(val);
  return val;
}

// $26: ROL $zp
vcputick_t instr_rol_am2(void) {
  uint16_t x = am2_getbufferoffset();
  bufferdata[x] = instrall_rol(bufferdata[x]);
  return rtm40t;
}

// $2A: ROL
vcputick_t instr_rol(void) {
  vcpuregs.a = instrall_rol(vcpuregs.a);
  return rtm15t;
}

// $2E: ROL $ghjk
vcputick_t instr_rol_am4(void) {
  uint8_t d;
  uint16_t a = am4_getaddress();
  d = readmemory(a);
  d = instrall_rol(d);
  writememory(a, d);
  return rtm50t;
}

// $36: ROL $zp,X
vcputick_t instr_rol_am6(void) {
  uint16_t x = am6_getbufferoffset();
  bufferdata[x] = instrall_rol(bufferdata[x]);
  return rtm40t;
}

// $3E: ROL $ghjk,X
vcputick_t instr_rol_am8(void) {
  uint8_t d;
  uint16_t a = am8_getaddress();
  d = readmemory(a);
  d = instrall_rol(d);
  writememory(a, d);
  return rtm50t;
}



/* === ROR: */
static inline uint8_t instrall_ror(uint8_t val) {
  uint8_t x = vcpuregs.sr&f_c;
  vcpuregs.sr &= ~(f_c);
  if (val & 0x01) vcpuregs.sr |= f_c;
  val = (val>>1)|(x<<7);
  setnzflags(val);
  return val;
}

// $66: ROR $zp
vcputick_t instr_ror_am2(void) {
  uint16_t x = am2_getbufferoffset();
  bufferdata[x] = instrall_ror(bufferdata[x]);
  return rtm40t;
}

// $6A: ROR
vcputick_t instr_ror(void) {
  vcpuregs.a = instrall_ror(vcpuregs.a);
  return rtm15t;
}

// $6E: ROR $ghjk
vcputick_t instr_ror_am4(void) {
  uint8_t d;
  uint16_t a = am4_getaddress();
  d = readmemory(a);
  d = instrall_ror(d);
  writememory(a, d);
  return rtm50t;
}

// $76: ROR $zp,X
vcputick_t instr_ror_am6(void) {
  uint16_t x = am6_getbufferoffset();
  bufferdata[x] = instrall_ror(bufferdata[x]);
  return rtm40t;
}

// $7E: ROR $ghjk,X
vcputick_t instr_ror_am8(void) {
  uint8_t d;
  uint16_t a = am8_getaddress();
  d = readmemory(a);
  d = instrall_ror(d);
  writememory(a, d);
  return rtm50t;
}



/* === INC: */
static inline uint8_t instrall_inc(uint8_t val) {
  val++;
  setnzflags(val);
  return val;
}

// $E6: INC $zp
vcputick_t instr_inc_am2(void) {
  uint16_t x = am2_getbufferoffset();
  bufferdata[x] = instrall_inc(bufferdata[x]);
  return rtm40t;
}

// $EE: INC $ghjk
vcputick_t instr_inc_am4(void) {
  uint8_t d;
  uint16_t a = am4_getaddress();
  d = readmemory(a);
  d = instrall_inc(d);
  writememory(a, d);
  return rtm50t;
}

// $F6: INC $zp,X
vcputick_t instr_inc_am6(void) {
  uint16_t x = am6_getbufferoffset();
  bufferdata[x] = instrall_inc(bufferdata[x]);
  return rtm40t;
}

// $FE: INC $ghjk,X
vcputick_t instr_inc_am8(void) {
  uint8_t d;
  uint16_t a = am8_getaddress();
  d = readmemory(a);
  d = instrall_inc(d);
  writememory(a, d);
  return rtm50t;
}



/* === DEC: */
static inline uint8_t instrall_dec(uint8_t val) {
  val--;
  setnzflags(val);
  return val;
}

// $C6: DEC $zp
vcputick_t instr_dec_am2(void) {
  uint16_t x = am2_getbufferoffset();
  bufferdata[x] = instrall_dec(bufferdata[x]);
  return rtm40t;
}

// $CE: DEC $ghjk
vcputick_t instr_dec_am4(void) {
  uint8_t d;
  uint16_t a = am4_getaddress();
  d = readmemory(a);
  d = instrall_dec(d);
  writememory(a, d);
  return rtm50t;
}

// $D6: DEC $zp,X
vcputick_t instr_dec_am6(void) {
  uint16_t x = am6_getbufferoffset();
  bufferdata[x] = instrall_dec(bufferdata[x]);
  return rtm40t;
}

// $DE: DEC $ghjk,X
vcputick_t instr_dec_am8(void) {
  uint8_t d;
  uint16_t a = am8_getaddress();
  d = readmemory(a);
  d = instrall_dec(d);
  writememory(a, d);
  return rtm50t;
}



/* === INX/Y/DEX/Y: */
// $E8: INX
vcputick_t instr_inx(void) {
  vcpuregs.x++;
  setnzflags(vcpuregs.x);
  return rtm15t;
}

// $C8: INY
vcputick_t instr_iny(void) {
  vcpuregs.y++;
  setnzflags(vcpuregs.y);
  return rtm15t;
}

// $CA: DEX
vcputick_t instr_dex(void) {
  vcpuregs.x--;
  setnzflags(vcpuregs.x);
  return rtm15t;
}

// $88: DEY
vcputick_t instr_dey(void) {
  vcpuregs.y--;
  setnzflags(vcpuregs.y);
  return rtm15t;
}



/* === BIT: */
static inline void instrall_bit(uint8_t val) {
  vcpuregs.sr = (vcpuregs.sr&(~(f_n|f_v|f_z)))|(val&(f_n|f_v)); // Copy B7/B6 to SR N/V flags
  if (!(val&vcpuregs.a)) vcpuregs.sr |= f_z;                    // Set Z flag
}

// $24: BIT $zp
vcputick_t instr_bit_am2(void) {
  instrall_bit(bufferdata[am2_getbufferoffset()]);
  return rtm20t;
}

// $2C: BIT $ghjk
vcputick_t instr_bit_am4(void) {
  uint8_t x = readmemory(am4_getaddress());
  instrall_bit(x);
  return rtm35t;
}



/* === JMP/JSR/RTS/RTI: */
static inline bool checkmemoryaddress(uint16_t addr) {
  if (addr < memorysize) return true;
  vcpuregs.interrupt |= VCPU_ERROR_ADDRESS;     // Set "address" flag
  set_vcpurunflag(0);
  return false;
}

// $4C: JMP $ghjk
vcputick_t instr_jmp_am4(void) {
  vcpuregs.pc = am4_getaddress();
  checkmemoryaddress(vcpuregs.pc);    // Check memory address
  return rtm30t;
}

// $6C: JMP ($ghjk)
vcputick_t instr_jmp_am10(void) {
  vcpuregs.pc = am4_getaddress();
  if (vcpuregs.pc >= (memorysize-1)) {
    vcpuregs.interrupt |= VCPU_ERROR_ADDRESS;     // Set "address" flag
    set_vcpurunflag(0);
    return 0;
  }
  instr_jmp_am4();
  return rtm50t;
}

// $20: JSR $ghjk
vcputick_t instr_jsr_am4(void) {
  uint16_t a, s;
  a = readbytepc();         // Read subroutine address B7..0
  s = vcpuregs.pc;          // Save actual PC
  a |= readbytepc()<<8;     // Read subroutine address B15..8
  pushtostack(s>>8);        // Save return address Hi to stack
  pushtostack(s&0xff);      // Save return address Lo to stack
  vcpuregs.pc = a;
  checkmemoryaddress(vcpuregs.pc);    // Check memory address
  return rtm25t;
}

// $60: RTS
vcputick_t instr_rts(void) {
  vcpuregs.pc = popfromstack();       // Restore PC lo from stack
  vcpuregs.pc |= popfromstack()<<8;   // Restore PC hi from stack
  vcpuregs.pc++;
  checkmemoryaddress(vcpuregs.pc);    // Check memory address
  return rtm25t;
}

// $40: RTI
vcputick_t instr_rti(void) {
  vcpuregs.sr = popfromstack();       // Restore SR
  vcpuregs.pc = popfromstack();       // Restore PC lo from stack
  vcpuregs.pc |= popfromstack()<<8;   // Restore PC hi from stack
  checkmemoryaddress(vcpuregs.pc);    // Check memory address
  return rtm35t;
}



/* === Bcc: */
// $10: BPL $rtyu
vcputick_t instr_bpl(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  if (!(vcpuregs.sr&f_n)) vcpuregs.pc += o;   // If "N" flag = 0
  return rtm20t;
}

// $30: BMI $rtyu
vcputick_t instr_bmi(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  if (vcpuregs.sr&f_n) vcpuregs.pc += o;      // If "N" flag = 1
  return rtm20t;
}

// $50: BVC $rtyu
vcputick_t instr_bvc(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  if (!(vcpuregs.sr&f_v)) vcpuregs.pc += o;   // If "V" flag = 0
  return rtm20t;
}

// $70: BVS $rtyu
vcputick_t instr_bvs(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  if (vcpuregs.sr&f_v) vcpuregs.pc += o;      // If "V" flag = 1
  return rtm20t;
}

// $90: BCC $rtyu
vcputick_t instr_bcc(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  if (!(vcpuregs.sr&f_c)) vcpuregs.pc += o;   // If "C" flag = 0
  return rtm20t;
}

// $B0: BCS $rtyu
vcputick_t instr_bcs(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  if (vcpuregs.sr&f_c) vcpuregs.pc += o;      // If "C" flag = 1
  return rtm20t;
}

// $D0: BNE $rtyu
vcputick_t instr_bne(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  if (!(vcpuregs.sr&f_z)) vcpuregs.pc += o;   // If "Z" flag = 0
  return rtm20t;
}

// $F0: BEQ $rtyu
vcputick_t instr_beq(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  if (vcpuregs.sr&f_z) vcpuregs.pc += o;      // If "Z" flag = 1
  return rtm20t;
}



/* === Txx: */
// $AA: TAX
vcputick_t instr_tax(void) {
  vcpuregs.x = vcpuregs.a;
  setnzflags(vcpuregs.x);
  return rtm20t;
}

// $8A: TXA
vcputick_t instr_txa(void) {
  vcpuregs.a = vcpuregs.x;
  setnzflags(vcpuregs.a);
  return rtm20t;
}

// $A8: TAY
vcputick_t instr_tay(void) {
  vcpuregs.y = vcpuregs.a;
  setnzflags(vcpuregs.y);
  return rtm20t;
}

// $98: TYA
vcputick_t instr_tya(void) {
  vcpuregs.a = vcpuregs.y;
  setnzflags(vcpuregs.a);
  return rtm20t;
}

// $BA: TSX
vcputick_t instr_tsx(void) {
  vcpuregs.x = vcpuregs.sp;
  setnzflags(vcpuregs.x);
  return rtm20t;
}

// $9A: TXS
vcputick_t instr_txs(void) {
  vcpuregs.sp = vcpuregs.x;
  return rtm20t;
}



/* === SEx/CLx: */
// $18: CLC
vcputick_t instr_clc(void) {
  vcpuregs.sr &= ~(f_c);
  return rtm15t;
}

// $38: SEC
vcputick_t instr_sec(void) {
  vcpuregs.sr |= f_c;
  return rtm15t;
}

// $B8: CLV
vcputick_t instr_clv(void) {
  vcpuregs.sr &= ~(f_v);
  return rtm15t;
}

// $EA: NOP
vcputick_t instr_nop(void) {
  return rtm10t;
}

// $78: SEI
vcputick_t instr_sei(void) {
  return rtm15t;
}

// $D8: CLD
vcputick_t instr_cld(void) {
  return rtm15t;
}



/* === PHx/PLx: */
// $48: PHA
vcputick_t instr_pha(void) {
  pushtostack(vcpuregs.a);
  return rtm15t;
}

// $68: PLA
vcputick_t instr_pla(void) {
  vcpuregs.a = popfromstack();
  setnzflags(vcpuregs.a);
  return rtm15t;
}

// $08: PHP
vcputick_t instr_php(void) {
  vcpuregs.sr |= (f_x|f_b);     // not used + Break bit set
  vcpuregs.sr &= ~(f_d);        // Decimal bit clear
  pushtostack(vcpuregs.sr);
  return rtm20t;
}

// $28: PLP
vcputick_t instr_plp(void) {
  vcpuregs.sr = popfromstack();
  vcpuregs.sr |= (f_x|f_b);     // not used + Break bit set
  vcpuregs.sr &= ~(f_d);        // Decimal bit clear
  return rtm20t;
}



/* === Extended instructions: */
static inline bool checkmemsize(uint8_t val) {
  if (val < memoryblockno) return true;
  vcpuregs.interrupt |= VCPU_ERROR_ADDRESS;     // Set "address" flag
  set_vcpurunflag(0);
  return false;
}

// $54: (IOp) TZPHY
vcputick_t einstr_tzphy(void) {
  vcpuregs.y = vcpuregs.zph;
  setnzflags(vcpuregs.y);
  return rtm15t;
}

// $44: (IOp) TYZPH
vcputick_t einstr_tyzph(void) {
  if (checkmemsize(vcpuregs.y)) vcpuregs.zph = vcpuregs.y;
  return rtm15t;
}

// $02: (IOp) LDZPH
vcputick_t einstr_ldzph(void) {
  uint8_t v = am3_read();
  if (checkmemsize(v)) vcpuregs.zph = v;
  return rtm20t;
}

// $F4: (IOp) TSPHY
vcputick_t einstr_tsphy(void) {
  vcpuregs.y = vcpuregs.sph;
  setnzflags(vcpuregs.y);
  return rtm15t;
}

// $D4: (IOp) TYSPH
vcputick_t einstr_tysph(void) {
  if (checkmemsize(vcpuregs.y)) vcpuregs.sph = vcpuregs.y;
  return rtm15t;
}

// $22: (IOp) LDSPH
vcputick_t einstr_ldsph(void) {
  uint8_t v = am3_read();
  if (checkmemsize(v))  vcpuregs.sph = v;
  return rtm20t;
}

// $0F: (IOp) (W/R65C02: BBR0 $zp,offs) USERR (R2)
vcputick_t einstr_userr(void) {
  vcpuregs.pc = vcpuregs.rr;
  return rtm10t;
}

// $1F: (IOp) (W/R65C02: BBR1 $zp,offs) USER1 (R2)
vcputick_t einstr_user1(void) {
  vcpuregs.rr = vcpuregs.pc;
  vcpuregs.pc = vcpuregs.u1r;
  return rtm15t;
}

// $2F: (IOp) (W/R65C02: BBR2 $zp,offs) USER2 (R2)
vcputick_t einstr_user2(void) {
  vcpuregs.rr = vcpuregs.pc;
  vcpuregs.pc = vcpuregs.u2r;
  return rtm15t;
}

// $CB: (IOp) (65C02: WAI) CONFG (VCPU configuration, 2+ BYTEs OpCode) (R2)
vcputick_t einstr_confg(void) {
  uint16_t v;
  uint8_t m = readbytepc();       // Read mode
  if (m <= 7) {
    switch (m & 0x07) {
      case 0x00:            // %000: PULRR
        v = popfromstack();
        v |= popfromstack()<<8;
        if (checkmemoryaddress(v)) vcpuregs.rr = v;
        break;
      case 0x02:            // %010: TYXU1
        v = vcpuregs.x|(vcpuregs.y<<8);
        if (checkmemoryaddress(v)) vcpuregs.u1r = v;
        break;
      case 0x04:            // %100: TYXU2
        v = vcpuregs.x|(vcpuregs.y<<8);
        if (checkmemoryaddress(v)) vcpuregs.u2r = v;
        break;
      case 0x06:            // %110: TYXRR
        v = vcpuregs.x|(vcpuregs.y<<8);
        if (checkmemoryaddress(v)) vcpuregs.rr = v;
        break;
      case 0x01:            // %001: PSHRR
        pushtostack(vcpuregs.rr>>8);
        pushtostack(vcpuregs.rr&0xff);
        break;
      case 0x03:            // %011: TU1YX
        vcpuregs.x = vcpuregs.u1r&0xff;
        vcpuregs.y = vcpuregs.u1r>>8;
        break;
      case 0x05:            // %101: TU2YX
        vcpuregs.x = vcpuregs.u2r&0xff;
        vcpuregs.y = vcpuregs.u2r>>8;
        break;
      case 0x07:            // %111: TRRYX
        vcpuregs.x = vcpuregs.rr&0xff;
        vcpuregs.y = vcpuregs.rr>>8;
        break;
    }
  }
#if CONFIG_FASTSERIAL_MODE >= 1
    else if ((m&0xf8) == 0x10) {
    if (m&0x01) {
      fastser_recven();
    } else {
      fastser_recvdis();
    }
  }
#endif
    else {
    ninstr_hangup();
    return 0;
  }
  return rtm40t;
}



/* === µOp instructions: */
#if VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER

// $03: (IOp) UWDTL
vcputick_t uinstr_op_0x03(void) {
  if (iec_bus_read()&IEC_BIT_DATA) vcpuregs.pc--;
  //while (!((!(iec_bus_read()&IEC_BIT_DATA))||(!vcpurun))) {}
  return 0;
}

// $23: (IOp) UWCKL
vcputick_t uinstr_op_0x23(void) {
  if (iec_bus_read()&IEC_BIT_CLOCK) vcpuregs.pc--;
  //while (!((!(iec_bus_read()&IEC_BIT_CLOCK))||(!vcpurun))) {}
  return 0;
}

// $43: (IOp) UWATL
vcputick_t uinstr_op_0x43(void) {
  if (iec_bus_read()&IEC_BIT_ATN) vcpuregs.pc--;
  //while (!((!(iec_bus_read()&IEC_BIT_ATN))||(!vcpurun))) {}
  return 0;
}

// $13: (IOp) UWDTH
vcputick_t uinstr_op_0x13(void) {
  if (!(iec_bus_read()&IEC_BIT_DATA)) vcpuregs.pc--;
  //while (!((iec_bus_read()&IEC_BIT_DATA)||(!vcpurun))) {}
  return 0;
}

// $33: (IOp) UWCKH
vcputick_t uinstr_op_0x33(void) {
  if (!(iec_bus_read()&IEC_BIT_CLOCK)) vcpuregs.pc--;
  //while (!((iec_bus_read()&IEC_BIT_CLOCK)||(!vcpurun))) {}
  return 0;
}

// $53: (IOp) UWATH
vcputick_t uinstr_op_0x53(void) {
  if (!(iec_bus_read()&IEC_BIT_ATN)) vcpuregs.pc--;
  //while (!((iec_bus_read()&IEC_BIT_ATN)||(!vcpurun))) {}
  return 0;
}

// $83: (IOp) USDTL
vcputick_t uinstr_op_0x83(void) {
  //vcputimer_waitnext05t();
  set_data(0);
  return rtm15t;
}

// $93: (IOp) USDTH
vcputick_t uinstr_op_0x93(void) {
  //vcputimer_waitnext05t();
  set_data(1);
  return rtm15t;
}

// $A3: (IOp) USCKL
vcputick_t uinstr_op_0xa3(void) {
  //vcputimer_waitnext05t();
  set_clock(0);
  return rtm15t;
}

// $B3: (IOp) USCKH
vcputick_t uinstr_op_0xb3(void) {
  //vcputimer_waitnext05t();
  set_clock(1);
  return rtm15t;
}

// $C3: (IOp) USATL
vcputick_t uinstr_op_0xc3(void) {
  //vcputimer_waitnext05t();
  set_atn(0);
  return rtm15t;
}

// $D3: (IOp) USATH
vcputick_t uinstr_op_0xd3(void) {
  //vcputimer_waitnext05t();
  set_atn(1);
  return rtm15t;
}

// $0B: (IOp) UCLDL
vcputick_t uinstr_op_0x0b(void) {
  //vcputimer_waitnext05t();
  //set_clock(0);
  //set_data(0);
  set_clockdata(0, 0);
  return rtm15t;
}

// $1B: (IOp) UCLDH
vcputick_t uinstr_op_0x1b(void) {
  //vcputimer_waitnext05t();
  //set_data(1);
  //set_clock(0);
  set_clockdata(0, 1);
  return rtm15t;
}

// $2B: (IOp) UCHDL
vcputick_t uinstr_op_0x2b(void) {
  //vcputimer_waitnext05t();
  //set_clock(1);
  //set_data(0);
  set_clockdata(1, 0);
  return rtm15t;
}

// $3B: (IOp) UCHDH
vcputick_t uinstr_op_0x3b(void) {
  //vcputimer_waitnext05t();
  //set_data(1);
  //set_clock(1);
  set_clockdata(1, 1);
  return rtm15t;
}

// $4B: (IOp) UATDT #$xx: "A" register bit to DAT line:
vcputick_t uinstr_op_0x4b(void) {
  uint8_t m = readbytepc();                   // Read bitmask
  vcputimer_waitnext05t();
  set_data(vcpuregs.a&m);
  return rtm20t;
}

// $5B: (IOp) UDTTA #$xx: DAT line to "A" register bit:
vcputick_t uinstr_op_0x5b(void) {
  vcputimer_waitnext05t();
  iec_bus_t t = iec_bus_read();
  uint8_t m = readbytepc();                   // Read bitmask
  vcpuregs.a |= m;
  if (!(t&IEC_BIT_DATA)) vcpuregs.a &= ~m;
  return rtm20t;
}

// $6B: (IOp) UATCK #$xx: "A" register bit to CLK line:
vcputick_t uinstr_op_0x6b(void) {
  uint8_t m = readbytepc();                   // Read bitmask
  vcputimer_waitnext05t();
  set_clock(vcpuregs.a&m);
  return rtm20t;
}

// $7B: (IOp) UCKTA #$xx: CLK line to "A" register bit:
vcputick_t uinstr_op_0x7b(void) {
  uint8_t m = readbytepc();                   // Read bitmask
  vcputimer_waitnext05t();
  iec_bus_t t = iec_bus_read();
  vcpuregs.a |= m;
  if (!(t&IEC_BIT_CLOCK)) vcpuregs.a &= ~m;
  return rtm20t;
}

// $8B: (IOp) UATCD #$xx, #$xx: A register bits to CLK/DAT Lines:
vcputick_t uinstr_op_0x8b(void) {
  uint8_t mc = readbytepc();                  // Read bitmask for CLK
  uint8_t md = readbytepc();                  // Read bitmask for DAT
  vcputimer_waitnext05t();
  //set_clock(vcpuregs.a&mc);
  //set_data(vcpuregs.a&md);
  set_clockdata(vcpuregs.a&mc, vcpuregs.a&md);
  return rtm25t;
}

// $9B: (IOp) UCDTA #$xx, #$xx: CLK/DAT lines to A register bits:
vcputick_t uinstr_op_0x9b(void) {
  uint8_t mc = readbytepc();                  // Read bitmask for CLK
  uint8_t md = readbytepc();                  // Read bitmask for DAT
  vcputimer_waitnext05t();
  iec_bus_t t = iec_bus_read();
  vcpuregs.a |= mc;
  vcpuregs.a |= md;
  if (!(t&IEC_BIT_CLOCK)) vcpuregs.a &= ~mc;
  if (!(t&IEC_BIT_DATA)) vcpuregs.a &= ~md;
  return rtm25t;
}

// $DB: (IOp) (65C02: STP) ULBIT
vcputick_t uinstr_op_0xdb(void) {
  vcputimer_waitnext05t();
  iec_bus_t t = iec_bus_read();
  if (t&IEC_BIT_CLOCK) vcpuregs.sr |= f_n; else vcpuregs.sr &= ~f_n;
  if (t&IEC_BIT_DATA) vcpuregs.sr |= f_z; else vcpuregs.sr &= ~f_z;
  if (t&IEC_BIT_ATN) vcpuregs.sr |= f_v; else vcpuregs.sr &= ~f_v;
  return rtm20t;
}

// $63: (IOp) USND1: Send A register bits to DAT:
vcputick_t uinstr_op_0x63(void) {
  uint8_t v = vcpuregs.a;
  uint8_t c;
  set_clock(1);             // Release CLK
  for (c = 0; c < 4; c++) {
    while (vcpurun)
      if (iec_bus_read()&IEC_BIT_CLOCK) break;
    set_data(v&0x01);
    if (!vcpurun) break;
    v >>= 1;
    while (vcpurun)
      if (!(iec_bus_read()&IEC_BIT_CLOCK)) break;
    set_data(v&0x01);
    v >>= 1;
    if (!vcpurun) break;
  }
  return 0;
}

// $73: (IOp) URCV1
vcputick_t uinstr_op_0x73(void) {
  uint8_t c;
  set_clock(1);             // Release CLK
  set_data(1);              // Release DAT
  for (c = 0; c < 4; c++) {
    while (vcpurun)
      if (!(iec_bus_read()&IEC_BIT_CLOCK)) break;
    if (!vcpurun) break;
    vcpuregs.a >>= 1;
    if (iec_bus_read()&IEC_BIT_DATA) vcpuregs.a |= 0x80;
    while (vcpurun)
      if (iec_bus_read()&IEC_BIT_CLOCK) break;
    if (!vcpurun) break;
    vcpuregs.a >>= 1;
    if (iec_bus_read()&IEC_BIT_DATA) vcpuregs.a |= 0x80;
  }
  return 0;
}

// $E3: (IOp) USND2: Send A register (bits-pairs) to CLK/DAT:
vcputick_t uinstr_op_0xe3(void) {
  uint8_t v = vcpuregs.a;
  uint8_t c;
  set_atn(1);               // Release ATN
  for (c = 0; c < 2; c++) {
    while (vcpurun)
      if (iec_bus_read()&IEC_BIT_ATN) break;
    //set_data(v&0x02);
    //set_clock(v&0x01);
    set_clockdata(v&0x01, v&0x02);
    if (!vcpurun) break;
    v >>= 2;
    while (vcpurun)
      if (!(iec_bus_read()&IEC_BIT_ATN)) break;
    //set_data(v&0x02);
    //set_clock(v&0x01);
    set_clockdata(v&0x01, v&0x02);
    v >>= 2;
    if (!vcpurun) break;
  }
  return 0;
}

// $F3: (IOp) URCV2
vcputick_t uinstr_op_0xf3(void) {
  uint8_t c;
  iec_bus_t t;
  //set_clock(1);             // Release CLK
  //set_data(1);              // Release DAT
  set_clockdata(1, 1);      // Release CLK+DAT
  set_atn(1);               // Release ATN
  for (c = 0; c < 2; c++) {
    while (vcpurun)
      if (!(iec_bus_read()&IEC_BIT_ATN)) break;
    if (!vcpurun) break;
    vcpuregs.a >>= 2;
    t = iec_bus_read();
    if (t&IEC_BIT_DATA) vcpuregs.a |= 0x40;
    if (t&IEC_BIT_CLOCK) vcpuregs.a |= 0x80;
    while (vcpurun)
      if (iec_bus_read()&IEC_BIT_ATN) break;
    if (!vcpurun) break;
    vcpuregs.a >>= 2;
    t = iec_bus_read();
    if (t&IEC_BIT_DATA) vcpuregs.a |= 0x40;
    if (t&IEC_BIT_CLOCK) vcpuregs.a |= 0x80;
  }
  return 0;
}

  #if CONFIG_FASTSERIAL_MODE >= 1
// $8F: (IOp) (W/R65C02: BBS0 $zp,offs) FSTXB
vcputick_t uinstr_op_0x8f(void) {
  fastser_sendbyte_vcpu(vcpuregs.a);
  return (8*9)+2+rtm30t;              // RunTime: "40T"
}

// $9F: (IOp) (W/R65C02: BBS1 $zp,offs) FSRXB
vcputick_t uinstr_op_0x9f(void) {
  vcputimer_waitnext05t();
  while (vcpurun) {
    vcputimer_waitnext05t();
    if (fastserrecvrdy_flag) {
      vcpuregs.a = fastserrecvb;
      fastserrecvrdy_flag = 0x00;
      return 0;
    }
  }
  return 0;
}

// $AF: (IOp) (W/R65C02: BBS2 $zp,offs) FSRXC
vcputick_t uinstr_op_0xaf(void) {
  if (fastserrecvrdy_flag) vcpuregs.sr |= f_v; else vcpuregs.sr &= ~(f_v);
  return rtm15t;
}
  #else     // Fast serial disable
#define uinstr_op_0x8f ninstr_hangup
#define uinstr_op_0x9f ninstr_hangup
#define uinstr_op_0xaf ninstr_hangup
  #endif    // Fast serial


  #ifdef HAVE_PARALLEL
// $07: (IOp) PPACK
vcputick_t uinstr_op_0x07(void) {
  vcputimer_waitnext05t();
  parallel_hskline_outset(0);
  vcputimer_waitnext05t();
  vcputimer_waitnext05t();
  parallel_hskline_outset(1);
  hskoutline = 1;
  return rtm25t;
}

// $17: (IOp) PPDRD
vcputick_t uinstr_op_0x17(void) {
  vcputimer_waitnext05t();
  vcpuregs.a = parallel_data_read();
  vcputimer_waitnext05t();
  parallel_rxflag = 0;
  return rtm20t;
}

// $27: (IOp) PPDWR
vcputick_t uinstr_op_0x27(void) {
  vcputimer_waitnext05t();
  parallel_data_write(vcpuregs.a);
  vcputimer_waitnext05t();
  parallel_rxflag = 0;
return rtm20t;
}

// $37: (IOp) PPWAI
vcputick_t uinstr_op_0x37(void) {
  vcputimer_waitnext05t();
  while (vcpurun) {
    vcputimer_waitnext05t();
    if (parallel_rxflag)
      while (vcpurun)
        if (parallel_hskline_get())
          return 0;
  }
  return 0;
}

// $47: (IOp) PPWDW
vcputick_t uinstr_op_0x47(void) {
  while (vcpurun) {
    vcputimer_waitnext05t();
    if (parallel_rxflag) {
      vcputimer_waitnext05t();
      (void)uinstr_op_0x27();
      return 0;
    }
  }
  return 0;
}
  #else     // No parallel interface
#define uinstr_op_0x07 ninstr_hangup
#define uinstr_op_0x17 ninstr_hangup
#define uinstr_op_0x27 ninstr_hangup
#define uinstr_op_0x37 ninstr_hangup
#define uinstr_op_0x47 ninstr_hangup
  #endif    // Parallel




#else       // VCPU_BUSTYPE != VCPU_INTERFACE_CBMSER
#define uinstr_op_0x03 ninstr_hangup
#define uinstr_op_0x23 ninstr_hangup
#define uinstr_op_0x43 ninstr_hangup
#define uinstr_op_0x13 ninstr_hangup
#define uinstr_op_0x33 ninstr_hangup
#define uinstr_op_0x53 ninstr_hangup
#define uinstr_op_0x83 ninstr_hangup
#define uinstr_op_0x93 ninstr_hangup
#define uinstr_op_0xa3 ninstr_hangup
#define uinstr_op_0xb3 ninstr_hangup
#define uinstr_op_0xc3 ninstr_hangup
#define uinstr_op_0xd3 ninstr_hangup
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
#define uinstr_op_0x63 ninstr_hangup
#define uinstr_op_0x73 ninstr_hangup
#define uinstr_op_0xe3 ninstr_hangup
#define uinstr_op_0xf3 ninstr_hangup
#define uinstr_op_0x8f ninstr_hangup
#define uinstr_op_0x9f ninstr_hangup
#define uinstr_op_0xaf ninstr_hangup
#endif      // VCPU_BUSTYPE == VCPU_INTERFACE_CBMSER



/* === Cycle instructions: */
// $AB: (IOp) UINDB
vcputick_t uinstr_op_0xab(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  vcpuregs.x++;
  vcpuregs.y--;
  if (vcpuregs.y != 0xff) {
    vcpuregs.pc += o;
    return rtm20t;
  }
  return rtm15t;
}

// $BB: (IOp) UDEDB
vcputick_t uinstr_op_0xbb(void) {
  int16_t o = (int8_t)readbytepc();           // Read jump offset
  vcpuregs.x--;
  vcpuregs.y--;
  if (vcpuregs.y != 0xff) {
    vcpuregs.pc += o;
    return rtm20t;
  }
  return rtm15t;
}

/* === Delay: */
// $FB: (IOp) UDELY
vcputick_t uinstr_op_0xfb(void) {
  uint8_t v = readbytepc();                   // Read delay value
  return rtm15t+v;
}



/* CPU commands jump table: */
vcputick_t (* const cpucommands[256])(void) = {
  &instr_brk,          // $00: BRK
  &instr_ora_am1,      // $01: ORA ($zp,X)
  &einstr_ldzph,       // $02: (IOp) LDZPH
  &uinstr_op_0x03,     // $03: (IOp) UWDTL
  &ninstr_hangup,      // $04: IOp
  &instr_ora_am2,      // $05: ORA $zp
  &instr_asl_am2,      // $06: ASL $zp
  &uinstr_op_0x07,     // $07: (IOp) PPACK
  &instr_php,          // $08: PHP
  &instr_ora_am3,      // $09: ORA #$gh
  &instr_asl,          // $0A: ASL
  &uinstr_op_0x0b,     // $0B: (IOp) UCLDL
  &ninstr_hangup,      // $0C: IOp
  &instr_ora_am4,      // $0D: ORA $ghjk
  &instr_asl_am4,      // $0E: ASL $ghjk
  &einstr_userr,       // $0F: (IOp) (W/R65C02: BBR0 $zp,offs) USERR (R2)

  &instr_bpl,          // $10: BPL $rtyu
  &instr_ora_am5,      // $11: ORA ($zp),Y
  &ninstr_hangup,      // $12: IOp
  &uinstr_op_0x13,     // $13: (IOp) UWDTH
  &ninstr_hangup,      // $14: IOp
  &instr_ora_am6,      // $15: ORA $zp,X
  &instr_asl_am6,      // $16: ASL $zp,X
  &uinstr_op_0x17,     // $17: (IOp) PPDRD
  &instr_clc,          // $18: CLC
  &instr_ora_am7,      // $19: ORA $ghjk,Y
  &ninstr_hangup,      // $1A: IOp
  &uinstr_op_0x1b,     // $1B: (IOp) UCLDH
  &ninstr_hangup,      // $1C: IOp
  &instr_ora_am8,      // $1D: ORA $ghjk,X
  &instr_asl_am8,      // $1E: ASL $ghjk,X
  &einstr_user1,       // $1F: (IOp) (W/R65C02: BBR1 $zp,offs) USER1 (R2)

  &instr_jsr_am4,      // $20: JSR $ghjk
  &instr_and_am1,      // $21: AND ($zp,X)
  &einstr_ldsph,       // $22: (IOp) LDSPH
  &uinstr_op_0x23,     // $23: (IOp) UWCKL
  &instr_bit_am2,      // $24: BIT $zp
  &instr_and_am2,      // $25: AND $zp
  &instr_rol_am2,      // $26: ROL $zp
  &uinstr_op_0x27,     // $27: (IOp) PPDWR
  &instr_plp,          // $28: PLP
  &instr_and_am3,      // $29: AND #$gh
  &instr_rol,          // $2A: ROL
  &uinstr_op_0x2b,     // $2B: (IOp) UCHDL
  &instr_bit_am4,      // $2C: BIT $ghjk
  &instr_and_am4,      // $2D: AND $ghjk
  &instr_rol_am4,      // $2E: ROL $ghjk
  &einstr_user2,       // $2F: (IOp) (W/R65C02: BBR2 $zp,offs) USER2 (R2)

  &instr_bmi,          // $30: BMI $rtyu
  &instr_and_am5,      // $31: AND ($zp),Y
  &ninstr_hangup,      // $32: IOp
  &uinstr_op_0x33,     // $33: (IOp) UWCKH
  &ninstr_hangup,      // $34: IOp
  &instr_and_am6,      // $35: AND $zp,X
  &instr_rol_am6,      // $36: ROL $zp,X
  &uinstr_op_0x37,     // $37: (IOp) PPWAI
  &instr_sec,          // $38: SEC
  &instr_and_am7,      // $39: AND $ghjk,Y
  &ninstr_hangup,      // $3A: IOp
  &uinstr_op_0x3b,     // $3B: (IOp) UCHDH
  &ninstr_hangup,      // $3C: IOp
  &instr_and_am8,      // $3D: AND $ghjk,X
  &instr_rol_am8,      // $3E: ROL $ghjk,X
  &ninstr_hangup,      // $3F: IOp

  &instr_rti,          // $40: RTI
  &instr_eor_am1,      // $41: EOR ($zp,X)
  &ninstr_hangup,      // $42: IOp
  &uinstr_op_0x43,     // $43: (IOp) UWATL
  &einstr_tyzph,       // $44: (IOp) TYZPH
  &instr_eor_am2,      // $45: EOR $zp
  &instr_lsr_am2,      // $46: LSR $zp
  &uinstr_op_0x47,     // $47: (IOp) PPWDW
  &instr_pha,          // $48: PHA
  &instr_eor_am3,      // $49: EOR #$gh
  &instr_lsr,          // $4A: LSR
  &uinstr_op_0x4b,     // $4B: (IOp) UATDT
  &instr_jmp_am4,      // $4C: JMP $ghjk
  &instr_eor_am4,      // $4D: EOR $ghjk
  &instr_lsr_am4,      // $4E: LSR $ghjk
  &ninstr_hangup,      // $4F: IOp

  &instr_bvc,          // $50: BVC $rtyu
  &instr_eor_am5,      // $51: EOR ($zp),Y
  &ninstr_hangup,      // $52: IOp
  &uinstr_op_0x53,     // $53: (IOp) UWATH
  &einstr_tzphy,       // $54: (IOp) TZPHY
  &instr_eor_am6,      // $55: EOR $zp,X
  &instr_lsr_am6,      // $56: LSR $zp,X
  &ninstr_hangup,      // $57: IOp
  &ninstr_hangup,      // $58: CLI: NOT implemented!
  &instr_eor_am7,      // $59: EOR $ghjk,Y
  &ninstr_hangup,      // $5A: IOp
  &uinstr_op_0x5b,     // $5B: (IOp) UDTTA
  &ninstr_hangup,      // $5C: IOp
  &instr_eor_am8,      // $5D: EOR $ghjk,X
  &instr_lsr_am8,      // $5E: LSR $ghjk,X
  &ninstr_hangup,      // $5F: IOp

  &instr_rts,          // $60: RTS
  &instr_adc_am1,      // $61: ADC ($zp,X)
  &ninstr_hangup,      // $62: IOp
  &uinstr_op_0x63,     // $63: (IOp) USND1
  &ninstr_hangup,      // $64: IOp
  &instr_adc_am2,      // $65: ADC $zp
  &instr_ror_am2,      // $66: ROR $zp
  &ninstr_hangup,      // $67: IOp
  &instr_pla,          // $68: PLA
  &instr_adc_am3,      // $69: ADC #$gh
  &instr_ror,          // $6A: ROR
  &uinstr_op_0x6b,     // $6B: (IOp) UATCK
  &instr_jmp_am10,     // $6C: JMP ($ghjk)
  &instr_adc_am4,      // $6D: ADC $ghjk
  &instr_ror_am4,      // $6E: ROR $ghjk
  &ninstr_hangup,      // $6F: IOp

  &instr_bvs,          // $70: BVS $rtyu
  &instr_adc_am5,      // $71: ADC ($zp),Y
  &ninstr_hangup,      // $72: IOp
  &uinstr_op_0x73,     // $73: (IOp) URCV1
  &ninstr_hangup,      // $74: IOp
  &instr_adc_am6,      // $75: ADC $zp,X
  &instr_ror_am6,      // $76: ROR $zp,X
  &ninstr_hangup,      // $77: IOp
  &instr_sei,          // $78: SEI
  &instr_adc_am7,      // $79: ADC $ghjk,Y
  &ninstr_hangup,      // $7A: IOp
  &uinstr_op_0x7b,     // $7B: (IOp) UCKTA
  &ninstr_hangup,      // $7C: IOp
  &instr_adc_am8,      // $7D: ADC $ghjk,X
  &instr_ror_am8,      // $7E: ROR $ghjk,X
  &ninstr_hangup,      // $7F: IOp

  &ninstr_hangup,      // $80: IOp
  &instr_sta_am1,      // $81: STA ($zp,X)
  &ninstr_hangup,      // $82: IOp
  &uinstr_op_0x83,     // $83: (IOp) USDTL
  &instr_sty_am2,      // $84: STY $zp
  &instr_sta_am2,      // $85: STA $zp
  &instr_stx_am2,      // $86: STX $zp
  &ninstr_hangup,      // $87: IOp
  &instr_dey,          // $88: DEY
  &ninstr_hangup,      // $89: IOp
  &instr_txa,          // $8A: TXA
  &uinstr_op_0x8b,     // $8B: (IOp) UATCD
  &instr_sty_am4,      // $8C: STY $ghjk
  &instr_sta_am4,      // $8D: STA $ghjk
  &instr_stx_am4,      // $8E: STX $ghjk
  &uinstr_op_0x8f,     // $8F: (IOp) (W/R65C02: BBS0 $zp,offs) FSTXB

  &instr_bcc,          // $90: BCC $rtyu
  &instr_sta_am5,      // $91: STA ($zp),Y
  &ninstr_hangup,      // $92: IOp
  &uinstr_op_0x93,     // $93: (IOp) USDTH
  &instr_sty_am6,      // $94: STY $zp,X
  &instr_sta_am6,      // $95: STA $zp,X
  &instr_stx_am9,      // $96: STX $zp,Y
  &ninstr_hangup,      // $97: IOp
  &instr_tya,          // $98: TYA
  &instr_sta_am7,      // $99: STA $ghjk,Y
  &instr_txs,          // $9A: TXS
  &uinstr_op_0x9b,     // $9B: (IOp) UCDTA
  &ninstr_hangup,      // $9C: IOp
  &instr_sta_am8,      // $9D: STA $ghjk,X
  &ninstr_hangup,      // $9E: IOp
  &uinstr_op_0x9f,     // $9F: (IOp) (W/R65C02: BBS1 $zp,offs) FSRXB

  &instr_ldy_am3,      // $A0: LDY #$gh
  &instr_lda_am1,      // $A1: LDA ($zp,X)
  &instr_ldx_am3,      // $A2: LDX #$gh
  &uinstr_op_0xa3,     // $A3: (IOp) USCKL
  &instr_ldy_am2,      // $A4: LDY $zp
  &instr_lda_am2,      // $A5: LDA $zp
  &instr_ldx_am2,      // $A6: LDX $zp
  &ninstr_hangup,      // $A7: IOp
  &instr_tay,          // $A8: TAY
  &instr_lda_am3,      // $A9: LDA #$gh
  &instr_tax,          // $AA: TAX
  &uinstr_op_0xab,     // $AB: (IOp) UINDB
  &instr_ldy_am4,      // $AC: LDY $ghjk
  &instr_lda_am4,      // $AD: LDA $ghjk
  &instr_ldx_am4,      // $AE: LDX $ghjk
  &uinstr_op_0xaf,     // $AF: (IOp) (W/R65C02: BBS2 $zp,offs) FSRXC

  &instr_bcs,          // $B0: BCS $rtyu
  &instr_lda_am5,      // $B1: LDA ($zp),Y
  &ninstr_hangup,      // $B2: IOp
  &uinstr_op_0xb3,     // $B3: (IOp) USCKH
  &instr_ldy_am6,      // $B4: LDY $zp,X
  &instr_lda_am6,      // $B5: LDA $zp,X
  &instr_ldx_am9,      // $B6: LDX $zp,Y
  &ninstr_hangup,      // $B7: IOp
  &instr_clv,          // $B8: CLV
  &instr_lda_am7,      // $B9: LDA $ghjk,Y
  &instr_tsx,          // $BA: TSX
  &uinstr_op_0xbb,     // $BB: (IOp) UDEDB
  &instr_ldy_am8,      // $BC: LDY $ghjk,X
  &instr_lda_am8,      // $BD: LDA $ghjk,X
  &instr_ldx_am7,      // $BE: LDX $ghjk,Y
  &ninstr_hangup,      // $BF: IOp

  &instr_cpy_am3,      // $C0: CPY #$gh
  &instr_cmp_am1,      // $C1: CMP ($zp,X)
  &ninstr_hangup,      // $C2: IOp
  &uinstr_op_0xc3,     // $C3: (IOp) USATL
  &instr_cpy_am2,      // $C4: CPY $zp
  &instr_cmp_am2,      // $C5: CMP $zp
  &instr_dec_am2,      // $C6: DEC $zp
  &ninstr_hangup,      // $C7: IOp
  &instr_iny,          // $C8: INY
  &instr_cmp_am3,      // $C9: CMP #$gh
  &instr_dex,          // $CA: DEX
  &einstr_confg,       // $CB: (IOp) (65C02: WAI) CONFG (VCPU configuration, 2+ BYTEs OpCode) (R2)
  &instr_cpy_am4,      // $CC: CPY $ghjk
  &instr_cmp_am4,      // $CD: CMP $ghjk
  &instr_dec_am4,      // $CE: DEC $ghjk
  &ninstr_hangup,      // $CF: IOp

  &instr_bne,          // $D0: BNE $rtyu
  &instr_cmp_am5,      // $D1: CMP ($zp),Y
  &ninstr_hangup,      // $D2: IOp
  &uinstr_op_0xd3,     // $D3: (IOp) USATH
  &einstr_tysph,       // $D4: (IOp) TYSPH
  &instr_cmp_am6,      // $D5: CMP $zp,X
  &instr_dec_am6,      // $D6: DEC $zp,X
  &ninstr_hangup,      // $D7: IOp
  &instr_cld,          // $D8: CLD
  &instr_cmp_am7,      // $D9: CMP $ghjk,Y
  &ninstr_hangup,      // $DA: IOp
  &uinstr_op_0xdb,     // $DB: (IOp) (65C02: STP) ULBIT
  &ninstr_hangup,      // $DC: IOp
  &instr_cmp_am8,      // $DD: CMP $ghjk,X
  &instr_dec_am8,      // $DE: DEC $ghjk,X
  &ninstr_hangup,      // $DF: IOp

  &instr_cpx_am3,      // $E0: CPX #$gh
  &instr_sbc_am1,      // $E1: SBC ($zp,X)
  &ninstr_hangup,      // $E2: (IOp) (deprecated BTASC)
  &uinstr_op_0xe3,     // $E3: (IOp) USND2
  &instr_cpx_am2,      // $E4: CPX $zp
  &instr_sbc_am2,      // $E5: SBC $zp
  &instr_inc_am2,      // $E6: INC $zp
  &ninstr_hangup,      // $E7: IOp
  &instr_inx,          // $E8: INX
  &instr_sbc_am3,      // $E9: SBC #$gh
  &instr_nop,          // $EA: NOP
  &ninstr_hangup,      // $EB: (IOp) (deprecated UTEST)
  &instr_cpx_am4,      // $EC: CPX $ghjk
  &instr_sbc_am4,      // $ED: SBC $ghjk
  &instr_inc_am4,      // $EE: INC $ghjk
  &ninstr_hangup,      // $EF: IOp

  &instr_beq,          // $F0: BEQ $rtyu
  &instr_sbc_am5,      // $F1: SBC ($zp),Y
  &ninstr_hangup,      // $F2: IOp
  &uinstr_op_0xf3,     // $F3: (IOp) URCV2
  &einstr_tsphy,       // $F4: (IOp) TSPHY
  &instr_sbc_am6,      // $F5: SBC $zp,X
  &instr_inc_am6,      // $F6: INC $zp,X
  &ninstr_hangup,      // $F7: IOp
  &ninstr_hangup,      // $F8: SED: NOT implemented!
  &instr_sbc_am7,      // $F9: SBC $ghjk,Y
  &ninstr_hangup,      // $FA: IOp
  &uinstr_op_0xfb,     // $FB: (IOp) UDELY
  &ninstr_hangup,      // $FC: IOp
  &instr_sbc_am8,      // $FD: SBC $ghjk,X
  &instr_inc_am8,      // $FE: INC $ghjk,X
  &ninstr_hangup       // $FF: IOp
};



/* 6502 emulation entry: */
void vcpu6502core(void) {
  uint8_t cr;
  vcputick_t rt;      // Command runtime
  vcputick_t tm;      // Actual time
  vcputick_t (*funcptr)(void);
  if (vcpuregs.sph >= memoryblockno) vcpuregs.sph = 0x00;
  if (vcpuregs.zph >= memoryblockno) vcpuregs.zph = 0x00;
  if (vcpuregs.rr >= memorysize) vcpuregs.rr = 0x0000;
  if (vcpuregs.u1r >= memorysize) vcpuregs.u1r = 0x0000;
  if (vcpuregs.u2r >= memorysize) vcpuregs.u2r = 0x0000;
#ifdef HAVE_PARALLEL
  hskoutline = 1;
#endif
  timedelay = 0;
  vcputimer_clear();
  rt = 1;
  while (vcpurun) {
    vcpuregs.lastopcode = readbytepc();
    funcptr = cpucommands[vcpuregs.lastopcode];
    while (vcputimer_getvalue() <= rt) {}
    vcputimer_clear();
    rt = funcptr();
    if (rt) {
      tm = vcputimer_getvalue();
      if (tm > rt) {
        if (timedelay == 0) {
          if (vcpurun) vcpuregs.interrupt |= VCPU_ERROR_ALIGN;
          set_vcpurunflag(0);
        }
        timedelay = 0;
      }
    }
  }
  vcpuregs.interrupt |= interruptcode;
  vcpuregs.sr |= (f_x|f_b);     // not used + Break bit set
  vcpuregs.sr &= ~(f_d);        // Decimal bit clear
  cr = VCPU_ERROR_ADDRESS;
  if (vcpuregs.interrupt & VCPU_FUNCTIONCALL) cr = VCPU_ERROR_ILLEGALFC;
  if (vcpuregs.pc >= memorysize) vcpuregs.interrupt |= cr;
}
