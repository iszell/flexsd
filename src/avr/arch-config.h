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


   arch-config.h: The main architecture-specific config header

*/

#ifndef ARCH_CONFIG_H
#define ARCH_CONFIG_H

#include <avr/io.h>
#include <avr/interrupt.h>
/* Include avrcompat.h to get the PA0..PD7 macros on 1284P */
#include "avrcompat.h"
#include "flags.h"

/* ----- Common definitions for all AVR hardware variants ------ */

/* Return value of buttons_read() */
typedef uint8_t rawbutton_t;

/* Interrupt handler for system tick */
#define SYSTEM_TICK_HANDLER ISR(TIMER1_COMPA_vect, ISR_NOBLOCK)

/* EEPROMFS: offset and size must be multiples of 4 */
/* to actually enable it, CONFIG_HAVE_EEPROMFS must be set in config */
#  define EEPROMFS_OFFSET     512
#  define EEPROMFS_SIZE       3584
#  define EEPROMFS_ENTRIES    8
#  define EEPROMFS_SECTORSIZE 64


#if CONFIG_HARDWARE_VARIANT == 1
/* ---------- Hardware configuration: Example ---------- */
/* This is a commented example for most of the available options    */
/* in case someone wants to build Yet Another[tm] hardware variant. */
/* Some of the values are chosen randomly, so this variant is not   */
/* expected to compile successfully.                                */

/*** SD card support ***/
/* If your device supports SD cards by default, define this symbol. */
#  define HAVE_SD

/* Declaration of the interrupt handler for SD card change */
#  define SD_CHANGE_HANDLER ISR(INT0_vect)

/* Declaration of the interrupt handler for SD card 2 change */
#  define SD2_CHANGE_HANDLER ISR(INT9_vect)

/* Initialize all pins and interrupts related to SD - except SPI */
static inline void sdcard_interface_init(void) {
  /* card detect (SD1) */
  DDRD  &= ~_BV(PD2);
  PORTD |=  _BV(PD2);
  /* write protect (SD1) */
  DDRD &= ~_BV(PD6);
  PORTD |= _BV(PD6);
  /* card change interrupt (SD1) */
  EICRA |= _BV(ISC00);
  EIMSK |= _BV(INT0);
  // Note: Wrapping SD2 in CONFIG_TWINSD may be a good idea
  /* chip select (SD2) */
  PORTD |= _BV(PD4);
  DDRD |= _BV(PD4);
  /* card detect (SD2) */
  DDRD &= ~_BV(PD3);
  PORTD |= _BV(PD3);
  /* write protect (SD2) */
  DDRD &= ~_BV(PD7);
  PORTD |= _BV(PD7);
  /* card change interrupt (SD2) */
  EICRA |=  _BV(ISC90); // Change interrupt
  EIMSK |=  _BV(INT9);  // Change interrupt
}

/* sdcard_detect() must return non-zero while a card is inserted */
/* This must be a pin capable of generating interrupts.          */
static inline uint8_t sdcard_detect(void) {
  return !(PIND & _BV(PD2));
}

/* Returns non-zero when the currently inserted card is write-protected */
static inline uint8_t sdcard_wp(void) {
  return PIND & _BV(PD6);
}

/* Support for a second SD card - use CONFIG_TWINSD=y in your config file to enable! */
/* Same as the two functions above, but for card 2 */
static inline uint8_t sdcard2_detect(void) {
  return !(PIND & _BV(PD3));
}
static inline uint8_t sdcard2_wp(void) {
  return PIND & _BV(PD7);
}

/* SD card 1 is assumed to use the standard SS pin   */
/* If that's not true, #define SDCARD_SS_SPECIAL and */
/* implement this function:                          */
//static inline __attribute__((always_inline)) void sdcard_set_ss(uint8_t state) {
//  if (state)
//    PORTZ |= _BV(PZ9);
//  else
//    PORTZ &= ~_BV(PZ9);
//}

/* SD card 2 CS pin */
static inline __attribute__((always_inline)) void sdcard2_set_ss(uint8_t state) {
  if (state)
    PORTD |= _BV(PD4);
  else
    PORTD &= ~_BV(PD4);
}

/* SD Card supply voltage - choose the one appropiate to your board */
/* #  define SD_SUPPLY_VOLTAGE (1L<<15)  / * 2.7V - 2.8V */
/* #  define SD_SUPPLY_VOLTAGE (1L<<16)  / * 2.8V - 2.9V */
/* #  define SD_SUPPLY_VOLTAGE (1L<<17)  / * 2.9V - 3.0V */
#  define SD_SUPPLY_VOLTAGE (1L<<18)  /* 3.0V - 3.1V */
/* #  define SD_SUPPLY_VOLTAGE (1L<<19)  / * 3.1V - 3.2V */
/* #  define SD_SUPPLY_VOLTAGE (1L<<20)  / * 3.2V - 3.3V */
/* #  define SD_SUPPLY_VOLTAGE (1L<<21)  / * 3.3V - 3.4V */
/* #  define SD_SUPPLY_VOLTAGE (1L<<22)  / * 3.4V - 3.5V */
/* #  define SD_SUPPLY_VOLTAGE (1L<<23)  / * 3.5V - 3.6V */

/* SPI clock divisors - the slow one must be 400KHz or slower,        */
/* the fast one can be as high as you thing your hardware will handle */
#  define SPI_DIVISOR_SLOW 32
#  define SPI_DIVISOR_FAST 4


/*** Device address selection ***/
/* Device address jumpers/switch port / input configuration */
#  define DEVICE_HW_ADDR_PORT PORTD
#  define DEVICE_HW_ADDR_DDR DDRD
#  define DEVICE_HW_ADDR_PIN PIND
/* Device address jumpers/switch pins (no. of bits) configuration */
#  define DEVICE_HW_ADDR_B0 PD7
#  define DEVICE_HW_ADDR_B1 PD5
/* If the hardware doesn't have device address jumpers/switches, define default unit no: */
/* #  define DEVICE_DEFAULT_UNITNO 8 */


/*** LEDs ***/
/* Please don't build single-LED hardware anymore... */

/*** LED definitions: ***/
#  define LED_BUSY_PORT PORTC
#  define LED_BUSY_DDR DDRC
#  define LED_BUSY_PIN PC0
#  define LED_DIRTY_PORT PORTC
#  define LED_DIRTY_INPUT PINC
#  define LED_DIRTY_DDR DDRC
#  define LED_DIRTY_PIN PC1
#  define LED_ACTIVE_LEVEL 1

/*** IEC signals ***/
#  define IEC_INPUT PINA
#  define IEC_DDR   DDRA
#  define IEC_PORT  PORTA

/* Pins assigned for the IEC lines */
#  define IEC_PIN_ATN   PA0
#  define IEC_PIN_DATA  PA1
#  define IEC_PIN_CLOCK PA2
#  define IEC_PIN_SRQ   PA3

/* Use separate input/output lines?                                    */
/* The code assumes that the input is NOT inverted, but the output is. */
//#  define IEC_SEPARATE_OUT
//#  define IEC_OPIN_ATN   PA4
//#  define IEC_OPIN_DATA  PA5
//#  define IEC_OPIN_CLOCK PA6
//#  define IEC_OPIN_SRQ   PA7

/* You can use different ports for input and output bits. The code tries */
/* to not stomp on the unused bits. IEC output is on IEC_PORT.           */
/* Not well-tested yet.                                                  */
//#  define IEC_DDRIN      DDRX
//#  define IEC_DDROUT     DDRY
//#  define IEC_PORTIN     PORTX

/* ATN interrupt (required) */
#  define IEC_ATN_INT_VECT    PCINT0_vect
#  define IEC_PCIFR_BIT         PCIF0

/* Fast serial support:
   If SRQ interrupt is PCI and the vector shared with ATN vector, define this: */
#  define IEC_SRQATN_VECT_SHARED
/* SRQ interrupt vector, if not shared with ATN vector: */
//#  define IEC_SRQ_INT_VECT      PCINTx_vect

static inline void iec_interrupts_init(void) {
  PCMSK0 = _BV(PCINT0);
  PCIFR |= _BV(IEC_PCIFR_BIT);
}

/* CLK interrupt (not required) */
/* Dreamload requires interrupts for both the ATN and CLK lines. If both are served by */
/* the same PCINT vector, define that as ATN interrupt above and define IEC_PCMSK.     */
//#  define IEC_PCMSK             PCMSK0
/* If the CLK line has its own dedicated interrupt, use the following definitions: */
//#  define IEC_CLK_INT           INT5
//#  define IEC_CLK_INT_VECT      INT5_vect
//static inline void iec_clock_int_setup(void) {
//  EICRB |= _BV(ISC50);
//}


/*** IEEE signals ***/
/* not documented yet, look at petSD/XS-1541 for guidance */

/*** User interface ***/

/*** Button definitions ***/
#  define BUTTONS_PORT PORTC
#  define BUTTONS_INPUT PINC
#  define BUTTONS_DDR DDRC
#  define BUTTONS_NEXT_PIN PC4
#  define BUTTONS_PREV_PIN PC3

/* Software I2C lines for the RTC and display */
#  define SOFTI2C_PORT    PORTC
#  define SOFTI2C_PIN     PINC
#  define SOFTI2C_DDR     DDRC
#  define SOFTI2C_BIT_SCL PC4
#  define SOFTI2C_BIT_SDA PC5
#  define SOFTI2C_DELAY   6


/*** board-specific initialisation ***/
/* Used on uIEC/CF, uIEC/SD, FlexSD drive */
//#define HAVE_BOARD_INIT
//static inline void board_init(void) {
//  // turn on power LED
//  DDRG  |= _BV(PG1);
//  PORTG |= _BV(PG1);
//}

/* VCPU "run flag" definition */
/* If this bit is 1: VCPU is running */
/* Requirements: only one bit in any I/O register in 0x00..0x1F range */
/*   for SBI/CBI/SBIS/SBIC usage */
/*   If set this definition to any unused port pin and connect LED to this, */
/*   this LED switch On/Off for VCPU usage */
/* If not defined these, one bit of "globalflags" will be used for this. */
#  ifdef CONFIG_VCPULED
#    define VCPURUNFLAG_REG DDRB
#    define VCPURUNFLAG_BIT PB0
/* If VCPU "run flag" is assigned to any port pin, set PORT name, */
/*   the LED init routine set the correct level of this pin */
/* If "run flag" is *NOT* use port pin, comment out this definition */
#    define VCPURUNFLAG_PORT PORTB
#  endif



/* Pre-configurated hardware variants */

#elif CONFIG_HARDWARE_VARIANT == 2
/* ---------- Hardware configuration: Shadowolf 1 ---------- */
#  define HAVE_SD
#  define SD_CHANGE_HANDLER     ISR(INT0_vect)
#  define SD_SUPPLY_VOLTAGE     (1L<<18)

/* 250kHz slow, 2MHz fast */
#  define SPI_DIVISOR_SLOW 32
#  define SPI_DIVISOR_FAST 4

static inline void sdcard_interface_init(void) {
  DDRD &= ~_BV(PD2);
  PORTD |= _BV(PD2);
  DDRD &= ~_BV(PD6);
  PORTD |= _BV(PD6);
  EICRA |= _BV(ISC00);
  EIMSK |= _BV(INT0);
}

static inline uint8_t sdcard_detect(void) {
  return !(PIND & _BV(PD2));
}

static inline uint8_t sdcard_wp(void) {
  return PIND & _BV(PD6);
}

#  define DEVICE_HW_ADDR_PORT PORTD
#  define DEVICE_HW_ADDR_DDR DDRD
#  define DEVICE_HW_ADDR_PIN PIND
#  define DEVICE_HW_ADDR_B0 PD7
#  define DEVICE_HW_ADDR_B1 PD5

#  define LED_BUSY_PORT PORTC
#  define LED_BUSY_DDR DDRC
#  define LED_BUSY_PIN PC0
#  define LED_DIRTY_PORT PORTC
#  define LED_DIRTY_INPUT PINC
#  define LED_DIRTY_DDR DDRC
#  define LED_DIRTY_PIN PC1
#  define LED_ACTIVE_LEVEL 1

#  define IEC_INPUT             PINA
#  define IEC_DDR               DDRA
#  define IEC_PORT              PORTA
#  define IEC_PIN_ATN           PA0
#  define IEC_PIN_DATA          PA1
#  define IEC_PIN_CLOCK         PA2
#  define IEC_PIN_SRQ           PA3
#  define IEC_ATN_INT_VECT      PCINT0_vect
#  define IEC_PCMSK             PCMSK0
#  define IEC_PCIFR_BIT         PCIF0
#  define IEC_SRQATN_VECT_SHARED

static inline void iec_interrupts_init(void) {
  PCICR |= _BV(PCIE0);
  PCIFR |= _BV(IEC_PCIFR_BIT);
}

#  define BUTTONS_PORT PORTC
#  define BUTTONS_INPUT PINC
#  define BUTTONS_DDR DDRC
#  define BUTTONS_NEXT_PIN PC4
#  define BUTTONS_PREV_PIN PC3

#  ifdef CONFIG_VCPULED
#    define VCPURUNFLAG_REG DDRB
#    define VCPURUNFLAG_BIT PB0
#    define VCPURUNFLAG_PORT PORTB
#  endif


#elif CONFIG_HARDWARE_VARIANT == 3
/* ---------- Hardware configuration: LarsP ---------- */
#  define HAVE_SD
#  define SD_CHANGE_HANDLER     ISR(INT0_vect)
#  define SD_SUPPLY_VOLTAGE     (1L<<21)

#  define SPI_DIVISOR_SLOW 32
#  define SPI_DIVISOR_FAST 4

static inline void sdcard_interface_init(void) {
  DDRD  &= ~_BV(PD2);
  PORTD |=  _BV(PD2);
  DDRD  &= ~_BV(PD6);
  PORTD |=  _BV(PD6);
  EICRA |=  _BV(ISC00);
  EIMSK |=  _BV(INT0);
}

static inline uint8_t sdcard_detect(void) {
  return !(PIND & _BV(PD2));
}

static inline uint8_t sdcard_wp(void) {
  return PIND & _BV(PD6);
}

#  define DEVICE_HW_ADDR_PORT PORTA
#  define DEVICE_HW_ADDR_DDR DDRA
#  define DEVICE_HW_ADDR_PIN PINA
#  define DEVICE_HW_ADDR_B0 PA2
#  define DEVICE_HW_ADDR_B1 PA3

#  define LED_BUSY_PORT PORTA
#  define LED_BUSY_DDR DDRA
#  define LED_BUSY_PIN PA0
#  define LED_DIRTY_PORT PORTA
#  define LED_DIRTY_INPUT PINA
#  define LED_DIRTY_DDR DDRA
#  define LED_DIRTY_PIN PA1
#  define LED_ACTIVE_LEVEL 0

#  define IEC_INPUT             PINC
#  define IEC_DDR               DDRC
#  define IEC_PORT              PORTC
#  define IEC_PIN_ATN           PC0
#  define IEC_PIN_DATA          PC1
#  define IEC_PIN_CLOCK         PC2
#  define IEC_PIN_SRQ           PC3
#  define IEC_ATN_INT_VECT      PCINT2_vect
#  define IEC_PCMSK             PCMSK2
#  define IEC_PCIFR_BIT         PCIF2
#  define IEC_SRQATN_VECT_SHARED

static inline void iec_interrupts_init(void) {
  PCICR |= _BV(PCIE2);
  PCIFR |= _BV(IEC_PCIFR_BIT);
}

#  define BUTTONS_PORT PORTA
#  define BUTTONS_INPUT PINA
#  define BUTTONS_DDR DDRA
#  define BUTTONS_NEXT_PIN PA4
#  define BUTTONS_PREV_PIN PA5

#  define SOFTI2C_PORT          PORTC
#  define SOFTI2C_PIN           PINC
#  define SOFTI2C_DDR           DDRC
#  define SOFTI2C_BIT_SCL       PC6
#  define SOFTI2C_BIT_SDA       PC5
#  define SOFTI2C_BIT_INTRQ     PC7
#  define SOFTI2C_DELAY         6

#  ifdef CONFIG_VCPULED
#    define VCPURUNFLAG_REG DDRB
#    define VCPURUNFLAG_BIT PB0
#    define VCPURUNFLAG_PORT PORTB
#  endif


#elif CONFIG_HARDWARE_VARIANT == 4
/* ---------- Hardware configuration: uIEC ---------- */
/* Note: This CONFIG_HARDWARE_VARIANT number is tested in system.c */
/*       240329: CONFIG_HARDWARE_VARIANT only found in xxx/arch-config.h */
#  define HAVE_ATA
#  ifndef CONFIG_NO_SD
#    define HAVE_SD
#  endif
#  define SPI_LATE_INIT
#  define CF_CHANGE_HANDLER     ISR(INT7_vect)
#  define SD_CHANGE_HANDLER     ISR(PCINT0_vect)
#  define SD_SUPPLY_VOLTAGE     (1L<<21)

/* 250kHz slow, 2MHz fast */
#  define SPI_DIVISOR_SLOW 32
#  define SPI_DIVISOR_FAST 4

#  define SINGLE_LED

static inline void cfcard_interface_init(void) {
  DDRE  &= ~_BV(PE7);
  PORTE |=  _BV(PE7);
  EICRB |=  _BV(ISC70);
  EIMSK |=  _BV(INT7);
}

static inline uint8_t cfcard_detect(void) {
  return !(PINE & _BV(PE7));
}

static inline void sdcard_interface_init(void) {
  DDRB   &= ~_BV(PB7);
  PORTB  |=  _BV(PB7);
  DDRB   &= ~_BV(PB6);
  PORTB  |=  _BV(PB6);
  PCMSK0 |=  _BV(PCINT7);
  PCICR  |=  _BV(PCIE0);
  PCIFR  |=  _BV(PCIF0);
}

static inline uint8_t sdcard_detect(void) {
  return !(PINB & _BV(PB7));
}

static inline uint8_t sdcard_wp(void) {
  return PINB & _BV(PB6);
}

/* No device jumpers on uIEC */
#  define DEVICE_DEFAULT_UNITNO 10

#  define LED_BUSY_PORT PORTE
#  define LED_BUSY_INPUT PINE
#  define LED_BUSY_DDR DDRE
#  define LED_BUSY_PIN PE3
#  define LED_ACTIVE_LEVEL 1

#  define IEC_INPUT             PINE
#  define IEC_DDR               DDRE
#  define IEC_PORT              PORTE
#  define IEC_PIN_ATN           PE6
#  define IEC_PIN_DATA          PE4
#  define IEC_PIN_CLOCK         PE5
#  define IEC_PIN_SRQ           PE2
#  define IEC_ATN_INT           INT6
#  define IEC_ATN_INT_VECT      INT6_vect
#  define IEC_CLK_INT           INT5
#  define IEC_CLK_INT_VECT      INT5_vect
/* On uIEC hardware, the SRQ line connected to PE2, but no interrupt
 * can be attached on this pin. (Maybe the analog comparator can do
 * any trick..? It's a joke.)
 * Therefore, no IEC_SRQ_INT_VECT defined. */
//#  define IEC_SRQ_INT_VECT      PCINTx_vect

static inline void iec_interrupts_init(void) {
  EICRB |= _BV(ISC60);
  EICRB |= _BV(ISC50);
}

#  define BUTTONS_PORT PORTG
#  define BUTTONS_INPUT PING
#  define BUTTONS_DDR DDRG
#  define BUTTONS_NEXT_PIN PG4
#  define BUTTONS_PREV_PIN PG3

#  define SOFTI2C_PORT          PORTD
#  define SOFTI2C_PIN           PIND
#  define SOFTI2C_DDR           DDRD
#  define SOFTI2C_BIT_SCL       PD0
#  define SOFTI2C_BIT_SDA       PD1
#  define SOFTI2C_BIT_INTRQ     PD2
#  define SOFTI2C_DELAY         6

/* parallel cable - conflicts with the SOFTI2C pins above! */
#  ifdef CONFIG_NO_SD
#    define HAVE_PARALLEL
#    define PARALLEL_HANDLER      ISR(PCINT0_vect)
#    define PARALLEL_PDDR         DDRD      // CONN2 pins 1,3,...,15
#    define PARALLEL_PPORT        PORTD
#    define PARALLEL_PPIN         PIND
#    define PARALLEL_HDDR         DDRB
#    define PARALLEL_HPORT        PORTB
#    define PARALLEL_HPIN         PINB
#    define PARALLEL_HSK_OUT_BIT  5         // CONN2 pin 14, to C64 FLAG2
#    define PARALLEL_HSK_IN_BIT   4         // CONN2 pin 16, to C64 PC2
#    define PARALLEL_PCMSK        PCMSK0
#    define PARALLEL_PCINT_GROUP  0

#    define PARALLEL_IRQEN_BIT      PARALLEL_PCINT_GROUP
#    define PARALLEL_IRQEN_REG      PCICR
#    define PARALLEL_IRQFLAG_REG    PCIFR

/* Configure parallel port lines and interrupt */
static inline void parallelport_init(void) {
  /* set data lines to input with pullup */
  PARALLEL_PDDR  = 0;
  PARALLEL_PPORT = 0xff;

  /* set HSK_OUT and _IN to input with pullup */
  PARALLEL_HDDR  &= ~(_BV(PARALLEL_HSK_OUT_BIT) |
                      _BV(PARALLEL_HSK_IN_BIT));
  PARALLEL_HPORT |= _BV(PARALLEL_HSK_OUT_BIT) |
                    _BV(PARALLEL_HSK_IN_BIT);

  /* configure interrupt for parallel handshake */
  /* excluse PCINT group */
  PARALLEL_PCMSK |= _BV(PARALLEL_HSK_IN_BIT);
}

#  elif defined(CONFIG_PARALLEL_DOLPHIN)
#    error CONFIG_PARALLEL_DOLPHIN on uIEC requires CONFIG_NO_SD=y !
#  endif

/* Use diskmux code to optionally turn off second IDE drive */
#  define NEED_DISKMUX

#  define HAVE_BOARD_INIT

static inline void board_init(void) {
  /* Force control lines of the external SRAM high */
  DDRG  = _BV(PG0) | _BV(PG1) | _BV(PG2);
  PORTG = _BV(PG0) | _BV(PG1) | _BV(PG2);
}

#  ifdef CONFIG_VCPULED
#    define VCPURUNFLAG_REG PORTA
#    define VCPURUNFLAG_BIT PA0
#  endif


#elif CONFIG_HARDWARE_VARIANT == 5
/* ---------- Hardware configuration: Shadowolf 2 aka sd2iec 1.x ---------- */
#  define HAVE_SD
#  define SD_CHANGE_HANDLER     ISR(INT0_vect)
#  define SD2_CHANGE_HANDLER    ISR(INT2_vect)
#  define SD_SUPPLY_VOLTAGE     (1L<<18)

/* 250kHz slow, 2MHz fast */
#  define SPI_DIVISOR_SLOW 32
#  define SPI_DIVISOR_FAST 4

static inline void sdcard_interface_init(void) {
  DDRD  &= ~_BV(PD2);
  PORTD |=  _BV(PD2);
  DDRD  &= ~_BV(PD6);
  PORTD |=  _BV(PD6);
  EICRA |=  _BV(ISC00);
  EIMSK |=  _BV(INT0);
#ifdef CONFIG_TWINSD
  PORTD |=  _BV(PD3); // CS
  DDRD  |=  _BV(PD3); // CS
  DDRC  &= ~_BV(PC7); // WP
  PORTC |=  _BV(PC7); // WP
  DDRB  &= ~_BV(PB2); // Detect
  PORTB |=  _BV(PB2); // Detect
  EICRA |=  _BV(ISC20); // Change interrupt
  EIMSK |=  _BV(INT2);  // Change interrupt
#endif
}

static inline uint8_t sdcard_detect(void) {
  return !(PIND & _BV(PD2));
}

static inline uint8_t sdcard_wp(void) {
  return PIND & _BV(PD6);
}

static inline uint8_t sdcard2_detect(void) {
  return !(PINB & _BV(PB2));
}

static inline uint8_t sdcard2_wp(void) {
  return PINC & _BV(PC7);
}

static inline __attribute__((always_inline)) void sdcard2_set_ss(uint8_t state) {
  if (state)
    PORTD |= _BV(PD3);
  else
    PORTD &= ~_BV(PD3);
}

#  define DEVICE_HW_ADDR_PORT PORTD
#  define DEVICE_HW_ADDR_DDR DDRD
#  define DEVICE_HW_ADDR_PIN PIND
#  define DEVICE_HW_ADDR_B0 PD7
#  define DEVICE_HW_ADDR_B1 PD5

#  define LED_BUSY_PORT PORTC
#  define LED_BUSY_DDR DDRC
#  define LED_BUSY_PIN PC0
#  define LED_DIRTY_PORT PORTC
#  define LED_DIRTY_INPUT PINC
#  define LED_DIRTY_DDR DDRC
#  define LED_DIRTY_PIN PC1
#  define LED_ACTIVE_LEVEL 1

#  define IEC_INPUT             PINA
#  define IEC_DDR               DDRA
#  define IEC_PORT              PORTA
#  define IEC_PIN_ATN           PA0
#  define IEC_PIN_DATA          PA1
#  define IEC_PIN_CLOCK         PA2
#  define IEC_PIN_SRQ           PA3
#  define IEC_SEPARATE_OUT
#  define IEC_OPIN_ATN          PA4
#  define IEC_OPIN_DATA         PA5
#  define IEC_OPIN_CLOCK        PA6
#  define IEC_OPIN_SRQ          PA7
#  define IEC_ATN_INT_VECT      PCINT0_vect
#  define IEC_PCMSK             PCMSK0
#  define IEC_PCIFR_BIT         PCIF0
#  define IEC_SRQATN_VECT_SHARED

static inline void iec_interrupts_init(void) {
  PCICR |= _BV(PCIE0);
  PCIFR |= _BV(IEC_PCIFR_BIT);
}

#  define BUTTONS_PORT PORTC
#  define BUTTONS_INPUT PINC
#  define BUTTONS_DDR DDRC
#  define BUTTONS_NEXT_PIN PC3
#  define BUTTONS_PREV_PIN PC2

#  define SOFTI2C_PORT          PORTC
#  define SOFTI2C_PIN           PINC
#  define SOFTI2C_DDR           DDRC
#  define SOFTI2C_BIT_SCL       PC4
#  define SOFTI2C_BIT_SDA       PC5
#  define SOFTI2C_BIT_INTRQ     PC6
#  define SOFTI2C_DELAY         6

#  ifdef CONFIG_VCPULED
#    define VCPURUNFLAG_REG DDRB
#    define VCPURUNFLAG_BIT PB0
#    define VCPURUNFLAG_PORT PORTB
#  endif


/* Hardware configuration 6 was old NKC MMC2IEC */


#elif CONFIG_HARDWARE_VARIANT == 7
/* ---------- Hardware configuration: uIEC v3 ---------- */
#  define HAVE_SD
#  define SD_CHANGE_HANDLER     ISR(INT6_vect)
#  define SD_SUPPLY_VOLTAGE     (1L<<21)

/* 250kHz slow, 2MHz fast */
#  define SPI_DIVISOR_SLOW 32
#  define SPI_DIVISOR_FAST 4

#  define SINGLE_LED

static inline void sdcard_interface_init(void) {
  DDRE  &= ~_BV(PE6);
  PORTE |=  _BV(PE6);
  DDRE  &= ~_BV(PE2);
  PORTE |=  _BV(PE2);
  EICRB |=  _BV(ISC60);
  EIMSK |=  _BV(INT6);
}

static inline uint8_t sdcard_detect(void) {
  return !(PINE & _BV(PE6));
}

static inline uint8_t sdcard_wp(void) {
  return PINE & _BV(PE2);
}

/* No device jumpers on uIEC v3 */
#  define DEVICE_DEFAULT_UNITNO 10

#  define LED_BUSY_PORT PORTG
#  define LED_BUSY_INPUT PING
#  define LED_BUSY_DDR DDRG
#  define LED_BUSY_PIN PG0
#  define LED_ACTIVE_LEVEL 1

#  define IEC_INPUT             PINB
#  define IEC_DDRIN             DDRB
#  define IEC_PORTIN            PORTB
#  define IEC_PIN_ATN           PB4
#  define IEC_PIN_DATA          PB5
#  define IEC_PIN_CLOCK         PB6
#  define IEC_PIN_SRQ           PB7
#  define IEC_SEPARATE_OUT
#  define IEC_PORT              PORTD
#  define IEC_DDROUT            DDRD
#  define IEC_OPIN_ATN          PD4
#  define IEC_OPIN_DATA         PD5
#  define IEC_OPIN_CLOCK        PD6
#  define IEC_OPIN_SRQ          PD7
#  define IEC_ATN_INT_VECT      PCINT0_vect
#  define IEC_PCMSK             PCMSK0
#  define IEC_PCIFR_BIT         PCIF0
#  define IEC_SRQATN_VECT_SHARED

static inline void iec_interrupts_init(void) {
  PCICR |= _BV(PCIE0);
  PCIFR |= _BV(IEC_PCIFR_BIT);
}

#  define BUTTONS_PORT PORTG
#  define BUTTONS_INPUT PING
#  define BUTTONS_DDR DDRG
#  define BUTTONS_NEXT_PIN PG4
#  define BUTTONS_PREV_PIN PG3

#  define HAVE_BOARD_INIT

static inline void board_init(void) {
  // turn on power LED
  DDRG  |= _BV(PG1);
  PORTG |= _BV(PG1);
}

#  ifdef CONFIG_VCPULED
#    define VCPURUNFLAG_REG PORTA
#    define VCPURUNFLAG_BIT PA0
#  endif


#elif CONFIG_HARDWARE_VARIANT == 8
/* ---------- Hardware configuration: petSD ---------- */
#  define HAVE_SD
#  define SD_CHANGE_HANDLER     ISR(PCINT3_vect)
#  define SD_SUPPLY_VOLTAGE (1L<<21)

/* 288 kHz slow, 2.304 MHz fast */
#  define SPI_DIVISOR_SLOW 64
#  define SPI_DIVISOR_FAST 8

static inline void sdcard_interface_init(void) {
  DDRD   &= ~_BV(PD4);            /* card detect */
  PORTD  |=  _BV(PD4);
  DDRC   &= ~_BV(PC3);            /* write protect  */
  PORTC  |=  _BV(PC3);
  PCMSK3 |=  _BV(PCINT28);        /* card change interrupt */
  PCICR  |=  _BV(PCIE3);
  PCIFR  |=  _BV(PCIF3);
}

static inline uint8_t sdcard_detect(void) {
  return (!(PIND & _BV(PD4)));
}

static inline uint8_t sdcard_wp(void) {
  return (PINC & _BV(PC3));
}

/* No device jumpers on petSD */
#  define DEVICE_DEFAULT_UNITNO 8

#  define LED_BUSY_PORT PORTD
#  define LED_BUSY_DDR DDRD
#  define LED_BUSY_PIN PD5
#  define LED_DIRTY_PORT PORTD
#  define LED_DIRTY_INPUT PIND
#  define LED_DIRTY_DDR DDRD
#  define LED_DIRTY_PIN PD6
#  define LED_ACTIVE_LEVEL 1

#  define HAVE_IEEE
#  define IEEE_ATN_INT          INT0    /* ATN interrupt (required!) */
#  define IEEE_ATN_INT_VECT     INT0_vect

static inline void ieee_interrupts_init(void) {
  DDRD &= ~_BV(PD2);
  PORTD |= _BV(PD2);
  EICRA |= _BV(ISC00);
  EIMSK |= _BV(INT0);
}

#  define HAVE_7516X            /* Device uses 75160/75161 bus drivers */
#  define IEEE_PORT_TE          PORTB   /* TE */
#  define IEEE_DDR_TE           DDRB
#  define IEEE_PIN_TE           PB0
#  define IEEE_PORT_DC          PORTC   /* DC */
#  define IEEE_DDR_DC           DDRC
#  define IEEE_PIN_DC           PC5
#  define IEEE_INPUT_ATN        PIND    /* ATN */
#  define IEEE_PORT_ATN         PORTD
#  define IEEE_DDR_ATN          DDRD
#  define IEEE_PIN_ATN          PD2
#  define IEEE_INPUT_NDAC       PINC    /* NDAC */
#  define IEEE_PORT_NDAC        PORTC
#  define IEEE_DDR_NDAC         DDRC
#  define IEEE_PIN_NDAC         PC6
#  define IEEE_INPUT_NRFD       PINC    /* NRFD */
#  define IEEE_PORT_NRFD        PORTC
#  define IEEE_DDR_NRFD         DDRC
#  define IEEE_PIN_NRFD         PC7
#  define IEEE_INPUT_DAV        PINB    /* DAV */
#  define IEEE_PORT_DAV         PORTB
#  define IEEE_DDR_DAV          DDRB
#  define IEEE_PIN_DAV          PB2
#  define IEEE_INPUT_EOI        PIND    /* EOI */
#  define IEEE_PORT_EOI         PORTD
#  define IEEE_DDR_EOI          DDRD
#  define IEEE_PIN_EOI          PD7
#  define IEEE_INPUT_SRQ                /* SRQ */
#  define IEEE_PORT_SRQ         PORTA
#  define IEEE_DDR_SRQ          DDRA
#  define IEEE_PIN_SRQ
#  define IEEE_INPUT_IFC                /* IFC */
#  define IEEE_PORT_IFC         PORTA
#  define IEEE_DDR_IFC          DDRA
#  define IEEE_PIN_IFC
#  define IEEE_INPUT_REN                /* REN */
#  define IEEE_PORT_REN         PORTA
#  define IEEE_DDR_REN          DDRA
#  define IEEE_PIN_REN
#  define IEEE_D_PIN            PINA    /* Data */
#  define IEEE_D_PORT           PORTA
#  define IEEE_D_DDR            DDRA
#  define IEEE_BIT_DC           _BV(IEEE_PIN_DC)
#  define IEEE_BIT_TE           _BV(IEEE_PIN_TE)
#  define IEEE_BIT_SRQ          0   /* Define as 0 if SRQ not connected */
#  define IEEE_BIT_REN          0   /* Define as 0 if REN not connected */
#  define IEEE_BIT_IFC          0   /* Define as 0 if IFC not connected */

static inline void ieee_interface_init(void) {
  IEEE_PORT_TE  &= (uint8_t) ~ IEEE_BIT_TE;         // Set TE low
  IEEE_PORT_DC  |= IEEE_BIT_DC;                     // Set DC high
  IEEE_PORT_SRQ |= IEEE_BIT_SRQ;                    // Set SRQ high
  IEEE_DDR_TE   |= IEEE_BIT_TE;                     // Define TE  as output
  IEEE_DDR_DC   |= IEEE_BIT_DC;                     // Define DC  as output
  IEEE_DDR_SRQ  |= IEEE_BIT_SRQ;                    // Define SRQ as output
  IEEE_PORT_ATN |= _BV(IEEE_PIN_ATN);               // Enable pull-up for ATN
  IEEE_PORT_REN |= IEEE_BIT_REN;                    // Enable pull-up for REN
  IEEE_PORT_IFC |= IEEE_BIT_IFC;                    // Enable pull-up for IFC
  IEEE_DDR_ATN  &= (uint8_t) ~ _BV(IEEE_PIN_ATN);   // Define ATN as input
  IEEE_DDR_REN  &= (uint8_t) ~ IEEE_BIT_REN;        // Define REN as input
  IEEE_DDR_IFC  &= (uint8_t) ~ IEEE_BIT_IFC;        // Define IFC as input
}

#  define BUTTONS_PORT PORTB
#  define BUTTONS_INPUT PINB
#  define BUTTONS_DDR DDRB
#  define BUTTONS_NEXT_PIN PB1
#  define BUTTONS_PREV_PIN PB3

#  define SOFTI2C_PORT          PORTC
#  define SOFTI2C_PIN           PINC
#  define SOFTI2C_DDR           DDRC
#  define SOFTI2C_BIT_SCL       PC0
#  define SOFTI2C_BIT_SDA       PC1
#  define SOFTI2C_BIT_INTRQ     PC2
#  define SOFTI2C_DELAY         6

#  define HAVE_BOARD_INIT

static inline void board_init(void) {
  DDRC  |= _BV(PC4);
  PORTC |= _BV(PC4);       /* Disable  ENC28J60 */
}


#elif CONFIG_HARDWARE_VARIANT == 9
/* ---------- Hardware configuration: XS-1541 ---------- */
#  define HAVE_SD
#  define SD_SUPPLY_VOLTAGE     (1L<<18)

/* 230 kHz slow, 1.8432 2MHz fast */
#  define SPI_DIVISOR_SLOW 64
#  define SPI_DIVISOR_FAST 8

static inline void sdcard_interface_init(void) {
  /* No card detect switch, no write protect switch... nothing */
  return;
}

static inline uint8_t sdcard_detect(void) {
  return 1; /* assume it's always there */
}

static inline uint8_t sdcard_wp(void) {
  return 0;
}

/* No device jumpers on XS-1541 */
#  define DEVICE_DEFAULT_UNITNO 8

/* busy LED onboard */
#  define LED_BUSY_PORT PORTC
#  define LED_BUSY_DDR DDRC
#  define LED_BUSY_PIN PC0
/* dirty LED extern */
#  define LED_DIRTY_PORT PORTB
#  define LED_DIRTY_INPUT PINB
#  define LED_DIRTY_DDR DDRB
#  define LED_DIRTY_PIN PB0
#  define LED_ACTIVE_LEVEL 1

// dual-interface device, currently only as a compile-time option
#ifdef CONFIG_HAVE_IEC
#  define IEC_INPUT             PORTD
#  define IEC_DDR               DDRD
#  define IEC_PORT              PORTD
#  define IEC_PIN_ATN           PD2
#  define IEC_PIN_DATA          PD4
#  define IEC_PIN_CLOCK         PD7
#  define IEC_PIN_SRQ           PD5
#  define IEC_ATN_INT_VECT      PCINT3_vect
#  define IEC_PCMSK             PCMSK3
#  define IEC_PCIFR_BIT         PCIF3
#  define IEC_SRQATN_VECT_SHARED

static inline void iec_interrupts_init(void) {
  PCICR |= _BV(PCIE3);
  PCIFR |= _BV(IEC_PCIFR_BIT);
}
#endif // CONFIG_HAVE_IEC

#ifdef CONFIG_HAVE_IEEE
#  define IEEE_ATN_INT          PCINT3
#  define IEEE_PCMSK            PCMSK3
#  define IEEE_PCINT            PCINT27
#  define IEEE_ATN_INT_VECT     PCINT3_vect

static inline void ieee_interrupts_init(void)  {
  /* clear interrupt flag */
  PCIFR |= _BV(PCIF3);

  /* enable ATN in pin change enable mask */
  IEEE_PCMSK |= _BV(IEEE_PCINT);

  /* Enable pin change interrupt 3 (PCINT31..24) */
  PCICR |= _BV(PCIE3);
}

#  define IEEE_C_PIN            PINC    /* Control signals */
#  define IEEE_C_DDR            DDRC
#  define IEEE_C_PORT           PORTC
#  define IEEE_C_ATN_PIN        PIND
#  define IEEE_C_ATN_PORT       PORTD
#  define IEEE_C_ATN_DDR        DDRD
#  define IEEE_PIN_TE           0       /* 7516x only */
#  define IEEE_PIN_DC           0       /* 7516x only */
#  define IEEE_PIN_NDAC         PC4
#  define IEEE_PIN_NRFD         PC5
#  define IEEE_PIN_DAV          PC6
#  define IEEE_PIN_EOI          PC7
#  define IEEE_PIN_ATN          PD3
#  define IEEE_PIN_SRQ          PC3
#  define IEEE_PIN_REN          PC1
#  define IEEE_PIN_IFC          PC2
#  define IEEE_D_PIN            PINA    /* Data */
#  define IEEE_D_DDR            DDRA
#  define IEEE_D_PORT           PORTA
#  define IEEE_BIT_DC           _BV(IEEE_PIN_DC)
#  define IEEE_BIT_TE           _BV(IEEE_PIN_TE)
#  define IEEE_BIT_SRQ          _BV(IEEE_PIN_SRQ)
#  define IEEE_BIT_REN          _BV(IEEE_PIN_REN)
#  define IEEE_BIT_IFC          _BV(IEEE_PIN_IFC)

static inline void ieee_interface_init(void) {
  /* Define TE, DC, SRQ as outputs */
  IEEE_C_DDR |= IEEE_BIT_TE | IEEE_BIT_DC | IEEE_BIT_SRQ;

  /* Define REN, IFC as inputs */
  IEEE_C_DDR &= (uint8_t) ~ (IEEE_BIT_REN | IEEE_BIT_IFC);

  /* DC and SRQ high, pull-up for REN and IFC */
  IEEE_C_PORT     |= IEEE_BIT_DC | IEEE_BIT_SRQ | IEEE_BIT_REN | IEEE_BIT_IFC;
  IEEE_C_ATN_DDR  &= (uint8_t)~_BV(IEEE_PIN_ATN); // ATN as input
  IEEE_C_ATN_PORT |= _BV(IEEE_PIN_ATN);           // enable ATN pullup
}

#endif // CONFIG_HAVE_IEEE

#  define BUTTONS_PORT PORTB
#  define BUTTONS_INPUT PINB
#  define BUTTONS_DDR DDRB
#  define BUTTONS_NEXT_PIN PB1
#  define BUTTONS_PREV_PIN PB2


#elif CONFIG_HARDWARE_VARIANT == 20
/* ---------- Hardware configuration: FlexSD Drive version 1 ---------- */
#  define HAVE_SD
#  define SD_CHANGE_HANDLER     ISR(INT2_vect)
#  define SD_SUPPLY_VOLTAGE     (1L<<21)

/* 250kHz slow, 2MHz fast */
#  define SPI_DIVISOR_SLOW 32
#  define SPI_DIVISOR_FAST 4

static inline void sdcard_interface_init(void) {
  // WP + CARD detect lines inicialized in board_init()
  EICRA |=  _BV(ISC20);
  EIMSK |=  _BV(INT2);
}

static inline uint8_t sdcard_detect(void) {
  return !(PINB & _BV(PB2));
}

static inline uint8_t sdcard_wp(void) {
  return PINB & _BV(PB3);
}

#  define DEVICE_HW_ADDR_MUX_PIN PINC
#  define DEVICE_HW_ADDR_PIN OCR0A      // For an explanation of this funny register choice, see board_init()
#  define DEVICE_HW_ADDR_B0 6
#  define DEVICE_HW_ADDR_B1 5
#  define DEVICE_HW_CONF_B2 4
#  define DEVICE_HW_CONF_B3 3

/* On the FlexSD hardware, the LEDs have the same port pins as the
   push buttons. To avoid collisions, the LEDs are driven in a
   simulated open-collector. Therefore, the LED ports use the data
   direction register, and the active level is '1', even though the
   LEDs are lit with output = low level. */
#  define LED_BUSY_PORT DDRB
#  define LED_BUSY_PIN PB0
#  define LED_DIRTY_PORT DDRB
#  define LED_DIRTY_PIN PB1
#  define LED_ACTIVE_LEVEL 1
#  define LED_VCPU_PORT PORTD
#  define LED_VCPU_PIN PD1

#  define IEC_INPUT             PIND
#  define IEC_DDRIN             DDRD
#  define IEC_PORTIN            PORTD
#  define IEC_PIN_ATN           PD4
#  define IEC_PIN_DATA          PD6
#  define IEC_PIN_CLOCK         PD5
#  define IEC_PIN_SRQ           PD3
#  define IEC_SEPARATE_OUT
#  define IEC_PORT              PORTC
#  define IEC_DDROUT            DDRC
#  define IEC_OPIN_ATN          PC4
#  define IEC_OPIN_DATA         PC6
#  define IEC_OPIN_CLOCK        PC5
#  define IEC_OPIN_SRQ          PC3
#  define IEC_ATN_INT_VECT      PCINT3_vect
#  define IEC_PCMSK             PCMSK3
#  define IEC_PCIFR_BIT         PCIF3
#  define IEC_SRQATN_VECT_SHARED

#  define IEC_INPUTS_INVERTED

static inline void iec_interrupts_init(void) {
  PCICR |= _BV(PCIE3);
  PCIFR |= _BV(IEC_PCIFR_BIT);
}

/* Parallel cable: */
#  define HAVE_PARALLEL
#  define PARALLEL_HANDLER      ISR(INT0_vect)
#  define PARALLEL_PDDR         DDRA      // CONN2 pins 1,3,...,15
#  define PARALLEL_PPORT        PORTA
#  define PARALLEL_PPIN         PINA
#  define PARALLEL_HDDR         DDRD
#  define PARALLEL_HPORT        PORTD
#  define PARALLEL_HPIN         PIND
#  define PARALLEL_HSK_OUT_BIT  7         // CONN2 pin 14, to C64 FLAG2
#  define PARALLEL_HSK_IN_BIT   2         // CONN2 pin 16, to C64 PC2
#  define PARALLEL_INTMODE      _BV(ISC01)  // Falling Edge
#  define PARALLEL_INTSEL       INT0

#  define PARALLEL_IRQEN_BIT    PARALLEL_INTSEL
#  define PARALLEL_IRQEN_REG    EIMSK
#  define PARALLEL_IRQFLAG_REG  EIFR

/* Configure parallel port lines and interrupt */
static inline void parallelport_init(void) {
  /* Parallel lines configured by board_init(),
     this init configure irq only */

  /* configure interrupt for parallel handshake */
  /* exclusive INTx line */
  EICRA |= PARALLEL_INTMODE;
}

/* I2C + display lines: */
#  define SOFTI2C_PORT          PORTC
#  define SOFTI2C_PIN           PINC
#  define SOFTI2C_DDR           DDRC
#  define SOFTI2C_BIT_SCL       PC0
#  define SOFTI2C_BIT_SDA       PC1
#  define SOFTI2C_BIT_INTRQ     PC2
#  define SOFTI2C_DELAY         6

#  define HAVE_BOARD_INIT
#  define IEC_IFINIT_IN_BRDINIT
static inline void board_init(void) {
  /* Read config switches in early stage, inputs are multiplexed
     with IEC output pins. The state of the bits is stored in an
     unused peripheral register. If the state of the switches has
     already been read by the bootloader, it is saved in the same
     register. The reason it is not saved to SRAM is because it
     is deleted by crt on startup, but the peripheral register
     remains unchanged. */
  if (!(IEC_DDROUT & (_BV(IEC_OPIN_ATN) | _BV(IEC_OPIN_CLOCK) | _BV(IEC_OPIN_DATA) | _BV(IEC_OPIN_SRQ)))) {
    IEC_PORT |= (_BV(IEC_OPIN_ATN) | _BV(IEC_OPIN_CLOCK) | _BV(IEC_OPIN_DATA) | _BV(IEC_OPIN_SRQ));
    DEVICE_HW_ADDR_PIN = DEVICE_HW_ADDR_MUX_PIN;
    IEC_PORT &= ~(_BV(IEC_OPIN_ATN) | _BV(IEC_OPIN_CLOCK) | _BV(IEC_OPIN_DATA) | _BV(IEC_OPIN_SRQ));
  }
  /* I/O directions set */
  DDRA = 0x00;      // parallel IN
  PORTA = 0xff;     // parallel pullups on
  DDRB &= 0xf0;
  PORTB = (PORTB & 0xf0) | 0x0c;  // Config portB B3210, B7654: SPI bus, init separately
  PORTC = 0xc7;     // CBMSER DAT lines drive to low, host wait...
  DDRC = 0xfb;
  PORTD = 0xff;
  DDRD = 0x02;
  /* The FlexSD drive hardware runs at 16 MHz clock speed,
   wich requires divide by 2 now */
  CLKPR = _BV(CLKPCE);
  CLKPR = _BV(CLKPS0);
}

#  define BUTTONS_MUX_DDR DDRB
#  define BUTTONS_MUX_PORT PORTB
#  define BUTTONS_INPUT PINB
#  define BUTTONS_NEXT_PIN PB0
#  define BUTTONS_PREV_PIN PB1

/* Since the push buttons share the LEDs, they need to be read a states
   in a special way. But there are many steps. The push buttons are
   (normally) read by the interrupt routine. So this function should
   not be used directly from the main program. */
#  define BUTTONS_SPECIAL
static inline rawbutton_t buttons_read(void) {
  rawbutton_t buttons;
  uint8_t p = BUTTONS_MUX_DDR;
  BUTTONS_MUX_DDR &= ~(_BV(BUTTONS_NEXT_PIN) | _BV(BUTTONS_PREV_PIN));
  BUTTONS_MUX_PORT |= (_BV(BUTTONS_NEXT_PIN) | _BV(BUTTONS_PREV_PIN));
  buttons = BUTTONS_INPUT;
  BUTTONS_MUX_PORT &= ~(_BV(BUTTONS_NEXT_PIN) | _BV(BUTTONS_PREV_PIN));
  BUTTONS_MUX_DDR = p;
  return (buttons & (_BV(BUTTONS_NEXT_PIN) | _BV(BUTTONS_PREV_PIN)));
}

#  ifdef CONFIG_VCPULED
#    define VCPU_LED_FUNCT
static inline __attribute__((always_inline)) void set_vcpu_led(uint8_t state) {
  if (state)
    LED_VCPU_PORT &= ~_BV(LED_VCPU_PIN);
  else
    LED_VCPU_PORT |= _BV(LED_VCPU_PIN);
}
#  endif


#else
#  error "CONFIG_HARDWARE_VARIANT is unset or set to an unknown value."
#endif


/* ---------------- End of user-configurable options ---------------- */

#if !defined(CONFIG_HAVE_IEC) && !defined(CONFIG_HAVE_IEEE)
#  error Need CONFIG_HAVE_IEC and/or CONFIG_HAVE_IEEE
// Please edit your config-<devicename> if this error occurs.
#endif

#if defined(CONFIG_HAVE_IEC) && defined(CONFIG_HAVE_IEEE)
#  error Sorry, dual-interface devices must select only one interface at compile time!
#endif

#if defined(CONFIG_UART_DEBUG) && (CONFIG_FASTSERIAL_MODE >= 2)
#  error Fast Serial and UART Debug cannot be used at the same time!
#endif

/* --- Buttons --- */

/* Button NEXT changes to the next disk image and enables sleep mode (held) */
#  define BUTTON_NEXT _BV(BUTTONS_NEXT_PIN)

/* Button PREV changes to the previous disk image */
#  define BUTTON_PREV _BV(BUTTONS_PREV_PIN)

/* Read the raw button state - a depressed button should read as 0 */
#  ifndef BUTTONS_SPECIAL
static inline rawbutton_t buttons_read(void) {
  return BUTTONS_INPUT & (BUTTON_NEXT | BUTTON_PREV);
}
#  endif

static inline void buttons_init(void) {
#  ifdef BUTTONS_DDR
  BUTTONS_DDR  &= (uint8_t)~(BUTTON_NEXT | BUTTON_PREV);
  BUTTONS_PORT |= BUTTON_NEXT | BUTTON_PREV;
#  endif
}



/* --- Device address functions --- */

#ifdef DEVICE_DEFAULT_UNITNO
/* Devices without address jumpers/switches: */
static inline uint8_t device_hw_address(void) {
  return DEVICE_DEFAULT_UNITNO;
}
static inline void device_hw_address_init(void) {
  return;
}
#else
/* device_hw_address() returns the hardware-selected device address */
static inline uint8_t device_hw_address(void) {
  return 8 + !(DEVICE_HW_ADDR_PIN & _BV(DEVICE_HW_ADDR_B0)) + 2*!(DEVICE_HW_ADDR_PIN & _BV(DEVICE_HW_ADDR_B1));
}
/* Configure hardware device address pins */
static inline void device_hw_address_init(void) {
#  ifdef DEVICE_HW_ADDR_DDR
  DEVICE_HW_ADDR_DDR  &= ~(_BV(DEVICE_HW_ADDR_B0) | _BV(DEVICE_HW_ADDR_B1));
  DEVICE_HW_ADDR_PORT |=   _BV(DEVICE_HW_ADDR_B0) | _BV(DEVICE_HW_ADDR_B1);
#  endif
}
#endif


/* --- LEDs --- */

/* Initialize ports for all LEDs */
static inline void leds_init(void) {
  /* Note: Depending on the chip and register these lines can compile */
  /*       to one instruction each on AVR. For two bits this is one   */
  /*       instruction shorter than "DDRC |= _BV(PC0) | _BV(PC1);"    */
#ifdef LED_BUSY_DDR
  LED_BUSY_DDR |= _BV(LED_BUSY_PIN);      /* Port pin set to OUTPUT */
#endif
#ifdef LED_DIRTY_DDR
#  ifdef LED_DIRTY_PIN
  LED_DIRTY_DDR |= _BV(LED_DIRTY_PIN);    /* Port pin set to OUTPUT */
#  endif
#endif

// VCPU RUN LED, if required, set output level to required value.
// For LED ON/OFF, set pin direction to OUT/IN.
#ifdef CONFIG_VCPUSUPPORT
#  ifdef VCPURUNFLAG_PORT
#    if (LED_ACTIVE_LEVEL == 1)
  VCPURUNFLAG_PORT |= _BV(VCPURUNFLAG_BIT);
#    else
  VCPURUNFLAG_PORT &= ~(_BV(VCPURUNFLAG_BIT));
#    endif
#  endif
#endif
}

#ifndef SINGLE_LED
/* --- "BUSY" led, recommended color: green (usage similiar to 1541 LED) --- */
static inline __attribute__((always_inline)) void set_busy_led(uint8_t state) {
  if (state)
#if (LED_ACTIVE_LEVEL == 1)
    LED_BUSY_PORT |= _BV(LED_BUSY_PIN);
#else
    LED_BUSY_PORT &= ~_BV(LED_BUSY_PIN);
#endif
  else
#if (LED_ACTIVE_LEVEL == 1)
    LED_BUSY_PORT &= ~_BV(LED_BUSY_PIN);
#else
    LED_BUSY_PORT |= _BV(LED_BUSY_PIN);
#endif
}

/* --- "DIRTY" led, recommended color: red (errors, unwritten data in memory) --- */
static inline __attribute__((always_inline)) void set_dirty_led(uint8_t state) {
  if (state)
#if (LED_ACTIVE_LEVEL == 1)
    LED_DIRTY_PORT |= _BV(LED_DIRTY_PIN);
#else
    LED_DIRTY_PORT &= ~_BV(LED_DIRTY_PIN);
#endif
  else
#if (LED_ACTIVE_LEVEL == 1)
    LED_DIRTY_PORT &= ~_BV(LED_DIRTY_PIN);
#else
    LED_DIRTY_PORT |= _BV(LED_DIRTY_PIN);
#endif
}

/* Toggle function used for error blinking */
static inline void toggle_dirty_led(void) {
#  ifdef LED_DIRTY_INPUT
  /* Sufficiently new AVR cores have a toggle function */
  LED_DIRTY_INPUT |= _BV(LED_DIRTY_PIN);      /* Compiler optimized to SBI instruction, no IN/ORI/OUT */
#  else
  LED_DIRTY_PORT ^= _BV(LED_DIRTY_PIN);
#  endif
}
#endif

/* Functions for one-LED hardwares: */
#ifdef SINGLE_LED
static inline __attribute__((always_inline)) void set_led(uint8_t state) {
  if (state)
#  if (LED_ACTIVE_LEVEL == 1)
    LED_BUSY_PORT |= _BV(LED_BUSY_PIN);
#  else
    LED_BUSY_PORT &= ~_BV(LED_BUSY_PIN);
#  endif
  else
#  if (LED_ACTIVE_LEVEL == 1)
    LED_BUSY_PORT &= ~_BV(LED_BUSY_PIN);
#  else
    LED_BUSY_PORT |= _BV(LED_BUSY_PIN);
#  endif
}

static inline void toggle_led(void) {
  LED_BUSY_INPUT |= _BV(LED_BUSY_PIN);
}
#endif



/* VCPU run flag set/clear */
/* This function is *NOT* LED-switch only, the bit-value (level) is *NOT* configurabe! */
#ifdef CONFIG_VCPUSUPPORT
  #ifndef VCPURUNFLAG_REG
    #define VCPURUNFLAG_REG globalflags
    #define VCPURUNFLAG_BIT VCPU_RUN_FLAG_BIT
  #endif
static inline __attribute__((always_inline)) void set_vcpurunflag(uint8_t state) {
  if (state)
      VCPURUNFLAG_REG |= (1 << VCPURUNFLAG_BIT);
  else
      VCPURUNFLAG_REG &= ~(1 << VCPURUNFLAG_BIT);
}
/* VCPU indicator led. AVR targets no separate VCPU led, unless you have already defined one. :) */
/* The previous function can still be used for status feedback! */
  #ifndef VCPU_LED_FUNCT
    #define set_vcpu_led(x) do {} while(0)
  #endif
#endif



/* --- IEC --- */
#ifdef CONFIG_HAVE_IEC

#define IEC_BIT_ATN      _BV(IEC_PIN_ATN)
#define IEC_BIT_DATA     _BV(IEC_PIN_DATA)
#define IEC_BIT_CLOCK    _BV(IEC_PIN_CLOCK)
#define IEC_BIT_SRQ      _BV(IEC_PIN_SRQ)

/* Return type of iec_bus_read() */
typedef uint8_t iec_bus_t;

/* OPIN definitions are only used in the assembler module */
#ifdef IEC_SEPARATE_OUT
#  define IEC_OBIT_ATN   _BV(IEC_OPIN_ATN)
#  define IEC_OBIT_DATA  _BV(IEC_OPIN_DATA)
#  define IEC_OBIT_CLOCK _BV(IEC_OPIN_CLOCK)
#  define IEC_OBIT_SRQ   _BV(IEC_OPIN_SRQ)
#  define IEC_OUTPUT     IEC_PORT
#else
#  define IEC_OPIN_ATN   IEC_PIN_ATN
#  define IEC_OPIN_DATA  IEC_PIN_DATA
#  define IEC_OPIN_CLOCK IEC_PIN_CLOCK
#  define IEC_OPIN_SRQ   IEC_PIN_SRQ
#  define IEC_OBIT_ATN   IEC_BIT_ATN
#  define IEC_OBIT_DATA  IEC_BIT_DATA
#  define IEC_OBIT_CLOCK IEC_BIT_CLOCK
#  define IEC_OBIT_SRQ   IEC_BIT_SRQ
#  define IEC_OUTPUT     IEC_DDR
#endif

#ifndef IEC_PORTIN
#  define IEC_PORTIN IEC_PORT
#endif

#ifndef IEC_DDRIN
#  define IEC_DDRIN  IEC_DDR
#  define IEC_DDROUT IEC_DDR
#endif

/* The AVR asm modules don't support noninverted output lines, */
/* so this can be staticalle defined for all configurations.   */
#define IEC_OUTPUTS_INVERTED

#ifdef IEC_PCMSK
   /* For hardware configurations using PCINT for IEC IRQs */
#  define set_atn_irq(x) \
     if (x) { IEC_PCMSK |= _BV(IEC_PIN_ATN); } \
     else { IEC_PCMSK &= (uint8_t)~_BV(IEC_PIN_ATN); }
#  define set_clock_irq(x) \
     if (x) { IEC_PCMSK |= _BV(IEC_PIN_CLOCK); } \
     else { IEC_PCMSK &= (uint8_t)~_BV(IEC_PIN_CLOCK); }
#  define HAVE_CLOCK_IRQ
#else
     /* Hardware ATN interrupt */
#  define set_atn_irq(x) \
     if (x) { EIMSK |= _BV(IEC_ATN_INT); } \
     else { EIMSK &= (uint8_t)~_BV(IEC_ATN_INT); }

#  ifdef IEC_CLK_INT
     /* Hardware has a CLK interrupt */
#    define set_clock_irq(x) \
       if (x) { EIMSK |= _BV(IEC_CLK_INT); } \
       else { EIMSK &= (uint8_t)~_BV(IEC_CLK_INT); }
#    define HAVE_CLOCK_IRQ
#  endif
#endif

/* IEC output functions */
#ifdef IEC_OUTPUTS_INVERTED
#  define COND_INV(x) (!(x))
#else
#  define COND_INV(x) (x)
#endif

static inline __attribute__((always_inline)) void set_atn(uint8_t state) {
  if (COND_INV(state))
    IEC_OUTPUT |= IEC_OBIT_ATN;
  else
    IEC_OUTPUT &= ~IEC_OBIT_ATN;
}

static inline __attribute__((always_inline)) void set_data(uint8_t state) {
  if (COND_INV(state))
    IEC_OUTPUT |= IEC_OBIT_DATA;
  else
    IEC_OUTPUT &= ~IEC_OBIT_DATA;
}

static inline __attribute__((always_inline)) void set_clock(uint8_t state) {
  if (COND_INV(state))
    IEC_OUTPUT |= IEC_OBIT_CLOCK;
  else
    IEC_OUTPUT &= ~IEC_OBIT_CLOCK;
}

#ifdef IEC_SEPARATE_OUT
static inline __attribute__((always_inline)) void set_srq(uint8_t state) {
  if (COND_INV(state))
    IEC_OUTPUT |= IEC_OBIT_SRQ;
  else
    IEC_OUTPUT &= ~IEC_OBIT_SRQ;
}
#else
/* this version of the function turns on the pullups when state is 1 */
/* note: same pin for in/out implies inverted output via DDR */
static inline __attribute__((always_inline)) void set_srq(uint8_t state) {
  if (state) {
    IEC_DDR  &= ~IEC_OBIT_SRQ;
    IEC_PORT |=  IEC_OBIT_SRQ;
  } else {
    IEC_PORT &= ~IEC_OBIT_SRQ;
    IEC_DDR  |=  IEC_OBIT_SRQ;
  }
}
#endif

#undef COND_INV

// for testing purposes only, probably does not do what you want!
#define toggle_srq() IEC_INPUT |= IEC_OBIT_SRQ

/* Parallel handshake interrupt enable/disable: */
#ifdef HAVE_PARALLEL
static inline void parallel_irq_enable(void) {
  PARALLEL_IRQFLAG_REG = _BV(PARALLEL_IRQEN_BIT);       // Clear pendig IRQ
  PARALLEL_IRQEN_REG |= _BV(PARALLEL_IRQEN_BIT);        // Enable parallel IRQ
}

static inline void parallel_irq_disable(void) {
  PARALLEL_IRQEN_REG &= ~(_BV(PARALLEL_IRQEN_BIT));     // Disable parallel IRQ
  //PARALLEL_IRQFLAG_REG = _BV(PARALLEL_IRQEN_BIT);       // Clear pendig IRQ
}
#endif


/* IEC lines initialisation */
static inline void iec_interface_init(void) {
#ifndef IEC_IFINIT_IN_BRDINIT
#  ifdef IEC_SEPARATE_OUT
  /* Set up the input port - pullups on all lines */
  IEC_DDRIN  &= (uint8_t)~(IEC_BIT_ATN  | IEC_BIT_CLOCK  | IEC_BIT_DATA  | IEC_BIT_SRQ);
  IEC_PORTIN |= IEC_BIT_ATN | IEC_BIT_CLOCK | IEC_BIT_DATA | IEC_BIT_SRQ;
  /* Set up the output port - all lines high */
  IEC_DDROUT |=            IEC_OBIT_ATN | IEC_OBIT_CLOCK | IEC_OBIT_DATA | IEC_OBIT_SRQ;
  IEC_PORT   &= (uint8_t)~(IEC_OBIT_ATN | IEC_OBIT_CLOCK | IEC_OBIT_DATA | IEC_OBIT_SRQ);
#  else
  /* Pullups would be nice, but AVR can't switch from */
  /* low output to hi-z input directly                */
  IEC_DDR  &= (uint8_t)~(IEC_BIT_ATN | IEC_BIT_CLOCK | IEC_BIT_DATA | IEC_BIT_SRQ);
  IEC_PORT &= (uint8_t)~(IEC_BIT_ATN | IEC_BIT_CLOCK | IEC_BIT_DATA);
  /* SRQ is special-cased because it may be unconnected */
  IEC_PORT |= IEC_BIT_SRQ;
#  endif
#endif
#ifdef HAVE_PARALLEL
  parallelport_init();    /* Configure parallel port lines and interrupt */
  //parallel_irq_enable();
  parallel_irq_disable();
#endif
}

/* weak-aliasing is resolved at link time, so it doesn't work */
/* for static inline functions - use a conditionally compiled */
/* wrapper instead                                            */
#  ifndef CONFIG_HAVE_IEEE
static inline void bus_interface_init(void) {
  iec_interface_init();
}
#  endif
#endif /* CONFIG_HAVE_IEC */


/* --- IEEE --- */
#ifdef CONFIG_HAVE_IEEE
#  ifdef IEEE_PCMSK
/* IEEE-488 ATN interrupt using PCINT */
static inline void set_atn_irq(uint8_t x) {
  if (x)
    IEEE_PCMSK |= _BV(IEEE_PCINT);
  else
    IEEE_PCMSK &= (uint8_t) ~_BV(IEEE_PCINT);
}
#  else
/* Hardware ATN interrupt */
static inline void set_atn_irq(uint8_t x) {
  if (x)
    EIMSK |= _BV(IEEE_ATN_INT);
  else
    EIMSK &= (uint8_t) ~_BV(IEEE_ATN_INT);
}
#  endif

/* same weak alias problem as in IEC version */
#  ifndef CONFIG_HAVE_IEC
static inline void bus_interface_init(void) {
  ieee_interface_init();
}
#  endif
#endif /* CONFIG_HAVE_IEEE */



/* The assembler module needs the vector names, */
/* so the _HANDLER macros are created here.     */
#define IEC_ATN_HANDLER   ISR(IEC_ATN_INT_VECT)
#define IEC_CLOCK_HANDLER ISR(IEC_CLK_INT_VECT)
#define IEEE_ATN_HANDLER  ISR(IEEE_ATN_INT_VECT)

/* SD SS pin default implementation */
#ifndef SDCARD_SS_SPECIAL
static inline __attribute__((always_inline)) void sdcard_set_ss(uint8_t state) {
  if (state)
    SPI_PORT |= SPI_SS;
  else
    SPI_PORT &= ~SPI_SS;
}
#endif

/* Display interrupt pin */
#ifdef CONFIG_REMOTE_DISPLAY
static inline void display_intrq_init(void) {
  /* Enable pullup on the interrupt line */
  SOFTI2C_PORT |= _BV(SOFTI2C_BIT_INTRQ);
}

static inline uint8_t display_intrq_active(void) {
  return !(SOFTI2C_PIN & _BV(SOFTI2C_BIT_INTRQ));
}
#endif

/* P00 name cache is in bss by default */
#ifndef P00CACHE_ATTRIB
#  define P00CACHE_ATTRIB
#endif

/* -- ensure that the timing for Dolphin is achievable        -- */
/* the C64 will switch to an alternate, not-implemented protocol */
/* if the answer to the XQ/XZ commands is too late and the       */
/* file name/command dump takes too long if the buffer is        */
/* smaller than the output from uart_trace                       */
#if defined(CONFIG_PARALLEL_DOLPHIN) && \
    defined(CONFIG_UART_DEBUG) && \
  CONFIG_UART_BUF_SHIFT < 8  // 7 may work with short file names
#  error Enabling both DolphinDOS and UART debugging requires CONFIG_UART_BUF_SHIFT >= 8 !
#endif

#endif
