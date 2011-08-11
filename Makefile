# Hey Emacs, this is a -*- makefile -*-

# Define version number
MAJOR = 0
MINOR = 10
PATCHLEVEL = 1
FIX =

# Forces bootloader version to 0, comment out or leave empty for release
PRERELEASE =

#----------------------------------------------------------------------------
# WinAVR Makefile Template written by Eric B. Weddington, Joerg Wunsch, et al.
#
# Released to the Public Domain
#
# Additional material for this makefile was written by:
# Peter Fleury
# Tim Henigan
# Colin O'Flynn
# Reiner Patommel
# Markus Pfaff
# Sander Pool
# Frederik Rouleau
# Carlos Lamas
#
#
# Extensively modified for sd2iec by Ingo Korb
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------

# Read configuration file
ifdef CONFIG
 CONFIGSUFFIX = $(CONFIG:config%=%)
else
 CONFIG = config
 CONFIGSUFFIX =
endif

# Enable verbose compilation with "make V=1"
ifdef V
 Q :=
 E := @:
else
 Q := @
 E := @echo
endif

# Include the configuration file
include $(CONFIG)

# Directory for all generated files
OBJDIR := obj-$(CONFIG_MCU:atmega%=m%)$(CONFIGSUFFIX)

# Output format. (can be srec, ihex, binary)
HEXFORMAT = ihex

# Target file name (without extension).
TARGET = $(OBJDIR)/sd2iec

# List C source files here. (C dependencies are automatically generated.)
SRC  = buffers.c fatops.c fileops.c main.c errormsg.c
SRC += doscmd.c ff.c m2iops.c d64ops.c diskchange.c
SRC += eeprom.c parser.c utils.c led.c diskio.c sdcard.c
SRC += timer.c $(CONFIG_ARCH)/arch-timer.c $(CONFIG_ARCH)/spi.c
SRC += $(CONFIG_ARCH)/system.c

ifneq ($(CONFIG_HAVE_IEC),y)
  ifneq ($(CONFIG_HAVE_IEEE),y)
    .PHONY: nobus
    nobus:
	@echo 'Neither CONFIG_HAVE_IEC nor CONFIG_HAVE_IEEE are set.'
	@echo "Please edit $(CONFIG)."
  endif
endif

ifeq ($(CONFIG_HAVE_IEC),y)
  SRC += iec.c fastloader.c
endif

ifeq ($(CONFIG_HAVE_IEEE),y)
  SRC += ieee.c
endif

ifeq ($(CONFIG_UART_DEBUG),y)
  SRC += $(CONFIG_ARCH)/uart.c
endif

ifeq ($(CONFIG_REMOTE_DISPLAY),y)
  SRC += display.c
  NEED_I2C := y
endif

ifeq ($(CONFIG_P00CACHE),y)
  SRC += p00cache.c
endif

# Additional hardware support enabled in the config file
ifdef CONFIG_ADD_SD
  SRC += sdcard.c
endif

ifdef CONFIG_ADD_ATA
  SRC += $(CONFIG_ARCH)/ata.c
endif

# Various RTC implementations
ifeq ($(CONFIG_RTC_VARIANT),5)
  SRC += rtc.c ds1307-3231.c
  NEED_I2C := y
endif

ifeq ($(CONFIG_RTC_VARIANT),4)
  SRC += rtc.c ds1307-3231.c
  NEED_I2C := y
endif
# Note: 3 is the LPC17xx internal RTC

ifeq ($(CONFIG_RTC_VARIANT),2)
  SRC += rtc.c pcf8583.c
  NEED_I2C := y
endif

ifeq ($(CONFIG_RTC_VARIANT),1)
  SRC += rtc.c avr/softrtc.c
endif

# Optimization level, can be [0, 1, 2, 3, s].
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
# Use s -mcall-prologues when you really need size...
#OPT = 2
OPT = s

# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DEBUG = dwarf-2


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRAINCDIRS =


# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99


# Place -D or -U options here
CDEFS = -DF_CPU=$(CONFIG_MCU_FREQ)UL

# Calculate bootloader version
ifdef PRERELEASE
BOOT_VERSION := 0
else
BOOT_VERSION := 0x$(MAJOR)$(MINOR)$(PATCHLEVEL)$(FIX)
endif

# Create a version number define
ifdef PATCHLEVEL
ifdef FIX
PROGRAMVERSION := $(MAJOR).$(MINOR).$(PATCHLEVEL).$(FIX)
else
PROGRAMVERSION := $(MAJOR).$(MINOR).$(PATCHLEVEL)
BOOT_VERSION := $(BOOT_VERSION)0
endif
else
PROGRAMVERSION := $(MAJOR).$(MINOR)
BOOT_VERSION := $(BOOT_VERSION)00
endif

ifdef PRERELEASE
PROGRAMVERSION := $(PROGRAMVERSION)$(PRERELEASE)
endif

LONGVERSION := -$(CONFIG_MCU:atmega%=m%)$(CONFIGSUFFIX)
CDEFS += -DVERSION=\"$(PROGRAMVERSION)\" -DLONGVERSION=\"$(LONGVERSION)\"


# Define programs and commands.
# CC must be defined here to generate the correct CFLAGS
SHELL = sh
REMOVE = rm -f
COPY = cp
WINSHELL = cmd
AWK = awk
CRCGEN = crcgen-new

# Include architecture-specific variables
include scripts/$(CONFIG_ARCH)/variables.mk

#---------------- Compiler Options ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS)
CFLAGS += -O$(OPT) -fno-strict-aliasing
CFLAGS += -Wall -Wstrict-prototypes -Werror
#CFLAGS += -Wa,-adhlns=$(OBJDIR)/$(<:.c=.lst)
CFLAGS += -I$(OBJDIR) -Isrc -Isrc/$(CONFIG_ARCH)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)
CFLAGS += -ffunction-sections -fdata-sections

ifdef NEED_I2C
  CFLAGS += -DHAVE_I2C
endif

#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -ahlms:    create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
ASFLAGS = -Wa,-gstabs -I$(OBJDIR) -Isrc -Isrc/$(CONFIG_ARCH)


#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections
ifeq ($(CONFIG_LINKER_RELAX),y)
  LDFLAGS += -Wl,-O9,--relax
endif


#============================================================================


# De-dupe the list of C source files
CSRC := $(patsubst %,src/%,$(sort $(SRC)))

# Add subdir to assembler source files
ASMSRC_DIR := $(patsubst %,src/%,$(ASMSRC))

# Define all object files.
OBJ := $(patsubst %,$(OBJDIR)/%,$(CSRC:.c=.o) $(ASMSRC_DIR:.S=.o))

# Define all listing files.
LST := $(patsubst %,$(OBJDIR)/%,$(CSRC:.c=.lst) $(ASMSRC_DIR:.S=.lst))

# Define the object directories
OBJDIRS := $(sort $(dir $(OBJ)))

# Compiler flags to generate dependency files.
GENDEPFLAGS = -MMD -MP -MF .dep/$(@F).d


# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS  = $(ARCH_CFLAGS)  $(CFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = $(ARCH_ASFLAGS) -x assembler-with-cpp $(ASFLAGS) $(CDEFS)
ALL_LDFLAGS = $(ARCH_LDFLAGS) $(LDFLAGS)


# Default target.
all: build

# include architecture-dependent rules
include scripts/$(CONFIG_ARCH)/targets.mk

build: elf bin hex
	$(E) "  SIZE   $(TARGET).elf"
	$(Q)$(ELFSIZE)|grep -v debug

elf: $(TARGET).elf
bin: $(TARGET).bin
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss
sym: $(TARGET).sym


# Doxygen output:
doxygen:
	-rm -rf doxyinput
	mkdir doxyinput
	cp src/*.h src/*.c doxyinput
	scripts/src2doxy.pl doxyinput/*.h doxyinput/*.c
	doxygen scripts/doxygen.conf

# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) -A $(TARGET).elf


# Generate autoconf.h from config
.PRECIOUS : $(OBJDIR)/autoconf.h
$(OBJDIR)/autoconf.h: $(CONFIG) | $(OBJDIR)
	$(E) "  CONF2H $(CONFIG)"
	$(Q)$(AWK) -f scripts/conf2h.awk $(CONFIG) > $(OBJDIR)/autoconf.h

# Generate macro-only asmconfig.h from config
.PRECIOUS: $(OBJDIR)/asmconfig.h
$(OBJDIR)/asmconfig.h: $(CONFIG) $(OBJDIR)/autoconf.h src/config.h src/$(CONFIG_ARCH)/arch-config.h | $(OBJDIR)
	$(E) "  CPP    config.h"
	$(Q)$(CC) -E -dM $(ALL_ASFLAGS) src/config.h | grep -v "^#define __" > $@

# Create final output files (.hex, .eep) from ELF output file.
ifeq ($(CONFIG_BOOTLOADER),y)
$(OBJDIR)/%.bin: $(OBJDIR)/%.elf
	$(E) "  BIN    $@"
	$(Q)$(OBJCOPY) -O binary -R .eeprom $< $@
	$(E) "  CRCGEN $@"
	-$(Q)$(CRCGEN) $@ $(BINARY_LENGTH) $(CONFIG_BOOT_DEVID) $(BOOT_VERSION)
	$(E) "  COPY   $(CONFIG_HARDWARE_NAME)-firmware-$(PROGRAMVERSION).bin"
	$(Q)$(COPY) $@ $(OBJDIR)/$(CONFIG_HARDWARE_NAME)-firmware-$(PROGRAMVERSION).bin
else
$(OBJDIR)/%.bin: $(OBJDIR)/%.elf
	$(E) "  BIN    $@"
	$(Q)$(OBJCOPY) -O binary -R .eeprom $< $@
endif


$(OBJDIR)/%.hex: $(OBJDIR)/%.elf
	$(E) "  HEX    $@"
	$(Q)$(OBJCOPY) -O $(HEXFORMAT) -R .eeprom $< $@

$(OBJDIR)/%.eep: $(OBJDIR)/%.elf
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(HEXFORMAT) $< $@

# Create extended listing file from ELF output file.
$(OBJDIR)/%.lss: $(OBJDIR)/%.elf
	$(E) "  LSS    $<"
	$(Q)$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
$(OBJDIR)/%.sym: $(OBJDIR)/%.elf
	$(E) "  SYM    $<"
	$(E)$(NM) -n $< > $@



# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
$(OBJDIR)/%.elf: $(OBJ)
	$(E) "  LINK   $@"
	$(Q)$(CC) $(ALL_CFLAGS) $^ --output $@ $(ALL_LDFLAGS)


# Compile: create object files from C source files.
$(OBJDIR)/%.o : %.c $(OBJDIR)/autoconf.h | $(OBJDIR)
	$(E) "  CC     $<"
	$(Q)$(CC) -c $(ALL_CFLAGS) $< -o $@


# Compile: create assembler files from C source files.
$(OBJDIR)/%.s : %.c $(OBJDIR)/autoconf.h | $(OBJDIR)
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
$(OBJDIR)/%.o : %.S $(OBJDIR)/asmconfig.h $(OBJDIR)/autoconf.h | $(OBJDIR)
	$(E) "  AS     $<"
	$(Q)$(CC) -c $(ALL_ASFLAGS) $< -o $@

# Create preprocessed source for use in sending a bug report.
$(OBJDIR)/%.i : %.c $(OBJDIR)/autoconf.h | $(OBJDIR)
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@

# Create the output directories
$(OBJDIR):
	$(E) "  MKDIR  $(OBJDIR)"
	-$(Q)mkdir -p $(OBJDIRS)

# Target: clean project.
clean:
	$(E) "  CLEAN"
	$(Q)$(REMOVE) $(TARGET).hex
	$(Q)$(REMOVE) $(TARGET).bin
	$(Q)$(REMOVE) $(TARGET).eep
	$(Q)$(REMOVE) $(TARGET).cof
	$(Q)$(REMOVE) $(TARGET).elf
	$(Q)$(REMOVE) $(TARGET).map
	$(Q)$(REMOVE) $(TARGET).sym
	$(Q)$(REMOVE) $(TARGET).lss
	$(Q)$(REMOVE) $(OBJ)
	$(Q)$(REMOVE) $(OBJDIR)/autoconf.h
	$(Q)$(REMOVE) $(OBJDIR)/asmconfig.h
	$(Q)$(REMOVE) $(OBJDIR)/*.bin
	$(Q)$(REMOVE) $(LST)
	$(Q)$(REMOVE) $(CSRC:.c=.s)
	$(Q)$(REMOVE) $(CSRC:.c=.d)
	$(Q)$(REMOVE) .dep/*
	$(Q)$(REMOVE) -rf codedoc
	$(Q)$(REMOVE) -rf doxyinput
	-$(Q)rmdir --ignore-fail-on-non-empty -p $(OBJDIRS)

# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# Listing of phony targets.
.PHONY : all sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list program debug gdb-config doxygen

