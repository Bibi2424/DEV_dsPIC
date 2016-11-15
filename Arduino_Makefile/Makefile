# Arduino makefile
#
# This makefile allows you to build sketches from the command line
# without the Arduino environment (or Java).
#
# The Makefile is doing the same steps as the Arduino IDE for compiling.
# It creates the core.a lib from all needed arduino source code and
# arduino librairies.
# It copy the .ino file into a .cpp one, add the prototypes for the
# setup and loop functions and include the Arduino.h header.
# It then compile your source file and link everything together with
# the core.a lib.
#
# Instructions for using the makefile:
#
#  1. Copy this file into the folder with your sketch.
#
#  2. Modify all needed line from "ADJUST FOR EVERY PROJECT"
#
#  3. Type "make build" to compile/verify your program.
#
#     Type "make upload" to upload your program to the Arduino board.
#
#	  Type "make" to build and upload the code.
#
#	  Type "make clean" to clean the intermediate file in both your 
#	  working folder and the arduino folder that came from the compilation
# 	  process. Only the .hex file is kept.
#
# $Id$

#########################################################################
#																		#
#	ADJUST FOR EVERY PROJECT											#
#																		#
#########################################################################

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# MCU Specifications
MCU = atmega328p
F_CPU = 16000000
FORMAT = ihex

# Path needed
ARDUINO = D:\Programmes\Arduino
ARDUINO_CORE = $(ARDUINO)\hardware\arduino\avr\cores\arduino
ARDUINO_VAR = $(ARDUINO)\hardware\arduino\avr\variants\standard
ARDUINO_LIB = $(ARDUINO)\libraries
AVR_TOOLS_PATH = $(ARDUINO)\hardware\tools\avr\bin
AVRDUDE_PATH = $(ARDUINO)\hardware\tools\avr\bin
AVRDUD_CONF = $(ARDUINO)\hardware\tools\avr\etc\avrdude.conf


# Project Specifications
TARGET = Blink
OUTPUT_DIR = .dep
# append the .c source file on the line C_APP and the .cpp in the CXX_APP
C_APP = 
CXX_APP = $(OUTPUT_DIR)/$(TARGET).cpp
# append the name of the user libs used (Arduino\librairies folder)
LIBS = 


# Programming support using avrdude. Settings and variables.
AVRDUDE_PROGRAMMER = arduino
AVRDUDE_PORT = COM5
AVRDUDE_BAUDRATE = 115200
AVRDUDE_WRITE_FLASH = -U flash:w:$(OUTPUT_DIR)/$(TARGET).hex
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) \
				-b $(AVRDUDE_BAUDRATE) -C $(AVRDUD_CONF)

#########################################################################
#																		#
#	SAME FOR EVERY PROJECT												#
#																		#
#########################################################################

# Arduino .c source code and librairies
C_MODULES =  $(wildcard $(ARDUINO_CORE)/*.c)
C_MODULES += $(foreach d, $(LIBS), $(wildcard $(ARDUINO_LIB)\$d/*.c))
C_MODULES += $(foreach d, $(LIBS), $(wildcard $(ARDUINO_LIB)\$d/**/*.c))

# Arduino .cpp source code and librairies
CXX_MODULES =  $(wildcard $(ARDUINO_CORE)/*.cpp)
CXX_MODULES += $(foreach d, $(LIBS), $(wildcard $(ARDUINO_LIB)\$d/*.cpp))
CXX_MODULES += $(foreach d, $(LIBS), $(wildcard $(ARDUINO_LIB)\$d/**/*.cpp))

MODULES = $(C_MODULES) $(CXX_MODULES)
SRC = $(C_MODULES) $(C_APP)
CXXSRC = $(CXX_MODULES) $(CXX_APP)

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = stabs

OPT = s

# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU) -DARDUINO=105
CXXDEFS = -DF_CPU=$(F_CPU) -DARUINO=105

# Place -I options here
ARDUINO_INCLUDE_LIB =  $(foreach d, $(LIBS), $(ARDUINO_LIB)\$d)
ARDUINO_INCLUDE_LIB += $(foreach d, $(LIBS), $(ARDUINO_LIB)\$d\utility)
ARDUINO_INCLUDE_LIB += $(foreach d, $(LIBS), $(ARDUINO_LIB)\$d\src)

CINCS = -I$(ARDUINO_CORE) -I$(ARDUINO_VAR)
CINCS += $(foreach d, $(ARDUINO_INCLUDE_LIB), -I$d)
CXXINCS = -I$(ARDUINO_CORE) $(ARDUINO_INCLUDE_LIB)
CXXINCS += $(foreach d, $(ARDUINO_INCLUDE_LIB), -I$d)
 
# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=c99
CDEBUG = -g$(DEBUG)
#CWARN = -Wall -Wstrict-prototypes
CWARN = -Wall 
CTUNING = -ffunction-sections -fdata-sections
CXXTUNING = -fno-exceptions -ffunction-sections -fdata-sections
#CEXTRA = -Wa,-adhlns=$(<:.c=.lst)
CSTRING = -Wno-write-strings


CFLAGS = $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CSTANDARD) $(CEXTRA)
CXXFLAGS = $(CDEFS) $(CINCS) -O$(OPT) $(CSTRING)
ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 
LDFLAGS = -O$(OPT) -Wl,--gc-sections

# Program settings
CC = $(AVR_TOOLS_PATH)\avr-gcc
CXX = $(AVR_TOOLS_PATH)\avr-g++
LD = $(AVR_TOOLS_PATH)\avr-gcc
OBJCOPY = $(AVR_TOOLS_PATH)\avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)\avr-objdump
AR  = $(AVR_TOOLS_PATH)\avr-ar
SIZE = avr-size
NM = $(AVR_TOOLS_PATH)\avr-nm
AVRDUDE = $(AVRDUDE_PATH)\avrdude
REMOVE = rm -f
MV = mv -f

# Define all object files.
OBJ = $(SRC:.c=.o) $(CXXSRC:.cpp=.o) $(ASRC:.S=.o)
OBJ_APP = $(C_APP:.c=.o) $(CXX_APP:.cpp=.o)
OBJ_MODULES = $(C_MODULES:.c=.o) $(CXX_MODULES:.cpp=.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(CXXSRC:.cpp=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)
ALL_CXXFLAGS = -mmcu=$(MCU) -I. $(CXXFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)
ALL_LDFLAGS = $(LDFLAGS) -mmcu=$(MCU)


# Default target.
all: build sizeafter upload

build: applet_files elf hex
	@echo ""
	@echo "Compilation Finished !! Congrats"
	@echo
	@echo "-------------------------------------------------------------------"
	@echo

applet_files: $(TARGET).ino
#$(OUTPUT_DIR)/$(TARGET).cpp: $(TARGET).ino
	@test -d $(OUTPUT_DIR) || mkdir $(OUTPUT_DIR)
	@echo '#include "Arduino.h"' > $(OUTPUT_DIR)/$(TARGET).cpp
	@echo 'void setup();' >> $(OUTPUT_DIR)/$(TARGET).cpp
	@echo 'void loop();' >> $(OUTPUT_DIR)/$(TARGET).cpp
	@cat $(TARGET).ino >> $(OUTPUT_DIR)/$(TARGET).cpp
	@echo "-------------------------------------------------------------------"
	@echo "---------------------  MAKING $(TARGET)  ---------------------"
	@echo "-------------------------------------------------------------------"
	@echo
	@echo "Start Compiling ..."
	@echo

elf: $(OUTPUT_DIR)/$(TARGET).elf
hex: $(OUTPUT_DIR)/$(TARGET).hex
eep: $(OUTPUT_DIR)/$(TARGET).eep
lss: $(OUTPUT_DIR)/$(TARGET).lss
sym: $(OUTPUT_DIR)/$(TARGET).sym

# Program the device.  
upload: 
	@echo
	@echo "-------------------------------------------------------------------"
	@echo
	@echo "Start Programming..."
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)
	@echo "-------------------------------------------------------------------"
	@echo "-------------------------------------------------------------------"
	
#
# Display size of file.
#
HEXSIZE = $(SIZE) --target=$(FORMAT) $(OUTPUT_DIR)/$(TARGET).elf
ELFSIZE = $(SIZE) -C $(OUTPUT_DIR)/$(TARGET).elf
sizebefore:
	@if [ -f $(OUTPUT_DIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(HEXSIZE); echo; fi

sizeafter:
	@echo
	@echo "Code size :"
	@if [ -f $(OUTPUT_DIR)/$(TARGET).elf ]; then echo $(MSG_SIZE_AFTER); $(HEXSIZE); echo; fi


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 


coff: $(OUTPUT_DIR)/$(TARGET).elf
	$(COFFCONVERT) -O coff-avr $(OUTPUT_DIR)/$(TARGET).elf $(TARGET).cof


extcoff: $(TARGET).elf
	$(COFFCONVERT) -O coff-ext-avr $(OUTPUT_DIR)/$(TARGET).elf $(TARGET).cof


.SUFFIXES: .elf .hex .eep .lss .sym

.elf.hex:
	@echo ""
	@echo "Creating HEX file : $@"
	@$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@



# Link: create ELF output file from object files.
$(OUTPUT_DIR)/$(TARGET).elf: $(OBJ_APP) $(OUTPUT_DIR)/core.a 
	@echo
	@echo "Linking : $@"
	@$(LD) $(ALL_LDFLAGS) -o $@ $(OBJ_APP) $(OUTPUT_DIR)/core.a -L core -lm

$(OUTPUT_DIR)/core.a: $(OBJ_MODULES)
	@echo "Done Compiling"
	@echo
	@echo "-------------------------------------------------------------------"
	@echo
	@echo "Making core.a Arduino librairies with following files :"
	@$(AR) rcsv $(OUTPUT_DIR)/core.a $(OBJ_MODULES)
	@echo "Librairie $@ created"
	@echo
	@echo "-------------------------------------------------------------------"
	@echo

# Compile: create object files from C++ source files.
.cpp.o:
	@echo "Compiling : $<"
	@$(CXX) -c $(ALL_CXXFLAGS) $< -o $@ 

# Compile: create object files from C source files.
.c.o:
	@echo "Compiling : $<"
	@$(CC) -c $(ALL_CFLAGS) $< -o $@ 

# Compile: create assembler files from C source files.
.c.s:
	@echo "Compiling : $<"
	@$(CC) -S $(ALL_CFLAGS) $< -o $@

# Assemble: create object files from assembler source files.
.S.o:
	@echo "Assembling : $<"
	@$(CC) -c $(ALL_ASFLAGS) $< -o $@

#
# Target: clean project.
#
clean:
	@$(REMOVE) $(OUTPUT_DIR)/$(TARGET).cpp $(OUTPUT_DIR)/$(TARGET).eep $(OUTPUT_DIR)/$(TARGET).cof \
  $(OUTPUT_DIR)/$(TARGET).elf $(OUTPUT_DIR)/$(TARGET).map $(OUTPUT_DIR)/$(TARGET).sym \
  $(OUTPUT_DIR)/$(TARGET).lss $(OUTPUT_DIR)/core.a \
  $(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d)
	@echo "Project $(TARGET) cleaned"

clean_all: clean
	@$(REMOVE) -r $(OUTPUT_DIR)

.PHONY: all build elf hex eep lss sym program coff extcoff applet_files sizebefore sizeafter

