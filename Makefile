# The following makefile is used to build a project on atmega382p, which is mounted on an arduino uno
# The programmer is already on the development board.

# Directories
AVRGCC_ROOT_DIR = /home/karzan/dev/tools/avr-gnu-gcc
AVRGCC_BIN_DIR = $(AVRGCC_ROOT_DIR)/bin
AVRGCC_INCLUDE_DIR = $(AVRGCC_ROOT_DIR)/avr/include
PORT_DIR = /home/kazanm/dev/ttyACM0
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj
BIN_DIR = $(BUILD_DIR)/bin

# AVRDUDE  variables
PROGRAMMER = arduino
CHIP = ATMEGA328P
PORT = /dev/ttyACM0
BAUD = 115200
MEMTYPE = flash

# Toolchain
CC = $(AVRGCC_BIN_DIR)/avr-gcc
CPPCHECK = cppcheck

# Files
TARGET = $(BIN_DIR)/blink

SOURCES = main.c
# 	\ append src.c for another source file

OBJECT_NAMES = $(SOURCES:.c=.o)
OBJECTS = $(patsubst %, $(OBJ_DIR)/%, $(OBJECT_NAMES))

# Flags
## compiler
MCU = atmega328p
WFLAGS = -Wall -Wextra -Werror -Wshadow
CFLAGS = -mmcu=$(MCU) -g -Os -DF_CPU=16000000U $(WFLAGS) $(addprefix -I, $(AVRGCC_INCLUDE_DIR)) 
LDFLAGS = -g -mmcu=$(MCU) 
#$(addprefix -L,$(AVRGCC_INCLUDE_DIR))
##  linker
PROFLAGS = -c $(PROGRAMMER)
MCUFLAGS = -p $(CHIP) -P $(PORT) -b $(BAUD)
MEMFLAGS = -F -V

# Build
## Flashing
$(TARGET).hex: $(TARGET).elf
	@echo "Generating flashable hex file..."
	avr-objcopy -O ihex -j .text -j .data $< $@
	@echo "...done!"

## Linking
$(TARGET).elf: $(OBJ_DIR)/main.o
	@mkdir -p $(dir $@)
	@echo "Linking..."
	$(CC) $(LDFLAGS) -o $@ $<
	@echo " ...done! \n"

## Compiling
$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "Compiling..."
	$(CC) $(CFLAGS) -c -o $@  $^
	@echo "...done! \n"

# Phonies
.PHONY: all clean flash cppcheck

all: $(TARGET) 

clean:
	@rm -rf $(BUILD_DIR) \
	$(info removed build directory)

flash: 
	@echo "Flashing..."
	avrdude $(MEMFLAGS) $(PROFLAGS) $(MCUFLAGS) -Uflash:w:$(TARGET).hex
	@echo "...done!"

cppcheck:
	@$(CPPCHECK) --quiet --enable=all --error-exitcode=1 \
	--inline-suppr \
	--suppress=missingInclude \
	$(SOURCES) 
