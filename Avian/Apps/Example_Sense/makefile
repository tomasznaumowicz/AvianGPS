###############################################################################
# Configuration: Application

# Name of the application and output files (up to 8 characters allowed)
APPNAME		:= Sense

# Application Version
APPVER_MAJOR	:= 1
APPVER_MINOR	:= 0

# Hardware Desc
MSB_TYPE			:= MSP430
MSB_EXT_BOARD_TYPE	:= AVIAN4

# Radio library
RADIO:=
# none yet ./System/cc1100lib/lpm3_routing.a

# Linker Script
LDSCRIPT  := ../../msp430x1611_avian.x

###############################################################################
# Configuration: Environment

INCLUDES := C:\SkomerIDE\Backend\mspgcc\msp430\include
LIBRARIES_ABSOLUTE = C:/SkomerIDE/Backend/mspgcc/msp430/lib/libmspgcc.a
LIBS := C:\SkomerIDE\Backend\mspgcc\lib

SYSTEM_DIRECTORY := ../../System
RADIO_DIRECTORY := ../../System/Radio
DRIVERS_DIRECTORY := ../../Drivers
APPLICATION_DIRECTORY := .

OUTPUT_DIRECTORY := ./Output

###############################################################################
# Microcontroller details
MCU			:= msp430x1611
GNUPREFIX	:= msp430-

###############################################################################
# Maketools
AR			:= $(GNUPREFIX)ar
CC			:= $(GNUPREFIX)gcc
OBJCOPY		:= $(GNUPREFIX)objcopy
SIZE		:= $(GNUPREFIX)size
MKDIR		:= mkdir -p

###############################################################################
# Do not modify the makefile below this line

###############################################################################
# Helpers
COMMA		:= ,
EMPTY		:=
SPACE		:= $(empty) $(empty)

CURTIME		?= $(shell "date" +%s)

# Length of application name
namelen		:= $(shell echo -n '$(strip $(APPNAME))' | wc -m)
#__APP_NAME="\"$(APPNAME)\"" __APPVER_MAJOR=$(APPVER_MAJOR) __APPVER_MINOR=$(APPVER_MINOR) 
# Infered defines
DEFS		:= __APP_NAME="\"$(APPNAME)\"" __APPVER_MAJOR=$(APPVER_MAJOR) __APPVER_MINOR=$(APPVER_MINOR) __PLATFORM_BOARD="\"$(MSB_TYPE)\"" __PLATFORM_EXTBOARD="\"$(MSB_EXT_BOARD_TYPE)\"" __COMPILE_TIME=$(CURTIME)

# Get all sources, headers and create obj file names.
SYSTEM_SOURCES := $(wildcard $(SYSTEM_DIRECTORY)/*.c)
SYSTEM_HEADERS := $(wildcard $(SYSTEM_DIRECTORY)/*.h)
SYSTEM_OBJS := $(patsubst $(SYSTEM_DIRECTORY)/%.c, $(OUTPUT_DIRECTORY)/%.o, $(SYSTEM_SOURCES))

RADIO_SOURCES := $(wildcard $(RADIO_DIRECTORY)/*.c)
RADIO_HEADERS := $(wildcard $(RADIO_DIRECTORY)/*.h)
RADIO_OBJS := $(patsubst $(RADIO_DIRECTORY)/%.c, $(OUTPUT_DIRECTORY)/%.o, $(RADIO_SOURCES))

DRIVERS_SOURCES := $(wildcard $(DRIVERS_DIRECTORY)/*.c)
DRIVERS_HEADERS := $(wildcard $(DRIVERS_DIRECTORY)/*.h)
DRIVERS_OBJS := $(patsubst $(DRIVERS_DIRECTORY)/%.c, $(OUTPUT_DIRECTORY)/%.o, $(DRIVERS_SOURCES))

APPLICATION_SOURCES := $(wildcard $(APPLICATION_DIRECTORY)/*.c)
APPLICATION_HEADERS := $(wildcard $(APPLICATION_DIRECTORY)/*.h)
APPLICATION_OBJS := $(patsubst $(APPLICATION_DIRECTORY)/%.c, $(OUTPUT_DIRECTORY)/%.o, $(APPLICATION_SOURCES))

APPLICATION_CONFIG := "./AppConfig.h"

LIBRARIES := 

OPTIMIZELEVEL := 2

.PHONY: clean

all: init $(SYSTEM_OBJS) $(RADIO_OBJS) $(DRIVERS_OBJS) $(APPLICATION_OBJS)  
	@echo ""
	@echo "Linking..."
	
	@msp430-gcc -Lout -Wl,-Map=./Output/$(APPNAME).map,--cref -Wall -Wl,-T"$(LDSCRIPT)" -mmcu=msp430x1611 -o ./Output/$(APPNAME).elf $(SYSTEM_OBJS) $(RADIO_OBJS) $(DRIVERS_OBJS) $(APPLICATION_OBJS) $(LIBRARIES) $(LIBRARIES_ABSOLUTE) -lm
	
	@$(GNUPREFIX)objcopy -O ihex "./Output/$(APPNAME).elf" "./Output/$(APPNAME).hex"
	@$(GNUPREFIX)objcopy -O binary "./Output/$(APPNAME).elf" "./Output/$(APPNAME).bin"

	@echo "Build complete."
	@echo ""
	@echo "Memory footprint:"
	@"$(SIZE)" -t $(wildcard ./Output/*.o)
	@echo ""
	@"$(SIZE)" "./Output/$(APPNAME).elf"
	
	@echo ""
	@msp430-ram-usage "./Output/$(APPNAME).elf"		
	@msp430-ram-usage "./Output/$(APPNAME).elf" -d > "./Output/$(APPNAME)-ram-usage.txt"
	@echo "Ram usage details: Output/$(APPNAME)-ram-usage.txt"	

# Quick update for eclipse autocompile
update: $(SYSTEM_OBJS) $(RADIO_OBJS) $(DRIVERS_OBJS) $(APPLICATION_OBJS) 

# All system files are built in the same way (pattern matching rule, all targets depend on their source and all firmware headers).
./Output/%.o: $(SYSTEM_DIRECTORY)/%.c $(SYSTEM_HEADERS)
	@echo " SYSTEM : $(@F)"
	@$(CC)  $(addprefix -I ,$(INCLUDES)) -include $(APPLICATION_CONFIG) -O$(OPTIMIZELEVEL) -g3 -Wall -Werror -c -fmessage-length=0 -mmcu=$(MCU) $(addprefix -D,$(DEFS)) -o $@ $<
	 
./Output/%.o: $(SYSTEM_DIRECTORY)/Radio/%.c $(SYSTEM_HEADERS) $(RADIO_HEADERS)
	@echo " RADIO  : $(@F)"
	@$(CC) $(addprefix -I ,$(INCLUDES)) -include $(APPLICATION_CONFIG) -O$(OPTIMIZELEVEL) -g3 -Wall -Werror -c -fmessage-length=0 -mmcu=$(MCU) $(addprefix -D,$(DEFS)) -o $@ $<
	 
./Output/%.o: $(DRIVERS_DIRECTORY)/%.c $(SYSTEM_HEADERS) $(RADIO_HEADERS) $(DRIVERS_HEADERS)
	@echo " DRIVER : $(@F)"
	@$(CC) $(addprefix -I ,$(INCLUDES)) -include $(APPLICATION_CONFIG) -O$(OPTIMIZELEVEL) -g3 -Wall -Werror -c -fmessage-length=0 -mmcu=$(MCU) $(addprefix -D,$(DEFS)) -o $@ $<
	 
./Output/%.o: ./%.c $(SYSTEM_HEADERS) $(RADIO_HEADERS) $(DRIVERS_HEADERS) $(APPLICATION_HEADERS)
	@echo " APP    : $(@F)"
	@$(CC) $(addprefix -I,$(INCLUDES)) -include $(APPLICATION_CONFIG) -O$(OPTIMIZELEVEL) -g3 -Wall -Werror -c -fmessage-length=0 -mmcu=$(MCU) $(addprefix -D,$(DEFS)) -o $@ $<

# Rebuild
rebuild: clean all

# Delete all objects.
clean:
	@echo "Cleaning '$(APPNAME)'"
	@$(RM) -r "Output"
	@echo "done."
	@echo ""
	
init:
ifeq (,$(findstring $(namelen),1 2 3 4 5 6 7 8))
	$(error "Application name '$(APPNAME)' invalid. Must be defined in the makefile. Max 8 chars allowed.")
endif
	@echo "Working in the directory:"
	@echo "$(abspath .)"
	@echo ""
	@echo "Building application: '$(APPNAME)' Version: $(APPVER_MAJOR).$(APPVER_MINOR)" for $(MSB_TYPE) on $(MSB_EXT_BOARD_TYPE)
	@echo -e "With defines:\n$(DEFS)"
	@echo ""
	@$(MKDIR) "Output"
	
test:
	echo "$(GENERATED_CODE_OBJS)"
	
flash:
	@echo ""
	@echo "Flashing application: '$(APPNAME)' Version: $(APPVER_MAJOR).$(APPVER_MINOR)"
	@$(GNUPREFIX)jtag -mr -lTIUSB -p "Output/$(APPNAME).HEX"
	
rbf: rebuild flash

bf: all flash

gdbproxy:
	@echo "Starting msp430-gdbproxy"
	@echo ""
	msp430-gdbproxy --port=3333 msp430 TIUSB
	
reset:
	@echo "Resetting the MSP430..."
	@echo ""
	@$(GNUPREFIX)jtag -r -lTIUSB
	
clearinfo:
	@$(GNUPREFIX)jtag -l TIUSB --erase=0x1000-0x1100
	