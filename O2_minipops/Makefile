## Halfhearted attempt to make an Arduino-alike build in a Makefile
## Following from my talk "Arduino without Arduino" 
## 11 May, 2015 
## This is quick and dirty, but works for me. :)

## "make" will compile the core libs and link your code against them
## "make flash" will create a hex file and upload it to your Arduino

## You may need to configure some stuff.

## This is the name of the file (drop the .ino) you're trying to build
TARGET = O2

## The location of arduino install.
UNAME	= $(shell uname)

ifeq ($(UNAME), Linux)
    ARDUINO_INSTALL = /usr/share/arduino
else ifeq ($(UNAME), FreeBSD)
    ARDUINO_INSTALL = /usr/share/local/arduino
else ifeq ($(UNAME), Darwin)
    ARDUINO_INSTALL = /Applications/Arduino.app/Contents/Java
endif

## These are standard subdirectories in Arduino.  This should work.
ARDUINO_TOOLS   = $(ARDUINO_INSTALL)/hardware/tools/avr/bin
CORE            = $(ARDUINO_INSTALL)/hardware/arduino/avr/cores/arduino
VARIANT         = $(ARDUINO_INSTALL)/hardware/arduino/avr/variants/standard

## These tools come with Arduino.  This should work.
## If not, you may need to specify the paths by hand
## Windows folks may need to append ".exe" for all I know.
CC      = $(ARDUINO_TOOLS)/avr-gcc
CXX     = $(ARDUINO_TOOLS)/avr-g++
AR      = $(ARDUINO_TOOLS)/avr-ar
OBJCOPY = $(ARDUINO_TOOLS)/avr-objcopy
OBJDUMP = $(ARDUINO_TOOLS)/avr-objdump
AVRSIZE = $(ARDUINO_TOOLS)/avr-size
AVRDUDE = $(ARDUINO_TOOLS)/avrdude

## Processor type and speed
MCU   = atmega328p
F_CPU = 16000000L

## To flash, you may need to change the -P port to match your system
## e.g. COM3 or /dev/tty.usbserialxxxxx or whatever
AVRDUDE_CONFIG  = $(ARDUINO_INSTALL)/hardware/tools/avr/etc/avrdude.conf
AVRDUDE_OPTIONS = -C $(AVRDUDE_CONFIG) -p $(MCU) -c arduino -P /dev/ttyACM0 -b 115200 -D
## AVRDUDE_OPTIONS = -C $(AVRDUDE_CONFIG) -p $(MCU) -c avrftdi

## These are the flags that the Arduino IDE compiles with.  They seem reasonable.
## -DARDUINO_AVR_UNO is specific to the Uno board
CFLAGS  = -c -g -Os -w -ffunction-sections -fdata-sections
CFLAGS += -MMD -mmcu=$(MCU) -DF_CPU=$(F_CPU)
CFLAGS += -DARDUINO=10604 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR
CFLAGS += -I. -I$(VARIANT) -I$(CORE)

CXXFLAGS  = -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics
CXXFLAGS += -MMD -mmcu=$(MCU) -DF_CPU=$(F_CPU)
CXXFLAGS += -DARDUINO=10604 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR
CXXFLAGS += -I. -I$(VARIANT) -I$(CORE)


######################### You shouldn't have to edit anything below here
######################### but have a look, because this is where the action's at.

## This sets up a virtual path to the core library
VPATH = $(CORE)

## Core library depends on every c/c++ file in the $(CORE) directory:
CORE_SOURCES       = $(wildcard $(CORE)/*.c $(CORE)/*.cpp)
CORE_OBJECTS       = $(addsuffix .o, $(basename $(CORE_SOURCES)))
LOCAL_CORE_OBJECTS = $(notdir $(CORE_OBJECTS))
HEADERS            = $(wildcard $(CORE)/*.h) $(VARIANT)/pins_arduino.h

all: $(TARGET).hex core.a flash

## Build Arduino core library from all of the object files
## Only rebuilds core when its source changes, cleans up the related object files
## If you want to force a core rebuild, just remove core.a
core.a: $(CORE_SOURCES) 
	@echo "---------- Rebuilding the core library"
	$(MAKE) $(LOCAL_CORE_OBJECTS)
	$(AR) rcs core.a $(LOCAL_CORE_OBJECTS)
	rm -f $(LOCAL_CORE_OBJECTS:.o=.d)
	rm -f $(LOCAL_CORE_OBJECTS)

## The Arduino .ino file is just a cpp file without the include files and function prototypes
## If you've defined other functions or linked to other libraries, you'll need to add them manually.
%.cpp: %.ino
	@echo "---------- Making $@ from $< and compiling"
	@echo "#include \"Arduino.h\"" | cat > $@
	@echo "void setup();"          | cat >> $@
	@echo "void loop();"           | cat >> $@
	@cat $< >> $@

# Build our target file's object
$(TARGET).o: $(TARGET).cpp

# Link target object against core.a library
$(TARGET).elf: $(TARGET).o core.a
	@echo "---------- Linking $< to core library"
	$(CC) -w -Os -Wl,--gc-sections -mmcu=$(MCU) -o $@ $< core.a -L. -lm	

$(TARGET).hex: $(TARGET).elf
	@echo "---------- Creating hex file: $@"
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(TARGET).eeprom: $(TARGET).elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ 

flash: $(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_OPTIONS) -U $<
	pyterm /dev/ttyACM0


clean:
	rm -f *.o
	rm -f *.d
	rm -f $(TARGET).cpp
	rm -f $(TARGET).elf
	rm -f $(TARGET).hex

