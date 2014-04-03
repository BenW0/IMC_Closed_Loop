CLOCK = 48000000

TEENSY_PATH = ..
COMPILER = $(TEENSY_PATH)/hardware/tools/arm-none-eabi/bin
VENDOR = $(TEENSY_PATH)/hardware/vendor/


CPPFLAGS = -Wall -g -Os -mcpu=cortex-m4 -mthumb -nostdlib -MMD -DF_CPU=$(CLOCK) -DUSB_SERIAL -I$(VENDOR) -D__MK20DX256__
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti
CFLAGS =
LDFLAGS = -Os -Wl,--gc-sections -mcpu=cortex-m4 -mthumb -T$(VENDOR)/mk20dx256.ld
LIBS = -lm
CC = $(COMPILER)/arm-none-eabi-gcc
CXX = $(COMPILER)/arm-none-eabi-g++
OBJCOPY = $(COMPILER)/arm-none-eabi-objcopy
SIZE = $(COMPILER)/arm-none-eabi-size

OBJECTS = main.o hardware.o stepper.o qdenc.o

VENDOR_C = $(wildcard $(VENDOR)/*.c)
VENDOR_OBJECTS = $(patsubst %.c,%.o,$(VENDOR_C))

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@


main.elf: $(OBJECTS) $(VENDOR_OBJECTS) 
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS) $(VENDOR_OBJECTS) $(LIBS) 

-include $(OBJS:.o=.d)

all: main.hex

clean:
	rm -f *.o *.d *.elf *.hex

