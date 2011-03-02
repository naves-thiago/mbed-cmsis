#  Project Name
PROJECT=mbed_test
#  List of the objects files to be compiled/assembled
OBJECTS=startup_LPC17xx.o core_cm3.o system_LPC17xx.o main_LPC17xx.o 
LSCRIPT=LPC17xx.ld

OPTIMIZATION = 0
DEBUG = -g
#LISTING = -ahls

#  Compiler Options
GCFLAGS = -Wall -fno-common -mcpu=cortex-m3 -mthumb -O$(OPTIMIZATION) $(DEBUG)
#GCFLAGS += -Wcast-align -Wcast-qual -Wimplicit -Wpointer-arith -Wswitch
#GCFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
LDFLAGS = -mcpu=cortex-m3 -mthumb -O$(OPTIMIZATION) -nostartfiles -Wl,-Map=$(PROJECT).map -T$(LSCRIPT)
ASFLAGS = $(LISTING) -mcpu=cortex-m3

#  Compiler/Assembler/Linker Paths
GCC = arm-eabi-gcc
AS = arm-eabi-as
LD = arm-eabi-ld
OBJCOPY = arm-eabi-objcopy
REMOVE = rm -f
SIZE = arm-eabi-size

#########################################################################

#all:: $(PROJECT).hex $(PROJECT).bin
$(phony all): $(PROJECT).hex $(PROJECT).bin %.c %.cpp %.h

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary -j .text -j .data $(PROJECT).elf $(PROJECT).bin

$(PROJECT).hex: $(PROJECT).elf
	$(OBJCOPY) -R .stack -O ihex $(PROJECT).elf $(PROJECT).hex

$(PROJECT).elf: $(OBJECTS)
	$(GCC) $(LDFLAGS) $(OBJECTS) -o $(PROJECT).elf

stats: $(PROJECT).elf
	$(SIZE) $(PROJECT).elf

clean:
	$(REMOVE) $(OBJECTS)
	$(REMOVE) $(PROJECT).hex
	$(REMOVE) $(PROJECT).elf
	$(REMOVE) $(PROJECT).map
	$(REMOVE) $(PROJECT).bin
	$(REMOVE) $(OBJECTS)

#########################################################################
#  Default rules to compile .c and .cpp file to .o
#  and assemble .s files to .o

%.o : %.c %.h
	$(GCC) $(GCFLAGS) -c $<

.cpp.o :
	$(GCC) $(GCFLAGS) -c $<

.S.o :
	$(AS) $(ASFLAGS) -o $(PROJECT)_crt.o $< > $(PROJECT)_crt.lst

#########################################################################
