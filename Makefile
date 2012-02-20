#  Project Name
PROJECT=mbed_test
#  List of the objects files to be compiled/assembled
OBJECTS=startup_LPC17xx.o core_cm3.o system_LPC17xx.o main_LPC17xx.o \
				ConfigDescriptor.o HIDReport.o
LSCRIPT=LPC17xx.ld

LUFA=LUFA/Drivers/USB/Class/Host/HID.o \
		 LUFA/Drivers/USB/Class/Common/HIDParser.o \
		 LUFA/Drivers/USB/Core/ConfigDescriptor.o \
		 LUFA/Drivers/USB/Core/Events.o \
		 LUFA/Drivers/USB/Core/HostStandardReq.o \
		 LUFA/Drivers/USB/Core/DeviceStandardReq.o \
		 LUFA/Drivers/USB/Core/LPC/Device_LPC.o \
		 LUFA/Drivers/USB/Core/LPC/EndpointStream_LPC.o \
		 LUFA/Drivers/USB/Core/LPC/HAL/HAL_LPC.o \
		 LUFA/Drivers/USB/Core/LPC/HAL/LPC17XX/HAL_LPC17xx.o \
		 LUFA/Drivers/USB/Core/LPC/HCD/EHCI/EHCI.o \
		 LUFA/Drivers/USB/Core/LPC/HCD/HCD.o \
		 LUFA/Drivers/USB/Core/LPC/Host_LPC.o \
		 LUFA/Drivers/USB/Core/LPC/Pipe_LPC.o \
		 LUFA/Drivers/USB/Core/LPC/PipeStream_LPC.o \
		 LUFA/Drivers/USB/Core/LPC/USBController_LPC.o \
		 LUFA/Drivers/USB/Core/USBMemory.o \
		 LUFA/Drivers/USB/Core/USBTask.o \
		 LUFA/Drivers/USB/Core/LPC/Endpoint_LPC.o \
		 LUFA/Drivers/USB/Core/LPC/HCD/OHCI/OHCI.o \
		 LUFA/Drivers/USB/Core/LPC/HAL/LPC11UXX/HAL_LPC11Uxx.o \
		 LUFA/Drivers/USB/Core/LPC/HAL/LPC18XX/HAL_LPC18xx.o \
		 LUFA/Drivers/USB/Core/LPC/DCD/LPC11UXX/Endpoint_LPC11Uxx.o \
		 LUFA/Drivers/USB/Core/LPC/DCD/LPC18XX/Endpoint_LPC18xx.o \
		 LUFA/Drivers/USB/Core/LPC/DCD/LPC17XX/Endpoint_LPC17xx.o \

#		 LUFA/Drivers/USB/Class/Device/Audio.o \
		 LUFA/Drivers/USB/Class/Device/CDC.o \
		 LUFA/Drivers/USB/Class/Device/HID.o \
		 LUFA/Drivers/USB/Class/Device/MassStorage.o \
		 LUFA/Drivers/USB/Class/Device/MIDI.o \
		 LUFA/Drivers/USB/Class/Device/RNDIS.o \
		 LUFA/Drivers/USB/Class/Host/Audio.o \
		 LUFA/Drivers/USB/Class/Host/CDC.o \
		 LUFA/Drivers/USB/Class/Host/MassStorage.o \
		 LUFA/Drivers/USB/Class/Host/MIDI.o \
		 LUFA/Drivers/USB/Class/Host/Printer.o \
		 LUFA/Drivers/USB/Class/Host/RNDIS.o \
		 LUFA/Drivers/USB/Class/Host/StillImage.o \
	
OBJECTS+=$(LUFA)

OPTIMIZATION = 0
DEBUG = -g
#LISTING = -ahls

#  Compiler Options
GCFLAGS = -Wall -fno-common -mcpu=cortex-m3 -mthumb -O$(OPTIMIZATION) $(DEBUG) -D __LPC17XX__ -std=gnu99
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

all: $(PROJECT).elf stats
	$(OBJCOPY) -O binary -j .text -j .data $(PROJECT).elf $(PROJECT).bin

$(PROJECT).elf: $(OBJECTS)
	$(GCC) $(LDFLAGS) $(OBJECTS) -o $(PROJECT).elf

startup_LPC17xx.o: startup_LPC17xx.s
	$(AS) $(ASFLAGS) -o startup_LPC17xx.o startup_LPC17xx.s > $(PROJECT)_crt.lst

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

%.o : %.c
	$(GCC) $(GCFLAGS) -c $< -o $@

.cpp.o :
	$(GCC) $(GCFLAGS) -c $< -o $@

#########################################################################
