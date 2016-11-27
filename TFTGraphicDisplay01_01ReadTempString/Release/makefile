################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -f

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include libraries/UTFT_Buttons/subdir.mk
-include libraries/UTFT/subdir.mk
-include libraries/URTouch/subdir.mk
-include core/variant/subdir.mk
-include core/core/avr/subdir.mk
-include core/core/USB/subdir.mk
-include core/core/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
arduino.ar \

ELF += \
TFTGraphicDisplay01_01ReadTempString.elf \

HEX += \
TFTGraphicDisplay01_01ReadTempString.hex \


# All Target
all: TFTGraphicDisplay01_01ReadTempString

# Tool invocations
TFTGraphicDisplay01_01ReadTempString: $(HEX)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"/usr/lib64/eclipse/arduinoPlugin/tools/arduino/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-size" -A "/home/rmelendez/git/SeniorDesign406/TFTGraphicDisplay01_01ReadTempString/Release/TFTGraphicDisplay01_01ReadTempString.elf"
	@echo 'Finished building target: $@'
	@echo ' '

arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"/usr/lib64/eclipse/arduinoPlugin/tools/arduino/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs "/home/rmelendez/git/SeniorDesign406/TFTGraphicDisplay01_01ReadTempString/Release/arduino.ar"  arduino.ar $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

TFTGraphicDisplay01_01ReadTempString.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"/usr/lib64/eclipse/arduinoPlugin/tools/arduino/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m3 -mthumb -Os -Wl,--gc-sections "-T/usr/lib64/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/variants/arduino_due_x/linker_scripts/gcc/flash.ld" "-Wl,-Map,/home/rmelendez/git/SeniorDesign406/TFTGraphicDisplay01_01ReadTempString/Release/TFTGraphicDisplay01_01ReadTempString.map" -o "/home/rmelendez/git/SeniorDesign406/TFTGraphicDisplay01_01ReadTempString/Release/TFTGraphicDisplay01_01ReadTempString.elf" "-L/home/rmelendez/git/SeniorDesign406/TFTGraphicDisplay01_01ReadTempString/Release" -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--start-group "/home/rmelendez/git/SeniorDesign406/TFTGraphicDisplay01_01ReadTempString/Release/core/syscalls_sam3.c.o"  $(LINK_OBJ) $(AR) "/usr/lib64/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/variants/arduino_due_x/libsam_sam3x8e_gcc_rel.a"  -Wl,--end-group -lm -gcc
	@echo 'Finished building: $@'
	@echo ' '

TFTGraphicDisplay01_01ReadTempString.hex: $(ELF)
	@echo 'Create eeprom image'
	${A.RECIPE.OBJCOPY.BIN.PATTERN}
	${A.RECIPE.OBJCOPY.HEX.PATTERN}
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(PDE_DEPS)$(C_DEPS)$(AR)$(CC_DEPS)$(AR_OBJ)$(C++_DEPS)$(LINK_OBJ)$(CXX_DEPS)$(ASM_DEPS)$(HEX)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(ELF)$(CPP_DEPS)
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
