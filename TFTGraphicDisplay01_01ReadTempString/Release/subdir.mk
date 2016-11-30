################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
INO_SRCS += \
..\TFTGraphicDisplay01_01ReadTempString.ino 

CPP_SRCS += \
..\.ino.cpp 

C_SRCS += \
..\Ubuntu.c 

C_DEPS += \
.\Ubuntu.c.d 

LINK_OBJ += \
.\.ino.cpp.o \
.\Ubuntu.c.o 

INO_DEPS += \
.\TFTGraphicDisplay01_01ReadTempString.ino.d 

CPP_DEPS += \
.\.ino.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
.ino.cpp.o: ../.ino.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\Users\Roberto\eclipse\arduinoPlugin\tools\arduino\arm-none-eabi-gcc\4.8.3-2014q1/bin/arm-none-eabi-g++" -c -g -Os -std=gnu++11 -ffunction-sections -fdata-sections -nostdlib -fno-threadsafe-statics --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -MMD -mcpu=cortex-m3 -mthumb -DF_CPU=84000000L -DARDUINO=10609 -DARDUINO_SAM_DUE -DARDUINO_ARCH_SAM -D__SAM3X8E__ -mthumb -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON "-DUSB_MANUFACTURER=\"Unknown\"" "-DUSB_PRODUCT=\"Arduino Due\"" "-IC:/Users/Roberto/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/system/libsam" "-IC:/Users/Roberto/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/system/CMSIS/CMSIS/Include/" "-IC:/Users/Roberto/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/system/CMSIS/Device/ATMEL/"  -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\cores\arduino" -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\variants\arduino_due_x" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\UTFT" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\URTouch" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\UTFT_Buttons" -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\libraries\SPI" -I"C:\Users\Roberto\eclipse\arduinoPlugin\libraries\SD\1.0.8" -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\libraries\SPI\src" -I"C:\Users\Roberto\eclipse\arduinoPlugin\libraries\SD\1.0.8\src" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\br3ttb-Arduino-PID-Library-fb095d8" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

TFTGraphicDisplay01_01ReadTempString.o: ../TFTGraphicDisplay01_01ReadTempString.ino
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\Users\Roberto\eclipse\arduinoPlugin\tools\arduino\arm-none-eabi-gcc\4.8.3-2014q1/bin/arm-none-eabi-g++" -c -g -Os -std=gnu++11 -ffunction-sections -fdata-sections -nostdlib -fno-threadsafe-statics --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -MMD -mcpu=cortex-m3 -mthumb -DF_CPU=84000000L -DARDUINO=10609 -DARDUINO_SAM_DUE -DARDUINO_ARCH_SAM -D__SAM3X8E__ -mthumb -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON "-DUSB_MANUFACTURER=\"Unknown\"" "-DUSB_PRODUCT=\"Arduino Due\"" "-IC:/Users/Roberto/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/system/libsam" "-IC:/Users/Roberto/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/system/CMSIS/CMSIS/Include/" "-IC:/Users/Roberto/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/system/CMSIS/Device/ATMEL/"  -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\cores\arduino" -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\variants\arduino_due_x" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\UTFT" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\URTouch" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\UTFT_Buttons" -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\libraries\SPI" -I"C:\Users\Roberto\eclipse\arduinoPlugin\libraries\SD\1.0.8" -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\libraries\SPI\src" -I"C:\Users\Roberto\eclipse\arduinoPlugin\libraries\SD\1.0.8\src" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\br3ttb-Arduino-PID-Library-fb095d8" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

Ubuntu.c.o: ../Ubuntu.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"C:\Users\Roberto\eclipse\arduinoPlugin\tools\arduino\arm-none-eabi-gcc\4.8.3-2014q1/bin/arm-none-eabi-gcc" -c -g -Os -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -Dprintf=iprintf -MMD -mcpu=cortex-m3 -mthumb -DF_CPU=84000000L -DARDUINO=10609 -DARDUINO_SAM_DUE -DARDUINO_ARCH_SAM -D__SAM3X8E__ -mthumb -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON "-DUSB_MANUFACTURER=\"Unknown\"" "-DUSB_PRODUCT=\"Arduino Due\"" "-IC:/Users/Roberto/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/system/libsam" "-IC:/Users/Roberto/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/system/CMSIS/CMSIS/Include/" "-IC:/Users/Roberto/eclipse/arduinoPlugin/packages/arduino/hardware/sam/1.6.7/system/CMSIS/Device/ATMEL/"  -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\cores\arduino" -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\variants\arduino_due_x" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\UTFT" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\URTouch" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\UTFT_Buttons" -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\libraries\SPI" -I"C:\Users\Roberto\eclipse\arduinoPlugin\libraries\SD\1.0.8" -I"C:\Users\Roberto\eclipse\arduinoPlugin\packages\arduino\hardware\sam\1.6.7\libraries\SPI\src" -I"C:\Users\Roberto\eclipse\arduinoPlugin\libraries\SD\1.0.8\src" -I"C:\Users\Roberto\Documents\GitHub\SeniorDesign406\libraries\br3ttb-Arduino-PID-Library-fb095d8" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '


