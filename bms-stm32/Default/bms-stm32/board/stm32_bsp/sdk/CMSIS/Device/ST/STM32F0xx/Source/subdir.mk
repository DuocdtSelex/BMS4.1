################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f030.s \
../bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f031.s \
../bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f042.s \
../bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f051.s \
../bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f072.s \
../bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f0xx.s 

S_DEPS += \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f030.d \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f031.d \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f042.d \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f051.d \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f072.d \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f0xx.d 

OBJS += \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f030.o \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f031.o \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f042.o \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f051.o \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f072.o \
./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f0xx.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/%.o: ../bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/%.s bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0 -g -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-bms-2d-stm32-2f-board-2f-stm32_bsp-2f-sdk-2f-CMSIS-2f-Device-2f-ST-2f-STM32F0xx-2f-Source

clean-bms-2d-stm32-2f-board-2f-stm32_bsp-2f-sdk-2f-CMSIS-2f-Device-2f-ST-2f-STM32F0xx-2f-Source:
	-$(RM) ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f030.d ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f030.o ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f031.d ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f031.o ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f042.d ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f042.o ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f051.d ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f051.o ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f072.d ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f072.o ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f0xx.d ./bms-stm32/board/stm32_bsp/sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f0xx.o

.PHONY: clean-bms-2d-stm32-2f-board-2f-stm32_bsp-2f-sdk-2f-CMSIS-2f-Device-2f-ST-2f-STM32F0xx-2f-Source

