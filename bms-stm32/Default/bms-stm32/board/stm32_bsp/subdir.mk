################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/board/stm32_bsp/stm32_bsp.c \
../bms-stm32/board/stm32_bsp/stm32f0xx_it.c \
../bms-stm32/board/stm32_bsp/system_stm32f0xx.c 

C_DEPS += \
./bms-stm32/board/stm32_bsp/stm32_bsp.d \
./bms-stm32/board/stm32_bsp/stm32f0xx_it.d \
./bms-stm32/board/stm32_bsp/system_stm32f0xx.d 

OBJS += \
./bms-stm32/board/stm32_bsp/stm32_bsp.o \
./bms-stm32/board/stm32_bsp/stm32f0xx_it.o \
./bms-stm32/board/stm32_bsp/system_stm32f0xx.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/board/stm32_bsp/%.o: ../bms-stm32/board/stm32_bsp/%.c bms-stm32/board/stm32_bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-board-2f-stm32_bsp

clean-bms-2d-stm32-2f-board-2f-stm32_bsp:
	-$(RM) ./bms-stm32/board/stm32_bsp/stm32_bsp.d ./bms-stm32/board/stm32_bsp/stm32_bsp.o ./bms-stm32/board/stm32_bsp/stm32f0xx_it.d ./bms-stm32/board/stm32_bsp/stm32f0xx_it.o ./bms-stm32/board/stm32_bsp/system_stm32f0xx.d ./bms-stm32/board/stm32_bsp/system_stm32f0xx.o

.PHONY: clean-bms-2d-stm32-2f-board-2f-stm32_bsp

