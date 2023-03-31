################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/board/stm32_bsp/hc05_hw/hc05_hw.c 

C_DEPS += \
./bms-stm32/board/stm32_bsp/hc05_hw/hc05_hw.d 

OBJS += \
./bms-stm32/board/stm32_bsp/hc05_hw/hc05_hw.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/board/stm32_bsp/hc05_hw/%.o: ../bms-stm32/board/stm32_bsp/hc05_hw/%.c bms-stm32/board/stm32_bsp/hc05_hw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-board-2f-stm32_bsp-2f-hc05_hw

clean-bms-2d-stm32-2f-board-2f-stm32_bsp-2f-hc05_hw:
	-$(RM) ./bms-stm32/board/stm32_bsp/hc05_hw/hc05_hw.d ./bms-stm32/board/stm32_bsp/hc05_hw/hc05_hw.o

.PHONY: clean-bms-2d-stm32-2f-board-2f-stm32_bsp-2f-hc05_hw

