################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/component/ext_flash/ext_flash.c 

C_DEPS += \
./bms-stm32/component/ext_flash/ext_flash.d 

OBJS += \
./bms-stm32/component/ext_flash/ext_flash.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/component/ext_flash/%.o: ../bms-stm32/component/ext_flash/%.c bms-stm32/component/ext_flash/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-component-2f-ext_flash

clean-bms-2d-stm32-2f-component-2f-ext_flash:
	-$(RM) ./bms-stm32/component/ext_flash/ext_flash.d ./bms-stm32/component/ext_flash/ext_flash.o

.PHONY: clean-bms-2d-stm32-2f-component-2f-ext_flash

