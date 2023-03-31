################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/component/keypad/keypad.c 

C_DEPS += \
./bms-stm32/component/keypad/keypad.d 

OBJS += \
./bms-stm32/component/keypad/keypad.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/component/keypad/%.o: ../bms-stm32/component/keypad/%.c bms-stm32/component/keypad/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-component-2f-keypad

clean-bms-2d-stm32-2f-component-2f-keypad:
	-$(RM) ./bms-stm32/component/keypad/keypad.d ./bms-stm32/component/keypad/keypad.o

.PHONY: clean-bms-2d-stm32-2f-component-2f-keypad

