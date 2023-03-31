################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
ELF_SRCS += \
../bms-stm32/selex_bms.elf 

C_SRCS += \
../bms-stm32/main.c 

C_DEPS += \
./bms-stm32/main.d 

OBJS += \
./bms-stm32/main.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/%.o: ../bms-stm32/%.c bms-stm32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32

clean-bms-2d-stm32:
	-$(RM) ./bms-stm32/main.d ./bms-stm32/main.o

.PHONY: clean-bms-2d-stm32

