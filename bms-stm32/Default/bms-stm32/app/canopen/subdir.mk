################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/app/canopen/canopen_init.c 

O_SRCS += \
../bms-stm32/app/canopen/canopen_init.o 

C_DEPS += \
./bms-stm32/app/canopen/canopen_init.d 

OBJS += \
./bms-stm32/app/canopen/canopen_init.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/app/canopen/%.o: ../bms-stm32/app/canopen/%.c bms-stm32/app/canopen/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-app-2f-canopen

clean-bms-2d-stm32-2f-app-2f-canopen:
	-$(RM) ./bms-stm32/app/canopen/canopen_init.d ./bms-stm32/app/canopen/canopen_init.o

.PHONY: clean-bms-2d-stm32-2f-app-2f-canopen

