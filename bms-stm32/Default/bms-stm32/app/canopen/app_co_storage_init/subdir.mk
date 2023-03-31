################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/app/canopen/app_co_storage_init/app_co_storage_init.c 

C_DEPS += \
./bms-stm32/app/canopen/app_co_storage_init/app_co_storage_init.d 

OBJS += \
./bms-stm32/app/canopen/app_co_storage_init/app_co_storage_init.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/app/canopen/app_co_storage_init/%.o: ../bms-stm32/app/canopen/app_co_storage_init/%.c bms-stm32/app/canopen/app_co_storage_init/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-app-2f-canopen-2f-app_co_storage_init

clean-bms-2d-stm32-2f-app-2f-canopen-2f-app_co_storage_init:
	-$(RM) ./bms-stm32/app/canopen/app_co_storage_init/app_co_storage_init.d ./bms-stm32/app/canopen/app_co_storage_init/app_co_storage_init.o

.PHONY: clean-bms-2d-stm32-2f-app-2f-canopen-2f-app_co_storage_init

