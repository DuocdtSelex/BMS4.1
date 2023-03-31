################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/service/auth_service/auth_service.c 

C_DEPS += \
./bms-stm32/service/auth_service/auth_service.d 

OBJS += \
./bms-stm32/service/auth_service/auth_service.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/service/auth_service/%.o: ../bms-stm32/service/auth_service/%.c bms-stm32/service/auth_service/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-service-2f-auth_service

clean-bms-2d-stm32-2f-service-2f-auth_service:
	-$(RM) ./bms-stm32/service/auth_service/auth_service.d ./bms-stm32/service/auth_service/auth_service.o

.PHONY: clean-bms-2d-stm32-2f-service-2f-auth_service

