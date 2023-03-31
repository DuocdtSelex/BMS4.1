################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/service/fault_handler/fault_handler.c 

C_DEPS += \
./bms-stm32/service/fault_handler/fault_handler.d 

OBJS += \
./bms-stm32/service/fault_handler/fault_handler.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/service/fault_handler/%.o: ../bms-stm32/service/fault_handler/%.c bms-stm32/service/fault_handler/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-service-2f-fault_handler

clean-bms-2d-stm32-2f-service-2f-fault_handler:
	-$(RM) ./bms-stm32/service/fault_handler/fault_handler.d ./bms-stm32/service/fault_handler/fault_handler.o

.PHONY: clean-bms-2d-stm32-2f-service-2f-fault_handler

