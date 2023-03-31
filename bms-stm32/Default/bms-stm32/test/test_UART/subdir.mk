################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/test/test_UART/test_UART.c 

C_DEPS += \
./bms-stm32/test/test_UART/test_UART.d 

OBJS += \
./bms-stm32/test/test_UART/test_UART.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/test/test_UART/%.o: ../bms-stm32/test/test_UART/%.c bms-stm32/test/test_UART/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-test-2f-test_UART

clean-bms-2d-stm32-2f-test-2f-test_UART:
	-$(RM) ./bms-stm32/test/test_UART/test_UART.d ./bms-stm32/test/test_UART/test_UART.o

.PHONY: clean-bms-2d-stm32-2f-test-2f-test_UART

