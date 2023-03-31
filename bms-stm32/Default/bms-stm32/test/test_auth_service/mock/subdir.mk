################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/test/test_auth_service/mock/hc05_mock.c 

C_DEPS += \
./bms-stm32/test/test_auth_service/mock/hc05_mock.d 

OBJS += \
./bms-stm32/test/test_auth_service/mock/hc05_mock.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/test/test_auth_service/mock/%.o: ../bms-stm32/test/test_auth_service/mock/%.c bms-stm32/test/test_auth_service/mock/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-test-2f-test_auth_service-2f-mock

clean-bms-2d-stm32-2f-test-2f-test_auth_service-2f-mock:
	-$(RM) ./bms-stm32/test/test_auth_service/mock/hc05_mock.d ./bms-stm32/test/test_auth_service/mock/hc05_mock.o

.PHONY: clean-bms-2d-stm32-2f-test-2f-test_auth_service-2f-mock

