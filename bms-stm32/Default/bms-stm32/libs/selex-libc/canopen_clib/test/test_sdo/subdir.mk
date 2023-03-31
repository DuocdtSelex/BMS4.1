################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/libs/selex-libc/canopen_clib/test/test_sdo/test_sdo.c 

C_DEPS += \
./bms-stm32/libs/selex-libc/canopen_clib/test/test_sdo/test_sdo.d 

OBJS += \
./bms-stm32/libs/selex-libc/canopen_clib/test/test_sdo/test_sdo.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/libs/selex-libc/canopen_clib/test/test_sdo/%.o: ../bms-stm32/libs/selex-libc/canopen_clib/test/test_sdo/%.c bms-stm32/libs/selex-libc/canopen_clib/test/test_sdo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-libs-2f-selex-2d-libc-2f-canopen_clib-2f-test-2f-test_sdo

clean-bms-2d-stm32-2f-libs-2f-selex-2d-libc-2f-canopen_clib-2f-test-2f-test_sdo:
	-$(RM) ./bms-stm32/libs/selex-libc/canopen_clib/test/test_sdo/test_sdo.d ./bms-stm32/libs/selex-libc/canopen_clib/test/test_sdo/test_sdo.o

.PHONY: clean-bms-2d-stm32-2f-libs-2f-selex-2d-libc-2f-canopen_clib-2f-test-2f-test_sdo

