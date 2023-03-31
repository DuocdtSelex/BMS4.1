################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/component/afe/afe.c \
../bms-stm32/component/afe/bq769x0.c \
../bms-stm32/component/afe/bq769x2.c \
../bms-stm32/component/afe/bq_crc.c 

C_DEPS += \
./bms-stm32/component/afe/afe.d \
./bms-stm32/component/afe/bq769x0.d \
./bms-stm32/component/afe/bq769x2.d \
./bms-stm32/component/afe/bq_crc.d 

OBJS += \
./bms-stm32/component/afe/afe.o \
./bms-stm32/component/afe/bq769x0.o \
./bms-stm32/component/afe/bq769x2.o \
./bms-stm32/component/afe/bq_crc.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/component/afe/%.o: ../bms-stm32/component/afe/%.c bms-stm32/component/afe/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-component-2f-afe

clean-bms-2d-stm32-2f-component-2f-afe:
	-$(RM) ./bms-stm32/component/afe/afe.d ./bms-stm32/component/afe/afe.o ./bms-stm32/component/afe/bq769x0.d ./bms-stm32/component/afe/bq769x0.o ./bms-stm32/component/afe/bq769x2.d ./bms-stm32/component/afe/bq769x2.o ./bms-stm32/component/afe/bq_crc.d ./bms-stm32/component/afe/bq_crc.o

.PHONY: clean-bms-2d-stm32-2f-component-2f-afe

