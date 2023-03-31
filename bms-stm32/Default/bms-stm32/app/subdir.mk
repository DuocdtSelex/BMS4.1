################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/app/afe_init.c \
../bms-stm32/app/bms_init.c \
../bms-stm32/app/data_logger_init.c \
../bms-stm32/app/inrush_limiter_init.c \
../bms-stm32/app/key_init.c \
../bms-stm32/app/led_init.c 

C_DEPS += \
./bms-stm32/app/afe_init.d \
./bms-stm32/app/bms_init.d \
./bms-stm32/app/data_logger_init.d \
./bms-stm32/app/inrush_limiter_init.d \
./bms-stm32/app/key_init.d \
./bms-stm32/app/led_init.d 

OBJS += \
./bms-stm32/app/afe_init.o \
./bms-stm32/app/bms_init.o \
./bms-stm32/app/data_logger_init.o \
./bms-stm32/app/inrush_limiter_init.o \
./bms-stm32/app/key_init.o \
./bms-stm32/app/led_init.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/app/%.o: ../bms-stm32/app/%.c bms-stm32/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-app

clean-bms-2d-stm32-2f-app:
	-$(RM) ./bms-stm32/app/afe_init.d ./bms-stm32/app/afe_init.o ./bms-stm32/app/bms_init.d ./bms-stm32/app/bms_init.o ./bms-stm32/app/data_logger_init.d ./bms-stm32/app/data_logger_init.o ./bms-stm32/app/inrush_limiter_init.d ./bms-stm32/app/inrush_limiter_init.o ./bms-stm32/app/key_init.d ./bms-stm32/app/key_init.o ./bms-stm32/app/led_init.d ./bms-stm32/app/led_init.o

.PHONY: clean-bms-2d-stm32-2f-app

