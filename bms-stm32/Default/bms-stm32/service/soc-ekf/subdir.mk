################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/service/soc-ekf/soc_ekf.c 

C_DEPS += \
./bms-stm32/service/soc-ekf/soc_ekf.d 

OBJS += \
./bms-stm32/service/soc-ekf/soc_ekf.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/service/soc-ekf/%.o: ../bms-stm32/service/soc-ekf/%.c bms-stm32/service/soc-ekf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-service-2f-soc-2d-ekf

clean-bms-2d-stm32-2f-service-2f-soc-2d-ekf:
	-$(RM) ./bms-stm32/service/soc-ekf/soc_ekf.d ./bms-stm32/service/soc-ekf/soc_ekf.o

.PHONY: clean-bms-2d-stm32-2f-service-2f-soc-2d-ekf

