################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/app/canopen/app_co_od/app_co_comm_od.c \
../bms-stm32/app/canopen/app_co_od/app_co_manu_od.c \
../bms-stm32/app/canopen/app_co_od/app_co_od.c 

C_DEPS += \
./bms-stm32/app/canopen/app_co_od/app_co_comm_od.d \
./bms-stm32/app/canopen/app_co_od/app_co_manu_od.d \
./bms-stm32/app/canopen/app_co_od/app_co_od.d 

OBJS += \
./bms-stm32/app/canopen/app_co_od/app_co_comm_od.o \
./bms-stm32/app/canopen/app_co_od/app_co_manu_od.o \
./bms-stm32/app/canopen/app_co_od/app_co_od.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/app/canopen/app_co_od/%.o: ../bms-stm32/app/canopen/app_co_od/%.c bms-stm32/app/canopen/app_co_od/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-app-2f-canopen-2f-app_co_od

clean-bms-2d-stm32-2f-app-2f-canopen-2f-app_co_od:
	-$(RM) ./bms-stm32/app/canopen/app_co_od/app_co_comm_od.d ./bms-stm32/app/canopen/app_co_od/app_co_comm_od.o ./bms-stm32/app/canopen/app_co_od/app_co_manu_od.d ./bms-stm32/app/canopen/app_co_od/app_co_manu_od.o ./bms-stm32/app/canopen/app_co_od/app_co_od.d ./bms-stm32/app/canopen/app_co_od/app_co_od.o

.PHONY: clean-bms-2d-stm32-2f-app-2f-canopen-2f-app_co_od

