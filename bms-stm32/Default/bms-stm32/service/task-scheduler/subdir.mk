################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/service/task-scheduler/scheduler.c \
../bms-stm32/service/task-scheduler/task.c 

C_DEPS += \
./bms-stm32/service/task-scheduler/scheduler.d \
./bms-stm32/service/task-scheduler/task.d 

OBJS += \
./bms-stm32/service/task-scheduler/scheduler.o \
./bms-stm32/service/task-scheduler/task.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/service/task-scheduler/%.o: ../bms-stm32/service/task-scheduler/%.c bms-stm32/service/task-scheduler/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-service-2f-task-2d-scheduler

clean-bms-2d-stm32-2f-service-2f-task-2d-scheduler:
	-$(RM) ./bms-stm32/service/task-scheduler/scheduler.d ./bms-stm32/service/task-scheduler/scheduler.o ./bms-stm32/service/task-scheduler/task.d ./bms-stm32/service/task-scheduler/task.o

.PHONY: clean-bms-2d-stm32-2f-service-2f-task-2d-scheduler

