################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/test/test_cell_monitor_adc_detect.c \
../bms-stm32/test/test_cell_monitor_balancing.c \
../bms-stm32/test/test_cell_monitor_cc.c \
../bms-stm32/test/test_cell_monitor_i2c.c \
../bms-stm32/test/test_cell_monitor_int.c \
../bms-stm32/test/test_cell_monitor_io.c \
../bms-stm32/test/test_nfc.c \
../bms-stm32/test/test_nfc_extint.c \
../bms-stm32/test/test_nfc_io.c \
../bms-stm32/test/test_nfc_send.c \
../bms-stm32/test/test_nfc_send_message.c \
../bms-stm32/test/test_nfc_spi.c \
../bms-stm32/test/test_nfc_timer.c 

C_DEPS += \
./bms-stm32/test/test_cell_monitor_adc_detect.d \
./bms-stm32/test/test_cell_monitor_balancing.d \
./bms-stm32/test/test_cell_monitor_cc.d \
./bms-stm32/test/test_cell_monitor_i2c.d \
./bms-stm32/test/test_cell_monitor_int.d \
./bms-stm32/test/test_cell_monitor_io.d \
./bms-stm32/test/test_nfc.d \
./bms-stm32/test/test_nfc_extint.d \
./bms-stm32/test/test_nfc_io.d \
./bms-stm32/test/test_nfc_send.d \
./bms-stm32/test/test_nfc_send_message.d \
./bms-stm32/test/test_nfc_spi.d \
./bms-stm32/test/test_nfc_timer.d 

OBJS += \
./bms-stm32/test/test_cell_monitor_adc_detect.o \
./bms-stm32/test/test_cell_monitor_balancing.o \
./bms-stm32/test/test_cell_monitor_cc.o \
./bms-stm32/test/test_cell_monitor_i2c.o \
./bms-stm32/test/test_cell_monitor_int.o \
./bms-stm32/test/test_cell_monitor_io.o \
./bms-stm32/test/test_nfc.o \
./bms-stm32/test/test_nfc_extint.o \
./bms-stm32/test/test_nfc_io.o \
./bms-stm32/test/test_nfc_send.o \
./bms-stm32/test/test_nfc_send_message.o \
./bms-stm32/test/test_nfc_spi.o \
./bms-stm32/test/test_nfc_timer.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/test/%.o: ../bms-stm32/test/%.c bms-stm32/test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-test

clean-bms-2d-stm32-2f-test:
	-$(RM) ./bms-stm32/test/test_cell_monitor_adc_detect.d ./bms-stm32/test/test_cell_monitor_adc_detect.o ./bms-stm32/test/test_cell_monitor_balancing.d ./bms-stm32/test/test_cell_monitor_balancing.o ./bms-stm32/test/test_cell_monitor_cc.d ./bms-stm32/test/test_cell_monitor_cc.o ./bms-stm32/test/test_cell_monitor_i2c.d ./bms-stm32/test/test_cell_monitor_i2c.o ./bms-stm32/test/test_cell_monitor_int.d ./bms-stm32/test/test_cell_monitor_int.o ./bms-stm32/test/test_cell_monitor_io.d ./bms-stm32/test/test_cell_monitor_io.o ./bms-stm32/test/test_nfc.d ./bms-stm32/test/test_nfc.o ./bms-stm32/test/test_nfc_extint.d ./bms-stm32/test/test_nfc_extint.o ./bms-stm32/test/test_nfc_io.d ./bms-stm32/test/test_nfc_io.o ./bms-stm32/test/test_nfc_send.d ./bms-stm32/test/test_nfc_send.o ./bms-stm32/test/test_nfc_send_message.d ./bms-stm32/test/test_nfc_send_message.o ./bms-stm32/test/test_nfc_spi.d ./bms-stm32/test/test_nfc_spi.o ./bms-stm32/test/test_nfc_timer.d ./bms-stm32/test/test_nfc_timer.o

.PHONY: clean-bms-2d-stm32-2f-test

