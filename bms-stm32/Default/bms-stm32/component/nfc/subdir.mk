################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/component/nfc/iso_7816_4.c \
../bms-stm32/component/nfc/iso_dep.c \
../bms-stm32/component/nfc/llcp.c \
../bms-stm32/component/nfc/mcu.c \
../bms-stm32/component/nfc/nfc.c \
../bms-stm32/component/nfc/nfc_a.c \
../bms-stm32/component/nfc/nfc_b.c \
../bms-stm32/component/nfc/nfc_controller.c \
../bms-stm32/component/nfc/nfc_dep.c \
../bms-stm32/component/nfc/nfc_f.c \
../bms-stm32/component/nfc/nfc_initiator.c \
../bms-stm32/component/nfc/nfc_rw_t2t.c \
../bms-stm32/component/nfc/nfc_rw_t3t.c \
../bms-stm32/component/nfc/nfc_rw_t5t.c \
../bms-stm32/component/nfc/nfc_spi.c \
../bms-stm32/component/nfc/nfc_target.c \
../bms-stm32/component/nfc/snep.c \
../bms-stm32/component/nfc/trf79x0.c 

C_DEPS += \
./bms-stm32/component/nfc/iso_7816_4.d \
./bms-stm32/component/nfc/iso_dep.d \
./bms-stm32/component/nfc/llcp.d \
./bms-stm32/component/nfc/mcu.d \
./bms-stm32/component/nfc/nfc.d \
./bms-stm32/component/nfc/nfc_a.d \
./bms-stm32/component/nfc/nfc_b.d \
./bms-stm32/component/nfc/nfc_controller.d \
./bms-stm32/component/nfc/nfc_dep.d \
./bms-stm32/component/nfc/nfc_f.d \
./bms-stm32/component/nfc/nfc_initiator.d \
./bms-stm32/component/nfc/nfc_rw_t2t.d \
./bms-stm32/component/nfc/nfc_rw_t3t.d \
./bms-stm32/component/nfc/nfc_rw_t5t.d \
./bms-stm32/component/nfc/nfc_spi.d \
./bms-stm32/component/nfc/nfc_target.d \
./bms-stm32/component/nfc/snep.d \
./bms-stm32/component/nfc/trf79x0.d 

OBJS += \
./bms-stm32/component/nfc/iso_7816_4.o \
./bms-stm32/component/nfc/iso_dep.o \
./bms-stm32/component/nfc/llcp.o \
./bms-stm32/component/nfc/mcu.o \
./bms-stm32/component/nfc/nfc.o \
./bms-stm32/component/nfc/nfc_a.o \
./bms-stm32/component/nfc/nfc_b.o \
./bms-stm32/component/nfc/nfc_controller.o \
./bms-stm32/component/nfc/nfc_dep.o \
./bms-stm32/component/nfc/nfc_f.o \
./bms-stm32/component/nfc/nfc_initiator.o \
./bms-stm32/component/nfc/nfc_rw_t2t.o \
./bms-stm32/component/nfc/nfc_rw_t3t.o \
./bms-stm32/component/nfc/nfc_rw_t5t.o \
./bms-stm32/component/nfc/nfc_spi.o \
./bms-stm32/component/nfc/nfc_target.o \
./bms-stm32/component/nfc/snep.o \
./bms-stm32/component/nfc/trf79x0.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/component/nfc/%.o: ../bms-stm32/component/nfc/%.c bms-stm32/component/nfc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-component-2f-nfc

clean-bms-2d-stm32-2f-component-2f-nfc:
	-$(RM) ./bms-stm32/component/nfc/iso_7816_4.d ./bms-stm32/component/nfc/iso_7816_4.o ./bms-stm32/component/nfc/iso_dep.d ./bms-stm32/component/nfc/iso_dep.o ./bms-stm32/component/nfc/llcp.d ./bms-stm32/component/nfc/llcp.o ./bms-stm32/component/nfc/mcu.d ./bms-stm32/component/nfc/mcu.o ./bms-stm32/component/nfc/nfc.d ./bms-stm32/component/nfc/nfc.o ./bms-stm32/component/nfc/nfc_a.d ./bms-stm32/component/nfc/nfc_a.o ./bms-stm32/component/nfc/nfc_b.d ./bms-stm32/component/nfc/nfc_b.o ./bms-stm32/component/nfc/nfc_controller.d ./bms-stm32/component/nfc/nfc_controller.o ./bms-stm32/component/nfc/nfc_dep.d ./bms-stm32/component/nfc/nfc_dep.o ./bms-stm32/component/nfc/nfc_f.d ./bms-stm32/component/nfc/nfc_f.o ./bms-stm32/component/nfc/nfc_initiator.d ./bms-stm32/component/nfc/nfc_initiator.o ./bms-stm32/component/nfc/nfc_rw_t2t.d ./bms-stm32/component/nfc/nfc_rw_t2t.o ./bms-stm32/component/nfc/nfc_rw_t3t.d ./bms-stm32/component/nfc/nfc_rw_t3t.o ./bms-stm32/component/nfc/nfc_rw_t5t.d ./bms-stm32/component/nfc/nfc_rw_t5t.o ./bms-stm32/component/nfc/nfc_spi.d ./bms-stm32/component/nfc/nfc_spi.o ./bms-stm32/component/nfc/nfc_target.d ./bms-stm32/component/nfc/nfc_target.o ./bms-stm32/component/nfc/snep.d ./bms-stm32/component/nfc/snep.o ./bms-stm32/component/nfc/trf79x0.d ./bms-stm32/component/nfc/trf79x0.o

.PHONY: clean-bms-2d-stm32-2f-component-2f-nfc

