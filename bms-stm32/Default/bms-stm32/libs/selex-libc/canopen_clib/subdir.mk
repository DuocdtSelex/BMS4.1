################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bms-stm32/libs/selex-libc/canopen_clib/CAN_Msg.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_CAN_Msg.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_EMCY.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_NMT.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_OD.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_OD_storage.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_Object.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_PDO.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_RPDO.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_SDO.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_SDOclient.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_SDOserver.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_SYNC.c \
../bms-stm32/libs/selex-libc/canopen_clib/CO_TPDO.c 

O_SRCS += \
../bms-stm32/libs/selex-libc/canopen_clib/CAN_Msg.o \
../bms-stm32/libs/selex-libc/canopen_clib/CO.o \
../bms-stm32/libs/selex-libc/canopen_clib/CO_NMT.o \
../bms-stm32/libs/selex-libc/canopen_clib/CO_OD.o \
../bms-stm32/libs/selex-libc/canopen_clib/CO_Object.o \
../bms-stm32/libs/selex-libc/canopen_clib/CO_PDO.o \
../bms-stm32/libs/selex-libc/canopen_clib/CO_RPDO.o \
../bms-stm32/libs/selex-libc/canopen_clib/CO_SDO.o \
../bms-stm32/libs/selex-libc/canopen_clib/CO_TPDO.o 

C_DEPS += \
./bms-stm32/libs/selex-libc/canopen_clib/CAN_Msg.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_CAN_Msg.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_EMCY.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_NMT.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_OD.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_OD_storage.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_Object.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_PDO.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_RPDO.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_SDO.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_SDOclient.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_SDOserver.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_SYNC.d \
./bms-stm32/libs/selex-libc/canopen_clib/CO_TPDO.d 

OBJS += \
./bms-stm32/libs/selex-libc/canopen_clib/CAN_Msg.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_CAN_Msg.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_EMCY.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_NMT.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_OD.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_OD_storage.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_Object.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_PDO.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_RPDO.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_SDO.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_SDOclient.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_SDOserver.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_SYNC.o \
./bms-stm32/libs/selex-libc/canopen_clib/CO_TPDO.o 


# Each subdirectory must supply rules for building sources it contributes
bms-stm32/libs/selex-libc/canopen_clib/%.o: ../bms-stm32/libs/selex-libc/canopen_clib/%.c bms-stm32/libs/selex-libc/canopen_clib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bms-2d-stm32-2f-libs-2f-selex-2d-libc-2f-canopen_clib

clean-bms-2d-stm32-2f-libs-2f-selex-2d-libc-2f-canopen_clib:
	-$(RM) ./bms-stm32/libs/selex-libc/canopen_clib/CAN_Msg.d ./bms-stm32/libs/selex-libc/canopen_clib/CAN_Msg.o ./bms-stm32/libs/selex-libc/canopen_clib/CO.d ./bms-stm32/libs/selex-libc/canopen_clib/CO.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_CAN_Msg.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_CAN_Msg.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_EMCY.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_EMCY.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_NMT.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_NMT.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_OD.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_OD.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_OD_storage.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_OD_storage.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_Object.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_Object.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_PDO.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_PDO.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_RPDO.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_RPDO.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_SDO.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_SDO.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_SDOclient.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_SDOclient.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_SDOserver.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_SDOserver.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_SYNC.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_SYNC.o ./bms-stm32/libs/selex-libc/canopen_clib/CO_TPDO.d ./bms-stm32/libs/selex-libc/canopen_clib/CO_TPDO.o

.PHONY: clean-bms-2d-stm32-2f-libs-2f-selex-2d-libc-2f-canopen_clib

