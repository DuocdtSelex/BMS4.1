TARGET		=bms
EXECUTABLE	:=$(TARGET).elf

TOOLCHAIN_INCLUDE=/mnt/workspace/SeleX.JSC/Firmware/toolchain/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include
CCPREFIX  	=arm-none-eabi-
CC       	= $(CCPREFIX)gcc
CP        	= $(CCPREFIX)objcopy
AS        	= $(CCPREFIX)gcc -x assembler-with-cpp
GDBTUI    	= $(CCPREFIX)gdbtui
SIZE	  	= $(CCPREFIX)size
HEX       	= $(CP) -O ihex
BIN       	= $(CP) -O binary -S
MCU     	= cortex-m0

#CC=arm-none-eabi-gcc
#LD=arm-none-eabi-gcc
#AR=arm-none-eabi-ar
#AS=arm-none-eabi-as
#CP=arm-none-eabi-objcopy
#OD=arm-none-eabi-objdump

BIN=$(CP) -O ihex

COMPONENTS=nfc 
DRIVERS= gpio exint spi
SERVICES= 
BOARD=
PROJECT_LIBS=

STM32_STD_LIB_DIR=lib/STM32F0xx_StdPeriph_Driver
STM32_CPAL_DIR=lib/STM32F0xx_CPAL_Driver

STM32_INCLUDES:=$(STM32_STD_LIB_DIR)/inc $(STM32_CPAL_DIR)/inc
CMSIS_DIR=lib/CMSIS
CMSIS_DEVICE:=$(CMSIS_DIR)/Device/ST/STM32F0xx
CMSIS_INCLUDES:=$(CMSIS_DIR)/Include $(CMSIS_DEVICE)/Include

SRC_DIRS:= component board service driver

SRC_DIRS+=$(addprefix component/,$(COMPONENTS))
SRC_DIRS+=$(addprefix board/,$(BOARD))
SRC_DIRS+=$(addprefix service,$(SERVICES))
SRC_DIRS+=$(addprefix driver/,$(DRIVERS))
SRC_DIRS+=$(addprefix lib/,$(PROJECT_LIBS))

INCLUDES=-I.
#INCLUDES=-I$(TOOLCHAIN_INCLUDE)
INCLUDES+=$(addprefix -I,$(SRC_DIRS))
INCLUDES+=$(addprefix -I,$(STM32_INCLUDES))
INCLUDES+=$(addprefix -I,$(CMSIS_INCLUDES))

DEFS:=-DSTM32F072 -DUSE_FULL_ASSERT -DUSE_STDPERIPH_DRIVER
STARTUP:=$(CMSIS_DEVICE)/Source/startup_stm32f0xx.s
LDSCRIPT_INC=$(CMSIS_DIR)/Device/ldscripts

MCFLAGS:= -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork

CFLAGS:= $(MCFLAGS) $(OPTIMIZE) $(DEFS) $(INCLUDES) -L$(LDSCRIPT_INC) -Wl,-T,$(LDSCRIPT_INC)/stm32f0xx.ld -ggdb 
AFLAGS:= $(MCFLAGS)

OPTIMIZE:= -O0

# list all used modules of stm32 standard libraries
STM32_STD_LIB_SRCS=stm32f0xx_adc.c \
		stm32f0xx_can.c \
		stm32f0xx_cec.c \
		stm32f0xx_crc.c \
		stm32f0xx_dac.c \
		stm32f0xx_dbgmcu.c \
		stm32f0xx_dma.c \
		stm32f0xx_exti.c \
		stm32f0xx_flash.c \
		stm32f0xx_gpio.c \
		stm32f0xx_i2c.c \
		stm32f0xx_iwdg.c \
		stm32f0xx_pwr.c \
		stm32f0xx_rcc.c \
		stm32f0xx_rtc.c \
		stm32f0xx_spi.c \
		stm32f0xx_tim.c \
		stm32f0xx_usart.c \
		stm32f0xx_wwdg.c \
		stm32f0xx_syscfg.c \
		stm32f0xx_misc.c

STM32_CPAL_SRCS=$(wildcard $(STM32_CPAL_DIR)/src/*.c)

STM32_STD_LIB_SRCS:=$(addprefix $(STM32_STD_LIB_DIR)/src/,$(STM32_STD_LIB_SRCS))

SRCS:=test/test_nfc.c
SRCS+=stm32f0xx_it.c system_stm32f0xx.c
SRCS+=$(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.c))
SRCS+=$(STM32_STD_LIB_SRCS)
#SRCS+=$(STM32_CPAL_SRCS)

OBJDIR=build
OBJ:= $(SRCS:%.c=$(OBJDIR)/%.o)
OBJ += Startup.o
DEPS:=$(SRCS:.c=.d)

#user libraries location
ULIBS_LOC=/mnt/Workspace/Projects/DesignReposistory/gcc-libs

.PHONY: all clean flash erase run debug

all: $(TARGET).hex

$(TARGET).hex: $(EXECUTABLE)
	$(HEX) $^ $@
	$(SIZE)	--format=sysv $(EXECUTABLE) --totals
-include $(DEPS)

$(EXECUTABLE): $(SRCS) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys  -o $@
%.d:%.c
	@echo "[Building" $< "..."]
	$(CC) $(CFLAGS) $< -MM -MT $(@:.d=.o) > $@
	@echo "[Done !"
%.d:%.s
	@echo "[Building" $< "..."]
	$(CC) $(CFLAGS) $< -MM -MT $(@:.d=.o) > $@
	@echo "[Done !"
clean:
	@echo "[Cleaning...]"]
	rm -f Startup.lst  $(TARGET)  $(TARGET).lst \
	$(DEPS) $(OBJ)  $(TARGET).out  $(TARGET).hex \
       	$(TARGET).map $(TARGET).dmp  $(EXECUTABLE) 
	@echo "[Done !"

OPENOCD_PATH=/usr/share/openocd
export OPENOCD_BIN= openocd
export OPENOCD_INTERFACE=$(OPENOCD_PATH)/scripts/interface/stlink-v2.cfg
export OPENOCD_TARGET=$(OPENOCD_PATH)/scripts/target/stm32f0x.cfg

OPENOCD_FLASH_CMDS=''
OPENOCD_FLASH_CMDS +=-c 'reset halt'
OPENOCD_FLASH_CMDS +=-c 'sleep 10'
OPENOCD_FLASH_CMDS +=-c 'stm32f1x unlock 0'
OPENOCD_FLASH_CMDS +=-c 'flash write_image erase $(TARGET).hex 0 ihex'
OPENOCD_FLASH_CMDS +=-c shutdown
export OPENOCD_FLASH_CMDS
OPENOCD_ERASE_CMDS= ''
OPENOCD_ERASE_CMDS += -c 'reset halt'
OPENOCD_ERASE_CMDS += -c 'sleep 10'
OPENOCD_ERASE_CMDS += -c 'sleep 10'
OPENOCD_ERASE_CMDS += -c 'stm32f0x mass_erase 0'
OPENOCD_ERASE_CMDS += -c shutdown
export OPENOCD_ERASE_CMDS
OPENOCD_RUN_CMDS= ''
OPENOCD_RUN_CMDS += -c 'reset halt'
OPENOCD_RUN_CMDS += -c 'sleep 10'
OPENOCD_RUN_CMDS += -c 'reset run'
OPENOCD_RUN_CMDS += -c 'sleep 10'
OPENOCD_RUN_CMDS += -c shutdown
export OPENOCD_RUN_CMDS

OPENOCD_DEBUG_CMDS=''
OPENOCD_DEBUG_CMDS +=-c 'halt'
OPENOCD_DEBUG_CMDS +=-c 'sleep 10'

flash:
	@echo "[Programming...]"]
	$(OPENOCD_BIN) \
	-f $(OPENOCD_INTERFACE) \
	-f $(OPENOCD_TARGET) \
	-c init $(OPENOCD_FLASH_CMDS)
	@echo "[Done !"
erase:
	@echo "[Erasing...]"]
	$(OPENOCD_BIN) 
	-f $(OPENOCD_INTERFACE) \
	-f $(OPENOCD_TARGET) \
	-c init $(OPENOCD_ERASE_CMDS)
	@echo "[Done !"
run:
	$(OPENOCD_BIN) 
	-f $(OPENOCD_INTERFACE) \
	-f $(OPENOCD_TARGET) \
	-c init $(OPENOCD_RUN_CMDS)
	@echo "[Done !"
debug:
	$(OPENOCD_BIN) 
	-f $(OPENOCD_INTERFACE) \
	-f $(OPENOCD_TARGET) \
	-c init $(OPENOCD_DEBUG_CMDS)
	@echo "[Done !"

