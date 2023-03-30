TARGET=bms
CCPREFIX  	=arm-none-eabi-
export CC      	= $(CCPREFIX)gcc
CP        	= $(CCPREFIX)objcopy
DUMP		= $(CCPREFIX)objdump
AS        	= $(CCPREFIX)gcc -x assembler-with-cpp
export AR      	= $(CCPREFIX)ar
GDBTUI    	= $(CCPREFIX)gdbtui
SIZE	  	= $(CCPREFIX)size
HEX       	= $(CP) -O ihex
BIN       	= $(CP) -O binary
MCU     	= cortex-m0

COMPONENTS=nfc afe
DRIVERS= gpio exint spi timer
SERVICES= 
BOARD=
PROJECT_LIBS=

STD_PERIPH_LIB=../lib

SRC_DIRS:= ../component ../board ../service ../driver

SRC_DIRS+=$(addprefix ../component/,$(COMPONENTS))
SRC_DIRS+=$(addprefix ../board/,$(BOARD))
SRC_DIRS+=$(addprefix ../service,$(SERVICES))
SRC_DIRS+=$(addprefix ../driver/,$(DRIVERS))
SRC_DIRS+=$(addprefix ../lib/,$(PROJECT_LIBS))

SRCS:= test_cell_monitor_balancing.c ../stm32f0xx_it.c ../bms.c ../system_stm32f0xx.c
SRCS+=$(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.c))

STARTUP:=$(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f0xx.s

SRCS+=$(STARTUP)

OBJDIR=build
OBJ:= $(SRCS:%.c=$(OBJDIR)/%.o)

LDSCRIPT_INC=$(STD_PERIPH_LIB)/CMSIS/Device/ldscripts
DEFS:=-DSTM32F072 -DUSE_FULL_ASSERT -DUSE_STDPERIPH_DRIVER


#--------------------------------------------------------

INCLUDES=-I../
INCLUDES+=$(addprefix -I,$(SRC_DIRS))

INCLUDES += -Iinc -I$(STD_PERIPH_LIB) -I$(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32F0xx/Include
INCLUDES += -I$(STD_PERIPH_LIB)/CMSIS/Include -I$(STD_PERIPH_LIB)/STM32F0xx_StdPeriph_Driver/inc
INCLUDES += -include$(STD_PERIPH_LIB)/stm32f0xx_conf.h

CFLAGS  = -Wall -g -std=c99 -O0  
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections -Wl,-Map=$(TARGET).map

CFLAGS += $(DEFS)
CFLAGS += $(INCLUDES)

# need if you want to build with -DUSE_CMSIS 
#SRCS += stm32f0_discovery.c
#SRCS += stm32f0_discovery.c stm32f0xx_it.c

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C $(STD_PERIPH_LIB)

proj: 	$(TARGET).elf

$(TARGET).elf: $(SRCS)
	@echo "[Building ...]"
	$(CC) $(CFLAGS) $^ -o $@ -L$(STD_PERIPH_LIB) -lstm32f0 -L$(LDSCRIPT_INC) -Tstm32f0xx.ld
	$(HEX)   $(TARGET).elf 	$(TARGET).hex
	$(BIN)   $(TARGET).elf  $(TARGET).bin
	$(DUMP) -St $(TARGET).elf >$(TARGET).lst
	$(SIZE)  $(TARGET).elf
	@echo "[Done !]"
clean:
	find ./ -name '*~' | xargs rm -f	
	rm -f *.o
	rm -f *.d
	rm -f $(TARGET).elf
	rm -f $(TARGET).hex
	rm -f $(TARGET).bin
	rm -f $(TARGET).map
	rm -f $(TARGET).lst
rebuild:
	make clean
	make all
reallyclean: clean
	$(MAKE) -C $(STD_PERIPH_LIB) clean

OPENOCD_PATH=/usr/share/openocd
export OPENOCD_BIN= openocd
export OPENOCD_INTERFACE=$(OPENOCD_PATH)/scripts/interface/stlink-v2.cfg
export OPENOCD_TARGET=$(OPENOCD_PATH)/scripts/target/stm32f0x.cfg

OPENOCD_FLASH_CMDS =""
OPENOCD_FLASH_CMDS +=-c "reset halt" 
OPENOCD_FLASH_CMDS +=-c "sleep 100"
OPENOCD_FLASH_CMDS +=-c "wait_halt 2"
OPENOCD_FLASH_CMDS +=-c "flash write_image erase $(TARGET).hex 0 ihex"
OPENOCD_FLASH_CMDS +=-c "sleep 100"
OPENOCD_FLASH_CMDS +=-c "verify_image $(TARGET).hex"
OPENOCD_FLASH_CMDS +=-c "sleep 100" 
OPENOCD_FLASH_CMDS +=-c "reset halt"
OPENOCD_FLASH_CMDS +=-c "shutdown"

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
	@echo "[Programming...]"
	$(OPENOCD_BIN) \
	-f $(OPENOCD_INTERFACE) \
	-f $(OPENOCD_TARGET) \
	-c init $(OPENOCD_FLASH_CMDS)
	@echo "[Done !"
erase:
	@echo "[Erasing...]"]
	$(OPENOCD_BIN) \
	-f $(OPENOCD_INTERFACE) \
	-f $(OPENOCD_TARGET) \
	-c init $(OPENOCD_ERASE_CMDS)
	@echo "[Done !"
run:
	$(OPENOCD_BIN) \
	-f $(OPENOCD_INTERFACE) \
	-f $(OPENOCD_TARGET) \
	-c init $(OPENOCD_RUN_CMDS)
	@echo "[Done !"
debug:
	@echo "[Debuging...]"
	$(OPENOCD_BIN) \
	-f $(OPENOCD_INTERFACE) \
	-f $(OPENOCD_TARGET) \
	-c init $(OPENOCD_DEBUG_CMDS)
	@echo "[Done !"

