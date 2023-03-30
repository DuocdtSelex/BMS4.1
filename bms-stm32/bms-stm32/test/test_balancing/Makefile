TARGET=cell_balancing
PROJ_ROOT=../..
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

COMPONENTS=nfc afe indicator_leds ext_flash adc can
DRIVERS= gpio exint spi timer it-flash
DRIVERS= gpio exint spi timer it-flash sys_clock i2c
SERVICES= CANopen config_server task-scheduler debug_print soc-ocv
BOARD=
PROJECT_LIBS=

STD_PERIPH_LIB=libs

SRC_DIRS:= component board service driver

SRC_DIRS+=$(addprefix component/,$(COMPONENTS))
SRC_DIRS+=$(addprefix board/,$(BOARD))
SRC_DIRS+=$(addprefix service/,$(SERVICES))
SRC_DIRS+=$(addprefix driver/,$(DRIVERS))
SRC_DIRS+=$(addprefix lib/,$(PROJECT_LIBS))

SRC_DIRS:=$(addprefix $(PROJ_ROOT)/,$(SRC_DIRS))

SRCS:=$(TARGET).c  $(PROJ_ROOT)/stm32f0xx_it.c  $(PROJ_ROOT)/system_stm32f0xx.c
SRCS+=$(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.c))

STARTUP:=$(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f072.s
SRCS+=$(PROJ_ROOT)/$(STARTUP)

OBJDIR=build
OBJ:= $(SRCS:%.c=$(OBJDIR)/%.o)

LDSCRIPT_INC=$(PROJ_ROOT)/$(STD_PERIPH_LIB)/CMSIS/Device/ldscripts
DEFS:=-DSTM32F072 -DUSE_FULL_ASSERT -DUSE_STDPERIPH_DRIVER


#--------------------------------------------------------

INCLUDES= .

INCLUDES += $(STD_PERIPH_LIB) $(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32F0xx/Include
INCLUDES += $(STD_PERIPH_LIB)/CMSIS/Include $(STD_PERIPH_LIB)/STM32F0xx_StdPeriph_Driver/inc
INCLUDES:=$(addprefix -I$(PROJ_ROOT)/,$(INCLUDES))
INCLUDES+=$(addprefix -I,$(SRC_DIRS))
INCLUDES += -include$(PROJ_ROOT)/$(STD_PERIPH_LIB)/stm32f0xx_conf.h

CFLAGS  = -Wall -g -std=gnu11 -O0  
#CFLAGS  = -Wall -std=c99 -O1  
CFLAGS += -specs=nosys.specs
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

.PHONY: lib proj lint

all: lib proj

lib:
	$(MAKE) -C $(PROJ_ROOT)/$(STD_PERIPH_LIB)

proj: 	$(TARGET).elf

$(TARGET).elf: $(SRCS)
	@echo $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -L$(PROJ_ROOT)/$(STD_PERIPH_LIB) -lstm32f0 -L$(LDSCRIPT_INC) -Tstm32f0xx.ld
	$(HEX)   $(TARGET).elf 	$(TARGET).hex
	$(BIN)   $(TARGET).elf  $(TARGET).bin
	$(DUMP) -St $(TARGET).elf >$(TARGET).lst
	$(SIZE)  $(TARGET).elf
clean:
	find ./ -name '*~' | xargs rm -f	
	rm -f *.o
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
OPENOCD_FLASH_CMDS +=-c "reset init" 
OPENOCD_FLASH_CMDS +=-c "halt" 
OPENOCD_FLASH_CMDS +=-c "flash write_image erase $(TARGET).bin 0x08000000"
OPENOCD_FLASH_CMDS +=-c "verify_image $(TARGET).bin 0x08000000"
OPENOCD_FLASH_CMDS +=-c "sleep 100"
OPENOCD_FLASH_CMDS +=-c "reset run"
OPENOCD_FLASH_CMDS +=-c "shutdown"
export OPENOCD_FLASH_CMDS

OPENOCD_ERASE_CMDS= ''
OPENOCD_ERASE_CMDS += -c 'reset halt'
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
OPENOCD_DEBUG_CMDS +=-c 'reset halt'
OPENOCD_DEBUG_CMDS +=-c 'sleep 10'

flash:
	@echo "[Programming...]"
	#st-flash --reset write "bms.bin" 0x8000000
	#st-flash write "bms.bin" 0x8000000
	st-flash --format ihex write bms.hex
#	$(OPENOCD_BIN) \
#	-f $(OPENOCD_INTERFACE) \
#	-f $(OPENOCD_TARGET) \
#	-c init $(OPENOCD_FLASH_CMDS)
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

lint:
	@echo "[Linting the code ....]"
	gcclint $(INCLUDES) main.c
