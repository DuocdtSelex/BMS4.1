# Prefix for the arm-eabi-none toolchain.
# I'm using codesourcery g++ lite compilers available here:
# http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/
# Microcontroller properties.

export TARGET=selex_bms
export PROJ_ROOT= ./
export BOARD_TYPE=stm32_bsp
include board/$(BOARD_TYPE)/$(BOARD_TYPE).mk
include gcc.mk

SRCS=board/board.c
	
SRCS+= component/afe/afe.c \
	component/afe/bq769x0.c \
	component/afe/bq769x2.c \
	component/cell_bank/cell_bank.c \
	component/cell_array/cell_array.c \
	component/switch/switch.c \
	component/inrush_limiter/inrush_limiter.c \
	component/ntc/ntc.c \
	component/keypad/keypad.c \
	component/adc_sensor/adc_sensor.c \
	component/hc05/hc05.c \
	component/flash_memory/flash_memory.c \
	component/flash_memory/at25sf161.c \
	util/string/string_util.c 
	
SRCS+=  libs/selex-libc/canopen_clib/CAN_Msg.c \
		libs/selex-libc/canopen_clib/CO_CAN_Msg.c \
		libs/selex-libc/canopen_clib/CO_EMCY.c \
		libs/selex-libc/canopen_clib/CO_NMT.c \
		libs/selex-libc/canopen_clib/CO_Object.c \
		libs/selex-libc/canopen_clib/CO_OD_storage.c \
		libs/selex-libc/canopen_clib/CO_OD.c \
		libs/selex-libc/canopen_clib/CO_PDO.c \
		libs/selex-libc/canopen_clib/CO_RPDO.c \
		libs/selex-libc/canopen_clib/CO_SDO.c \
		libs/selex-libc/canopen_clib/CO_SDOclient.c \
		libs/selex-libc/canopen_clib/CO_SDOserver.c \
		libs/selex-libc/canopen_clib/CO_SYNC.c \
		libs/selex-libc/canopen_clib/CO_TPDO.c \
		libs/selex-libc/canopen_clib/CO.c 
		
SRCS+= app/canopen/app_co_init/app_co_init.c \
		app/canopen/app_co_od/app_co_comm_od.c \
		app/canopen/app_co_od/app_co_manu_od.c \
		app/canopen/app_co_od/app_co_od.c \
		app/canopen/app_co_storage_init/app_co_storage_init.c \
		app/canopen/canopen_init.c 
		
SRCS+=service/soc-ocv/soc-ocv.c \
	service/soc_est/soc_est.c \
	service/soh_est/soh_est.c \
	service/estimator/estimator.c \
	service/bms/bms.c \
	service/soc-ekf/soc_ekf.c \
	service/soc-spkf/soc_spkf.c \
	service/soh_awtls/soh_awtls.c \
	service/data_logger/data_logger.c


BSP_SRCS:=$(addprefix board/$(BOARD_TYPE)/,$(BSP_SRCS))
BSP_INCLUDES:=$(addprefix board/$(BOARD_TYPE)/,$(BSP_INCLUDES))

INCLUDES:=. app_config board service component util app app/canopen
INCLUDES+= 	component/afe \
			component/cell_array \
			component/cell_bank \
			component/switch \
			component/inrush_limiter \
			component/ntc \
			component/keypad \
			component/hc05 \
			component/adc_sensor \
			component/flash_memory 

INCLUDES+=service/soc-ocv \
			service/soh_est \
			service/soc_est \
			service/estimator \
			service/bms \
			service/soc-ekf \
			service/soc-spkf \
			service/soh_awtls \
			service/data_logger 
			
INCLUDES+= libs/selex-libc/canopen_clib 

INCLUDES+= app/canopen/app_co_init \
			app/canopen/app_co_od \
			app/canopen/app_co_storage_init \
			app/canopen 

INCLUDES+= util/delay util/string

#USER_LIB_INCLUDES=$(PROJ_ROOT)/libs/selex-libc/canopen_clib 
#USER_LIBS=CANopen
#
#LIB_INCLUDES=libs 
OBJDIR=build

INCLUDES+=$(LIB_INCLUDES)			
INCLUDES+=$(BSP_INCLUDES)
INCLUDES:=$(addprefix -I$(PROJ_ROOT)/,$(INCLUDES))
INCLUDES+=$(addprefix -I,$(USER_LIB_INCLUDES))

SRCS+=$(BSP_SRCS)
SRCS+= main.c  app/afe_init.c app/bms_init.c  app/key_init.c \
       app/inrush_limiter_init.c \
       app/data_logger_init.c
OBJS:=$(addprefix $(PROJ_ROOT)/$(OBJDIR)/,$(SRCS))
OBJS:= $(patsubst %.c,%.o,$(OBJS))
OBJS:= $(patsubst %.s,%.o,$(OBJS))
DEPS:= $(patsubst %.o,%.d,$(OBJS))
SRCS:=$(addprefix $(PROJ_ROOT)/,$(SRCS))

LDSCRIPT_INC=
DEFS:=-DFW_VERSION_V4_1
#DEFS:=-DFW_VERSION_V4_0

#--------------------------------------------------------

OPTIMIZE=-O0

# Option arguments for C compiler.
CFLAGS+= $(INCLUDES)
CFLAGS+= $(OPTIMIZE)
CFLAGS+= $(DEFS)
CFLAGS+=-fmessage-length=0
CFLAGS+=-fsigned-char
CFLAGS+=-Wall -Winline -ggdb -lm -MMD -MP -Wno-unused-function -Wextra -Wstrict-prototypes
CFLAGS+=-std=gnu11
CFLAGS+=--specs=nosys.specs
CFLAGS+=-ffunction-sections -fdata-sections

LFLAGS:=-T$(PROJ_ROOT)/board/$(BOARD_TYPE)/$(LD_FILES)
LFLAGS	+=-nostartfiles -Xlinker --gc-sections
LFLAGS  +=-Wl,-Map=$(TARGET).map

LINK_LIBS:= $(addprefix -L,$(USER_LIB_INCLUDES))
LINK_LIBS+= $(addprefix -l,$(USER_LIBS))

#LFLAGS  += -flto -fuse-linker-plugin
# Flags for objcopy
CPFLAGS = -Obinary

# flags for objectdump
ODFLAGS = -S

# Uploader tool path.
FLASHER=JFlashLite
JLINKGDB=JLinkGDBServer
# Flags for the uploader program.
FLASHER_FLAGS=
#include $(DEPS)
###################################################

.PHONY:all proj debug libs test_suit test_suit_clean flash

all: proj $(DEPS)
	$(HEX)   $(TARGET).elf 	$(TARGET).hex
	$(BIN)   $(TARGET).elf  $(TARGET).bin
	$(DUMP) -St $(TARGET).elf >$(TARGET).lst
	$(SIZE)  $(OBJS) $(TARGET).elf $(TARGET).hex

proj: 	$(TARGET).elf

$(PROJ_ROOT)/$(OBJDIR)/%.o: $(PROJ_ROOT)/%.c
	@echo Compiling $<...
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) ${<} -o ${@}

$(PROJ_ROOT)/$(OBJDIR)/%.o: $(PROJ_ROOT)/%.s
	@echo Compiling $<...
	@mkdir -p $(dir $@)
	$(AS) -c $(CFLAGS) $< -o $@

$(TARGET).elf: $(OBJS)
	@echo Linking...
	$(CC) $(CFLAGS) $(LFLAGS) -o ${TARGET}.elf $(OBJS) $(LINK_LIBS)

clean:
	find ./ -name '*~' | xargs rm -f	
	rm -f *.o
	rm -f $(OBJS)
	rm -f $(DEPS)
	rm -f $(OBJDIR)/*.o
	rm -f $(OBJDIR)/*.d
	rm -f $(TARGET).elf
	rm -f $(TARGET).hex
	rm -f $(TARGET).bin
	rm -f $(TARGET).map
	rm -f $(TARGET).lst

debug:
	$(JLINKGDB) -xc jlinkgdb.conf -device PAC55XX -singlerun -strict -timeout 0 -nogui

libs:
	make -C libs/selex-libc/canopen_clib all
test_suit:
	make -C libs/selex-libc/test-suit/ -f test_suit.mk all
test_suit_clean:
	make -C libs/selex-libc/test-suit/ -f test_suit.mk clean
	
flash:
	./flasher.sh
	
