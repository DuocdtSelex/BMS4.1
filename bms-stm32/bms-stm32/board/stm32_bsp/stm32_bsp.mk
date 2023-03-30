BSP_SRCS=  \
	system_stm32f0xx.c \
	stm32f0xx_it.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_adc.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_can.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_cec.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_comp.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_dma.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_exti.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_gpio.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_i2c.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_misc.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_pwr.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_rcc.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_spi.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_tim.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_usart.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_syscfg.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_flash.c \
#        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_crc.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_crs.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_wwdg.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_rtc.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_iwdg.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_dac.c \
        sdk/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_dbgmcu.c \
        
 

BSP_SRCS+=$(BOARD_TYPE).c  \
	current_sense_hardware/current_sense_hardware.c \
	temp_sense_hardware/temp_sense_hardware.c \
	debug_com_port_hardware/debug_com_port_hardware.c \
	hc05_hw/hc05_hw.c \
	bq_hardware/bq_hardware.c \
	inrush_limiter_hardware/inrush_limiter_hardware.c \
	can_hardware/can_hardware.c \
	core_hw/core_hw.c \
	key_hw/key_hw.c \
	rgb_hw/rgb_hw.c \
	delay_hw/delay_hw.c \
	node_id_hw/node_id_hw.c\
	bq_hw/bq_hw.c\
	soc_hw/soc_hw.c \
	internal_flash_hw/internal_flash_hw.c \
	ext_flash_hardware/ext_flash_hardware.c 

# ASM sources
BSP_ASM_SRCS = sdk/CMSIS/Device/ST/STM32F0xx/Source/startup_stm32f072.s
BSP_SRCS+=$(BSP_ASM_SRCS)

# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
CFLAGS+= $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
ASFLAGS+= $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_STDPERIPH_DRIVER \
-DSTM32F072

# C includes
BSP_INCLUDES = . current_sense_hardware debug_com_port_hardware \
	bq_hardware \
	can_hardware \
	temp_sense_hardware \
	inrush_limiter_hardware \
	delay_hw \
	core_hw \
	key_hw \
	hc05_hw \
	rgb_hw \
	node_id_hw \
	bq_hw \
	soc_hw \
	internal_flash_hw \
	ext_flash_hardware \
	sdk/CMSIS/Include \
	sdk/STM32F0xx_StdPeriph_Driver/inc \
	sdk/CMSIS/Device/ST/STM32F0xx/Include \

# compile gcc flags
ASFLAGS+= $(AS_DEFS) $(AS_INCLUDES)

CFLAGS+= $(C_DEFS) $(C_INCLUDES)

#######################################
# LDFLAGS
#######################################
# link script
LD_FILES:=sdk/CMSIS/Device/ldscripts/stm32f0xx.ld
