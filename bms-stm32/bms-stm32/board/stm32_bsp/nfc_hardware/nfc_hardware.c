#include "nfc_hardware.h"
#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_misc.h"

static void nfc_gpio_hardware_init();
static void nfc_clock_source_init();
static void nfc_spi_hardware_init();
static void nfc_interrupt_init();

void nfc_hardware_init(){
	nfc_clock_source_init();
	nfc_gpio_hardware_init();
	nfc_spi_hardware_init();
	nfc_interrupt_init();
}

static void nfc_clock_source_init(){
	RCC_AHBPeriphClockCmd(NFC_SPI_CS_CLK,ENABLE);
	RCC_AHBPeriphClockCmd(NFC_SPI_MOSI_GPIO_CLK,ENABLE);
	RCC_AHBPeriphClockCmd(NFC_SPI_MISO_GPIO_CLK,ENABLE);
	RCC_AHBPeriphClockCmd(NFC_SPI_SCK_GPIO_CLK,ENABLE);
	RCC_AHBPeriphClockCmd(NFC_ENABLE_CLK,ENABLE);
	RCC_AHBPeriphClockCmd(NFC_INTERRUPT_CLK,ENABLE);
	/* NFC_SPI Periph clock enable */
	RCC_APB1PeriphClockCmd(NFC_SPI_CLK, ENABLE); 
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}

static void nfc_gpio_hardware_init(){
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Configure NFC_SPI pins: SCK */
	GPIO_InitStructure.GPIO_Pin = NFC_SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(NFC_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure NFC_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = NFC_SPI_MISO_PIN;
	GPIO_Init(NFC_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure NFC_SPI pins: MOSI */
	GPIO_InitStructure.GPIO_Pin = NFC_SPI_MOSI_PIN;
	GPIO_Init(NFC_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = NFC_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(NFC_SPI_CS_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = NFC_ENABLE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(NFC_ENABLE_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = NFC_INTERRUPT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(NFC_INTERRUPT_PORT, &GPIO_InitStructure);
	
	/* Connect PXx to NFC_SPI_SCK */
	GPIO_PinAFConfig(NFC_SPI_SCK_GPIO_PORT, NFC_SPI_SCK_SOURCE, NFC_SPI_SCK_AF);
	
	/* Connect PXx to NFC_SPI_MISO */
	GPIO_PinAFConfig(NFC_SPI_MISO_GPIO_PORT, NFC_SPI_MISO_SOURCE, NFC_SPI_MISO_AF); 
	
	/* Connect PXx to NFC_SPI_MOSI */
	GPIO_PinAFConfig(NFC_SPI_MOSI_GPIO_PORT, NFC_SPI_MOSI_SOURCE, NFC_SPI_MOSI_AF);  
}

static void nfc_spi_hardware_init(){
	SPI_InitTypeDef   SPI_InitStructure;
		
	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(NFC_SPI_DEVICE);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(NFC_SPI_DEVICE, &SPI_InitStructure);
	
	SPI_RxFIFOThresholdConfig(NFC_SPI_DEVICE, SPI_RxFIFOThreshold_QF);
	
	SPI_Cmd(NFC_SPI_DEVICE, ENABLE); /* NFC_SPI enable */
}

static void nfc_interrupt_init(){

	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	SYSCFG_EXTILineConfig(NFC_INTERRUPT_PORT_SOURCE, NFC_INTERRUPT_PIN_SOURCE);
	
	EXTI_InitStructure.EXTI_Line =NFC_INTERRUPT_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Enable and set EXTI4_15 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = NFC_INTERRUPT_VECTOR;
	NVIC_InitStructure.NVIC_IRQChannelPriority =  NFC_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

