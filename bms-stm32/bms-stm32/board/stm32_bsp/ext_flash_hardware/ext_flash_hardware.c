#include "ext_flash_hardware.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

#define FLASH_TIMEOUT       30000

static void ext_flash_clock_source_init(void);
static void ext_flash_gpio_hardware_init(void);
static void ext_flash_spi_hardware_init(void);

static bool flash_wait_for_flag(uint16_t flag, uint32_t status);

void external_flash_hardware_init(){
    ext_flash_clock_source_init();
    ext_flash_gpio_hardware_init();
    ext_flash_spi_hardware_init();
}

static void ext_flash_clock_source_init(void){
	RCC_AHBPeriphClockCmd(EXT_FLASH_SPI_CS_CLK,ENABLE);
	RCC_AHBPeriphClockCmd(EXT_FLASH_SPI_MOSI_GPIO_CLK,ENABLE);
	RCC_AHBPeriphClockCmd(EXT_FLASH_SPI_MISO_GPIO_CLK,ENABLE);
	RCC_AHBPeriphClockCmd(EXT_FLASH_SPI_SCK_GPIO_CLK,ENABLE);
	/* EXT_FLASH_SPI Periph clock enable */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	RCC_APB2PeriphClockCmd(EXT_FLASH_SPI_CLK, ENABLE);
}

static void ext_flash_gpio_hardware_init(void){

	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Configure EXT_FLASH_SPI pins: SCK */
	GPIO_InitStructure.GPIO_Pin = EXT_FLASH_SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(EXT_FLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure EXT_FLASH_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = EXT_FLASH_SPI_MISO_PIN;
	GPIO_Init(EXT_FLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure EXT_FLASH_SPI pins: MOSI */
	GPIO_InitStructure.GPIO_Pin = EXT_FLASH_SPI_MOSI_PIN;
	GPIO_Init(EXT_FLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = EXT_FLASH_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(EXT_FLASH_SPI_CS_PORT, &GPIO_InitStructure);
	
	/* Connect PXx to EXT_FLASH_SPI_SCK */
	GPIO_PinAFConfig(EXT_FLASH_SPI_SCK_GPIO_PORT, EXT_FLASH_SPI_SCK_SOURCE, EXT_FLASH_SPI_SCK_AF);
	
	/* Connect PXx to EXT_FLASH_SPI_MISO */
	GPIO_PinAFConfig(EXT_FLASH_SPI_MISO_GPIO_PORT, EXT_FLASH_SPI_MISO_SOURCE, EXT_FLASH_SPI_MISO_AF);
	
	/* Connect PXx to EXT_FLASH_SPI_MOSI */
	GPIO_PinAFConfig(EXT_FLASH_SPI_MOSI_GPIO_PORT, EXT_FLASH_SPI_MOSI_SOURCE, EXT_FLASH_SPI_MOSI_AF);

	GPIO_SetBits(EXT_FLASH_SPI_CS_PORT,EXT_FLASH_SPI_CS_PIN);
}

static void ext_flash_spi_hardware_init(void){
	SPI_InitTypeDef   SPI_InitStructure;
		
	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(EXT_FLASH_SPI_DEV);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

	/* Configure SPI mode 0, buadrate = 48/2/32(MHz) */
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(EXT_FLASH_SPI_DEV, &SPI_InitStructure);
	
	SPI_RxFIFOThresholdConfig(EXT_FLASH_SPI_DEV, SPI_RxFIFOThreshold_QF);
	
	/* disable CRC */
	SPI1->CR1 &= ~(1<<13);

	SPI_Cmd(EXT_FLASH_SPI_DEV, ENABLE); /* EXT_FLASH_SPI enable */
}

void external_flash_chip_select_active(void){
	GPIO_ResetBits(EXT_FLASH_SPI_CS_PORT, EXT_FLASH_SPI_CS_PIN);
}

void external_flash_chip_select_deactive(void){
	GPIO_SetBits(EXT_FLASH_SPI_CS_PORT, EXT_FLASH_SPI_CS_PIN);
}

static bool flash_wait_for_flag(uint16_t flag, uint32_t status) {
	uint32_t timeout = 0;
	while (SPI_I2S_GetFlagStatus(EXT_FLASH_SPI_DEV, flag) == status) {
		if (timeout++ > FLASH_TIMEOUT)
			return false;
	}
	return true;
}

int ext_flash_send_bytes(const uint8_t* p_data, const uint8_t len) {

	uint8_t data = 0;
	uint8_t index = 0;
	while (index < len) {
		/* Wait until the transmit buffer is empty */
		if (!flash_wait_for_flag(SPI_I2S_FLAG_TXE, RESET))
			return FLASH_FAIL;

		/* Send the byte */
		SPI_SendData8(EXT_FLASH_SPI_DEV, p_data[index]);
		/* Wait to receive a byte*/
		if (!flash_wait_for_flag(SPI_I2S_FLAG_RXNE, RESET))
			return FLASH_FAIL;
		/* Return the byte read from the SPI bus */
		data = SPI_ReceiveData8(EXT_FLASH_SPI_DEV);
		index++;
	}
	return 0;
}

int ext_flash_get_bytes(uint8_t* p_data, const uint8_t len) {

	uint8_t index = 0;
	while (index < len) {
		/* Wait until the transmit buffer is empty */
		if (!flash_wait_for_flag(SPI_I2S_FLAG_TXE, RESET))
			return FLASH_FAIL;
		/* Send the byte */
		SPI_SendData8(EXT_FLASH_SPI_DEV, EXT_FLASH_DUMMY_BYTE);

		/* Wait until a data is received */
		if (!flash_wait_for_flag(SPI_I2S_FLAG_RXNE, RESET))
			return FLASH_FAIL;

		/* Return the shifted data */
		p_data[index] = SPI_ReceiveData8(EXT_FLASH_SPI_DEV);
		index++;
	}
	return 0;
}

void ext_flash_flush_rx_buffer(void) {
	uint8_t data = 0;
	/* flush rx buffer */
	while (SPI_GetReceptionFIFOStatus(EXT_FLASH_SPI_DEV)
			!= SPI_ReceptionFIFOStatus_Empty) {
		data = SPI_ReceiveData8(EXT_FLASH_SPI_DEV);
	}

}

void ext_flash_send_dummy_byte(void) {

	uint8_t dummy_data = 0;
	/* Wait until the transmit buffer is empty */
	if (!flash_wait_for_flag(SPI_I2S_FLAG_TXE, RESET))
		return;
	/* Send the byte */
	SPI_SendData8(EXT_FLASH_SPI_DEV, EXT_FLASH_DUMMY_BYTE);

	/* Wait until a data is received */
	if (!flash_wait_for_flag(SPI_I2S_FLAG_RXNE, RESET))
		return;

	/* Return the shifted data */
	dummy_data = SPI_ReceiveData8(EXT_FLASH_SPI_DEV);
}

