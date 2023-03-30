#ifndef EXTERNAL_FLASH_HARDWARE_H_ 
#define EXTERNAL_FLASH_HARDWARE_H_

#include "stdbool.h"
#include "stdint.h"

#define EXT_FLASH_SPI_DEV          			SPI1

#define EXT_FLASH_SPI_CS_PORT				GPIOA
#define EXT_FLASH_SPI_CS_PIN				GPIO_Pin_4
#define EXT_FLASH_SPI_CS_CLK				RCC_AHBPeriph_GPIOA

#define EXT_FLASH_SPI_CLK                   RCC_APB2Periph_SPI1

#define EXT_FLASH_SPI_SCK_PIN               GPIO_Pin_5
#define EXT_FLASH_SPI_SCK_GPIO_PORT         GPIOA
#define EXT_FLASH_SPI_SCK_GPIO_CLK          RCC_AHBPeriph_GPIOA  
#define EXT_FLASH_SPI_SCK_SOURCE            GPIO_PinSource5
#define EXT_FLASH_SPI_SCK_AF                GPIO_AF_0

#define EXT_FLASH_SPI_MISO_PIN              GPIO_Pin_6
#define EXT_FLASH_SPI_MISO_GPIO_PORT        GPIOA
#define EXT_FLASH_SPI_MISO_GPIO_CLK         RCC_AHBPeriph_GPIOA
#define EXT_FLASH_SPI_MISO_SOURCE           GPIO_PinSource6
#define EXT_FLASH_SPI_MISO_AF               GPIO_AF_0

#define EXT_FLASH_SPI_MOSI_PIN              GPIO_Pin_7 
#define EXT_FLASH_SPI_MOSI_GPIO_PORT        GPIOA
#define EXT_FLASH_SPI_MOSI_GPIO_CLK         RCC_AHBPeriph_GPIOA  
#define EXT_FLASH_SPI_MOSI_SOURCE           GPIO_PinSource7
#define EXT_FLASH_SPI_MOSI_AF               GPIO_AF_0

#define EXT_FLASH_DUMMY_BYTE		    	0x00
#define EXT_FLASH_PAGE_SIZE          	   (1<<8) /* in byte */
#define EXT_FLASH_SECTOR_SIZE			   (1<<16)

enum{
    FLASH_SUCESS   = 0,
    FLASH_FAIL      = -1
};

void external_flash_hardware_init(void);

void external_flash_chip_select_active(void);
void external_flash_chip_select_deactive(void);

int ext_flash_send_bytes(const uint8_t* p_data, const uint8_t len);
int ext_flash_get_bytes(uint8_t* p_data, const uint8_t len);
void ext_flash_flush_rx_buffer(void);
void ext_flash_send_dummy_byte(void);

#endif
