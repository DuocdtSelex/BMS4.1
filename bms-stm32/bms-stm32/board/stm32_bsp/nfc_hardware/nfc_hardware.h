#ifndef _NFC_HARDWARE_H_

#define NFC_SPI_DEVICE 				        SPI2
#define NFC_INTERRUPT_CHANNEL			    9
#define NFC_INTERRUPT_VECTOR			    EXTI4_15_IRQn
#define NFC_INTERRUPT_LINE			        EXTI_Line9
#define NFC_INTERRUPT_PRIORITY				1

#define NFC_ENABLE_PORT				        GPIOB
#define NFC_ENABLE_PIN				        GPIO_Pin_4
#define NFC_ENABLE_CLK				        RCC_AHBPeriph_GPIOB

#define NFC_INTERRUPT_PORT			        GPIOA
#define NFC_INTERRUPT_PIN			        GPIO_Pin_9
#define NFC_INTERRUPT_CLK			        RCC_AHBPeriph_GPIOA
#define NFC_INTERRUPT_PORT_SOURCE          	EXTI_PortSourceGPIOA
#define NFC_INTERRUPT_PIN_SOURCE           	EXTI_PinSource9


#define NFC_SPI_CS_PORT				        GPIOB
#define NFC_SPI_CS_PIN				        GPIO_Pin_12
#define NFC_SPI_CS_CLK				        RCC_AHBPeriph_GPIOB

#define NFC_SPI_CLK                   		RCC_APB1Periph_SPI2

#define NFC_SPI_SCK_PIN               		GPIO_Pin_13 
#define NFC_SPI_SCK_GPIO_PORT         		GPIOB
#define NFC_SPI_SCK_GPIO_CLK          		RCC_AHBPeriph_GPIOB  
#define NFC_SPI_SCK_SOURCE            		GPIO_PinSource13
#define NFC_SPI_SCK_AF                		GPIO_AF_0

#define NFC_SPI_MISO_PIN              		GPIO_Pin_14
#define NFC_SPI_MISO_GPIO_PORT        		GPIOB
#define NFC_SPI_MISO_GPIO_CLK         		RCC_AHBPeriph_GPIOB
#define NFC_SPI_MISO_SOURCE           		GPIO_PinSource14
#define NFC_SPI_MISO_AF               		GPIO_AF_0

#define NFC_SPI_MOSI_PIN              		GPIO_Pin_15 
#define NFC_SPI_MOSI_GPIO_PORT        		GPIOB
#define NFC_SPI_MOSI_GPIO_CLK         		RCC_AHBPeriph_GPIOB  
#define NFC_SPI_MOSI_SOURCE           		GPIO_PinSource15
#define NFC_SPI_MOSI_AF               		GPIO_AF_0

#define _NFC_HARDWARE_H_
void nfc_hardware_init();
#endif
