#ifndef HC05_H_
#define HC05_H_
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

typedef struct HC05_HW_t HC05_HW;
struct HC05_HW_t{
	USART_TypeDef* uart_module;
};

extern HC05_HW hc05_port;

#define HC05_COM                        USART2
#define HC05_COM_CLK                    RCC_APB1Periph_USART2

#define HC05_COM_TX_PIN                 GPIO_Pin_2
#define HC05_COM_TX_GPIO_PORT           GPIOA
#define HC05_COM_TX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define HC05_COM_TX_SOURCE              GPIO_PinSource2
#define HC05_COM_TX_AF                  GPIO_AF_1

#define HC05_COM_RX_PIN                 GPIO_Pin_3
#define HC05_COM_RX_GPIO_PORT           GPIOA
#define HC05_COM_RX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define HC05_COM_RX_SOURCE              GPIO_PinSource3
#define HC05_COM_RX_AF                  GPIO_AF_1

#define HC05_COM_IRQn                   USART2_IRQn
#define HC05_COM_HANLDE			USART2_IRQHandler
#define HC05_COM_IRQn_Priority		3

typedef void (*Uart_Receive_Handle)(const char c);

//void debug_port_set_receive_handle(Uart_Receive_Handle handle);
//void USART_Write_String(char *a);
void hc05_hw_init(void);
void hc05_hw_sends(HC05_HW* p_hw,const uint8_t* s);
void hc05_send(HC05_HW* p_hw,const char c);
void hc05_set_receive_handle(Uart_Receive_Handle handle);
uint16_t hc05_receive_characters(HC05_HW *p_hw);

#endif
