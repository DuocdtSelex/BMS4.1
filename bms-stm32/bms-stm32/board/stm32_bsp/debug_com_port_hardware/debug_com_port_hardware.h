#ifndef DEBUG_PORT_H_
#define DEBUG_PORT_H_
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

typedef struct DEBUG_COM_HW_t DEBUG_COM_HW;
struct DEBUG_COM_HW_t{
	USART_TypeDef* uart_module;
};

extern DEBUG_COM_HW debug_port;

void debug_com_hw_init(void);
void debug_com_sends(DEBUG_COM_HW* p_hw,const uint8_t* s);


#define DEBUG_PORT_COM                        USART2
#define DEBUG_PORT_COM_CLK                    RCC_APB1Periph_USART2

#define DEBUG_PORT_COM_TX_PIN                 GPIO_Pin_2
#define DEBUG_PORT_COM_TX_GPIO_PORT           GPIOA
#define DEBUG_PORT_COM_TX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define DEBUG_PORT_COM_TX_SOURCE              GPIO_PinSource2
#define DEBUG_PORT_COM_TX_AF                  GPIO_AF_1

#define DEBUG_PORT_COM_RX_PIN                 GPIO_Pin_3
#define DEBUG_PORT_COM_RX_GPIO_PORT           GPIOA
#define DEBUG_PORT_COM_RX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define DEBUG_PORT_COM_RX_SOURCE              GPIO_PinSource3
#define DEBUG_PORT_COM_RX_AF                  GPIO_AF_1

//#define DEBUG_PORT_COM_IRQn                   USART2_IRQn
//#define DEBUG_PORT_COM_HANLDE				  USART2_IRQHandler
//#define DEBUG_PORT_COM_IRQn_Priority		  3

//typedef void (*Uart_Receive_Handle)();

void debug_com_hw_init(void);
//void debug_port_set_receive_handle(Uart_Receive_Handle handle);
//void USART_Write_String(char *a);
void debug_sends(DEBUG_COM_HW* p_hw,const uint8_t* s);
#endif
