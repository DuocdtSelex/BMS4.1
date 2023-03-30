#include "debug_com_port_hardware.h"

#include "stdio.h"
#include "stdlib.h"

DEBUG_COM_HW debug_port;

void debug_com_hw_init(void){
	debug_port.uart_module=DEBUG_PORT_COM;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(DEBUG_PORT_COM_TX_GPIO_CLK,ENABLE);

  	RCC_APB1PeriphClockCmd(DEBUG_PORT_COM_CLK, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(DEBUG_PORT_COM_TX_GPIO_PORT,DEBUG_PORT_COM_TX_SOURCE,DEBUG_PORT_COM_TX_AF);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(DEBUG_PORT_COM_RX_GPIO_PORT,DEBUG_PORT_COM_RX_SOURCE,DEBUG_PORT_COM_RX_AF);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = DEBUG_PORT_COM_TX_PIN | DEBUG_PORT_COM_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(DEBUG_PORT_COM_TX_GPIO_PORT, &GPIO_InitStructure);


   	USART_ITConfig(DEBUG_PORT_COM, USART_IT_RXNE, ENABLE);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(DEBUG_PORT_COM,&USART_InitStructure);

	USART_ITConfig(DEBUG_PORT_COM,USART_IT_RXNE,ENABLE);
	/* Enable USART */
	USART_Cmd(DEBUG_PORT_COM, ENABLE);

}

void debug_sends(DEBUG_COM_HW* p_hw,const uint8_t* s){

    while(*s){
    	      USART_SendData(p_hw->uart_module,*s);
    	      s++;
    }
}
//Uart_Receive_Handle USART2_Receive_Handle;

///////////
void Delay(__IO uint32_t nTime) {
	uint32_t a = nTime * 480;
	while (a-- != 0);
}

///////////

#if 0
void DEBUG_PORT_COM_HANLDE(void){
	if(USART_GetITStatus(DEBUG_PORT_COM, USART_IT_RXNE) != RESET)
	{
		if(USART2_Receive_Handle != NULL){
			USART2_Receive_Handle();
		}

	}
}

void debug_port_set_receive_handle(Uart_Receive_Handle handle){
	USART2_Receive_Handle =handle;
}
#endif

