#include "hc05_hw.h"
#include "stdio.h"
#include "stdlib.h"

HC05_HW hc05_port;

static Uart_Receive_Handle hc05_receive_handle;
static void hc05_nvic_init(void);

void hc05_hw_init(void){
	hc05_port.uart_module=HC05_COM;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(HC05_COM_TX_GPIO_CLK,ENABLE);

  	RCC_APB1PeriphClockCmd(HC05_COM_CLK, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(HC05_COM_TX_GPIO_PORT,HC05_COM_TX_SOURCE,HC05_COM_TX_AF);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(HC05_COM_RX_GPIO_PORT,HC05_COM_RX_SOURCE,HC05_COM_RX_AF);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = HC05_COM_TX_PIN | HC05_COM_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(HC05_COM_TX_GPIO_PORT, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_OverrunDetectionConfig(HC05_COM,USART_OVRDetection_Disable);

	/* USART configuration */
	USART_Init(HC05_COM,&USART_InitStructure);

   	USART_ITConfig(HC05_COM, USART_IT_RXNE, ENABLE);
	/* Enable USART */
	USART_Cmd(HC05_COM, ENABLE);
	hc05_nvic_init();
}

static void hc05_nvic_init(void)
{
	NVIC_InitTypeDef nvic_init_structure;
	nvic_init_structure.NVIC_IRQChannel = HC05_COM_IRQn;
	nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;
	nvic_init_structure.NVIC_IRQChannelPriority = HC05_COM_IRQn_Priority;
	NVIC_Init(&nvic_init_structure);
}

void hc05_hw_sends(HC05_HW* p_hw,const uint8_t* s){

    while(*s){
    	      USART_SendData(p_hw->uart_module,*s);
    	      s++;
    }
}

void hc05_send(HC05_HW* p_hw,const char c){
        USART_SendData(p_hw->uart_module,c);
}

uint16_t hc05_receive_characters(HC05_HW *p_hw){
	return USART_ReceiveData(p_hw->uart_module);
}

#if 0
void HC05_COM_HANLDE(void){
	if(USART_GetITStatus(HC05_COM, USART_IT_RXNE) != RESET)
	{
		if(hc05_receive_handle != NULL){
		        hc05_receive_handle(USART_ReceiveData(HC05_COM));
		}
	}

	if(USART_GetITStatus(HC05_COM,USART_IT_ORE)){
	        USART_ClearFlag(HC05_COM,USART_FLAG_ORE);
	}

}
#endif
void hc05_set_receive_handle(Uart_Receive_Handle handle){
	hc05_receive_handle =handle;
}

