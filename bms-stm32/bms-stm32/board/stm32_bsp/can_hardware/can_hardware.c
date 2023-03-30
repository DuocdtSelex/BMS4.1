#include "can_hardware.h"
#include "string.h"

CAN_Hw can1;
static void can_set_baudrate_impl(CAN_Hw* p_hw,uint32_t baudrate) ;
static void can_send_impl(CAN_Hw* p_hw,CanTxMsg* p_msg);

static CAN_InitTypeDef CAN_InitStructure;
static void can_hardware_init_clk(void);
static void can_hardware_init_gpio(void);
static void can_hardware_init_module(void);
static void can_hardware_init_nvic(void);

void can_hardware_init(void) {
	can_hardware_init_clk();
	can_hardware_init_gpio();
	can_hardware_init_module();
	can_hardware_init_nvic();
	can1.can_module=CAN;
	can1.send=can_send_impl;
	can1.set_baudrate=can_set_baudrate_impl;
}

static void can_hardware_init_clk(void) {
	RCC_APB1PeriphClockCmd(CAN_DEV_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(CAN_TX_PORT_CLK, ENABLE);
}

static void can_hardware_init_gpio(void) {

	GPIO_InitTypeDef GPIO_InitStructure;


	GPIO_PinAFConfig(CAN_TX_PORT, CAN_TX_PINSOURCE, CAN_AF_PORT);
	GPIO_PinAFConfig(CAN_RX_PORT, CAN_RX_PINSOURCE, CAN_AF_PORT);

	GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN_TX_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN_RX_PORT, &GPIO_InitStructure);
}

static void can_hardware_init_module(void) {

	CAN_FilterInitTypeDef CAN_FilterInitStructure;

	CAN_DeInit(CAN);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = ENABLE;
	CAN_InitStructure.CAN_NART = ENABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_Init(CAN, &CAN_InitStructure);
    /* buadrate = pclk/(BS1+1 + BS2+1 + 1)/Prescaler  */
	/* this configure is corresponding to 500 kbs */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_7tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
	CAN_InitStructure.CAN_Prescaler = CAN_PRE_DEFAULT;

	CAN_Init(CAN, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN, CAN_IT_FMP0, ENABLE);

}

static void can_hardware_init_nvic(void) {

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = CAN_IRQN;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPriority = CAN_IRQN_PRIORITY;
	NVIC_Init(&NVIC_InitStructure);
}

uint8_t can_tx_available(CAN_Hw* p_hw){

  /* Select one empty transmit mailbox */
  if ((p_hw->can_module->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
  {
	  return 1;
  }

  if ((p_hw->can_module->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
  {
	  return 1;
  }

  if ((p_hw->can_module->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
  {
	  return 1;
  }
  return 0;
}
static void can_send_impl(CAN_Hw* p_hw,CanTxMsg* p_msg){
#if 0
	uint32_t timeout=0;
	while(!can_tx_available(p_hw)){
		if(timeout++ > 100000)
			return;
	}
#endif
	CAN_Transmit(p_hw->can_module,p_msg);
}

static void can_set_baudrate_impl(CAN_Hw* p_hw,uint32_t baudrate) {
	p_hw->baudrate=baudrate;
	uint32_t pclk_val=48000000UL;
	 CAN_InitStructure.CAN_Prescaler = (uint32_t)(pclk_val/baudrate*((CAN_InitStructure.CAN_BS1+1+CAN_InitStructure.CAN_BS2+1+1)));
	 CAN->BTR = ((uint32_t) CAN_InitStructure.CAN_Prescaler - 1);
}

void CEC_CAN_IRQHandler(void) {

	if (CAN_GetITStatus(CAN, CAN_IT_FMP0) != RESET) {
		CAN_Receive(CAN, CAN_FIFO0,&can1.rx_msg);
		if (can1.receive_handle != NULL)
			can1.receive_handle(&can1);
		CAN_ClearITPendingBit(CAN, CAN_IT_FMP0);
	}
}



