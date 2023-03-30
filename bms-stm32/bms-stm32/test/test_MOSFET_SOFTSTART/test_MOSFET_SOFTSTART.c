/*
 * test_MOSFET_SOFTSTART.c
 *
 *  Created on: Sep 9, 2020
 *      Author: son19
 */
#include "stm32f0xx_conf.h"
#include "debug_com_port_hardware.h"

#define SOFTSTART_Pin	GPIO_Pin_1
#define SOFTSTART_Port	GPIOB
#define SOFTSTART_CLK	RCC_AHBPeriph_GPIOB

GPIO_InitTypeDef GPIO_InitStructure;

void Setup()
{
	RCC_AHBPeriphClockCmd(SOFTSTART_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin   = SOFTSTART_Pin;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(SOFTSTART_Port,&GPIO_InitStructure);

	debug_com_hw_init();
}

void main (void )
{
	uint8_t count = 0;
	Setup();
	GPIO_SetBits(SOFTSTART_Port, SOFTSTART_Pin);

	for(count = 0; count < 50; count++)
	{
		if (GPIO_ReadOutputDataBit(SOFTSTART_Port, SOFTSTART_Pin) == Bit_SET)
		{
			debug_sends(&debug_port,"ON\n");
			GPIO_ResetBits(SOFTSTART_Port, SOFTSTART_Pin);
			hw_delay_ms(1000);
		}else{
			debug_sends(&debug_port,"OFF\n");
			GPIO_SetBits(SOFTSTART_Port, SOFTSTART_Pin);
			hw_delay_ms(1000);
		}
	while(1);
	}

}

