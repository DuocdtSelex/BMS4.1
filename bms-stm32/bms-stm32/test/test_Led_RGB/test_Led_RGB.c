/*
 * test_Led_RGB.c
 *
 *  Created on: Sep 9, 2020
 *      Author: son19
 */
#include "stm32f0xx_conf.h"
#include "delay_hw.h"

#define LED_GREEN_PIN			GPIO_Pin_2
#define LED_GREEN_PORT			GPIOB
#define LED_GREEN_CLK			RCC_AHBPeriph_GPIOB
#define LED_GREEN_INDEX			1

#define LED_RED_PIN				GPIO_Pin_3
#define LED_RED_PORT			GPIOB
#define LED_RED_CLK				RCC_AHBPeriph_GPIOB
#define LED_RED_INDEX			0

#define LED_BLUE_PIN			GPIO_Pin_12
#define LED_BLUE_PORT			GPIOA
#define LED_BLUE_CLK			RCC_AHBPeriph_GPIOA
#define LED_BLUE_INDEX			2

GPIO_InitTypeDef GPIO_InitStructure;

uint16_t INDICATOR_LED_PIN[3] ={
		LED_RED_PIN,
		LED_GREEN_PIN,
		LED_BLUE_PIN,
};

GPIO_TypeDef * INDICATOR_LED_PORT[3] = {
		LED_RED_PORT,
		LED_GREEN_PORT,
		LED_BLUE_PORT,
};

void GPIO_InitStruct(void)
{
	RCC_AHBPeriphClockCmd(LED_RED_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(LED_BLUE_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin   = LED_RED_PIN|LED_GREEN_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN;
	GPIO_Init(LED_BLUE_PORT,&GPIO_InitStructure);


}


void led_switch_off(uint8_t index)
{
	GPIO_ResetBits(INDICATOR_LED_PORT[index], INDICATOR_LED_PIN[index]);
}
void led_switch_on(uint8_t index)
{
	GPIO_SetBits(INDICATOR_LED_PORT[index], INDICATOR_LED_PIN[index]);
}

void led_switch_off_all()
{
    uint8_t led_index=0;
    for(led_index=0;led_index<3;led_index++){
        led_switch_off(led_index);
    }
}


void led_switch_on_all()
{
    uint8_t led_index=0;
    for(led_index=0;led_index<3;led_index++){
        led_switch_on(led_index);
    }
}


int main(void)
{
	GPIO_InitStruct();


	uint8_t count = 0;
	for (int i =0; i <3; i++)
	{
		for(count = 0;count < 15;count++)
		{
			led_switch_on(i);
			hw_delay_ms(250);
			led_switch_off(i);
			hw_delay_ms(250);
		}
	}
		led_switch_on_all();
		hw_delay_ms(120000); // 2 minute
		led_switch_off_all();

	while(1);

	return 1;
}

