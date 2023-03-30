/*
 * key_hw.c
 *
 *  Created on: Oct 20, 2020
 *      Author: quangnd
 */

#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

#define KEY_PIN					GPIO_Pin_13
#define KEY_PORT				GPIOC
#define KEY_CLK					RCC_AHBPeriph_GPIOC

void key_hw_init(void){
		RCC_AHBPeriphClockCmd(KEY_CLK, ENABLE);
		GPIO_InitTypeDef GPIO_InitStructure;
		/* Configure PB2 and PB3 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = KEY_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(KEY_PORT, &GPIO_InitStructure);
}

uint8_t key_read(void){
        return GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN);
}


