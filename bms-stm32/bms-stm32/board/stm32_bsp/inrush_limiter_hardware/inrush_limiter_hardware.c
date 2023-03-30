/*
 * inrush_limiter_hardware.c
 *
 *  Created on: Aug 25, 2020
 *      Author: quangnd
 */

#include "inrush_limiter_hardware.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

#define INRUSH_LIM_PIN					GPIO_Pin_1
#define INRUSH_LIM_PORT					GPIOB
#define INRUSH_LIM_CLK					RCC_AHBPeriph_GPIOB


void inrush_limter_hardware_init(void){

		RCC_AHBPeriphClockCmd(INRUSH_LIM_CLK, ENABLE);
		GPIO_InitTypeDef GPIO_InitStructure;
		/* Configure PB2 and PB3 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = INRUSH_LIM_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(INRUSH_LIM_PORT, &GPIO_InitStructure);
}

#ifdef FW_VERSION_V4_1
void inrush_limiter_switch_on(void){

		GPIO_SetBits(INRUSH_LIM_PORT,INRUSH_LIM_PIN);
}
void inrush_limiter_switch_off(void){

		GPIO_ResetBits(INRUSH_LIM_PORT,INRUSH_LIM_PIN);

}
#endif

#ifdef FW_VERSION_V4_0
void inrush_limiter_switch_on(void){

		GPIO_ResetBits(INRUSH_LIM_PORT,INRUSH_LIM_PIN);

}
void inrush_limiter_switch_off(void){

		GPIO_SetBits(INRUSH_LIM_PORT,INRUSH_LIM_PIN);
}
#endif

