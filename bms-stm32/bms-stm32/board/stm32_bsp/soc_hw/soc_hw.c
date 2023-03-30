/*
 * soc_hw.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Admin
 */


#include "soc_hw.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

static void timer_soc_hardware_clock_init(void) {
	RCC_APB2PeriphClockCmd(TIMER_SOC_CLK,ENABLE);
}

static void timer_soc_hardware_timer_init(void){

	TIM_TimeBaseInitTypeDef TimBase_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	TimBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimBase_InitStructure.TIM_Prescaler  = TIMER_SOC_PRESCALER;
	TimBase_InitStructure.TIM_Period =TIMER_SOC_PERIOD;
	TimBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimBase_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMER_SOC,&TimBase_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPriority = TIMER_SOC_INTERRUPT_PRIORITY;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearFlag(TIMER_SOC,TIM_FLAG_Update);
	TIM_ITConfig(TIMER_SOC,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIMER_SOC,ENABLE);
}

void soc_hw_init(void){
	timer_soc_hardware_clock_init();
	timer_soc_hardware_timer_init();
}
