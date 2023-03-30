/*
 * rgb_hw.c
 *
 *  Created on: Oct 25, 2020
 *      Author: nguyenquang
 */
#include "rgb_hw.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

#define R_LED_PORT                    GPIOB
#define R_LED_PIN                     GPIO_Pin_10
#define R_LED_CLK                     RCC_AHBPeriph_GPIOB

#define G_LED_PORT                    GPIOA
#define G_LED_PIN                     GPIO_Pin_12
#define G_LED_CLK                     RCC_AHBPeriph_GPIOA

#define B_LED_PORT                    GPIOB
#define B_LED_PIN                     GPIO_Pin_11
#define B_LED_CLK                     RCC_AHBPeriph_GPIOB


#define R_ON                          GPIO_SetBits(R_LED_PORT,R_LED_PIN)
#define G_ON                          GPIO_SetBits(G_LED_PORT,G_LED_PIN)
#define B_ON                          GPIO_SetBits(B_LED_PORT,B_LED_PIN)

#define R_OFF                          GPIO_ResetBits(R_LED_PORT,R_LED_PIN)
#define G_OFF                          GPIO_ResetBits(G_LED_PORT,G_LED_PIN)
#define B_OFF                          GPIO_ResetBits(B_LED_PORT,B_LED_PIN)

#define TIMER_TIM_DEV					TIM3
#define TIM_PRESCALER					65000
#define TIM_PERIOD						100
#define TIMER_TIM_DEV_CLK				RCC_APB1Periph_TIM3
#define TIMER_TIM_INTERRUPT_PRIORITY    1
#define TIMER_UPDATE_HANDLE				TIM3_IRQHandler

static void rgb_clock_init(void);
static void rgb_gpio_init(void);
static void rgb_hw_blink_timer_init(void);

static volatile uint8_t blink_enable=0;
static volatile uint8_t blink_state=0;

static volatile uint8_t led_r;
static volatile uint8_t led_g;
static volatile uint8_t led_b;

void rgb_hw_init(void){
        rgb_clock_init();
        rgb_gpio_init();
        rgb_hw_blink_timer_init();
}

void rgb_set_color(const uint8_t r,const uint8_t g,const uint8_t b,const uint8_t blink){

	blink_enable=blink;
    led_r=r;
    led_g=g;
    led_b=b;
}

void rgb_blink_enable(){
	if(blink_enable==0){
		blink_enable=1;
	}
}

void rgb_blink_disable(void){
	blink_enable=0;
}

void rgb_off(void){
	R_OFF;
	G_OFF;
	B_OFF;
}


static void rgb_clock_init(void){

        RCC_AHBPeriphClockCmd(R_LED_CLK, ENABLE);
        RCC_AHBPeriphClockCmd(G_LED_CLK, ENABLE);
        RCC_AHBPeriphClockCmd(B_LED_CLK, ENABLE);
}

static void rgb_gpio_init(void){
        GPIO_InitTypeDef GPIO_InitStructure;
        /* Configure PB2 and PB3 in output pushpull mode */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

        GPIO_InitStructure.GPIO_Pin = R_LED_PIN;
        GPIO_Init(R_LED_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = G_LED_PIN;
        GPIO_Init(G_LED_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = B_LED_PIN;
        GPIO_Init(B_LED_PORT, &GPIO_InitStructure);
}

static void timer_hardware_clock_init(void) {
	RCC_APB1PeriphClockCmd(TIMER_TIM_DEV_CLK,ENABLE);
}

static void timer_hardware_timer_init(void){

	TIM_TimeBaseInitTypeDef TimBase_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	TimBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
	TimBase_InitStructure.TIM_Prescaler  = TIM_PRESCALER;
	TimBase_InitStructure.TIM_Period =TIM_PERIOD;
	TimBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimBase_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMER_TIM_DEV,&TimBase_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPriority = TIMER_TIM_INTERRUPT_PRIORITY;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearFlag(TIMER_TIM_DEV,TIM_FLAG_Update);
	TIM_ITConfig(TIMER_TIM_DEV,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIMER_TIM_DEV,ENABLE);
}

void TIMER_UPDATE_HANDLE(void) {

	if(TIM_GetITStatus(TIMER_TIM_DEV, TIM_IT_Update) != RESET) {

		if(blink_enable){
				blink_state=~blink_state;
				if(blink_state){
					rgb_off();
				}else{
					(led_r>0)?R_ON:R_OFF;
					(led_g>0)?G_ON:G_OFF;
					(led_b>0)?B_ON:B_OFF;
				}
		}else{
			(led_r>0)?R_ON:R_OFF;
			(led_g>0)?G_ON:G_OFF;
			(led_b>0)?B_ON:B_OFF;
		}

		TIM_ClearFlag(TIMER_TIM_DEV, TIM_FLAG_Update);
	}
}

static void rgb_hw_blink_timer_init(void){
	timer_hardware_clock_init();
	timer_hardware_timer_init();
}


