//*****************************************************************************
//
// mcu.c - MCU Configuration and host interface APIs
//
// Copyright (c) 2015 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************
// Include Header Files
#include "mcu.h"
#include "board.h"
#include "exint_driver.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "timer.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_tim.h"

volatile uint8_t * g_timeout_ptr;
uint32_t g_ui32RemainingTime;
volatile uint8_t g_ui8TimeoutFlag;
/* 
 * Defines whether the timer is used for milliseconds (TIMER_COUNT_MS) 
 * or microseconds (TIME_COUNT_US)
 */
volatile uint8_t g_ui8TimerResolution;
bool g_bSerialConnectionEstablished = false;

#define 	TIMER_COUNT_MS		(uint8_t)0x01
#define		TIMER_COUNT_US		(uint8_t)0x02


static void start_timeout_timer_tick(uint8_t resolution,uint16_t tick_number);
static void stop_timeout_timer_tick();
static void mcu_delay(const uint8_t res,const uint16_t interval);

/**
 * @brief <b>Function Name</b>: MCU_init
 * @brief <b>Description</b>: Initialize MSP430, such as Clock, Port, MPU and
 * System Time.
 * @param Input value: None
 * @return Return value:None
 **/
void MCU_init(void){
    TIM_DeInit(TIM3);
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
#if 0
	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

  	TIM_Cmd(TIM3, DISABLE);
}

static void mcu_delay(const uint8_t res,const uint16_t interval){

   	g_timeout_ptr = (uint8_t *) &g_ui8TimeoutFlag;
	*g_timeout_ptr = 0x00;

	start_timeout_timer_tick(res,interval);
	while(*g_timeout_ptr != 0x01)
	{
	}
	MCU_timerDisable();
}

void MCU_delayMillisecond(uint16_t n_ms){
    
    mcu_delay(TIMER_COUNT_MS,n_ms);
}

void MCU_delayMicrosecond(uint16_t n_us){

    mcu_delay(TIMER_COUNT_US,n_us);
}

void MCU_timerInit(uint16_t timeout_ms, uint8_t * timeout_flag){

	g_timeout_ptr = (uint8_t *) timeout_flag;
	*g_timeout_ptr = 0x00;
	start_timeout_timer_tick(TIMER_COUNT_MS,timeout_ms);
}

void MCU_timerDisable(void){

	stop_timeout_timer_tick();
}

uint8_t convertNibbleToAscii(uint8_t ui8Nibble){

	uint8_t	ui8AsciiOut = ui8Nibble;

	if(ui8Nibble > 9)	// If req ASCII A-F then add 7(hex)
	{
		ui8AsciiOut = ui8AsciiOut + 0x07;
	}

	// Add offset to convert to Ascii
	ui8AsciiOut = ui8AsciiOut + 0x30;

	return(ui8AsciiOut);
}

void convertByteToAscii(uint8_t ui8byte, uint8_t pui8Buffer[3]){
	uint8_t	ui8Temp1 = 0;

	// Convert High Nibble
	ui8Temp1 = (ui8byte >> 4) & 0x0F;			// get high nibble
	pui8Buffer[0] = convertNibbleToAscii(ui8Temp1);		// convert to ASCII

	// Convert Low Nibble
	ui8Temp1 = ui8byte & 0x0F;					// get low nibble
	pui8Buffer[1] = convertNibbleToAscii(ui8Temp1);		// convert to ASCII

	pui8Buffer[2] = 0x00;
}

void convertWordToAscii(uint16_t ui16Word, uint8_t pui8Buffer[5]){
	uint8_t	ui8Temp1 = 0;

	// Convert High Nibble
	ui8Temp1 = (ui16Word >> 12) & 0x0F;			// get high nibble
	pui8Buffer[0] = convertNibbleToAscii(ui8Temp1);		// convert to ASCII

	ui8Temp1 = (ui16Word >> 8) & 0x0F;
	pui8Buffer[1] = convertNibbleToAscii(ui8Temp1);		// convert to ASCII

	ui8Temp1 = (ui16Word >> 4) & 0x0F;
	pui8Buffer[2] = convertNibbleToAscii(ui8Temp1);		// convert to ASCII

	// Convert Low Nibble
	ui8Temp1 = ui16Word & 0x0F;					// get low nibble
	pui8Buffer[3] = convertNibbleToAscii(ui8Temp1);		// convert to ASCII

	pui8Buffer[4] = 0x00;
}

void trf_enable(){
	uint16_t trf_enalbe_pin = NFC_ENABLE_PIN;
	gpio_set_bit((void*)NFC_ENABLE_PORT,(void*)&trf_enalbe_pin);
}

void trf_disable(){
	uint16_t trf_enalbe_pin = NFC_ENABLE_PIN;
	gpio_clear_bit((void*)NFC_ENABLE_PORT,(void*)&trf_enalbe_pin);
}

void trf_enable_interrupt(){
	extint_enable(NFC_INTERRUPT_CHANNEL);
}

void trf_disable_interrupt(){
	extint_disable(NFC_INTERRUPT_CHANNEL);
}

void trf_clear_interrupt_flag(){
	extint_clear_interrupt_flag(NFC_INTERRUPT_CHANNEL);
}

void trf_cs_set(){
	uint16_t spi_cs_pin=NFC_SPI_CS_PIN;
	gpio_set_bit((void*)NFC_SPI_CS_PORT,(void*)&spi_cs_pin);
}

void trf_cs_clear(){
	uint16_t spi_cs_pin=NFC_SPI_CS_PIN;
	gpio_clear_bit((void*)NFC_SPI_CS_PORT,(void*)&spi_cs_pin);
}

bool trf_interrupt_is_set(){

	GPIO_IN_LEVEL level=IN_LOW;
	uint16_t nfc_interrupt_pin=NFC_INTERRUPT_PIN;
	gpio_get_bit((void*)NFC_INTERRUPT_PORT,(void*)&nfc_interrupt_pin,&level);
	if(level==IN_HIGH){
		return true;
	}
	return false;
}

static void start_timeout_timer_tick(uint8_t resolution,uint16_t tick_number){

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t PrescalerValue =0;
	
	/* Compute the prescaler value */
	/* Witch SystemCoreClock=48Mhz */
	if(resolution == TIMER_COUNT_US){
		PrescalerValue = (uint16_t) (SystemCoreClock  / 1000000) - 1;
	}else if(resolution==TIMER_COUNT_MS){
		PrescalerValue = (uint16_t) (SystemCoreClock  / 1000) - 1;
	}

	g_ui8TimerResolution = resolution;
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = tick_number;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
    /*
    TIM_SetCounter(TIM3,0);
    TIM_ARRPreloadConfig(TIM3,ENABLE);
    TIM_SetAutoreload(TIM3,tick_number);
    */

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

	/* TIM3 enable counter */
	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void stop_timeout_timer_tick(){

	NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable the TIM3 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM3, DISABLE);
}

void TIM3_IRQHandler(){

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		*g_timeout_ptr = 0x01;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

