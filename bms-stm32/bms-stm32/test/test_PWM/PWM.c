#include "stm32f0xx.h"

uint16_t TimerPrescaler = 0;
uint16_t Channel1Pulse = 0;
static void TIM_Config(void);


static void GPIO_OUT_config(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	/* GPIOA Configuration: Channel 1 as alternate function push-pull */
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void GPIO_AF_config(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	/* GPIOA Configuration: Channel 1 as alternate function push-pull */
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

int main(void)
{

  TIM_Config();

  while (1)
  {

	  GPIO_OUT_config();
	  delay_ms(1000);
	  GPIO_AF_config();
  }
}

static void TIM_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA, GPIOB and GPIOE Clocks enable */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);

  /* GPIOA Configuration: Channel 1 as alternate function push-pull */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  //GPIO_Init(GPIOA, &GPIO_InitStructure);

  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);

  /* GPIOB Configuration: Channel 1 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_0);

  TimerPrescaler = (SystemCoreClock /10000 ) - 1;

  /* TIM1 clock enable */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPrescaler;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM14,&TIM_TimeBaseStructure);


  uint16_t   pulse_length = ((TimerPrescaler + 1) * 70) / 100 - 1;
  /* Channel 1 PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = pulse_length;
  //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //low voltage in active mode
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  //TIM_OC1Init(TIM1, &TIM_OCInitStructure); //TIM_OC1Init for enable the channel 1
  TIM_OC1Init(TIM14, &TIM_OCInitStructure);
  /* TIM1 counter enable */
  //TIM_Cmd(TIM1, ENABLE);
  TIM_Cmd(TIM14, ENABLE);
  /* TIM1 Main Output Enable */
  //TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM14, ENABLE);
}
