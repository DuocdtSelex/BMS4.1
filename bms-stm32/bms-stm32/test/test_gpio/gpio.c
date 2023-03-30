#include "gpio.h"
#include "stm32f0xx_gpio.h"

#define LED_0_PIN                 GPIO_Pin_1

void GPIO_init_pin() {
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOB Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* Configure PB2 and PB3 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = LED_0_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

int main(void) {
	GPIO_init_pin();
	while (1) {
		GPIO_SetBits(GPIOB, LED_0_PIN );

		delay_ms(1000);
		GPIO_ResetBits(GPIOB, LED_0_PIN);

		delay_ms(1000);
	}
}
