/*
 * soc_hw.h
 *
 *  Created on: Sep 23, 2021
 *      Author: Admin
 */

#ifndef BOARD_STM32_BSP_SOC_HW_SOC_HW_H_
#define BOARD_STM32_BSP_SOC_HW_SOC_HW_H_

#include "stdint.h"

#define TIMER_SOC						TIM15
#define TIMER_SOC_PRESCALER				39999
#define TIMER_SOC_PERIOD				1199
#define TIMER_SOC_CLK					RCC_APB2Periph_TIM15
#define TIMER_SOC_INTERRUPT_PRIORITY    2
#define TIMER_SOC_UPDATE_HANDLE			TIM15_IRQHandler

void soc_hw_init(void);

#endif /* BOARD_STM32_BSP_SOC_HW_SOC_HW_H_ */
