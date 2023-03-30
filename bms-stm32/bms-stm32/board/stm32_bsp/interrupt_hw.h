/*
 * interrupt_hw.h
 *
 *  Created on: Aug 22, 2020
 *      Author: quangnd
 */

#ifndef BOARD_STM32_BSP_INTERRUPT_HW_H_
#define BOARD_STM32_BSP_INTERRUPT_HW_H_


#define STATE_MACHINE_UPDATE_TICK SysTick_Handler
#define DISABLE_STM_UPDATE_TICK	 		SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |\
                   SysTick_CTRL_TICKINT_Msk   |\
                   SysTick_CTRL_ENABLE_Msk

#define ENABLE_STM_UPDATE_TICK SysTick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk |\
                   SysTick_CTRL_TICKINT_Msk   |\
                   SysTick_CTRL_ENABLE_Msk)

#endif /* BOARD_STM32_BSP_INTERRUPT_HW_H_ */
