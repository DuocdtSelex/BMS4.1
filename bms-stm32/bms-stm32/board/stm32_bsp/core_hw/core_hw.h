/*
 * core_hw.h
 *
 *  Created on: Sep 16, 2020
 *      Author: quangnd
 */

#ifndef BOARD_STM32_BSP_CORE_HW_H_
#define BOARD_STM32_BSP_CORE_HW_H_
#define SYSTICK_FREQ_Hz                                     100
void core_hw_init(void);
int32_t core_read_id(char* id);

#endif /* BOARD_STM32_BSP_CORE_HW_H_ */
