/*
 * key_hw.h
 *
 *  Created on: Oct 20, 2020
 *      Author: quangnd
 */

#ifndef BOARD_STM32_BSP_KEY_HW_KEY_HW_H_
#define BOARD_STM32_BSP_KEY_HW_KEY_HW_H_
#include "stdint.h"

void key_hw_init(void);
uint8_t key_read(void);
#endif /* BOARD_STM32_BSP_KEY_HW_KEY_HW_H_ */
