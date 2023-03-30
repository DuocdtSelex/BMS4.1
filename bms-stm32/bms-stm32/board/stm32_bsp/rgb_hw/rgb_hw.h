/*
 * rgb_hw.h
 *
 *  Created on: Oct 25, 2020
 *      Author: nguyenquang
 */

#ifndef BOARD_STM32_BSP_RGB_HW_RGB_HW_H_
#define BOARD_STM32_BSP_RGB_HW_RGB_HW_H_
#include "stdint.h"

void rgb_hw_init(void);
void rgb_set_color(const uint8_t r,const uint8_t g,const uint8_t b,const uint8_t blink);
void rgb_blink_enable(void);
void rgb_blink_disable(void);
void rgb_off(void);


#endif /* BOARD_STM32_BSP_RGB_HW_RGB_HW_H_ */
