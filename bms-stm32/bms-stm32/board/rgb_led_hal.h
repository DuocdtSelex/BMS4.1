/*
 * rgb_led_hal.h
 *
 *  Created on: Aug 22, 2020
 *      Author: quangnd
 */

#ifndef BOARD_RGB_LED_HAL_H_
#define BOARD_RGB_LED_HAL_H_
#include "compiler_optimize.h"
#include "stdint.h"
#include "rgb_hw.h"

void rgb_hw_init(void) WEAK;
void rgb_set_color(const uint8_t r,const uint8_t g,const uint8_t b,const uint8_t blink) WEAK;

#endif /* BOARD_RGB_LED_HAL_H_ */
