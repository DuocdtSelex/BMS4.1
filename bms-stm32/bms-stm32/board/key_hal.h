/*
 * key_hal.h
 *
 *  Created on: Oct 20, 2020
 *      Author: quangnd
 */

#ifndef BOARD_KEY_HAL_H_
#define BOARD_KEY_HAL_H_

#include "compiler_optimize.h"
#include "stdint.h"
#include "key_hw.h"

void key_hw_init(void) WEAK;
uint8_t key_read(void) WEAK;
#endif /* BOARD_KEY_HAL_H_ */
