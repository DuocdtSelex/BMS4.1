/*
 * inrush_limiter_hal.h
 *
 *  Created on: Aug 25, 2020
 *      Author: quangnd
 */

#ifndef BOARD_INRUSH_LIMITER_HAL_H_
#define BOARD_INRUSH_LIMITER_HAL_H_
#include "compiler_optimize.h"
#include "inrush_limiter_hardware.h"

void inrush_limiter_switch_on(void)WEAK;
void inrush_limiter_switch_off(void)WEAK;

#endif /* BOARD_INRUSH_LIMITER_HAL_H_ */
