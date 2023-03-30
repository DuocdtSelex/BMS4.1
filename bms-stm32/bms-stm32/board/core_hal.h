/*
 * core.h
 *
 *  Created on: Sep 16, 2020
 *      Author: quangnd
 */

#ifndef BOARD_CORE_HAL_H_
#define BOARD_CORE_HAL_H_
#include "compiler_optimize.h"
#include "core_hw.h"
#define HAL_SYSTICK_FREQ_Hz                                     SYSTICK_FREQ_Hz
void core_hw_init(void) WEAK;
int32_t core_read_id(char* id) WEAK;
void global_interrupt_enable(void) WEAK;
void global_interrupt_disable(void) WEAK;

#endif /* BOARD_CORE_HAL_H_ */
