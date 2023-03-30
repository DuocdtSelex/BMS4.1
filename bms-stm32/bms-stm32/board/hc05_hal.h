/*
 * hc05_hal.h
 *
 *  Created on: Nov 3, 2020
 *      Author: quangnd
 */

#ifndef BOARD_HC05_HAL_H_
#define BOARD_HC05_HAL_H_
#include "hc05_hw.h"
#include "compiler_optimize.h"

void hc05_hw_init(void) WEAK;
void hc05_hw_sends(HC05_HW* p_hw,const uint8_t* s) WEAK;
void hc05_set_receive_handle(Uart_Receive_Handle handle)WEAK;
uint16_t hc05_receive_characters(HC05_HW *p_hw)WEAK;

#endif /* BOARD_HC05_HAL_H_ */
