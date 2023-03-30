/*
 * ext_flash_hardware_hal.h
 *
 *  Created on: Nov 23, 2021
 *      Author: Admin
 */

#ifndef BOARD_EXT_FLASH_HARDWARE_HAL_H_
#define BOARD_EXT_FLASH_HARDWARE_HAL_H_

#include "ext_flash_hardware.h"
#include "compiler_optimize.h"
#include "stdint.h"

void external_flash_hardware_init(void)WEAK;
void external_flash_chip_select_active(void)WEAK;
void external_flash_chip_select_deactive(void)WEAK;
int ext_flash_send_bytes(const uint8_t* p_data, const uint8_t len)WEAK;
int ext_flash_get_bytes(uint8_t* p_data, const uint8_t len)WEAK;
void ext_flash_flush_rx_buffer(void)WEAK;
void ext_flash_send_dummy_byte(void)WEAK;

#endif /* BOARD_EXT_FLASH_HARDWARE_HAL_H_ */
