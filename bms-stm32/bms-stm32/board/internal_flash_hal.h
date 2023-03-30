/*
 * internal_flash_hal.h
 *
 *  Created on: Nov 5, 2021
 *      Author: Admin
 */

#ifndef BOARD_INTERNAL_FLASH_HAL_H_
#define BOARD_INTERNAL_FLASH_HAL_H_

#include "internal_flash_hw.h"
#include "compiler_optimize.h"
#include "stdint.h"

int int_flash_erase_data(uint32_t start_page_address, uint32_t end_page_address)WEAK;
int int_flash_write_data(uint32_t start_page_address, uint32_t end_page_address, uint32_t data, uint32_t *index)WEAK;
int int_flash_read_data(uint32_t start_address, uint32_t end_address, uint32_t *data, uint32_t *index)WEAK;

#endif /* BOARD_INTERNAL_FLASH_HAL_H_ */
