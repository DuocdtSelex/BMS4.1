/*
 * temp_sense_hal.h
 *
 *  Created on: Aug 24, 2020
 *      Author: quangnd
 */

#ifndef BOARD_TEMP_SENSE_HAL_H_
#define BOARD_TEMP_SENSE_HAL_H_
#include "stdint.h"
#include "temp_sense_hardware.h"
#include "compiler_optimize.h"

int32_t temp_sense_read_adc(void) WEAK;
int32_t temp_sense_read_offset(void)WEAK;

#endif /* BOARD_TEMP_SENSE_HAL_H_ */
