/*
 * current_sense_hal.h
 *
 *  Created on: Aug 19, 2020
 *      Author: quangnd
 */

#ifndef BOARD_STM32_BSP_CURRENT_SENSE_HAL_H_
#define BOARD_STM32_BSP_CURRENT_SENSE_HAL_H_

#include "compiler_optimize.h"
#include "current_sense_hardware.h"

#define HAL_ADC_VREF_mV							ADC_VREF_mV
#define HAL_CURRENT_SENSE_R_SHUNT_mOhm			CURRENT_SENSE_R_SHUNT_mOhm

#if 0
void temp_sense_hardware_init(void) WEAK;
int32_t current_sense_calculate_gain(void) WEAK;
int32_t current_sense_read_adc(void) WEAK;
int32_t current_sense_read_offset(void)WEAK;
#endif

void temp_sense_hardware_init(void) WEAK;
int32_t ts5_temp_sense_read_adc(void) WEAK;
int32_t ts6_temp_sense_read_adc(void) WEAK;
#endif /* BOARD_STM32_BSP_CURRENT_SENSE_HAL_H_ */
