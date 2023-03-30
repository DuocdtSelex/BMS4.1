/*
 * soc_hal.h
 *
 *  Created on: Sep 23, 2021
 *      Author: Admin
 */

#ifndef BOARD_SOC_HAL_H_
#define BOARD_SOC_HAL_H_


#include "compiler_optimize.h"
#include "stdint.h"
#include "soc_hw.h"

void soc_hw_init(void) WEAK;


#endif /* BOARD_SOC_HAL_H_ */
