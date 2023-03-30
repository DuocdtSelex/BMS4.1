/*
 * node_id_hal.h
 *
 *  Created on: Apr 2, 2021
 *      Author: Admin
 */

#ifndef BOARD_NODE_ID_HAL_H_
#define BOARD_NODE_ID_HAL_H_

#include "node_id_hw.h"
#include "compiler_optimize.h"

void node_id_hw_init(void) WEAK;
uint8_t node_id_read_input(void) WEAK;
#endif /* BOARD_NODE_ID_HAL_H_ */
