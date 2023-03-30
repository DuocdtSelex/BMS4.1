/*
 * cell_array.h
 *
 *  Created on: Aug 19, 2020
 *      Author: quangnd
 */

#ifndef COMPONENT_CELL_ARRAY_CELL_ARRAY_H_
#define COMPONENT_CELL_ARRAY_CELL_ARRAY_H_

#include "cell_bank.h"

typedef struct Cell_Array_t Cell_Array;

struct Cell_Array_t{

	Cell_Bank* cells;
	uint8_t serial_cells;
};

uint8_t cell_array_get_min_voltage_cell(const Cell_Bank* const p_cells,const uint8_t len,uint32_t* vol);

#endif /* COMPONENT_CELL_ARRAY_CELL_ARRAY_H_ */
