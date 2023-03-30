/*
 * cell_bank.h
 *
 *  Created on: Aug 19, 2020
 *      Author: quangnd
 */

#ifndef COMPONENT_CELL_BANK_CELL_BANK_H_
#define COMPONENT_CELL_BANK_CELL_BANK_H_

#include "stdint.h"

typedef struct Cell_Bank_t Cell_Bank;

struct Cell_Bank_t{
	uint32_t capacity;
	uint32_t voltage;
	uint8_t is_short;
};

static inline uint8_t cell_is_short(Cell_Bank* cell){
	return cell->is_short;
}


#endif /* COMPONENT_CELL_BANK_CELL_BANK_H_ */
