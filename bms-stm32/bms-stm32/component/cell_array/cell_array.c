/*
 * cell_array.c
 *
 *  Created on: Aug 19, 2020
 *      Author: quangnd
 */

#include "cell_bank.h"
#include "cell_array.h"

uint8_t cell_array_get_min_voltage_cell(const Cell_Bank* const p_cells,const uint8_t len,uint32_t* vol){
	uint32_t min=0xFFFFFFFF;
	uint8_t index=0;

	for(int i=0;i<len;i++){
		if(p_cells[i].is_short==0){
			if(p_cells[i].voltage<min){
				index=i;
				min=p_cells[i].voltage;
			}
		}
	}
	*vol=min;
	return index;
}

