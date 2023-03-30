/*
 * data_logger.c
 *
 *  Created on: Nov 5, 2021
 *      Author: Admin
 */

#include "data_logger.h"
#include "stdlib.h"

Data_Logger* data_logger_create(void){

	Data_Logger* this=(Data_Logger*)malloc(sizeof(Data_Logger));
	while(this==NULL){};
	return this;
}

void data_logger_update_error(Data_Logger* p_data, uint32_t error){
	p_data->data_error = error;
}

