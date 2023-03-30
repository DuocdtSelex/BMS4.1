/*
 * data_logger_init.c
 *
 *  Created on: Nov 5, 2021
 *      Author: Admin
 */
#include "data_logger_init.h"
#include "internal_flash_hal.h"

Data_Logger* data_logger;

#define INT_FLASH_PAGE_127_START_ADDR   ((uint32_t)0x0801F800)   /* Start @ of user Flash area */
#define INT_FLASH_PAGE_127_END_ADDR     ((uint32_t)0x0801FFFF)   /* End @ of user Flash area */

static void int_flash_erase_data_impl(Data_Logger *p_data);
static void int_flash_write_data_impl(Data_Logger *p_data);
static void int_flash_read_data_impl(Data_Logger *p_data);

void data_logger_init(void){
	data_logger = data_logger_create();
	data_logger->start_address = INT_FLASH_PAGE_127_START_ADDR;
	data_logger->finish_address = INT_FLASH_PAGE_127_END_ADDR;
	data_logger->index =0;
	data_logger->data_error =0;
	data_logger->erase_data_error = int_flash_erase_data_impl;
	data_logger->write_data_error = int_flash_write_data_impl;
	data_logger->read_data_error = int_flash_read_data_impl;
}

static void int_flash_erase_data_impl(Data_Logger *p_data){
	int_flash_erase_data(p_data->start_address, p_data->finish_address);
}
static void int_flash_write_data_impl(Data_Logger *p_data){
	int_flash_write_data(p_data->start_address, p_data->finish_address, p_data->data_error, &p_data->index);
}
static void int_flash_read_data_impl(Data_Logger *p_data){
	int_flash_read_data(p_data->start_address, p_data->finish_address, p_data->data_buffer, &p_data->index);
}
