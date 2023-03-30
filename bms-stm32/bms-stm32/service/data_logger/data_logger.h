  /*
 * data_logger.h
 *
 *  Created on: Nov 5, 2021
 *      Author: Admin
 */

#ifndef SERVICE_DATA_LOGGER_DATA_LOGGER_H_
#define SERVICE_DATA_LOGGER_DATA_LOGGER_H_

#include "stdint.h"
/*
 *  Table Fault/Events of BMS
 */

// Voltage/Current Fault Event
#define SCD_ERROR			1
#define OCD_ERROR			2
#define OCC_ERROR			3
#define COV_ERROR			4
#define CUV_ERROR 			5

// Temperature Fault Event
#define OTF_ERROR			6
#define OTINT_ERROR			7
#define OTD_ERROR			8
#define OTC_ERROR			9
#define UTINT_ERROR			10
#define UTD_ERROR			11
#define UTC_ERROR			12

/*
 * Comunication Fault Event
 */

// I2C Fault Event
#define AFE_ERROR_NO_EVENTS						13
#define AFE_ERROR_COM_LINK_EVENTS				14
#define AFE_ERROR_READ_MISMATCH_EVENTS			15
#define AFE_HW_ERROR							16

// CAN Fault Event
#define ASSIGN_WAIT_CONFIRM_ERROR				17
#define ASSIGN_CONFRIM_ERROR					18
#define ASSIGN_WAIT_SLAVE_SELECT_ERROR			19
#define AUTHENICATING_ERROR						20
#define SWITCH_STATE_ERROR						21

// BMS State occur error
#define SOFT_START_ERROR						22

// Node ID occur error
#define NODE_ID_ERROR							23

typedef struct Data_Logger_t		Data_Logger;
struct Data_Logger_t{
	uint32_t start_address;
	uint32_t finish_address;
	uint32_t data_error;
	uint32_t index;
	uint32_t data_buffer[512];
	void(*write_data_error)(Data_Logger* this);
	void(*read_data_error)(Data_Logger* this);
	void(*erase_data_error)(Data_Logger* this);
};

Data_Logger* data_logger_create(void);

static inline void data_logger_write_error(Data_Logger* this){
	this->write_data_error(this);
}

static inline void data_logger_read_error(Data_Logger* this){
	this->read_data_error(this);
}

static inline void data_logger_erase_error(Data_Logger* this){
	this->erase_data_error(this);
}

void data_logger_update_error(Data_Logger* p_data, uint32_t error);

#endif /* SERVICE_DATA_LOGGER_DATA_LOGGER_H_ */
