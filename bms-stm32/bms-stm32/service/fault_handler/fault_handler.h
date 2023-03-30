/*
 * fault_handler.h
 *
 *  Created on: Jan 19, 2019
 *      Author: quangnd
 */

#ifndef FAULT_HANDLER_H_
#define FAULT_HANDLER_H_
#include "stdint.h"

#define ERROR_SEVERITY_HIGHEST	0
#define ERROR_SEVERITY_MEDIUM	8
#define ERROR_SEVERITY_LOWEST	15
#define ERROR_NUM_SEVERITY		4
#define ERROR_NUM_MODULE		8
#define ERROR_NUM_TYPE			4
#define ERROR_NUM_ID			16

typedef union ERROR_CODE ERROR_CODE;

#define DEVICE_NODE_ID		3


typedef void (*fault_process)(ERROR_CODE);

union ERROR_CODE{
	uint32_t code;
	struct error_params{
		uint32_t error_id:16;
		uint32_t error_type:4;
		uint32_t error_module:8;
		uint32_t error_severity:4;
	}error_params;

};

typedef struct fault_service{
	ERROR_CODE error_code;
	fault_process fault_process[ERROR_NUM_SEVERITY] ;

};
void fault_service_process_error(ERROR_CODE code);
void fault_handler_set_node_id(const uint8_t node_id);



#endif /* FAULT_HANDLER_H_ */
