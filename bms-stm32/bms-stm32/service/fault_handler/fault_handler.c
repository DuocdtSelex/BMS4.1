/*
 * fault_handler.c
 *
 *  Created on: Jan 19, 2019
 *      Author: quangnd
 */

#include "fault_handler.h"


static uint8_t device_node_id;

void fault_process(ERROR_CODE code) {

	motor_disable(get_motor());
	uint32_t error_severity = code.error_params.error_severity;
	switch (error_severity) {
	case ERROR_SEVERITY_HIGHEST:
    		__disable_irq();
		while (1);
		break;
	case ERROR_SEVERITY_MEDIUM:
		motor_disable(get_motor());
    		__disable_irq();
		while (1);
		break;
	case ERROR_SEVERITY_LOWEST:
		motor_disable(get_motor());
    		__disable_irq();
		while (1);
		break;
	default:
		motor_disable(get_motor());
    		__disable_irq();
		while (1);
		break;
	}
}

void fault_handler_set_node_id(const uint8_t node_id){
	device_node_id=node_id;	
}
