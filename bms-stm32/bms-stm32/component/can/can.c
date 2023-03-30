
#include "can.h"

static void can_process_rx_buffer();

void can_init(void){
	can_set_receive_handle(can_process_rx_buffer);
}

static void can_process_rx_buffer(CanRxMsg* p_msg){
}

