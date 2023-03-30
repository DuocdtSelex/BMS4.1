/*
 * CAN_Msg.h
 *
 *  Created on: May 3, 2019
 *      Author: quangnd
 */

#ifndef CAN_MSG_H_
#define CAN_MSG_H_

#include "stdint.h"
#include "stdbool.h"

#include "../../../libs/selex-libc/canopen_clib/CO_utils.h"

typedef struct CAN_Msg_t CAN_Msg;

struct CAN_Msg_t{
	uint16_t cob_id;
        uint8_t node_id;
	uint8_t data[8];
	uint8_t data_len;
        bool is_new;
};

static inline void can_rx_msg_parse(CAN_Msg* p_msg,const uint8_t* p_data,const uint16_t len){

	if(len <11 ) return;
        uint16_t msg_id= CO_getUint16(p_data);
        p_msg->data_len=p_data[2];
        p_msg->node_id= (uint8_t) (msg_id & 0x007FUL); /* node id is 7bit LSB */
        p_msg->cob_id= msg_id & 0x0780UL;
        CO_memcpy(p_msg->data,p_data+3,p_msg->data_len);
}

#endif /* CAN_MSG_H_ */
