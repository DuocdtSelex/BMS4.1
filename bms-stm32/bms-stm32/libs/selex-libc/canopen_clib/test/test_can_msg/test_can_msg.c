/*
 * test_can_msg.c
 *
 *  Created on: Jun 29, 2019
 *      Author: quangnd
 */
#include "../../../../../libs/selex-libc/canopen_clib/CAN_Msg.h"

int main(int argc,char** args){

	uint8_t msg_bytes[]={0x81,0x02,0x08,0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77};

	CAN_Msg msg;

	can_rx_msg_parse(&msg,msg_bytes,11);

	while(1){

	}
	return 0;
}

