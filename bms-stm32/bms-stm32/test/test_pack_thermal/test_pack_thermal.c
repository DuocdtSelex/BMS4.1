/*
 * main.c
 *
 *  Created on: Aug 18, 2020
 *      Author: quangnd
 */
#include "board.h"
#include "afe_init.h"
#include "bms_init.h"
#include "afe.h"
#include "delay.h"
#include "can_hal.h"
#include "string_util.h"
#include "debug_com_port_hal.h"
#include "ntc.h"
#include "afe_config.h"

extern BMS selex_bms;
static void can_sends(const uint8_t* s);
static void build_pack_data(const BMS* const p_bms,uint8_t* s);
static CanTxMsg tx_msg;
static void can_receive_handle(CAN_Hw* p_hw);

static void app_init(void){
	global_interrupt_disable();
	board_init();
	afe_init();
	bms_init();
	bms_set_state(&selex_bms,BMS_ST_IDLE);
	can_set_receive_handle(&can1,can_receive_handle);
	global_interrupt_enable();
}

static uint8_t pack_data_buffer[512];

int main(void){
	app_init();
	afe_reset_error(&app_afe);
	//delay_ms(5000);
	bms_set_state(&selex_bms,BMS_ST_DISCHARGING);
	//uint8_t data=0;
	//bq_read_reg_byte_with_crc(lower_bq.hw, PROTECT2, &data);
	while(1){
	}
	return 0;
}

void HAL_STATE_MACHINE_UPDATE_TICK(void){

	static uint32_t update_counter=0;
	static uint8_t* buffer;

	update_counter++;
	if (update_counter == 50) {

		bms_update_current(&selex_bms);
		bms_update_status(&selex_bms);
		uint32_t status = selex_bms.status;
		if (status & BMS_STS_AFE_ERROR) {
			bms_set_state(&selex_bms, BMS_ST_FAULT);
		}
		bms_update_pack_zone_temperature(&selex_bms);
		bms_update_pack_voltage(&selex_bms);
		bms_update_cell_voltages(&selex_bms);
	}

	if(update_counter==60){
		buffer=pack_data_buffer;
		build_pack_data(&selex_bms,buffer);
		can_sends(buffer);
		update_counter=0;
	}
}

static void can_receive_handle(CAN_Hw* p_hw){

	uint8_t cmd=p_hw->rx_msg.Data[0];
	switch(cmd){
	case 'C':
		bms_set_state(&selex_bms,BMS_ST_CHARGING);
		break;
	case 'D':
		bms_set_state(&selex_bms,BMS_ST_DISCHARGING);
		break;
	case 'S':
		bms_set_state(&selex_bms,BMS_ST_IDLE);
		break;
	default:
		break;
	}
}

static void build_pack_data(const BMS* const p_bms,uint8_t* s){

	*s=':';
	s++;
	s+=long_to_string(afe_get_pack_voltage(p_bms->afe), s);
	*s=',';
	s++;
	s+=slong_to_string(bms_get_current_mA(p_bms),s);
	*s=',';
	s++;
	s+=slong_to_string(p_bms->status, s);
	*s=',';
    for(int i =0; i <20; i++){
    	if(p_bms->afe->cell_array->cells[i].is_short==0){
    	s++;
		s+=long_to_string(afe_get_cell_voltage(p_bms->afe,i), s);
		*s=',';
    	}
    }
    for(int j =0; j <4; j++){
    	s++;
    	s+=slong_to_string(ntc_get_temp(p_bms->pack_zone_temp_sensors[j]), s);
    	*s =',';
    }
	*s='*';
	s++;
	*s='\n';
	s++;
	*s ='\0';
}


static void can_sends(const uint8_t* s){

	uint8_t len=0;
	while(*s){
		len=0;
		tx_msg.RTR = 0; //khung truyen la data Frame
		tx_msg.StdId= 0x33;//id
		tx_msg.IDE =0;// khung chuan
        for(int i= 0; i< 8; i++){
        	len++;
        	tx_msg.Data[i]= *s;
        	s++;
        	if(tx_msg.Data[i]=='\0'){
        	    tx_msg.DLC = len; //quy dinh chieu dai cua truong du lieu
        		can_send(&can1, &tx_msg);
        		return;
        	}
        }
        tx_msg.DLC = 8; //quy dinh chieu dai cua truong du lieu
        can_send(&can1, &tx_msg);
        delay_us(500);
	}

}

