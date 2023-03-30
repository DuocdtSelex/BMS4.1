/*
 * canopen_init.c
 *
 *  Created on: Oct 22, 2020
 *      Author: quangnd
 */

#include "canopen_init.h"
#include "CO.h"
#include "bms_init.h"
#include "can_hal.h"
#include "afe_config.h"
#include "core_hal.h"
#include "node_id_hal.h"
#include "stdio.h"
#include "string.h"
#include "delay.h"
#include "app_co_init.h"

#define BMS_SDO_ADDRESS                         (0x600+BMS_NODE_ID)
#define CAN_MASTER_SDO_ADDRESS                  (0x600 + CAN_MASTER_DEFAULT_NODE_ID)
#define BMS_SN_LENGHT	13
static char bms_serial_number[32];

Mating_Dev BP_mating;

static CanTxMsg tx_msg;
bool string_cmp(char str1[],char str2[], uint32_t lenght);
static uint32_t bms_sdo_address = 0;
CO_SDO_SERVER bms_sdo;
CO_PDO_Tx bms_pdo;

static void node_read_serial_number(char *buff);
static void read_vehicle_serial_number(Mating_Dev *p_mat);
static uint8_t write_serial_number(char *buff);
static void can_receive_handle(CAN_Hw *p_hw);
 bool is_para_write_flash(Mating_Dev *p_mating);

//uint32_t bms_node_id=0;

void canopen_service_init(void) {
	can_set_receive_handle(&can1, can_receive_handle);
	node_read_serial_number(bms_serial_number);
	read_vehicle_serial_number(&BP_mating);
	bms_sdo_address = BMS_SDO_ADDRESS;
	bms_sdo.state = SDO_ST_IDLE;
	bms_sdo.rx_address = 0x580 + BMS_DEFAULT_NODE_ID;
	bms_sdo.tx_address = 0x600 + BMS_DEFAULT_NODE_ID;
	bms_sdo.p_hw = &can1;
	bms_sdo.has_new_msg = 0;
	bms_sdo.bms_node_id = 0;
	app_co_init();
	CO_set_node_id(&selex_bms.co_app,0);
}

void co_send_bp_vol_cur_tpdo(void) {
	CO_setUint16(tx_msg.Data,
			(uint16_t) (selex_bms.afe->pack_voltage / 10));
	CO_setUint16(tx_msg.Data + 2,
			(uint16_t) ((int16_t) (selex_bms.current)));
	can1.tx_msg.Data[4] = (uint8_t)selex_bms.soc;
	can1.tx_msg.Data[5] = bms_get_state(&selex_bms);
	CO_setUint16(can1.tx_msg.Data + 6, (uint16_t) (selex_bms.status));
	can1.tx_msg.StdId = BP_VOL_CUR_TPDO_COBID + bms_pdo.bms_node_id;
	can1.tx_msg.DLC = 8;
	can_send(&can1,&tx_msg);
}

static inline void co_send_cell_voltage(Cell_Bank *p_bank,
		const uint16_t cob_id) {
	uint8_t cell_id = 0;
	uint8_t index = 0;
	while (cell_id < 8) {
		if (p_bank[index].is_short == 0) {
			can1.tx_msg.Data[cell_id] = (uint8_t) (p_bank[index].voltage
					/ 100);
			cell_id++;
		}
		index++;
	}
	can1.tx_msg.StdId = cob_id;
	can1.tx_msg.DLC = 8;
	can_send(&can1,&tx_msg);
}

void co_send_bp_low_cell_tpdo(void) {

	co_send_cell_voltage(selex_bms.afe->cell_array->cells,
			(uint16_t)(BP_LOW_CELLS_VOL_TPDO_COBID + bms_pdo.bms_node_id));
}

void co_send_bp_high_cell_tpdo(void) {
	co_send_cell_voltage(selex_bms.afe->cell_array->cells + BQ_MAX_CELL / 2,
			(uint16_t)(BP_HIGH_CELLS_VOL_TPDO_COBID + bms_pdo.bms_node_id));

}



void co_send_bp_temp_tpdo(void) {

}
void co_send_test(void){
	can1.tx_msg.Data[4] = (uint8_t) 10;
	can1.tx_msg.Data[5] = (uint8_t) 10;
	can1.tx_msg.StdId = 0x111;
	can1.tx_msg.DLC = 8;
	can_send(&can1,&tx_msg);
}

bool new_load_BMS_SN = false;
uint8_t new_bms_sn_request = 0;
char bms_sn_buff[8];

char SERIAL_NUMBER_MEM_ADDR[32] = { 0 };

static void can_receive_handle(CAN_Hw *p_hw) {

	uint32_t cob_id = p_hw->rx_msg.StdId;

	if(cob_id != 0x80){

	}

	if (bms_get_state(&selex_bms) == BMS_ST_SOFTSTART) {
		return;
	}
	if(CO_can_receive_basic_handle(&selex_bms.co_app, cob_id, p_hw->rx_msg.Data)){
		return;
	}

	if ((cob_id == CAN_NODE_ID_ASSIGN_COBID)) {
		switch (bms_get_state(&selex_bms)) {
		case BMS_ST_ID_ASSIGN_WAIT_CONFIRM:
			bms_set_state(&selex_bms, BMS_ST_ID_ASSIGN_CONFIRMED);
			break;
		case BMS_ST_ID_ASSIGN_WAIT_SLAVE_SELECT:
			bms_sdo.bms_node_id = p_hw->rx_msg.Data[0];
			p_hw->tx_msg.StdId = CAN_NODE_ID_ASSIGN_COBID;
			p_hw->tx_msg.Data[0] = bms_sdo.bms_node_id;
			p_hw->tx_msg.DLC = 1;
			can_send(p_hw,&p_hw->tx_msg);
			CO_set_node_id(&selex_bms.co_app,p_hw->rx_msg.Data[0]);
			bms_sdo.rx_address =(uint32_t)(0x580UL + bms_sdo.bms_node_id);
			bms_sdo.tx_address = (uint32_t)(0x600UL + bms_sdo.bms_node_id);
			bms_set_state(&selex_bms, BMS_ST_START_AUTHENTICATE);
			break;
		default:
			break;
		}
		return;
	}
	/* send BP serial code*/
	if(cob_id == 0x60){
		p_hw->tx_msg.StdId = 0x061;
		memcpy( p_hw->tx_msg.Data,(uint8_t*)SERIAL_NUMBER_MEM_ADDR,8);
		p_hw->tx_msg.DLC = 8;
		can_send(p_hw,&p_hw->tx_msg);
	}
#if 0
	/* receive and write BP serial code
	 * check the validity of the character data[6] must be in the character range A->B*/
	if(cob_id == 0x62){
		if(p_hw->rx_msg.data[6] >= 0x41 && p_hw->rx_msg.data[6] <= 0x5A){
			new_bms_sn_request = 1;
			memcpy(bms_sn_buff,p_hw->rx_msg.data,8);
		}
	}
#endif
}

uint8_t new_firm_request = 0;

void sysreset_new_firmware(void) {

}


void write_bms_sn(CAN_Hw *p_hw) {
	if(new_bms_sn_request != 1 ) return;
	new_bms_sn_request = 0;
	if(write_serial_number(bms_sn_buff) != 0) return;
	memcpy( bms_serial_number, (uint8_t*)SERIAL_NUMBER_MEM_ADDR, BMS_SN_LENGHT );
	p_hw->tx_msg.StdId = 0x061;
	memcpy( p_hw->tx_msg.Data,(uint8_t*)SERIAL_NUMBER_MEM_ADDR,8);
	p_hw->tx_msg.DLC = 8;
	can_send(p_hw,&p_hw->tx_msg);

}
void sdo_response(CO_SDO_SERVER *p_sdo) {
	can_send(p_sdo->p_hw,&p_sdo->p_hw->tx_msg);
}
void co_od_get_object_data_buff(const uint32_t mux, uint8_t **buff,
		uint16_t *len, uint8_t rw) {
	(void)rw;
	if (mux == BMS_SERIAL_NUMBER_OBJECT_INDEX) {

		memcpy( bms_serial_number, (uint8_t*)SERIAL_NUMBER_MEM_ADDR, BMS_SN_LENGHT );
		*buff = (uint8_t*) SERIAL_NUMBER_MEM_ADDR;
		*len = BMS_SN_LENGHT;
	}
	else if (mux == BMS_MAINSWITCH_INDEX) {

		*buff = &selex_bms.switch_state;
		*len = 1;
	}
	else if (mux == SLAVE_ID_NUMBER_OBJECT_INDEX) {

		*buff = &bms_sdo.bms_node_id;
		*len = 1;
	}
	else if (mux == VEHICLE_SN_OBJECT_INDEX) {
		read_vehicle_serial_number(&BP_mating);
		*buff = (uint8_t*)BP_mating.mated_dev;
		*len = 32;
	}
	else if (mux == BP_MATING_STATE_SN_OBJECT_INDEX) {
		read_vehicle_serial_number(&BP_mating);
		*buff = &BP_mating.mating_state;
		*len = 1;
	}
	else if (mux == NEW_FIRMWARE_REQUEST) {
		*buff = &new_firm_request;
		*len = 1;
	}
}

void co_od_set_nodeID(uint8_t nodeID, uint8_t *rx_msg){
	if(&bms_sdo.bms_node_id != rx_msg) return;
	CO_set_node_id(&selex_bms.co_app,nodeID);

}
void co_update_sdo_port(CO_SDO_SERVER *p_sdo) {
	p_sdo->rx_address =(uint32_t)(0x580UL + p_sdo->bms_node_id);
	p_sdo->tx_address =(uint32_t)(0x600UL + p_sdo->bms_node_id);
}

static void node_read_serial_number(char *buff) {
	int32_t len = core_read_id(buff);
	if (len <= 0)return;
}

char VEHICLE_SERIAL_NUMBER_MEM_ADDR[32] = { 0 };

static void read_vehicle_serial_number(Mating_Dev *p_mat){
	uint8_t buff_size = 32;
	memcpy( &p_mat->mating_state, (uint8_t*)VEHICLE_SERIAL_NUMBER_MEM_ADDR + 4, 1);
	memcpy( p_mat->mated_dev, (uint8_t*)VEHICLE_SERIAL_NUMBER_MEM_ADDR + 5, buff_size);

}
bool write_vehicle_sn(void) {
	uint8_t buff_size = 32;
	if(is_para_write_flash(&BP_mating) == true ) return 1;
	sw_delay_ms(1000);
	uint8_t vehicle_sn_buff[64] = {0};
	memcpy( vehicle_sn_buff , (uint8_t*)VEHICLE_SERIAL_NUMBER_MEM_ADDR , 64 );
	vehicle_sn_buff[4] = BP_mating.mating_state;
	memcpy( vehicle_sn_buff + 5, BP_mating.mated_dev, buff_size );
	flash_hp_data_flash_operations(vehicle_sn_buff,VEHICLE_SERIAL_NUMBER_MEM_ADDR,1,64);
	read_vehicle_serial_number(&BP_mating);
	return 0;
}
 bool is_para_write_flash(Mating_Dev *p_mating){
	uint8_t buff_size = 32;
	char vehicle_sn[buff_size];
	uint8_t mating_st;
	memcpy( vehicle_sn, (uint8_t*)VEHICLE_SERIAL_NUMBER_MEM_ADDR + 5, buff_size );
	memcpy( &mating_st, (uint8_t*)VEHICLE_SERIAL_NUMBER_MEM_ADDR + 4, 1 );
	if((string_cmp(vehicle_sn,p_mating->mated_dev,32) == 1)
			&& (mating_st == p_mating->mating_state )){
		return 1;
	}
	return 0;
}
bool string_cmp(char str1[],char str2[], uint32_t lenght){
	for( uint32_t i = 0 ; i < lenght; i++){
		if(str1[i] != str2[i]) return 0;
	}
	return 1;
}
static uint8_t write_serial_number(char *buff){
	uint8_t err = 0;
	uint8_t buff_size = 64;
	uint8_t new_firm[buff_size];
	memcpy( new_firm, (uint8_t*)SERIAL_NUMBER_MEM_ADDR, buff_size );
	memcpy( new_firm,buff,8);
	new_firm[8] = 'E';
	new_firm[9] = 'B';
	new_firm[10] = '1';
	new_firm[11] = '0';
	new_firm[12] = '1';
	__enable_irq();
	err = flash_hp_data_flash_operations(new_firm,SERIAL_NUMBER_MEM_ADDR,1,64);
	delay_ms(100);
	return err;
}

void co_sdo_set_state(SDO_STATE state) {
	bms_sdo.state = state;
}

SDO_STATE co_sdo_get_state(void) {
	return bms_sdo.state;
}

void co_send_node_assgin_request(CAN_Hw *p_hw) {
	p_hw->tx_msg.StdId = CAN_NODE_ID_ASSIGN_COBID;
	p_hw->tx_msg.DLC = 0;
	can_send(p_hw,&p_hw->tx_msg);
}
void co_read_node_id_signal(node_id *p_node_id) {
        p_node_id->previous_state = p_node_id->current_state;
        p_node_id->current_state = node_id_read_input();
}

uint8_t co_check_node_id_pin(node_id *p_node_id) {
        return (p_node_id->current_state == NODE_ID_DETECT_PIN_HIGH_LEVEL) ?
                        1 : 0;
}
