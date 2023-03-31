/*
 * app_co_od.c
 *
 *  Created on: Jun 17, 2022
 *      Author: Dakaka
 *
 *       Only edit in USER CODE 1,2  if you haven't read the documents or love simplicity!
 */

#include "app_co_od.h"
#include "CO.h"
#include "CO_OD.h"
#include "app_config.h"
#include "canopen_init.h"

/*******************************************************************************
 * 		1.	Declare and define ALL sub-object in OD
 *******************************************************************************/

/*------------------------------------------------------------------------------------
 *  1.1 Declare and define all communication profile initializer sub-object
 *  		WARNING: Should not be adjust!									***/

CO_Sub_Object device_type[] = {
		{(void*)&od_comm_prof_init_data.x1000_device_type, ODA_SDO_R, 4, NULL}
};/* 0x1000 VAR Device type */

CO_Sub_Object error_reg[] = {
		{(void*)&CO_DEVICE.emcy.error_reg.std, ODA_SDO_R, 1, NULL}
};/* 0x1001 VAR Error register */

CO_Sub_Object status[] = {
		{(void*)&CO_DEVICE.status, ODA_SDO_R, 4, NULL}
}; /* 0x1002 VAR Manufacturer status register */

CO_Sub_Object error_field[]={
		{(void*)&CO_DEVICE.emcy.predefined_error_fields.error_no, ODA_SDO_R, 1, NULL},
		{(void*)&(CO_DEVICE.emcy.predefined_error_fields.error_code[0].w), ODA_SDO_R, 4, NULL},
		{(void*)&(CO_DEVICE.emcy.predefined_error_fields.error_code[1].w), ODA_SDO_R, 4, NULL},
		{(void*)&(CO_DEVICE.emcy.predefined_error_fields.error_code[2].w), ODA_SDO_R, 4, NULL},
		{(void*)&(CO_DEVICE.emcy.predefined_error_fields.error_code[3].w), ODA_SDO_R, 4, NULL},
		{(void*)&(CO_DEVICE.emcy.predefined_error_fields.error_code[4].w), ODA_SDO_R, 4, NULL},
		{(void*)&(CO_DEVICE.emcy.predefined_error_fields.error_code[5].w), ODA_SDO_R, 4, NULL},
		{(void*)&(CO_DEVICE.emcy.predefined_error_fields.error_code[6].w), ODA_SDO_R, 4, NULL},
		{(void*)&(CO_DEVICE.emcy.predefined_error_fields.error_code[7].w), ODA_SDO_R, 4, NULL}
}; /* 0x1003 ARRAY Error field */

CO_Sub_Object sync_cob_id[]={
		{(void*)&od_comm_prof_init_data.x1005_cob_id_sync_message, ODA_SDO_RW, 4, NULL}
}; /* 0x1005 VAR COB-ID Sync message */

CO_Sub_Object comm_cycle[]={
		{(void*)&od_comm_prof_init_data.x1006_comm_cycle_period_ms, ODA_SDO_RW, 2, NULL}
}; /* 0x1006 VAR Communication cycle period [ms] */

CO_Sub_Object sync_window_len[]={
		{(void*)&od_comm_prof_init_data.x1007_sync_window_length_ms, ODA_SDO_RW, 2, NULL}
}; /* 0x1007 VAR Synchronous window length */

CO_Sub_Object device_name[]={
		{(void*)&od_comm_prof_init_data.x1008_device_name, ODA_SDO_R, CO_CF_1008_DEVICE_NAME_SIZE, NULL}
}; /* 0x1008 VAR Manufacturer device name */

CO_Sub_Object hw_version[]={
		{(void*)&od_comm_prof_init_data.x1009_hardware_version, ODA_SDO_R, 4, NULL}
}; /* 0x1009 VAR Manufacturer hardware version */

CO_Sub_Object sw_version[]={
		{(void*)&od_comm_prof_init_data.x100A_software_version, ODA_SDO_R, 4, NULL}
}; /* 0x100A VAR Manufacturer software version */

CO_Sub_Object guard_time[]={
		{(void*)&od_comm_prof_init_data.x100C_guard_time_ms, ODA_SDO_RW, 2, NULL}
}; /* 0x100C VAR Guard time [ms] */

CO_Sub_Object life_time_factor[]={
		{(void*)&od_comm_prof_init_data.x100D_life_time_factor, ODA_SDO_RW, 32, NULL}
}; /* 0x100D VAR Life time factor */


CO_Sub_Object time_cob_id[]={
		{(void*)&od_comm_prof_init_data.x1012_cob_id_time_stamp_object, ODA_SDO_RW, 4, NULL}
}; /* 0x1012 VAR COB-ID TIME */

CO_Sub_Object high_res_timestamp[]={
		{(void*)&CO_DEVICE.high_resolution_timestamp, ODA_SDO_RW, 4, NULL}
}; /* 0x1013 VAR Guard time */

CO_Sub_Object emcy_cob_id[]={
		{(void*)&od_comm_prof_init_data.x1014_cob_id_emcy, ODA_SDO_RW, 4, NULL}
}; /* 0x1014 VAR COB-ID EMCY */

CO_Sub_Object consumer_heartbeat[]={
		{(void*)&od_comm_prof_init_data.x1016_consumer_heartbeat_time_sub0, ODA_SDO_R, 1, NULL},
		{(void*)&od_comm_prof_init_data.x1016_consumer_heartbeat_time[0], ODA_SDO_RW, 4, NULL},
		{(void*)&od_comm_prof_init_data.x1016_consumer_heartbeat_time[1], ODA_SDO_RW, 4, NULL},
		{(void*)&od_comm_prof_init_data.x1016_consumer_heartbeat_time[2], ODA_SDO_RW, 4, NULL},
		{(void*)&od_comm_prof_init_data.x1016_consumer_heartbeat_time[3], ODA_SDO_RW, 4, NULL},
		{(void*)&od_comm_prof_init_data.x1016_consumer_heartbeat_time[4], ODA_SDO_RW, 4, NULL}
}; /* 0x1016 Consumer heart-beat time */

CO_Sub_Object producer_heartbeat[]={
		{(void*)&od_comm_prof_init_data.x1017_producer_heartbeat_time, ODA_SDO_RW, 2, NULL}
}; /* 0x1017 Producer heart-beat time */

CO_Sub_Object identity[]={
		{(void*)&od_comm_prof_init_data.x1018_identity.vendor_id, ODA_SDO_RW, 4, NULL},
		{(void*)&od_comm_prof_init_data.x1018_identity.product_code, ODA_SDO_R, 4, NULL},
		{(void*)&od_comm_prof_init_data.x1018_identity.revision_number, ODA_SDO_R, 4, NULL},
		{(void*)&od_comm_prof_init_data.x1018_identity.serial_number, ODA_SDO_R, 4, NULL}
}; /* 0x1018 ARRAY Identity */

CO_Sub_Object sync_cter_over_flow_value[]={
		{(void*)&od_comm_prof_init_data.x1019_sync_cter_over_flow_value, ODA_SDO_RW, 2, NULL}
}; /* 0x1019 Synchronous counter overflow value [1-30000]*/

CO_Sub_Object sync_bit_pos_over_flow_value[]={
		{(void*)&od_comm_prof_init_data.x1030_sync_bit_pos_over_flow_value, ODA_SDO_RW, 1, NULL}
}; /* 0x1030 Synchronous bit position overflow value [0-31]*/

CO_Sub_Object sdo_server_comm_para[]={
		{(void*)&od_comm_prof_init_data.x1031_sdo_server_comm_para.node_id, ODA_SDO_RW, 1, NULL},
		{(void*)&od_comm_prof_init_data.x1031_sdo_server_comm_para.trans_type, ODA_SDO_RW, 2, NULL},
		{(void*)&od_comm_prof_init_data.x1031_sdo_server_comm_para.allow_timeout_ms, ODA_SDO_RW, 2, NULL}
}; /* 0x1031 SDO server communication parameter*/

CO_Sub_Object sdo_client_comm_para[]={
		{(void*)&od_comm_prof_init_data.x1032_sdo_client_comm_para.trans_type, ODA_SDO_RW, 2, NULL},
		{(void*)&od_comm_prof_init_data.x1032_sdo_client_comm_para.sync_mask_reg, ODA_SDO_RW, 4, NULL}
}; /* 0x1032 SDO client communication parameter*/



/*--------------------------------------------------------------------------------------------
 * 1.2 Declare and define all manufacturer-specific profile sub-object
 */

/* USER CODE 1 BEGIN */
static uint8_t bms_node_id_shadow;
static bool is_change_nodeid;
void co_od_update_nodeid(void){
	if(!is_change_nodeid) return;
	is_change_nodeid = false;
	CO_set_node_id(&selex_bms.co_app,bms_node_id_shadow);
}

static CO_Sub_Object_Ext_Confirm_Func_t co_od_set_nodeid(void){
	is_change_nodeid = true;
	return CO_EXT_CONFIRM_success;
}
CO_Sub_Object_Ext_t bms_set_nodeid = {
		.p_shadow_data = &bms_node_id_shadow,
		.confirm_func = co_od_set_nodeid
};/* Segment data extension */


char mated_dev_shadow[32] = {0};

static CO_Sub_Object_Ext_Confirm_Func_t confirm_mated_dev(void){
	memcpy(BP_mating.mated_dev,mated_dev_shadow,32);
	return CO_EXT_CONFIRM_success;
}

CO_Sub_Object_Ext_t confirm_mated = {
		.p_shadow_data = (uint8_t *)mated_dev_shadow,
		.confirm_func = confirm_mated_dev
};

extern char SERIAL_NUMBER_MEM_ADDR[32];

CO_Sub_Object bms_object[] = {
		{(void*)(uint8_t*)SERIAL_NUMBER_MEM_ADDR,  ODA_SDO_R, 13, NULL},
		{(void*)&selex_bms.switch_state,  	ODA_SDO_RW, 1, NULL},
		{(void*)&bms_sdo.bms_node_id,  		ODA_SDO_RW, 1, &bms_set_nodeid},
};
CO_Sub_Object mating_object[] = {
		{(void*)&BP_mating.mating_state,  ODA_SDO_RW, 1, NULL},
		{(void*)&BP_mating.mated_dev,  ODA_SDO_RW, 32, &confirm_mated},

};
CO_Sub_Object new_firmware[] = {
		{(void*)NULL,  ODA_SDO_RW, 1, NULL},
		{(void*)NULL,  ODA_SDO_RW, 1, NULL},
		{(void*)NULL,  ODA_SDO_RW, 1, NULL},
		{(void*)NULL,  ODA_SDO_RW, 1, NULL},
		{(void*)NULL,  ODA_SDO_RW, 1, NULL},
		{(void*)NULL,  ODA_SDO_RW, 1, NULL},
		{(void*)NULL,  ODA_SDO_RW, 1, NULL},
		{(void*)&selex_bms.is_request_upgrade,  ODA_SDO_RW, 1, NULL},

};
/* USER CODE 1 END */

/*******************************************************************************************************
 * 			2. Declare and define OD
 *
 * 			Warning: Must sort the object so that the index value ascending from smallest to largest!
 *******************************************************************************************************/
static CO_Object smc_od_list[]=
{
		/*** Communication profile objects***/

		/** General communication object**/
		{.index = 0x1000, .sub_number = 1, .subs = device_type},		/* 0x1000 VAR Device type */
		{.index = 0x1001, .sub_number = 1, .subs = error_reg},			/* 0x1001 VAR Error register */
		{.index = 0x1002, .sub_number = 1, .subs = status},				/* 0x1002 VAR Manufacturer status register */
		{.index = 0x1003, .sub_number = 9, .subs = error_field}, 		/* 0x1003 ARRAY Error field */

		{.index = 0x1005, .sub_number = 1, .subs = sync_cob_id}, 		/* 0x1005 VAR COB-ID Sync message */
		{.index = 0x1006, .sub_number = 1, .subs = comm_cycle}, 		/* 0x1006 VAR Communication cycle period [ms] */
		{.index = 0x1007, .sub_number = 1, .subs = sync_window_len},	/* 0x1007 VAR Synchronous window length */
		{.index = 0x1008, .sub_number = 1, .subs = device_name}, 		/* 0x1008 VAR Manufacturer device name */
		{.index = 0x1009, .sub_number = 1, .subs = hw_version}, 		/* 0x1009 VAR Manufacturer hardware version */
		{.index = 0x100A, .sub_number = 1, .subs = sw_version}, 		/* 0x100A VAR Manufacturer software version */
		{.index = 0x100C, .sub_number = 1, .subs = guard_time}, 		/* 0x100C VAR Guard time [ms] */
		{.index = 0x100D, .sub_number = 1, .subs = life_time_factor},	/* 0x100D VAR Life time factor */
		{.index = 0x1012, .sub_number = 1, .subs = time_cob_id}, 		/* 0x100D VAR Time cob id */
		{.index = 0x1013, .sub_number = 1, .subs = high_res_timestamp},/* 0x1013 VAR Guard time */
		{.index = 0x1014, .sub_number = 1, .subs = emcy_cob_id}, 		/* VAR COB-ID emergency */

		{.index = 0x1016, .sub_number = 6, .subs = consumer_heartbeat},/* 0x1016 ARRAY Consumer heart-beat time */
		{.index = 0x1017, .sub_number = 1, .subs = producer_heartbeat},/* 0x1017 VAR Producer heart-beat time */
		{.index = 0x1018, .sub_number = 4, .subs = identity},			/* 0x1018 ARRAY Identity */
		{.index = 0x1019, .sub_number = 1, .subs = sync_cter_over_flow_value},	/* 0x1019 VAR Synchronous counter overflow value [0-30000]*/
		{.index = 0x1030, .sub_number = 1, .subs = sync_bit_pos_over_flow_value},	/* 0x1030 VAR Auxiliary synchronous counter overflow value [0-31]*/
		{.index = 0x1031, .sub_number = 3, .subs = sdo_server_comm_para},	/* 0x1031 ARRY SDO server communication parameter*/
		{.index = 0x1032, .sub_number = 2, .subs = sdo_client_comm_para},	/* 0x1032 ARRY SDO client communication parameter*/

		/** SDO parameter object [0x1200- 0x12FF] **/
		/** PDO parameter object [0x1400- 0x1BFF] **/

		/*** Manufacturer-specific profile object ***/

		/* USER CODE 2 BEGIN */
		{.index = NEW_FIRMWARE_REQUEST_INDEX, .sub_number = 8, .subs = new_firmware},
		{.index = BMS_OBJECT_INDEX, .sub_number = 3, .subs = bms_object},
		{.index = MATING_INDEX, .sub_number = 2, .subs = mating_object}
		/* USER CODE 2 END */
};

static CO_OD smc_od =
{
		(sizeof(smc_od_list) / sizeof(smc_od_list[0])), smc_od_list
};

CO_OD *p_co_od = &smc_od;

