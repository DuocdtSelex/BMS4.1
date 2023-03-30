/*
 * app_co_init.c
 *
 *  Created on: Jun 17, 2022
 *      Author: Dakaka
 */

#include "app_co_init.h"
#include "canopen_init.h"
#include <stdbool.h>

#if 0
/* Save all communication profile data value in flash to use after reset */
static void app_co_save_all_comm_data_in_flash(void){}
/* Save all manufacturer-specific profile data value in flash to use after reset */
static void app_co_save_all_app_data_in_flash(void){}

/* Update all communication profile initializer data value from flash */
static void app_co_update_all_init_comm_data_from_flash(void){}
/* Update all manufacturer-specific profile initializer data value from flash */
static void app_co_update_all_init_app_data_from_flash(void){}
#endif

/********************************************************************************************
 * 									MANDATORY
 * *******************************************************************************************/
/* Declare for build TPDO message function when using "tpdo_build_data_manually" mode*/
static void tpdo1_build_data_impl(uint8_t* buffer);
static void tpdo2_build_data_impl(uint8_t* buffer);
static void tpdo3_build_data_impl(uint8_t* buffer);
static void tpdo4_build_data_impl(uint8_t* buffer);
 void tpdo5_build_data_impl(uint8_t* buffer);
static void tpdo6_build_data_impl(uint8_t* buffer);

void* tpdo_build_data_impl[TPDO_NUMBER] =
{
		tpdo1_build_data_impl,
		tpdo2_build_data_impl,
		tpdo3_build_data_impl,
		tpdo4_build_data_impl,
		tpdo5_build_data_impl,
		tpdo6_build_data_impl
};
// pack_voltage,current ...
static void tpdo1_build_data_impl(uint8_t* buffer)
{
	CO_setUint16(buffer,
			(uint16_t) (selex_bms.afe->pack_voltage / 10));
	CO_setUint16(buffer + 2,
			(uint16_t) ((int16_t) (selex_bms.current)));
	buffer[4] = (uint8_t)selex_bms.soc;
	buffer[5] = bms_get_state(&selex_bms);
	CO_setUint16(buffer + 6, (uint16_t) (selex_bms.status));
}
//voltage from cell 1 to cell 4
static void tpdo2_build_data_impl(uint8_t* buffer)
{

	uint8_t cell_id = 0;
	uint8_t index = 0;
	while (cell_id < 8) {
		if (selex_bms.afe->cell_array->cells[index].is_short == 0) {
			buffer[cell_id] = (uint8_t) (selex_bms.afe->cell_array->cells[index].voltage);
			buffer[cell_id + 1] = (uint8_t) ((selex_bms.afe->cell_array->cells)[index].voltage >> 8);
			cell_id = cell_id + 2;
		}
		index++;
	}

}
//voltage from cell 5 to cell 8
static void tpdo3_build_data_impl(uint8_t* buffer)
{
	uint8_t cell_id = 0;
	uint8_t index = 0;
	while (cell_id < 8) {
		if ((selex_bms.afe->cell_array->cells+4)[index].is_short == 0) {
			buffer[cell_id] = (uint8_t) ((selex_bms.afe->cell_array->cells+4)[index].voltage);
			buffer[cell_id + 1] = (uint8_t) ((selex_bms.afe->cell_array->cells+4)[index].voltage >> 8);
			cell_id = cell_id + 2;
		}
		index++;
	}
}

static void tpdo4_build_data_impl(uint8_t* buffer)
{
    uint32_t i = 0;
    NTC *p_ss = 0;
    while ((i < 8) && (i < selex_bms.pack_zone_temp_sensor_num)) {
            p_ss = selex_bms.pack_zone_temp_sensors[i];
            buffer[i] = (uint8_t) p_ss->temp;
            i++;
    }
    buffer[4] = (uint8_t) selex_bms.ts5_temp->temp;
    buffer[5] = (uint8_t) selex_bms.ts6_temp->temp;
//    buffer[6] = (uint8_t) selex_bms.afe->fet_status
//    buffer[7] = (uint8_t) selex_bms.ts6_temp->temp;
}

//voltage from cell 9 to cell 12
 void tpdo5_build_data_impl(uint8_t* buffer)
{
	uint8_t cell_id = 0;
	uint8_t index = 0;
	while (cell_id < 8) {
		if ((selex_bms.afe->cell_array->cells+8)[index].is_short == 0) {
			buffer[cell_id] = (uint8_t) ((selex_bms.afe->cell_array->cells+8)[index].voltage);
			buffer[cell_id + 1] = (uint8_t) ((selex_bms.afe->cell_array->cells+8)[index].voltage >> 8);
			cell_id = cell_id + 2;
		}
		index++;
	}
}
//voltage from cell 13 to cell 16
static void tpdo6_build_data_impl(uint8_t* buffer)
{
	uint8_t cell_id = 0;
	uint8_t index = 0;
	while (cell_id < 8) {
		if ((selex_bms.afe->cell_array->cells+12)[index].is_short == 0) {
			buffer[cell_id] = (uint8_t) ((selex_bms.afe->cell_array->cells+12)[index].voltage);
			buffer[cell_id + 1] = (uint8_t) ((selex_bms.afe->cell_array->cells+12)[index].voltage >> 8);
			cell_id = cell_id + 2;
		}
		index++;
	}
}

/* Define can_send message function */
static void app_co_can_send_impl(CO_CAN_Msg* p_msg)
{
	CO_memcpy(can1.tx_msg.Data, p_msg->data, 8);
	can1.tx_msg.StdId = p_msg->id.can_id;
	can1.tx_msg.DLC = p_msg->data_len;
	can_send(&can1,&can1.tx_msg);
}
/* Call in receive can interrupt */
void app_co_can_receive_handle(const uint32_t can_id, uint8_t* data)
{
	CO_can_receive_basic_handle(&CO_DEVICE, can_id, data);

}

void app_co_init(void)
{
	app_co_init_storage();
	/* [Mandatory] Set CO_CAN_send interface */
	CO_CAN_set_can_send_interface(app_co_can_send_impl);

	/* [Mandatory] Init CO object */
	CO_init_basic(&CO_DEVICE,
			od_comm_prof_init_data.x1000_device_type,
			&od_comm_prof_init_data.x1018_identity,
			p_co_od);

	CO_SYNC_init(&CO_DEVICE.sync, &od_comm_prof_init_data);

	for(uint8_t i = 0; i < TPDO_NUMBER; i++)
	{
		CO_TPDO_init(&CO_DEVICE.tpdos[i],
				&CO_DEVICE.sync,
				&od_comm_prof_init_data.x1A0x_tpdo_map_para[i],
				CO_DEVICE.p_od,
				&od_comm_prof_init_data.x180x_tpdo_comm_para[i],
				CO_TPDO_build_data_manually, tpdo_build_data_impl[i]);
	}
	CO_SDOserver_init(&CO_DEVICE.sdo_server,
			&od_comm_prof_init_data,
			&CO_DEVICE.sync,
			CO_DEVICE.p_od);
	CO_SDOclient_init(&CO_DEVICE.sdo_client,
			&od_comm_prof_init_data,
			&CO_DEVICE.sync);
//	CO_DEVICE.sdo_server.co_get_od_data_buff	= co_od_get_object_data_buff;
//	CO_DEVICE.sdo_client.co_get_od_data_buff	= co_od_get_object_data_buff;
//	CO_DEVICE.sdo_server.co_od_set_nodeID	= co_od_set_nodeID;
//	CO_DEVICE.sdo_client.co_od_set_nodeID	= co_od_set_nodeID;
}


/********************************************************************************************
 * 									USER CODE
 * *******************************************************************************************/
/* Example for bms */
/*void app_co_bms_set_node_id(node_id)
{
	switch(node_id)
	case 5:
		CO_set_node_id(&CO_DEVICE, node_id);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 9);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 10);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 11);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 12);
		break;

	case 6:
		CO_set_node_id(&CO_DEVICE, node_id);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 13);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 14);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 15);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 16);
		CO_SDO_set_sync_mask_reg(&CO_DEVICE.sdo_client, mask);
		break;
	case 7:
		CO_set_node_id(&CO_DEVICE, node_id);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 17);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 18);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 19);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 20);
		CO_SDO_set_sync_mask_reg(&CO_DEVICE.sdo_client, mask);
		break;
	case 7:
		CO_set_node_id(&CO_DEVICE, node_id);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 17);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 18);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 19);
		CO_TPDO_set_transmission_type(&CO_DEVICE.tpdos[1], 20);
		CO_SDO_set_sync_mask_reg(&CO_DEVICE.sdo_client, mask);
		break;
	default:
		break;
}*/
