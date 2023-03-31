/*
 * canopen_init.h
 *
 *  Created on: Oct 22, 2020
 *      Author: quangnd
 */

#ifndef APP_CANOPEN_CANOPEN_INIT_H_
#define APP_CANOPEN_CANOPEN_INIT_H_
#include "CO.h"
#include "bms.h"
#include "can_hal.h"
#include "app_config.h"
#include "app_co_od.h"
#define BMS_NODE_ID                                     1

extern char SERIAL_NUMBER_MEM_ADDR[32];

#define CAN_NODE_ID_ASSIGN_COBID						0x70
#define BMS_SERIAL_NUMBER_OBJECT_INDEX					0x200300
#define BMS_MAINSWITCH_INDEX							0x200301
#define SLAVE_ID_NUMBER_OBJECT_INDEX					0x200302
#define BP_MATING_STATE_SN_OBJECT_INDEX					0x200400
#define VEHICLE_SN_OBJECT_INDEX							0x200401
#define NEW_FIRMWARE_REQUEST							0x200107

#define NODE_ID_DETECT_PIN_LOW_LEVEL 0
#define NODE_ID_DETECT_PIN_HIGH_LEVEL 1
#define MC_DEFAULT_NODE_ID 2
#define CAN_MASTER_DEFAULT_NODE_ID 3
#define BMS_DEFAULT_NODE_ID 4
#define TIME_OUT_100_ms 100
#define TIME_OUT_500_ms  500
#define TIME_OUT_1000_ms 1000

#define BP_VOL_CUR_TPDO_COBID                           CO_CAN_ID_TPDO_1
#define BP_LOW_CELLS_VOL_TPDO_COBID                     CO_CAN_ID_TPDO_2
#define BP_HIGH_CELLS_VOL_TPDO_COBID                    CO_CAN_ID_TPDO_3
#define BP_TEMP_TPDO_COBID                              CO_CAN_ID_TPDO_4

#define SDO_RX_BUFFER_SIZE                 (32UL)
typedef struct CO_SDO_SERVER_t CO_SDO_SERVER;
typedef struct CO_PDO_Tx_t CO_PDO_Tx;

#define SDO_CS_INIT_READ                0
#define SDO_CS_SEGMENT_READ             1
#define SDO_CS_FINISH_READ              2

#define SDO_CS_INIT_WRITE               4
#define SDO_CS_SEGMENT_WRITE            5
#define SDO_CS_FINISH_WRITE             6
#define SDO_CS_ABORT                    7

typedef enum SDO_STATE_t {
        SDO_ST_IDLE = 0, SDO_ST_SENT = 1, SDO_ST_SUCCESS = 2, SDO_ST_FAIL = 3
} SDO_STATE;
struct CO_SDO_SERVER_t {
        CAN_Hw *p_hw;
        uint32_t timeout;
        SDO_STATE state;
        uint32_t rx_index;
        uint32_t tx_index;
        uint32_t tx_address;
        uint32_t rx_address;
        uint32_t object_mux;
        uint32_t object_data_len;
        uint8_t *rx_data_buff;
        uint8_t *tx_data_buff;
        uint32_t buff_offset;
        CanRxMsg rx_msg_buff;
        uint8_t has_new_msg;
        uint8_t bms_node_id;
};

struct CO_PDO_Tx_t {
        uint32_t bms_node_id;
        uint32_t is_synchronize;
};
extern CO_PDO_Tx bms_pdo;
//extern uint32_t bms_node_id;
extern CO_SDO_SERVER bms_sdo;

enum CO_STATE_T {
        NODE_ID_ST_IDLE = 0,
        NODE_ID_ST_CONNECT,
        NODE_ID_ST_WAIT_ASSIGN,
        NODE_ID_ST_AUTHENTIC
};
typedef struct  {
        uint8_t mating_state;
        uint8_t new_save_flash;
        char mated_dev[32];
}Mating_Dev;



extern Mating_Dev BP_mating;
extern volatile uint32_t test_can;
extern volatile uint32_t test_can_se;

extern volatile uint32_t pdo_cnt_re;
extern volatile uint32_t pdo_cnt_se;
void co_process(const uint32_t timestamp);
void co_od_get_object_data_buff(const uint32_t mux, uint8_t **buff, uint16_t*, uint8_t rw);
void co_od_set_nodeID(uint8_t nodeID, uint8_t *rx_msg);
void co_update_sdo_port(CO_SDO_SERVER *p_sdo);
void sdo_response(CO_SDO_SERVER *p_sdo);
void co_sdo_set_state(SDO_STATE state);
SDO_STATE co_sdo_get_state(void);

void co_send_node_assgin_request(CAN_Hw *p_hw);

void canopen_service_init(void);
void co_send_bp_vol_cur_tpdo(void);
void co_send_bp_low_cell_tpdo(void);
void co_send_bp_high_cell_tpdo(void);
void co_send_bp_temp_tpdo(void);
void co_send_test(void);
bool write_vehicle_sn(void) ;
void sysreset_new_firmware(void);
void write_bms_sn(CAN_Hw *p_hw);
uint8_t co_check_node_id_pin(node_id* p_node_id);
void co_read_node_id_signal(node_id* p_node_id);
#endif /* APP_CANOPEN_CANOPEN_INIT_H_ */
