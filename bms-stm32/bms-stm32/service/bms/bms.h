#ifndef _BMS_H_
#define _BMS_H_

#include "board.h"
#include "stdbool.h"

#include "ntc.h"
#include "soc-ocv.h"
#include "afe.h"
#include "switch.h"
#include "adc_sensor.h"
#include "bms_error.h"
#include "soc_est.h"
#include "soh_est.h"
#include "inrush_limiter.h"
#include "model_battery.h"
//#include "soc_ekf.h"
#include "soc_spkf.h"
#include "soh_awtls.h"
#include "data_logger.h"
#include "CO.h"
#include "node_id_hw.h"

#define BMS_STS_AFE_ERROR							AFE_STS_HW_ERROR
#define BMS_STS_OCD									AFE_STS_OCD
#define BMS_STS_SCD									AFE_STS_SCD
#define BMS_STS_OV 	    			        		AFE_STS_OV
#define BMS_STS_UV			 	    			    AFE_STS_UV
#define BMS_STS_OT				      			    AFE_STS_OT
#define BMS_STS_UT				      			    AFE_STS_UT

enum BMS_State
{
	BMS_ST_INIT=0,
	BMS_ST_IDLE,
	BMS_ST_SOFTSTART,
	BMS_ST_DISCHARGING,
	BMS_ST_CHARGING,
	BMS_ST_FAULT,
	BMS_ST_SHIPMODE,
	BMS_ST_SYSTEM_BOOST_UP,
	BMS_ST_ID_ASSIGN_START,
	BMS_ST_ID_ASSIGN_WAIT_CONFIRM,
	BMS_ST_ID_ASSIGN_CONFIRMED,
	BMS_ST_ID_ASSIGN_WAIT_SLAVE_SELECT,
	BMS_ST_START_AUTHENTICATE,
	BMS_ST_AUTHENTICATING,
	BMS_ST_STANDBY,
	BMS_ST_ONLY_DISCHARGING
};


#define BMS_ACTIVE_STATUS_BIT                   (uint32_t)(1<<31)
#define BMS_SOFTSTART_FAULT     (1<<13)

#define BMS_PARAM_SYS_EXT_FLASH_ADDR		0x00
#define BMS_PARAM_SYS_BYTES					24

#define PACK_ZONE_NTC_NUM                     4
#define CELL_GROUP_NUM 4
#define CELL_NUMBER 20
#define BALANCING_OFFSET 	30
#define BALANCING_VOLTAGE_THRESHOLD		3500

typedef enum BMS_State BMS_State;
typedef struct BMS BMS;
typedef enum NFC_State NFC_State;

enum NFC_State {
	NFC_CONNECTED = 0x00,
	NFC_DISCONNECTED = 0x01
};

typedef struct Node_Select_Pin_t Node_Select_Pin;

struct Node_Select_Pin_t {
        NODE_ID_PIN_STATE current_state;
        NODE_ID_PIN_STATE previous_state;
        uint32_t id;
        uint32_t time_out_counter_mS;
        uint8_t time_out;
};


typedef union longlong_ascii1 {
	uint32_t  in_32bits;
	char ascii[7];
} longlong_ascii1;

typedef union longlong_ascii2 {
	uint32_t  in_32bits;
	char ascii[8];
} longlong_ascii2;

typedef struct stm32_uid {
	uint16_t x, y;
	uint8_t WAF_NUM;
	longlong_ascii1 LOT_NUM1;
	longlong_ascii2 LOT_NUM2;
} stm32_uid;


struct BMS {
	stm32_uid device_id;
	NFC_State nfc_state;
	int32_t capacity;
	uint8_t remain_percent;
	int32_t remain_capacity;
	int16_t OT_threshold; /* over temperature protect threshold */
	int16_t UT_threshold; /* under temperature protect threshold */
	uint32_t nfc_link_retry;
	int32_t OCC_threshold;
	uint32_t OV_pack_threshold;
	uint32_t UV_pack_threshold;
	uint16_t soc_calibrate_times;
/*-----------------------------------------------------------*/
	uint32_t fault_recover_timeout;
	uint32_t soc;
	uint32_t soh;
	uint32_t status;
	uint32_t battery_status;
	uint32_t fet_status;
	uint32_t error;
	uint32_t last_error[5];
	BMS_State state;
	AFE* afe;
	Switch* charge_sw;
	Switch* discharge_sw;
	Switch* charge_discharge_sw;
	Switch* softstart_sw;
	SOC_Est* soc_est;
	SOH_Est* soh_est;
	ADC_Sensor* current_sensor;
	int32_t current;
    Inrush_Limiter* ic_lim;
    uint32_t inrush_lim_time;

	NTC* ts5_temp;
	NTC* ts6_temp;

	uint8_t switch_state;
	uint32_t pack_zone_temp_sensor_num;
	NTC** pack_zone_temp_sensors;
	uint16_t balancing_mask;
	uint8_t balancing_enabled;
	//ekf_data ekf_data;
	spkf_data spkf_data;
	soh_awtls soh_awtls_data;
	float soc_previous;
	float soc_cc;
	float current_cc;
	uint32_t id_assign_timeout;
	Data_Logger* data_logger;

	Node_Select_Pin node_select_pin;
	uint32_t node_id_pin_debounce_counter_ms;
	uint32_t node_id_pin_debounce_counter_timeout;
	uint32_t sync_counter;
	CO co_app;
	uint8_t is_request_upgrade;

	void (*update_state_indicator)(const BMS* const bms);
};

typedef enum CO_STATE_T co_node_id_state;
typedef struct GPIO_In_t node_id;
struct GPIO_In_t{
	uint8_t current_state;
	uint8_t previous_state;
	uint32_t id;
    uint32_t time_out_counter_mS;
    uint8_t time_out;
    uint8_t contact_bouncing_detect;
} ;

extern node_id node_id_selex;

void bms_turn_off_discharge(BMS* p_bms);
void bms_turn_on_discharge(BMS* p_bms);
void bms_turn_off_charge(BMS* p_bms);
void bms_turn_on_charge(BMS* p_bms);
void bms_turn_on_all_charge_discharge(BMS* p_bms);
void bms_turn_off_all_charge_discharge(BMS* p_bms);
void bms_turn_off(BMS* bms);
void bms_turn_on(BMS* bms);
void bms_stop_balancing(BMS* p_bms);
uint32_t bms_get_nomial_capacity(const BMS* const p_pms);
static inline BMS_State bms_get_state(const BMS* const bms){
		return bms->state;

}
void bms_set_state(BMS* bms,const BMS_State state);
void bms_update_switch_state(BMS* bms);
void bms_update_status(BMS* bms);
void bms_update_battery_status(BMS* bms);
void bms_update_fet_status(BMS* p_bms);
void bms_update_safety_alert_status(BMS* p_bms);
void bms_update_permanent_fail_state(BMS* p_bms);
void bms_reset_error(BMS* bms);
void bms_update_current(BMS* p_bms);
void bms_update_soc(BMS* p_bms);
void bms_update_soh(BMS* p_bms);


static inline uint32_t bms_get_soc(const BMS* const p_bms){
	return p_bms->soc;
}

static inline uint32_t bms_get_soh(const BMS* const p_bms){
	return p_bms->soh;
}

static inline int32_t bms_get_current_mA(const BMS* const p_bms){
	return p_bms->current;
}

/*-----------------------------------------------------*/

void bms_calib_soc(BMS* p_bms);
void bms_construct(BMS* bms);
void bms_caculate_coulomb_counter(BMS* p_bms);
void bms_update_pack_voltage(BMS* p_bms);
void bms_update_cell_voltages(BMS* p_bms);
void bms_recover_from_fault(BMS* p_bms);
void bms_set_shutdown_mode(BMS *p_bms);
void bms_check_connecting_status(BMS* bms);
void bms_active_balancing(BMS* p_bms);
uint8_t bms_get_remain_percent(BMS* p_bms);
void bms_read_current(BMS* p_bms);
void bms_sync_object(BMS* p_bms,const uint16_t index,const uint8_t sub_index);
void bms_sync_data(BMS* p_bms);

void bms_clear_afe_flag(uint8_t flag_state, I2C_TypeDef* i2c_dev);
void bms_exit_ship_mode(void);
void bms_reset_sw_btn_object(void);
bool bms_check_active_conditions(BMS* p_bms);
void bms_read_data(void);
//void bms_update_soc_ocv(BMS* p_bms);

void bms_update_pack_zone_temperature(BMS* p_bms);
void bms_update_temperature_ts5(BMS* p_bms);
void bms_update_temperature_ts6(BMS* p_bms);
void bms_update_int_temperature_C(BMS *p_bms) ;

void bms_get_uid(BMS *selex_bms);
void bms_init_soc_ekf(BMS* p_bms);
void bms_update_soc_ekf(BMS* p_bms);

void bms_init_soc_spkf(BMS* p_bms);
void bms_update_soc_spkf(BMS* p_bms);
void bms_update_soc_cc(BMS* p_bms);
void bms_init_soc_cc(BMS* p_bms);

void bms_init_soh_awtls(BMS* p_bms);
void bms_update_soh_awtls(BMS* p_bms);

void build_pack_data(const BMS* const p_bms,uint8_t* s);
void build_log_error_data(const BMS* const p_bms,uint8_t* s);

void bms_update_log_data_error(BMS* p_bms, uint32_t error);
void bms_write_data_error(BMS* p_bms);
void bms_read_data_error(BMS* p_bms);
void bms_erase_data_error(BMS* p_bms);
void bms_update_and_write_data_error(BMS*p_bms, uint32_t error);

void bms_check_temperature_condition(BMS* p_bms);
void bms_check_status_operation(BMS* p_bms);

void bms_cell_over_voltage_handle(BMS* p_bms);

//add from BMS RA
bool bms_check_fet_status(BMS* p_bms, uint32_t status_check);
NODE_ID_PIN_STATE bms_get_node_select_state(BMS* p_bms);
bool bms_is_slave_select_request(const BMS* const p_bms);

#endif
