#include "service/bms/bms.h"

#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "delay.h"
#include "string_util.h"
#include "node_id_hw.h"

#define BMS_SOC_FLASH_ADR			0x0801F008
#define BMS_SOH_FLASH_ADR			0x0801F00C
#define BMS_OT_THRESH       		50
#define BMS_UT_THRESH       		(-4)
#define BMS_OVP_THRESH				67000 //61000//60000 //
#define BMS_UVP_THRESH				55000 //38000 //

#define BMS_UC_THRESH				100
#define BMS_VOL_HIS_THRESH			1000
#define BMS_VOL_CALIB_THRESH		30
#define BMS_SOC_CALIB_TIM_THRESH	100
#define MAX_FAULT_GROUP				4
#define MAX_FAULT_STATE_GROUP		4

#define CHAR_TYPE_BYTES				1
#define SHORT_TYPE_BYTES			2
#define INTEGER_TYPE_BYTES			4

#define BMS_OCC_THRESH			    (int32_t)17000    //15000        //10000
#define BMS_NORMAL_CAPACITY			10000

#define BMS_COV_HANLDE_VOL			4350
#define BMS_COV_HANDLE_CUR 			32
#define BMS_MAX_CELL				16

node_id node_id_selex;
uint32_t cnt_success_on = 0;

#define ADDR_UID 0x1FFFF7AC

static const uint16_t fault_state_mask[MAX_FAULT_GROUP] = { 0x0103, 0x180C,
		0x0600, 0x0020 };

void bms_construct(BMS *bms) {

	bms->remain_capacity = 86400000;
	bms->state = BMS_ST_INIT;
	bms->capacity = BMS_NORMAL_CAPACITY; /* 24Ah - 86400000mAs */
	bms->nfc_state = NFC_DISCONNECTED;
	bms->soh = 100.0;
	bms->soc = 100.0;
	bms->fault_recover_timeout = 0;
	bms->OT_threshold = BMS_OT_THRESH;
	bms->UT_threshold = BMS_UT_THRESH;
	bms->OCC_threshold = BMS_OCC_THRESH;
	bms->OV_pack_threshold = BMS_OVP_THRESH;
	bms->UV_pack_threshold = BMS_UVP_THRESH;
	bms->soc_calibrate_times = 0;
	bms->node_id_pin_debounce_counter_ms=0;
	bms->node_id_pin_debounce_counter_timeout=4000;
}

uint32_t bms_get_nomial_capacity(const BMS* const p_pms){

	return p_pms->afe->cell_array->cells->capacity;
}

void bms_get_uid(BMS *selex_bms)
{
	selex_bms->device_id.x = (*(uint32_t*) ADDR_UID) & 0xFFFF;

	selex_bms->device_id.y = ((*(uint32_t*) ADDR_UID) & 0xFFFF0000) >> 16;
	selex_bms->device_id.WAF_NUM = (*(uint32_t*) (ADDR_UID + 0x04)) & 0xFF;
	selex_bms->device_id.LOT_NUM1.in_32bits =  ((*(uint32_t*) (ADDR_UID + 0x04)) & 0xFFFFFFFFFFFFFF00)>>8;
	selex_bms->device_id.LOT_NUM2.in_32bits = ((*(uint32_t*) (ADDR_UID + 0x08)) & 0xFFFFFFFFFFFFFFFF);
}

void bms_update_status(BMS* bms){
	afe_update_status(bms->afe);
	bms->status |= afe_get_status(bms->afe);
}

void bms_update_battery_status(BMS* bms){
	afe_update_battery_status(bms->afe);
	bms->battery_status = afe_get_battery_status(bms->afe);
}
void bms_update_current(BMS* p_bms){
	afe_update_current_cA(p_bms->afe);
	p_bms->current = afe_get_current(p_bms->afe);
//	adc_sensor_update_result(p_bms->current_sensor);
//	p_bms->current=adc_sensor_get_result(p_bms->current_sensor);
}

void bms_update_fet_status(BMS* p_bms){
	afe_update_fet_status(p_bms->afe);
	p_bms->fet_status = afe_get_fet_status(p_bms->afe);
}

void bms_update_safety_alert_status(BMS* p_bms){
	afe_update_safety_alert_status(p_bms->afe);
}

void bms_update_permanent_fail_state(BMS* p_bms){
	afe_update_permanent_fail_state(p_bms->afe);
}

void bms_update_switch_state(BMS* bms){
	if(bms->switch_state==0){
		if((bms_get_state(bms)==BMS_ST_DISCHARGING)||
				(bms_get_state(bms)==BMS_ST_CHARGING)){
			bms_set_state(bms, BMS_ST_STANDBY);
		}
	}
	else if(bms->switch_state==1){
		if(bms_get_state(bms)==BMS_ST_STANDBY){
			bms_set_state(bms, BMS_ST_DISCHARGING);
		}else{
			bms->switch_state=0;
		}
	}
	else if(bms->switch_state==2){
		if(bms_get_state(bms)==BMS_ST_STANDBY){
			bms_set_state(bms, BMS_ST_CHARGING);
		}else{
			bms->switch_state=0;
		}
	}
	else if(bms->switch_state==3){
		if(bms_get_state(bms)==BMS_ST_STANDBY){
			bms_set_state(bms, BMS_ST_DISCHARGING);
		}
	}
	else if(bms->switch_state==4){
		bms_turn_off_charge(bms);
	}
	else{
		bms_set_state(bms, BMS_ST_FAULT);
		bms->switch_state=0;
		bms_update_and_write_data_error(bms, SOFT_START_ERROR);
	}
}


void bms_set_state(BMS* bms, const BMS_State state) {
        switch (state) {
        case BMS_ST_IDLE:
                bms_turn_off(bms);
          		bms->error=BMS_ERROR_NO;
          		inrush_limiter_stop(bms->ic_lim);
          		bms->node_id_pin_debounce_counter_ms=0;
                break;
        case BMS_ST_ID_ASSIGN_START:
        		break;
        case BMS_ST_ID_ASSIGN_WAIT_CONFIRM:
        		break;
        case BMS_ST_ID_ASSIGN_CONFIRMED:
        	break;
        case BMS_ST_ID_ASSIGN_WAIT_SLAVE_SELECT:
        	break;
        case BMS_ST_START_AUTHENTICATE:
        		break;
        case BMS_ST_AUTHENTICATING:
        		break;
        case BMS_ST_SOFTSTART:
                inrush_limiter_active(bms->ic_lim);
                bms_turn_on_all_charge_discharge(bms);
                hw_delay_ms(500);
                bms->inrush_lim_time=0;
                break;
        case BMS_ST_SYSTEM_BOOST_UP:
        		bms_stop_balancing(bms);
        		bms_turn_on(bms);
        		//inrush_limiter_stop(bms->ic_lim);
        		break;
        case BMS_ST_STANDBY:
        		bms->node_id_pin_debounce_counter_ms=0;
        		node_id_selex.contact_bouncing_detect=0;
        		bms_turn_off(bms);
        		break;
        case BMS_ST_CHARGING:
                bms_turn_on(bms);
          		inrush_limiter_stop(bms->ic_lim);
          		bms->node_id_pin_debounce_counter_ms=0;
                break;
        case BMS_ST_DISCHARGING:
                bms_stop_balancing(bms);
                bms_turn_on(bms);
          		inrush_limiter_stop(bms->ic_lim);
          		bms->node_id_pin_debounce_counter_ms=0;
                break;
    	case BMS_ST_ONLY_DISCHARGING:
    		bms_turn_off_charge(bms);
    		break;
        case BMS_ST_FAULT:
        		bms_turn_off(bms);
          		inrush_limiter_stop(bms->ic_lim);
          		bms->switch_state=0;
                bms_stop_balancing(bms);
                bms->fault_recover_timeout=0;
                break;
        case BMS_ST_SHIPMODE:
        	    bms_set_shutdown_mode(bms);
        	    break;
        default:
        	return;
        }
       bms->state = state;
}

NODE_ID_PIN_STATE bms_get_node_select_state(BMS *p_bms) {

	return p_bms->node_select_pin.current_state;
}

bool bms_is_slave_select_request(const BMS *const p_bms) {

	if ((p_bms->node_select_pin.current_state == NODE_ID_PIN_ST_SELECT)
			&& (p_bms->node_select_pin.previous_state == NODE_ID_PIN_ST_DESELECT)) {
		return true;
	}
	return false;
}

void bms_set_shutdown_mode(BMS *p_bms){
		afe_set_shutdown_mode(p_bms->afe);
}
void bms_recover_from_fault(BMS *p_bms) {

	bms_update_status(p_bms);
	if(p_bms->status!=0){
		bms_set_state(p_bms,BMS_ST_FAULT);
//		return;
	}
	bms_reset_error(p_bms);
	bms_set_state(p_bms,BMS_ST_STANDBY);
}

void bms_reset_error(BMS* p_bms){
	afe_reset_error(p_bms->afe);
	p_bms->error=BMS_ERROR_NO;
	p_bms->status=0;
}

void bms_turn_off(BMS *bms) {
	//bms->switch_state=0x00;
	bms_turn_off_all_charge_discharge(bms);
	//bms_turn_off_charge(bms);
}

void bms_turn_on(BMS *bms) {
	//bms->switch_state=0x03;
	bms_turn_on_all_charge_discharge(bms);
//	bms_turn_on_charge(bms);
}


void bms_turn_off_discharge(BMS* p_bms) {
	sw_off(p_bms->discharge_sw);
	//p_bms->switch_state &=~(1<<0);
}

void bms_turn_on_discharge(BMS* p_bms) {
	sw_on(p_bms->discharge_sw);
//	p_bms->switch_state |=(1<<0);
}

void bms_turn_off_charge(BMS* p_bms) {
	//p_bms->switch_state &=~(1<<1);
	sw_off(p_bms->charge_sw);
}

void bms_turn_on_charge(BMS* p_bms) {
	//p_bms->switch_state |=(1<<1);
	sw_on(p_bms->charge_sw);
}

void bms_turn_on_all_charge_discharge(BMS* p_bms){
	sw_on(p_bms->charge_discharge_sw);
}

void bms_turn_off_all_charge_discharge(BMS* p_bms){
	sw_off(p_bms->charge_discharge_sw);
}

static void bms_update_soc_est_inputs(BMS* p_bms){

	SOC_Est_State_Vector* p_soc_vector=(SOC_Est_State_Vector*)
			est_get_input_buffer((Estimator*) p_bms->soc_est);

	p_soc_vector->current=p_bms->current;
	p_soc_vector->voltage=afe_get_pack_voltage(p_bms->afe);
}

static void bms_update_soh_est_inputs(BMS* p_bms){

	SOH_Est_State_Vector* p_soh_vector=(SOH_Est_State_Vector*)
			est_get_input_buffer((Estimator*) p_bms->soh_est);

	p_soh_vector->current=p_bms->current;
	p_soh_vector->voltage=afe_get_pack_voltage(p_bms->afe);
}

void bms_update_soc(BMS* p_bms){

	bms_update_soc_est_inputs(p_bms);
	est_update((Estimator*) p_bms->soc_est);
}

void bms_update_soh(BMS* p_bms){

	bms_update_soh_est_inputs(p_bms);
	est_update((Estimator*) p_bms->soc_est);
}
#if 0
void bms_init_soc_ekf(BMS* p_bms){
  p_bms->ekf_data.input_voltage =(float)p_bms->afe->pack_voltage/1000;
  p_bms->ekf_data.xhat_zk = SOC_from_OCV(p_bms->ekf_data.input_voltage);
  ekf_soc_init(&p_bms->ekf_data);
}
void bms_update_soc_ekf(BMS* p_bms){
	p_bms->ekf_data.input_current =-(float)p_bms->current/100;
	p_bms->ekf_data.input_voltage =(float)p_bms->afe->pack_voltage/1000;
	ekf_soc_iter(&p_bms->ekf_data);
	// output soc = xhat_zk;
	//p_bms->soc = soc_is_round(p_bms->ekf_data.xhat_zk*100);
	p_bms->soc = p_bms->ekf_data.xhat_zk*100;
}
#endif

void bms_init_soc_spkf(BMS* p_bms){
  p_bms->spkf_data.input_voltage = (float)p_bms->afe->cell_array->cells[0].voltage/1000;
  spkf_soc_init(&p_bms->spkf_data);
}

void bms_update_soc_spkf(BMS* p_bms){
#if 0
	uint32_t new_soc=p_bms->soc;

	if((p_bms->current > -2) && (p_bms->current < 2))
	{
		p_bms->spkf_data.input_current = 0;
	}
	else {
	p_bms->spkf_data.input_current = -(float)p_bms->current/(100*4);
	}
	p_bms->spkf_data.input_voltage = (float)p_bms->afe->cell_array->cells[0].voltage/1000;
	spkf_soc_iter(&p_bms->spkf_data);
	// output soc = xhat_zk;
	new_soc = p_bms->spkf_data.xhat[2][0]*100;
	if((new_soc>100) || (new_soc<=0)){
		spkf_soc_init(&p_bms->spkf_data);
	}else{
		p_bms->soc=new_soc;
	}
#endif
	p_bms->soc=90;
}
#if 0
void bms_update_soc_ocv(BMS* p_bms){
	p_bms->soc= get_soc_from_ocv(p_bms->afe->pack_voltage, p_bms->current);
}
#endif
static float get_min_1(float value1, float value2)
{
    float min;
    min = (value1 < value2) ? value1 : value2;
	return min;
}
static float get_max_1(float value1, float value2)
{
	float max;
	max = (value1 > value2) ? value1: value2;
	return max;
}
void bms_init_soc_cc(BMS* p_bms){
	p_bms->soc_cc = get_soc_from_ocv((float)p_bms->afe->cell_array->cells[0].voltage/1000);
}
void bms_update_soc_cc(BMS* p_bms){
	if ((p_bms->current >-2) && (p_bms->current < 2))
	{
		p_bms->current_cc = 0;
	}
	else
	{
		p_bms->current_cc = -(float)p_bms->current/(100*4);
	}
	p_bms->soc_cc = p_bms->soc_cc - (1.0* p_bms->current_cc)/(3600*4); // deltat*I /(3600*Q)
    p_bms->soc_cc = get_min_1(1.0f, get_max_1(0.0f, p_bms->soc_cc));
}

void bms_init_soh_awtls(BMS* p_bms)
{
	p_bms->soh_awtls_data.measureX = 0;
	p_bms->soh_awtls_data.measureY = 0;
	soh_awtls_init(&p_bms->soh_awtls_data);
}

void bms_update_soh_awtls(BMS* p_bms){
	p_bms->soh_awtls_data.measureX = p_bms->spkf_data.xhat[2][0] - p_bms->soc_previous;
	p_bms->soh_awtls_data.measureY = ((float)(p_bms->current)/(4.0f*100.0f*3600.0f))*(DELTAT_SOH);
	soh_awtls_iter(&p_bms->soh_awtls_data);
	p_bms->soh = p_bms->soh_awtls_data.soh_estimate*100;
	if(p_bms->soh>100) p_bms->soh=100;
}


void bms_check_temperature_status(BMS *p_bms) {
	(void) p_bms;
#if 0
	uint8_t ntc_index = 0;
	int16_t ntc_temp = 0;
	bool is_ot = false;
	bool is_ut = false;
	afe_read_temperature();
	for (ntc_index = 0; ntc_index < NTC_NUM; ntc_index++) {
		ntc_temp = p_bms->pack_ntc_temperatures[ntc_index];
		itoa(ntc_temp, buffer, 10);
		if (ntc_temp >= (p_bms->OT_threshold + ntc_ot_offset[ntc_index])) {
			is_ot = true;
		}
		if (ntc_temp <= p_bms->UT_threshold)
			is_ut = true;
	}
	if (is_ot == true) {
		p_bms->error |= (uint32_t) OT_BIT;
	} else {
		p_bms->error &= ~((uint32_t) OT_BIT);
	}
	if (is_ut == true) {
		p_bms->error |= (uint32_t) UT_BIT;
	} else {
		p_bms->error &= ~((uint32_t) UT_BIT);
	}
#endif
}


void bms_update_cell_voltages(BMS *p_bms) {
	afe_update_cell_voltage(p_bms->afe);
}

void bms_update_pack_voltage(BMS *p_bms) {
	afe_update_pack_voltate(p_bms->afe);
}

void bms_update_pack_zone_temperature(BMS* p_bms){

	for(uint8_t i=0;i<p_bms->pack_zone_temp_sensor_num;i++){
		ntc_update_temp_from_bq(p_bms->pack_zone_temp_sensors[i]);
	}
}

void bms_update_temperature_ts5(BMS* p_bms){
	ntc_update_temp(p_bms->ts5_temp);
}

void bms_update_temperature_ts6(BMS* p_bms){
	ntc_update_temp(p_bms->ts6_temp);
}

void bms_update_int_temperature_C(BMS *p_bms) {
	afe_update_int_temperature_C(p_bms->afe);
}

void bms_active_balancing(BMS* p_bms){

	uint32_t min_cell_voltage;
	cell_array_get_min_voltage_cell(p_bms->afe->cell_array->cells,
				p_bms->afe->cell_array->serial_cells,&min_cell_voltage);
	afe_enable_cell_balancing(p_bms->afe,min_cell_voltage);
}

void bms_stop_balancing(BMS *p_bms) {
	afe_disable_cell_balancing(p_bms->afe);
}

uint8_t bms_get_remain_percent(BMS *p_bms) {
	return p_bms->remain_percent;
}

uint8_t bms_get_fault_group(BMS *p_bms) {
	uint8_t fault_group = 0;
	uint16_t error_code = (uint16_t) (p_bms->error);
	uint8_t group_id = 0;
	while (group_id < MAX_FAULT_GROUP) {
		if (error_code & fault_state_mask[group_id])
			fault_group += (1 << group_id);
		group_id++;
	}
	return fault_group;
}

void bms_fault_state_indicator(BMS *p_bms) {
	uint8_t fault_group = bms_get_fault_group(p_bms);
}

void bms_update_log_data_error(BMS* p_bms, uint32_t error){
	data_logger_update_error(p_bms->data_logger, error);
}
void bms_write_data_error(BMS* p_bms){
	data_logger_write_error(p_bms->data_logger);
}
void bms_read_data_error(BMS* p_bms){
	data_logger_read_error(p_bms->data_logger);
}
void bms_erase_data_error(BMS* p_bms){
	data_logger_erase_error(p_bms->data_logger);
}

void bms_update_and_write_data_error(BMS* p_bms, uint32_t error){
	bms_update_log_data_error(p_bms, error);
	bms_write_data_error(p_bms);
}

void bms_check_status_operation(BMS* p_bms){
	switch(p_bms->status) {
		case 0:
			break;
		case (1<<16):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, AFE_HW_ERROR);
				break;
		case (1<<2):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, CUV_ERROR);
				break;
		case (1<<3):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, COV_ERROR);
				break;
		case (1<<4):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, OCC_ERROR);
				break;
		case (1<<5):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, OCD_ERROR);
				break;
		case (1<<6):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, OCD_ERROR);
				break;
		case (1<<7):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, SCD_ERROR);
				break;
		case 0x60:
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, OCD_ERROR);
				break;
		default:
			break;
	}
}

void bms_check_temperature_condition(BMS* p_bms){
	switch(p_bms->afe->safety.safety_status_b){
		case 0:
				break;
		case (1<<0):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, UTC_ERROR);
				break;
		case (1<<1):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, UTD_ERROR);
				break;
		case (1<<2):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, UTINT_ERROR);
				break;
		case (0x03):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, UTD_ERROR);
				break;
		case (1<<4):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, OTC_ERROR);
				break;
		case (1<<5):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, OTD_ERROR);
				break;
		case (1<<6):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, OTINT_ERROR);
				break;
		case (1<<7):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, OTF_ERROR);
				break;
		case (0x60):
			//	bms_set_state(p_bms, BMS_ST_FAULT);
				bms_update_and_write_data_error(p_bms, OTD_ERROR);
				break;
		default:
			break;
	}
}
void build_pack_data(const BMS* const p_bms,uint8_t* s){

	*s=':';
	s++;
	s+=long_to_string(p_bms->soc, s);
	*s=',';
	s++;
	s+=long_to_string(afe_get_pack_voltage(p_bms->afe), s);
	*s=',';
	s++;
	s+=slong_to_string(afe_get_current(p_bms->afe),s);
	*s=',';
	s++;
	s+=slong_to_string(p_bms->status, s);
	*s=',';
	s++;
	s+=slong_to_string(p_bms->battery_status, s);
	*s=',';
	s++;
	s+=long_to_string(p_bms->fet_status, s);
	*s=',';
    for(int i =0; i <16; i++){
    	s++;
		s+=long_to_string(afe_get_cell_voltage(p_bms->afe,i), s);
		*s=',';
    }
    for(int j =0; j <4; j++){
    	s++;
    	s+=slong_to_string(ntc_get_temp(p_bms->pack_zone_temp_sensors[j]), s);
    	*s =',';
    }
	s++;
	s+=long_to_string(ntc_get_temp(p_bms->ts5_temp), s);
	*s=',';
	s++;
	s+=long_to_string(ntc_get_temp(p_bms->ts6_temp), s);
	*s=',';
	s++;
	s+=slong_to_string(afe_get_int_temperature(p_bms->afe), s);
	*s=',';
	s++;
	s+=long_to_string(p_bms->afe->balancing_mask, s);
	*s=',';
	s++;

	s+=long_to_string(p_bms->last_error[0], s);
	*s=',';
	s++;

	s+=long_to_string(p_bms->last_error[1], s);
	*s=',';
	s++;

	s+=long_to_string(p_bms->last_error[2], s);
	*s=',';
	s++;

	s+=long_to_string(p_bms->last_error[3], s);
	*s=',';
	s++;

	s+=long_to_string(p_bms->last_error[4], s);
	*s=',';
	s++;

	s+=long_to_string(p_bms->afe->safety.safety_alert_c, s);
	*s=',';
	s++;
	s+=long_to_string(p_bms->afe->safety.safety_status_c, s);
	*s=',';
	s++;

	s+=long_to_string(p_bms->state, s);
	*s=',';
	s++;

	s+=long_to_string(node_id_selex.contact_bouncing_detect, s);
	*s=',';
	s++;

	s+=long_to_string(p_bms->sync_counter, s);
	*s=',';
	s++;

	*s='*';
	s++;
	*s='\n';
	s++;
	*s ='\0';
}


void bms_cell_over_voltage_handle(BMS* p_bms){

	int i = 0;
	uint32_t max_cell_voltage;

	max_cell_voltage = p_bms->afe->cell_array->cells[0].voltage;
	for (i = 0; i < BMS_MAX_CELL; i++)
	{
		if (p_bms->afe->cell_array->cells[i].voltage > max_cell_voltage){
			max_cell_voltage = p_bms->afe->cell_array->cells[i].voltage;
		}
	}

	if(max_cell_voltage > BMS_COV_HANLDE_VOL)
	{
		bms_set_state(p_bms, BMS_ST_IDLE);
	}
}

void build_log_error_data(const BMS* const p_bms, uint8_t* s){
}

void TIM7_IRQHandler(void) {
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) {
	}
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
 }


//add from BMS RA
bool bms_check_fet_status(BMS* p_bms, uint32_t status_check){
	return p_bms->fet_status == status_check;

}
