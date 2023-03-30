/*
 * bms_init.c
 *
 *  Created on: Aug 21, 2020
 *      Author: quangnd
 */

#include "bms_init.h"

#include "bms.h"
#include "afe_init.h"
#include "switch.h"
#include "bq769x0.h"
#include "adc_sensor.h"
#include "soc_est.h"
#include "soh_est.h"
#include "temp_sense_hal.h"
#include "stdlib.h"
#include "ntc.h"
#include "ntc_data.h"
#include "app_config.h"
#include "inrush_limiter_init.h"
#include "rgb_led_hal.h"
#include "bms_config.h"
#include "bq769x2.h"
#include "data_logger_init.h"

BMS selex_bms;
static Switch discharge_sw;
static Switch charge_sw;
static Switch charge_discharge_sw;
static ADC_Sensor current_sense;
static NTC ts5_temp_sense;
static NTC ts6_temp_sense;

static NTC upper_ntc;
static NTC middle_ntc;
static NTC side_ntc;
static NTC bottom_ntc;

static void bq_sw_turn_off_discharge(Switch* p_sw);
static void bq_sw_turn_on_discharge(Switch* p_sw);
static void bq_sw_turn_on_charge(Switch* p_sw);
static void bq_sw_turn_off_charge(Switch* p_sw);
static void bq_sw_turn_on_all_fet(Switch* p_sw);
static void bq_sw_turn_off_all_fet(Switch* p_sw);

//static void current_sense_update_adc_impl(ADC_Sensor* p_ss);
//static void current_sense_update_offset_impl(ADC_Sensor* p_ss);

static void ntc_ts5_update_impedance_impl(NTC* p_ntc);
static void ntc_ts6_update_impedance_impl(NTC* p_ntc);

static void upper_ntc_update_temperature_impl(NTC* p_ntc);
static void middle_ntc_update_temperature_impl(NTC* p_ntc);
static void side_ntc_update_temperature_impl(NTC* p_ntc);
static void bottom_ntc_update_temperature_impl(NTC* p_ntc);

static void bms_update_state_indicator_impl(const BMS* const p_bms);

void bms_init(void){

        inrush_limiter_init();
        data_logger_init();
        selex_bms.ic_lim=inrush_limiter;
        selex_bms.data_logger = data_logger;
	selex_bms.state=BMS_ST_INIT;
	selex_bms.last_error[0]=BMS_ERROR_NO;
	selex_bms.last_error[1]=BMS_ERROR_NO;
	selex_bms.last_error[2]=BMS_ERROR_NO;
	selex_bms.last_error[3]=BMS_ERROR_NO;
	selex_bms.last_error[4]=BMS_ERROR_NO;

	selex_bms.afe=&app_afe;

	charge_sw.sw_off=bq_sw_turn_off_charge;
	charge_sw.sw_on=bq_sw_turn_on_charge;
	discharge_sw.sw_off=bq_sw_turn_off_discharge;
	discharge_sw.sw_on=bq_sw_turn_on_discharge;
	charge_discharge_sw.sw_on = bq_sw_turn_on_all_fet;
	charge_discharge_sw.sw_off = bq_sw_turn_off_all_fet;

	selex_bms.charge_sw=&charge_sw;
	selex_bms.discharge_sw=&discharge_sw;
	selex_bms.charge_discharge_sw = &charge_discharge_sw;
	selex_bms.switch_state=0;

//	adc_sensor_init(&current_sense);
//	current_sense.update_adc=current_sense_update_adc_impl;
//	current_sense.update_offset=current_sense_update_offset_impl;
//	current_sense.gain=current_sense_calculate_gain();

	selex_bms.current_sensor=&current_sense;

	selex_bms.ts5_temp=&ts5_temp_sense;
	ts5_temp_sense.update_impedance = ntc_ts5_update_impedance_impl;
	selex_bms.ts5_temp->lut =ntc_selex_lut;
	selex_bms.ts5_temp->max_temp=NTC_SELEX_LUT_MAX_TEMPERATURE;
	selex_bms.ts5_temp->min_temp=NTC_SELEX_LUT_MIN_TEMPERATURE;
	selex_bms.ts5_temp->lut_size=NTC_SELEX_LUT_SIZE;

	selex_bms.ts6_temp=&ts6_temp_sense;
	ts6_temp_sense.update_impedance = ntc_ts6_update_impedance_impl;
	selex_bms.ts6_temp->lut =ntc_selex_lut;
	selex_bms.ts6_temp->max_temp=NTC_SELEX_LUT_MAX_TEMPERATURE;
	selex_bms.ts6_temp->min_temp=NTC_SELEX_LUT_MIN_TEMPERATURE;
	selex_bms.ts6_temp->lut_size=NTC_SELEX_LUT_SIZE;

	selex_bms.soc_est=soc_est_create();
	est_init((Estimator*)selex_bms.soc_est);

	selex_bms.soh_est=soh_est_create();
	est_init((Estimator*)selex_bms.soh_est);

	//ekf_soc_init(&selex_bms.ekf_data);

	selex_bms.pack_zone_temp_sensor_num=PACK_ZONE_NTC_NUM;
	selex_bms.pack_zone_temp_sensors=(NTC**)malloc(PACK_ZONE_NTC_NUM*sizeof(NTC*));

	upper_ntc.update_temperature=upper_ntc_update_temperature_impl;
	middle_ntc.update_temperature=middle_ntc_update_temperature_impl;
	side_ntc.update_temperature=side_ntc_update_temperature_impl;
	bottom_ntc.update_temperature=bottom_ntc_update_temperature_impl;

	selex_bms.pack_zone_temp_sensors[0]=&upper_ntc;
	selex_bms.pack_zone_temp_sensors[1]=&middle_ntc;
	selex_bms.pack_zone_temp_sensors[2]=&side_ntc;
	selex_bms.pack_zone_temp_sensors[3]=&bottom_ntc;

	for(int i=0;i<PACK_ZONE_NTC_NUM;i++){
		selex_bms.pack_zone_temp_sensors[i]->lut=ntc_selex_lut;
		selex_bms.pack_zone_temp_sensors[i]->min_temp=NTC_SELEX_LUT_MIN_TEMPERATURE;
		selex_bms.pack_zone_temp_sensors[i]->max_temp=NTC_SELEX_LUT_MAX_TEMPERATURE;
		selex_bms.pack_zone_temp_sensors[i]->lut_size=NTC_SELEX_LUT_SIZE;
	}

	selex_bms.update_state_indicator=bms_update_state_indicator_impl;
	selex_bms.soc=65;
	selex_bms.soh=100;
	selex_bms.balancing_enabled=BMS_BALANCING_ENABLE;

	selex_bms.node_id_pin_debounce_counter_timeout=BMS_NODE_ID_PIN_DEBOUNCE_TIMEOUT_mS;
	node_id_selex.contact_bouncing_detect=0;

	selex_bms.sync_counter=0;
}

//static void current_sense_update_adc_impl(ADC_Sensor* p_ss){
//	p_ss->adc=current_sense_read_adc();
//}
//
//static void current_sense_update_offset_impl(ADC_Sensor* p_ss){
//	p_ss->offset=current_sense_read_offset();
//}

static void bq_sw_turn_off_discharge(Switch* p_sw){
	uint8_t fet_status;
	bq_turn_off_fet_DSG_PDSG(&bq);
	fet_status=bq_read_fet_status_reg(&bq);
	p_sw->state=SW_ST_OFF;
}

static void bq_sw_turn_off_charge(Switch* p_sw){
	uint8_t fet_status;
	bq_turn_off_fet_CHG_PCHG(&bq);
	fet_status=bq_read_fet_status_reg(&bq);
	p_sw->state=SW_ST_OFF;
}

static void bq_sw_turn_on_discharge(Switch* p_sw){
	uint8_t fet_status;
	bq_turn_on_all_fet(&bq);
	fet_status=bq_read_fet_status_reg(&bq);
	p_sw->state=SW_ST_ON;
}

static void bq_sw_turn_on_charge(Switch* p_sw){
	bq_turn_on_all_fet(&bq);
	bq_read_fet_status_reg(&bq);
	p_sw->state=SW_ST_ON;
}

static void bq_sw_turn_on_all_fet(Switch* p_sw){
	bq_turn_on_all_fet(&bq);
	bq_read_fet_status_reg(&bq);
	p_sw->state=SW_ST_ON;
}

static void bq_sw_turn_off_all_fet(Switch* p_sw){
	uint8_t fet_status;
	bq_turn_off_all_fet(&bq);
	fet_status=bq_read_fet_status_reg(&bq);
	p_sw->state=SW_ST_OFF;
}

static void ntc_ts5_update_impedance_impl(NTC* p_ntc){
	p_ntc->impedance=ts5_ntc_read_impedance();
}

static void ntc_ts6_update_impedance_impl(NTC* p_ntc){
	p_ntc->impedance=ts6_ntc_read_impedance();
}
static void upper_ntc_update_temperature_impl(NTC* p_ntc){

	p_ntc->temp=bq_read_ntc_temperature_TS1(&bq);
}

static void middle_ntc_update_temperature_impl(NTC* p_ntc){

	p_ntc->temp=bq_read_ntc_temperature_TS3(&bq);
}

static void side_ntc_update_temperature_impl(NTC* p_ntc){

	p_ntc->temp=bq_read_ntc_temperature_HDQ_TEMP(&bq);
}

static void bottom_ntc_update_temperature_impl(NTC* p_ntc){

	p_ntc->temp=bq_read_ntc_temperature_CFETOFF_TEMP(&bq);
}

static void bms_update_state_indicator_impl(const BMS* const p_bms){
        switch(bms_get_state(p_bms)){
                case BMS_ST_FAULT:
                        rgb_set_color(1, 0, 0,1);
                        break;
                case BMS_ST_CHARGING:
                case BMS_ST_DISCHARGING:
                        if(p_bms->afe->pack_voltage<56000){
                                rgb_set_color(1,0,1,1);
                        }
                        if(p_bms->afe->pack_voltage>57000){
                                rgb_set_color(0,0,1,0);
                        }
                        else
                        {
                        	rgb_set_color(0,0,1,0);
                        }
                        break;
                case BMS_ST_SOFTSTART:
                        rgb_set_color(1, 0, 1,0);
                        break;
                case BMS_ST_IDLE:
                        rgb_set_color(0, 1, 0,0);
                        break;
                case BMS_ST_SYSTEM_BOOST_UP:
                		rgb_set_color(0, 1, 1, 0);
                		break;
                case BMS_ST_ID_ASSIGN_START:
                case BMS_ST_ID_ASSIGN_WAIT_CONFIRM:
                case BMS_ST_ID_ASSIGN_CONFIRMED:
                case BMS_ST_ID_ASSIGN_WAIT_SLAVE_SELECT:
                case BMS_ST_START_AUTHENTICATE:
                case BMS_ST_AUTHENTICATING:
                		rgb_set_color(1, 1, 0, 1);
                		break;
                case BMS_ST_STANDBY:
                		rgb_set_color(0, 1, 0, 1);
                		break;
                default:
                        rgb_set_color(0, 0, 0,0);
                        break;
        }
}
