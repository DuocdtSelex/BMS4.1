/*
 * afe_init.c
 *
 *  Created on: Aug 20, 2020
 *      Author: quangnd
 */

#include "afe_init.h"
#include "afe_config.h"
#include "bms_config.h"

BQ769x2 bq;

AFE app_afe;
Cell_Array pack_cell_array;
Cell_Array cell_array;
Cell_Bank cells[PACK_MAX_CELL];
static uint8_t short_cell_mask[PACK_MAX_CELL]={
		0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0
};

static void afe_disable_cell_balancing_impl(AFE* this);
static void afe_enable_cell_balancing_impl(AFE* this,const uint32_t target_vol);
static void afe_set_occ_threshold_mA_impl(AFE* this, uint32_t th);
static void afe_set_scd_threshold_mA_impl(AFE* this, uint32_t th);
static void afe_set_cuv_threshold_mV_impl(AFE* this, uint32_t th);
static void afe_set_cov_threshold_mV_impl(AFE* this, uint32_t th);
static void afe_set_ocd1_threshold_mA_impl(AFE* this, uint32_t th);
static void afe_set_ocd2_threshold_mA_impl(AFE* this, uint32_t th);
static void afe_set_occ_delay_ms_impl(AFE* this, uint32_t ms);
static void afe_set_scd_delay_us_impl(AFE* this, uint32_t us);
static void afe_set_cuv_delay_ms_impl(AFE* this, uint32_t ms);
static void afe_set_cov_delay_ms_impl(AFE* this, uint32_t ms);
static void afe_set_ocd1_delay_ms_impl(AFE* this, uint32_t ms);
static void afe_set_ocd2_delay_ms_impl(AFE* this, uint32_t ms);
static void afe_set_cov_recovery_hysteresis_mV_impl(AFE* this, uint32_t th);
static void afe_set_cuv_recovery_hysteresis_mV_impl(AFE* this, uint32_t th);
static void afe_set_scd_recovery_time_s_impl(AFE* this, uint32_t s);
static void afe_set_occ_recovery_threhshold_mA_impl(AFE* this, int32_t th);
static void afe_set_ocd_recovery_threhshold_mA_impl(AFE* this, int32_t th);
static void afe_update_cell_voltage_impl(AFE* this);
static void afe_update_pack_voltage_impl(AFE* this);
static void afe_update_status_impl(AFE* this);
static void afe_update_battery_status_impl(AFE* this);
static void afe_reset_error_impl(AFE* this);
static void bq_interrupt_handle_impl(BQ_Hw* p_hw);
static void afe_set_shutdown_mode_impl(AFE* this);
static void afe_update_current_impl(AFE* this);
static void afe_update_fet_status_impl(AFE* this);
static void afe_update_safety_alert_status_impl(AFE* this);
static void afe_update_permanent_fail_state_impl(AFE* this);
static void afe_set_otd_threshold_C_impl(AFE* this, uint8_t th);
static void afe_set_otc_threshold_C_impl(AFE* this, uint8_t th);
static void afe_set_otint_threshold_C_impl(AFE* this, uint8_t th);
static void afe_set_otf_threshold_C_impl(AFE* this, uint8_t th);
static void afe_set_otc_recovery_C_impl(AFE* this, uint8_t th);
static void afe_set_utd_threshold_C_impl(AFE* this, uint8_t th);
static void afe_set_utc_threshold_C_impl(AFE* this, uint8_t th);
static void afe_set_utint_threshold_C_impl(AFE* this, uint8_t th);
static void afe_update_int_temperature_C_impl(AFE* this);

static AFE_Interface app_afe_interface={
	afe_disable_cell_balancing_impl,
	afe_enable_cell_balancing_impl,
	afe_set_occ_threshold_mA_impl,
	afe_set_scd_threshold_mA_impl,
	afe_set_cuv_threshold_mV_impl,
	afe_set_cov_threshold_mV_impl,
	afe_set_ocd1_threshold_mA_impl,
	afe_set_ocd2_threshold_mA_impl,
	afe_set_occ_delay_ms_impl,
	afe_set_scd_delay_us_impl,
	afe_set_cuv_delay_ms_impl,
	afe_set_cov_delay_ms_impl,
	afe_set_ocd1_delay_ms_impl,
	afe_set_ocd2_delay_ms_impl,
	afe_set_cov_recovery_hysteresis_mV_impl,
	afe_set_cuv_recovery_hysteresis_mV_impl,
	afe_set_scd_recovery_time_s_impl,
	afe_set_occ_recovery_threhshold_mA_impl,
	afe_set_ocd_recovery_threhshold_mA_impl,
	afe_update_cell_voltage_impl,
	afe_update_pack_voltage_impl,
	afe_update_status_impl,
	afe_update_battery_status_impl,
	afe_reset_error_impl,
	afe_set_shutdown_mode_impl,
	afe_update_current_impl,
	afe_update_fet_status_impl,
	afe_update_safety_alert_status_impl,
	afe_update_permanent_fail_state_impl,
	afe_set_otd_threshold_C_impl,
	afe_set_otc_threshold_C_impl,
	afe_set_otint_threshold_C_impl,
	afe_set_otf_threshold_C_impl,
	afe_set_otc_recovery_C_impl,
	afe_set_utd_threshold_C_impl,
	afe_set_utc_threshold_C_impl,
	afe_set_utint_threshold_C_impl,
	afe_update_int_temperature_C_impl
};

void afe_init(void){
	afe_set_interface(&app_afe,&app_afe_interface);
	bq.hw =&bq_hw;
	bq_set_interrupt_handle_1(&bq_hw,bq_interrupt_handle_impl);
	bq_init_1(&bq);

	pack_cell_array.cells=&cells[0];
	pack_cell_array.serial_cells=PACK_MAX_CELL;
	for(int i=0;i<pack_cell_array.serial_cells;i++){
		pack_cell_array.cells[i].capacity=CELL_CAPACITY;
		pack_cell_array.cells[i].voltage=3700UL;
		pack_cell_array.cells[i].is_short=short_cell_mask[i];
	}
	app_afe.cell_array=&pack_cell_array;
	app_afe.battery_status =1;

	bq.unshort_cell =16;

	cell_array.cells=&cells[0];
	bq.base.cell_array=&cell_array;
	bq.base.cell_array->serial_cells=BQ_MAX_CELL;

	bq_enter_config_update_mode(&bq);

	afe_set_ocd1_threshold_mA(&app_afe,OCD1_THRESHOLD_mA);
	afe_set_ocd1_delay_ms(&app_afe,OCD1_DELAY_mS);

	afe_set_ocd2_threshold_mA(&app_afe,OCD2_THRESHOLD_mA);
	afe_set_ocd2_delay_ms(&app_afe,OCD2_DELAY_mS);
	afe_set_ocd_recovery_threshold_mA(&app_afe, OCD_RECOVERY_THRESHOLD_mA);

	afe_set_occ_threshold_mA(&app_afe,OCC_THRESHOLD_mA);
	afe_set_occ_delay_ms(&app_afe,OCC_DELAY_mS);
	afe_set_occ_recovery_threshold_mA(&app_afe, OCC_HYSTERESIS_mA);

	afe_set_scd_threshold_mA(&app_afe,SCD_THRESHOLD_mA);
	afe_set_scd_delay_us(&app_afe,SCD_DELAY_uS);
	afe_set_scd_recovery_time_s(&app_afe, SCD_RECOVERY_TIME_s);

	afe_set_cov_threshold_mV(&app_afe,COV_THRESHOLD_mV);
	afe_set_cov_delay_ms(&app_afe,COV_DELAY_mS);
	afe_set_cov_recovery_hysteresis(&app_afe,COV_HYSTERESIS_mV);

	afe_set_cuv_threshold_mV(&app_afe,CUV_THRESHOLD_mV);
	afe_set_cuv_delay_ms(&app_afe,CUV_DELAY_mS);
	afe_set_cuv_recovery_hysteresis(&app_afe, CUV_HYSTERESIS_mV);

	afe_set_otd_threshold_C(&app_afe, OTD_THRESHOLD_DEG_C);
	afe_set_otc_threshold_C(&app_afe, OTC_THRESHOLD_DEG_C);
	afe_set_otint_threshold_C(&app_afe, OTINT_THRESHOLD_DEG_C);
	afe_set_otf_threshold_C(&app_afe, OTF_THRESHOLD_DEG_C);

	afe_set_otc_recovery_C(&app_afe, OTC_RECOVERY_C);

	afe_set_utd_threshold_C(&app_afe, UTD_THRESHOLD_DEG_C);
	afe_set_utc_threshold_C(&app_afe, UTC_THRESHOLD_DEG_C);
	afe_set_utint_threshold_C(&app_afe, UTINT_THRESHOLD_DEG_C);

	bq_exit_config_update_mode(&bq);
}

static void afe_update_pack_voltage_impl(AFE* p_afe){

	afe_update_pack_voltate((AFE*)&bq);
	p_afe->pack_voltage = bq.base.pack_voltage;
}

static void afe_disable_cell_balancing_impl(AFE* this){

	afe_disable_cell_balancing((AFE*)&bq);
    this->balancing_mask = bq.base.balancing_mask;
}

static void afe_enable_cell_balancing_impl(AFE* this,const uint32_t target_vol){
    afe_enable_cell_balancing((AFE*)&bq,target_vol);
    this->balancing_mask = bq.base.balancing_mask;
}

static void afe_set_occ_threshold_mA_impl(AFE* this, uint32_t th){
	afe_set_occ_threshold_mA((AFE*)&bq,th);
}

static void afe_set_scd_threshold_mA_impl(AFE* this, uint32_t th){
	afe_set_scd_threshold_mA((AFE*)&bq,th);
}

static void afe_set_cuv_threshold_mV_impl(AFE* this, uint32_t th){
	afe_set_cuv_threshold_mV((AFE*)&bq,th);
}

static void afe_set_cov_threshold_mV_impl(AFE* this, uint32_t th){
	afe_set_cov_threshold_mV((AFE*)&bq,th);
}

static void afe_set_ocd1_threshold_mA_impl(AFE* this, uint32_t th){
	afe_set_ocd1_threshold_mA((AFE*)&bq, th);
}

static void afe_set_ocd2_threshold_mA_impl(AFE* this, uint32_t th){
	afe_set_ocd2_threshold_mA((AFE*)&bq, th);
}

static void afe_set_occ_delay_ms_impl(AFE* this, uint32_t ms){
	afe_set_occ_delay_ms((AFE*)&bq,ms);
}

static void afe_set_scd_delay_us_impl(AFE* this, uint32_t us){
	afe_set_scd_delay_us((AFE*)&bq,us);
}

static void afe_set_cuv_delay_ms_impl(AFE* this, uint32_t ms){
	afe_set_cuv_delay_ms((AFE*)&bq,ms);
}

static void afe_set_cov_delay_ms_impl(AFE* this, uint32_t ms){
	afe_set_cov_delay_ms((AFE*)&bq,ms);
}

static void afe_set_ocd1_delay_ms_impl(AFE* this, uint32_t ms){
	afe_set_ocd1_delay_ms((AFE*)&bq, ms);
}

static void afe_set_ocd2_delay_ms_impl(AFE* this, uint32_t ms){
	afe_set_ocd2_delay_ms((AFE*)&bq, ms);
}

static void afe_set_cov_recovery_hysteresis_mV_impl(AFE* this, uint32_t th){
	afe_set_cov_recovery_hysteresis((AFE*)&bq, th);
}

static void afe_set_cuv_recovery_hysteresis_mV_impl(AFE* this, uint32_t th){
	afe_set_cuv_recovery_hysteresis((AFE*)&bq, th);
}
static void afe_set_scd_recovery_time_s_impl(AFE* this, uint32_t s){
	afe_set_scd_recovery_time_s((AFE*)&bq, s);
}

static void afe_set_occ_recovery_threhshold_mA_impl(AFE* this, int32_t th){
	afe_set_occ_recovery_threshold_mA((AFE*)&bq, th);
}
static void afe_set_ocd_recovery_threhshold_mA_impl(AFE* this, int32_t th){
	afe_set_ocd_recovery_threshold_mA((AFE*)&bq, th);
}
static void afe_update_cell_voltage_impl(AFE* this){

	afe_update_cell_voltage((AFE*)&bq);
}

static void afe_update_status_impl(AFE* this){

	afe_update_status((AFE*)&bq);
//	this->status&= 0xFFFFFF00;
	this->status=bq.base.status ;
	int32_t last_error=afe_get_last_error((AFE*)&bq);
	if(last_error != AFE_ERROR_NO){
		afe_set_error(this,last_error);
		this->status |= AFE_STS_HW_ERROR;
	}
}

static void afe_update_battery_status_impl(AFE* this){
	afe_update_battery_status((AFE*)&bq);
	this->battery_status = bq.base.battery_status;
}

static void afe_update_current_impl(AFE* this){
	afe_update_current_cA((AFE*)&bq);
	this->current = bq.base.current;
}

static void afe_update_fet_status_impl(AFE* this){
	afe_update_fet_status((AFE*)&bq);
	this->fet_status =bq.base.fet_status;
}

static void afe_update_safety_alert_status_impl(AFE* this){
	afe_update_safety_alert_status((AFE*)&bq);
	this->safety.safety_alert_a  = bq.base.safety.safety_alert_a;
	this->safety.safety_status_a = bq.base.safety.safety_status_a;
	this->safety.safety_alert_b  = bq.base.safety.safety_alert_b;
	this->safety.safety_status_b = bq.base.safety.safety_status_b;
	this->safety.safety_alert_c  = bq.base.safety.safety_alert_c;
	this->safety.safety_status_c = bq.base.safety.safety_status_c;
}

static void afe_update_permanent_fail_state_impl(AFE* this){
	afe_update_permanent_fail_state((AFE*)&bq);
	this->permanent_fail.pf_alert_a  = bq.base.permanent_fail.pf_alert_a;
	this->permanent_fail.pf_status_a = bq.base.permanent_fail.pf_status_a;
	this->permanent_fail.pf_alert_b  = bq.base.permanent_fail.pf_alert_b;
	this->permanent_fail.pf_status_b = bq.base.permanent_fail.pf_status_b;
	this->permanent_fail.pf_alert_c  = bq.base.permanent_fail.pf_alert_c;
	this->permanent_fail.pf_status_c = bq.base.permanent_fail.pf_status_c;
	this->permanent_fail.pf_alert_d  = bq.base.permanent_fail.pf_alert_d;
	this->permanent_fail.pf_status_d = bq.base.permanent_fail.pf_status_d;
}

static void afe_set_otd_threshold_C_impl(AFE* this, uint8_t th){
	afe_set_otd_threshold_C((AFE*)&bq, th);
}

static void afe_set_otc_threshold_C_impl(AFE* this, uint8_t th){
	afe_set_otc_threshold_C((AFE*)&bq, th);
}

static void afe_set_otint_threshold_C_impl(AFE* this, uint8_t th){
	afe_set_otint_threshold_C((AFE*)&bq, th);
}

static void afe_set_otf_threshold_C_impl(AFE* this, uint8_t th){
	afe_set_otf_threshold_C((AFE*)&bq, th);
}

static void afe_set_otc_recovery_C_impl(AFE* this, uint8_t th){
	afe_set_otc_recovery_C((AFE*)&bq, th);
}
static void afe_set_utd_threshold_C_impl(AFE* this, uint8_t th){
	afe_set_utd_threshold_C((AFE*)&bq, th);
}

static void afe_set_utc_threshold_C_impl(AFE* this, uint8_t th){
	afe_set_utc_threshold_C((AFE*)&bq, th);
}

static void afe_set_utint_threshold_C_impl(AFE* this, uint8_t th){
	afe_set_utint_threshold_C((AFE*)&bq, th);
}

static void afe_update_int_temperature_C_impl(AFE* this){
	afe_update_int_temperature_C((AFE*)&bq);
	this->int_temperature = bq.base.int_temperature;
}
static void afe_reset_error_impl(AFE* this){

	afe_reset_error((AFE*)&bq);
	this->error=AFE_ERROR_NO;
	this->status=0;
}

static void bq_interrupt_handle_impl(BQ_Hw* p_hw){
	(void)p_hw;
}

static void afe_set_shutdown_mode_impl(AFE* this){
	afe_set_shutdown_mode((AFE*)&bq);
}
