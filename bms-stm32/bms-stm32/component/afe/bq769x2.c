/*
 * bq769x2.c
 *
 *  Created on: Apr 13, 2021
 *      Author: Admin
 */
#include "bq769x2.h"
#include "afe.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "current_sense_hal.h"

#define COV_CUV_THRESHOLD_UINT_mV 		50.6f
#define COV_CUV_DELAY_UNIT_ms			3.3f
#define COV_CUV_RECOVERY_HYSTERESIS_mV	50.6f
#define SCD_DEALY_UNIT_us				15
#define OCC_THRESHOLD_UNIT_mV			2
#define OCC_DELAY_UNIT_ms				3.3f
#define OCC_RECOVERY_THRESHOLD_mA		1
#define OCD_THRESHOLD_UINT_mV			2
#define OCD_DELAY_UNIT_ms				3.3f
#define OCD_COVERY_HYSTERESIS_mA		1
#define STACK_VOLTAGE_UINT_mV			10
#define SHUTDOWN_DELAY_ms				0.25f
#define BALANCING_STOP_DELTA			30
#define BALANCING_VOLTAGE_MIN_THRESHOLD 3500

#define MIN_CUV_THRESHOLD				20
#define MAX_CUV_THRESHOLD				80
#define DEFAULT_CUV_THRESHOLD			50

#define MIN_COV_THRESHOLD				20
#define MAX_COV_THRESHOLD				110
#define DEFAULT_COV_THRESHOLD			86

#define MIN_OCC_THRESHOLD				2
#define MAX_OCC_THRESHOLD				62
#define DEFAULT_OCC_THRESHOLD			2

#define MIN_OCD1_THRESHOLD 				2
#define MAX_OCD1_THRESHOLD				100
#define DEFAULT_OCD1_THRESHOLD			4

#define MIN_OCD2_THREHSOLD				2
#define MAX_OCD2_THRESHOLD				100
#define DEFAULT_OCD2_THESHOLD			3

// cell balancing
static void bq_host_disable_cell_balancing_impl(AFE *this);
static void bq_host_enable_cell_balancing_impl(AFE *this, uint32_t voltage_target);
/*
 * Protection COV, CUV, SCD, OCC, OCC
 */
	// protection threshold
static void bq_set_cov_threshold_mV_impl(AFE *this, uint32_t th);
static void bq_set_cuv_threshold_mV_impl(AFE *this, uint32_t th);
static void bq_set_occ_threshold_mA_impl(AFE *this, uint32_t th);
static void bq_set_ocd1_threshold_mA_impl(AFE *this, uint32_t th);
static void bq_set_ocd2_threshold_mA_impl(AFE *this, uint32_t th);
static void bq_set_scd_threshold_mA_impl(AFE *this, uint32_t th);
	// protection delay
static void bq_set_cov_delay_ms_impl(AFE *this, uint32_t th);
static void bq_set_cuv_delay_ms_impl(AFE *this, uint32_t th);
static void bq_set_occ_delay_ms_impl(AFE *this, uint32_t th);
static void bq_set_ocd1_delay_ms_impl(AFE *this, uint32_t th);
static void bq_set_ocd2_delay_mA_impl(AFE *this, uint32_t th);
static void bq_set_scd_delay_us_impl(AFE *this, uint32_t th);
	// protection hysteresis
static void bq_set_cov_recovery_hysteresis_impl(AFE *this, uint32_t th);
static void bq_set_cuv_recovery_hysteresis_impl(AFE *this, uint32_t th);
static void bq_set_occ_recovery_threshold_mA_impl(AFE *this, int32_t th);
static void bq_set_ocd_recovery_hysteresis_mA_impl(AFE *this, int32_t th);
static void bq_set_scd_recovery_time_s_impl(AFE *this, uint32_t s);

/*
 * Protection over temperature
 */
static void bq_set_recovery_time(AFE* this, uint8_t th);
static void bq_set_otc_threshold_impl(AFE *this, uint8_t th);
static void bq_set_otd_threshold_impl(AFE *this, uint8_t th);
static void bq_set_otf_threshold_impl(AFE *this, uint8_t th);
static void bq_set_otint_threshold_impl(AFE *this, uint8_t th);
static void bq_set_utc_threshold_impl(AFE *this, uint8_t th);
static void bq_set_utd_threshold_impl(AFE *this, uint8_t th);
static void bq_set_utint_threshold_impl(AFE *this, uint8_t th);

static void bq_set_otc_delay_impl(AFE *this, uint8_t th);
static void bq_set_otd_delay_impl(AFE *this, uint8_t th);
static void bq_set_otf_delay_impl(AFE *this, uint8_t th);
static void bq_set_otint_delay_impl(AFE *this, uint8_t th);
static void bq_set_utc_delay_impl(AFE *this, uint8_t th);
static void bq_set_utd_delay_impl(AFE *this, uint8_t th);
static void bq_set_utint_delay_impl(AFE *this, uint8_t th);

static void bq_set_otc_recovery_impl(AFE *this, uint8_t th);
static void bq_set_otd_recovery_impl(AFE *this, uint8_t th);
static void bq_set_otf_recovery_impl(AFE *this, uint8_t th);
static void bq_set_otint_recovery_impl(AFE *this, uint8_t th);
static void bq_set_utc_recovery_impl(AFE *this, uint8_t th);
static void bq_set_utd_recovery_impl(AFE *this, uint8_t th);
static void bq_set_utint_recovery_impl(AFE *this, uint8_t th);
// measurement voltage
static void bq_update_cell_voltage_impl(AFE *this);
static void bq_update_pack_voltage_impl(AFE *this);
static void bq_update_status_impl(AFE *this);       // status of Safety Alert A register
static void bq_update_battery_status_impl(AFE *this);
static void bq_update_current_cA_impl(AFE* this);
static void bq_update_fet_status_impl(AFE* this);
static void bq_update_safety_alert_status_impl(AFE* this);
static void bq_update_permanent_fail_state_impl(AFE* this);
static void bq_update_int_temperature_impl(AFE* this);
// handle error
static void bq_reset_error_impl(AFE *this);
// power mode
//default initial allow enter or exit sleep mode
static void bq_enable_sleep_mode_impl(AFE *this);
static void bq_disable_sleep_mode_impl(AFE *this);
static void bq_enable_deep_sleep_mode_impl(AFE *this);
static void bq_disable_deep_sleep_mode_impl(AFE *this);
static void bq_enable_shutdown_mode_impl(AFE *this);

//cell balancing

// Threshold SCD for Predischarge
static void bq_set_scd_pdsg_threshold_mA_(AFE *this, uint32_t th);

static AFE_Interface bq_interface = {
		bq_host_disable_cell_balancing_impl,
		bq_host_enable_cell_balancing_impl,
		bq_set_occ_threshold_mA_impl,
		bq_set_scd_threshold_mA_impl,
		bq_set_cuv_threshold_mV_impl,
		bq_set_cov_threshold_mV_impl,
		bq_set_ocd1_threshold_mA_impl,
		bq_set_ocd2_threshold_mA_impl,
		bq_set_occ_delay_ms_impl,
		bq_set_scd_delay_us_impl,
		bq_set_cuv_delay_ms_impl,
		bq_set_cov_delay_ms_impl,
		bq_set_ocd1_delay_ms_impl,
		bq_set_ocd2_delay_mA_impl,
		bq_set_cov_recovery_hysteresis_impl,
		bq_set_cuv_recovery_hysteresis_impl,
		bq_set_scd_recovery_time_s_impl,
		bq_set_occ_recovery_threshold_mA_impl,
		bq_set_ocd_recovery_hysteresis_mA_impl,
		bq_update_cell_voltage_impl,
		bq_update_pack_voltage_impl,
		bq_update_status_impl,
		bq_update_battery_status_impl,
		bq_reset_error_impl,
		bq_enable_shutdown_mode_impl,
		bq_update_current_cA_impl,
		bq_update_fet_status_impl,
		bq_update_safety_alert_status_impl,
		bq_update_permanent_fail_state_impl,
		bq_set_otd_threshold_impl,
		bq_set_otc_threshold_impl,
		bq_set_otint_threshold_impl,
		bq_set_otf_threshold_impl,
		bq_set_otc_recovery_impl,
		bq_set_utd_threshold_impl,
		bq_set_utc_threshold_impl,
		bq_set_utint_threshold_impl,
		bq_update_int_temperature_impl
};


/**
 * Helper functions
 */
static void bq_set_number_of_cells(BQ769x2* p_bq, uint16_t mask);
static int32_t bq_i2c_data_ram_write_byte_verify(BQ769x2* p_bq, uint16_t reg_addr, uint8_t reg_data);
static int32_t bq_i2c_data_ram_write_word_verify(BQ769x2* p_bq, uint16_t reg_addr, uint16_t reg_data);
static int32_t bq_i2c_subcommand_write_word_verify(BQ769x2* p_bq, uint16_t reg_addr, uint16_t reg_data);
static uint8_t bq_get_scd_from_threshold(uint32_t mA);
/*
 * helper function for configure bq 76952
 */
static void bq_config_TS3_pin(BQ769x2* p_bq, uint8_t data);
static void bq_config_CFETOFF_pin(BQ769x2* p_bq, uint8_t data);
static void bq_config_HDQ_pin(BQ769x2* p_bq, uint8_t data);
static void bq_config_ALERT_pin(BQ769x2* p_bq, uint8_t data);
static void bq_config_default_alarm_mask(BQ769x2* p_bq, uint16_t data);
static void bq_config_enable_protection_A(BQ769x2* p_bq, uint8_t data);
static void bq_config_reg12(BQ769x2* p_bq, uint8_t data);
static void bq_config_reg0(BQ769x2* p_bq, uint8_t data);
static void bq_config_mfg_status_init(BQ769x2* p_bq, uint16_t data);
static void bq_config_da_configuration(BQ769x2* p_bq, uint8_t data);
static void bq_enable_host_control_fet(BQ769x2* p_bq);
static void bq_config_fet_options_reg(BQ769x2* p_bq, uint8_t data);
static void bq_config_chg_pump_control_reg(BQ769x2* p_bq, uint8_t data);

/*  must configure CHG FET Protections A (default 0x98) and DSG FET Protections A (default 0xE4)
 *  to control FET action taken
 */
static void bq_config_enable_protection_B(BQ769x2* p_bq, uint8_t data);
/*
 * must configure CHG FET Protection B (default 0xD5)
 * an DSG FET Protection B (default 0xE6) to control FET action taken
 */
static void bq_set_sleep_hysteresis_time_s(BQ769x2* p_bq, uint8_t time);
static void bq_set_sleep_current_mA(BQ769x2* p_bq, uint16_t current_mA);
static void bq_set_sleep_voltage_time_s(BQ769x2* p_bq, uint8_t voltage_time); //recommend set 5s or (4*n+1) seconds
static void bq_set_sleep_wake_comparator_current_mA(BQ769x2* p_bq, uint16_t current);
static void bq_set_sleep_charge_pack_tos_delta_mV(BQ769x2* p_bq, uint16_t delta);
static void bq_set_sleep_charge_voltage_threshold(BQ769x2* p_bq, uint16_t threshold);
static int bq_set_bit_power_config_reg(BQ769x2* p_bq, uint16_t bit_mask);
//shutdown base on shut_down_cell_voltage consider disable (value 0)
static void bq_set_shutdown_stack_voltage_mV(BQ769x2* p_bq, uint16_t threshold);
static void bq_set_low_v_shutdown_delay_s(BQ769x2* p_bq, uint8_t time);
// fet_off_delay must less than command_delay
static void bq_set_shutdown_fet_off_dealy_ms(BQ769x2* p_bq, uint8_t time);
static void bq_set_shutdown_command_delay_ms(BQ769x2* p_bq, uint8_t time);
static void bq_set_auto_shutdown_time_min(BQ769x2* p_bq, uint8_t time);
static void bq_set_shutdown_temperature_threshold(BQ769x2* p_bq, uint8_t threshold);
static void bq_set_shutdown_temperature_delay_ms(BQ769x2* p_bq, uint8_t time);
// cell balancing configure default: enable host control balancing
static void bq_set_cell_balance_max_cells(BQ769x2* p_bq, uint8_t number);
static void bq_set_level_voltage_threshold(BQ769x2* p_bq, uint16_t threshold);
static void bq_get_time_balancing_active(BQ769x2* p_bq, uint16_t data);
static void bq_set_min_cell_temp(BQ769x2* p_bq, uint8_t min_temp);
static void bq_set_max_cell_temp(BQ769x2* p_bq, uint8_t max_temp);
static void bq_set_max_internal_temp(BQ769x2* p_bq, uint8_t max_temp);
static void bq_set_cell_balance_interval(BQ769x2* p_bq, uint8_t time_s);

// calibration
static void bq_get_calibration_cell_voltage(BQ769x2* p_bq, int16_t* cell_gain);
static void bq_get_cc_gain(BQ769x2* p_bq, float cc_gain);

void bq_init_1(BQ769x2 *p_bq) {
	bq_set_interface(p_bq, &bq_interface);
	uint16_t battery_status;
	bq_enter_config_update_mode(p_bq);  // enter write data ram
	bq_i2c_read_reg_word(p_bq->hw, BATTERY_STATUS, &battery_status);
	// wait until bit CFGUPDATE is 1
// 	while((battery_status && 0x01)!=1){
//		bq_enter_config_update_mode(p_bq);
//		bq_i2c_read_reg_word(p_bq->hw, BATTERY_STATUS, &battery_status);
//	}
	bq_config_reg0(p_bq, 0x01);
	bq_config_reg12(p_bq, 0x0D);
	bq_config_mfg_status_init(p_bq, 0x0050);
	bq_config_fet_options_reg(p_bq, 0x0F);
	bq_config_chg_pump_control_reg(p_bq, 0x01);
	bq_set_number_of_cells(p_bq, 0XFFFF);		 // configure 16 cell is connect
	bq_config_enable_protection_A(p_bq, 0xFC);	 // enable SCD, OCC, OCD1, OCD2 , CUV, COV protection
	bq_config_enable_protection_B(p_bq, 0xF7); 	// enable all over temperature protection
	bq_config_ALERT_pin(p_bq, 0x2A);
	//bq_config_default_alarm_mask(p_bq, 0xF8FE); // set as default
	bq_config_da_configuration(p_bq, 0x06);     // configure user-volt:10mV,user-ampe:10mA
    bq_config_TS3_pin(p_bq, 0x07);
	bq_config_CFETOFF_pin(p_bq, 0x07); 			// configure CFETOFF as temperature sensor pin, use for cell protection
    bq_config_HDQ_pin(p_bq, 0x0F);				// configure HDQ as temperature sensor pin, use for fet protection
    bq_config_body_diode_threshold(p_bq, 10000);
    bq_set_cell_balance_max_cells(p_bq, 8);
    bq_set_level_voltage_threshold(p_bq, 3500);
    bq_set_min_cell_temp(p_bq, 0);
    bq_set_max_cell_temp(p_bq, 60);
    bq_exit_config_update_mode(p_bq); //exit write data ram
    bq_i2c_read_reg_word(p_bq->hw, BATTERY_STATUS, &battery_status);
}

void bq_set_cell_group(BQ769x2* p_bq,const uint8_t group){

	p_bq->cell_group=group;
}
static void bq_reset_error_impl(AFE *this){
	BQ769x2 *p_bq = (BQ769x2*) this;
	bq_clear_alert_bit(p_bq);
	this->error=AFE_ERROR_NO;
}

static void bq_set_number_of_cells(BQ769x2* p_bq, uint16_t mask){
    bq_i2c_data_ram_write_word(p_bq->hw, VCELL_MODE, mask);
}
int bq_set_byte_register_bit(BQ769x2* p_bq,uint8_t reg, const uint8_t bit_mask){
	int success = 0;
	uint8_t reg_data = 0;
	success = bq_i2c_read_reg_byte(p_bq->hw, reg, &reg_data);
	if (success != 0) {
		return success;
	}
	reg_data |= bit_mask;
	success = bq_i2c_write_reg_byte(p_bq->hw, reg, reg_data);
	return success;
}

int bq_clear_byte_register_bit(BQ769x2* p_bq,uint8_t reg, const uint8_t bit_mask){
	int success = 0;
	uint8_t control_reg = 0;
	success = bq_i2c_read_reg_byte(p_bq->hw, reg, &control_reg);
	if (success != 0) {
		return success;
	}
	control_reg &= ~bit_mask;
	success = bq_i2c_write_reg_byte(p_bq->hw, reg, control_reg);
	return success;
}

int bq_set_word_register_bit(BQ769x2* p_bq,uint16_t reg, const uint16_t bit_mask){
	int success = 0;
	uint16_t control_reg = 0;
	success = bq_i2c_read_reg_word(p_bq->hw, reg, &control_reg);
	if (success != 0) {
		return success;
	}
	control_reg |= bit_mask;
	success = bq_i2c_write_reg_word(p_bq->hw, reg, control_reg);
	return success;
}

int bq_clear_word_register_bit(BQ769x2* p_bq,uint16_t reg, const uint16_t bit_mask){
	int success = 0;
	uint16_t control_reg = 0;
	success = bq_i2c_read_reg_word(p_bq->hw, reg, &control_reg);
	if (success != 0) {
		return success;
	}
	control_reg &= ~bit_mask;
	success = bq_i2c_write_reg_word(p_bq->hw, reg, control_reg);
	return success;
}

static int32_t bq_i2c_data_ram_write_byte_verify(BQ769x2* p_bq, uint16_t reg_addr, uint8_t reg_data){
	uint8_t verify = 0;
	int32_t success = bq_i2c_data_ram_write_byte(p_bq->hw, reg_addr, reg_data);
	if(success != 0) return success;
	success = bq_i2c_data_ram_read_byte(p_bq->hw, reg_addr, &verify);
	if(success != 0) return success;

	if(reg_data != verify){
		return -2;
	}
	return 0;
}

static int32_t bq_i2c_data_ram_write_word_verify(BQ769x2* p_bq, uint16_t reg_addr, uint16_t reg_data){
	uint16_t verify = 0;
	int32_t success = bq_i2c_data_ram_write_word(p_bq->hw, reg_addr, reg_data);
	if(success != 0) return success;
	success = bq_i2c_data_ram_read_word(p_bq->hw, reg_addr, &verify);
	if(success != 0) return success;

	if(reg_data != verify){
		return -2;
	}
	return 0;
}

static int32_t bq_i2c_subcommand_write_word_verify(BQ769x2* p_bq, uint16_t reg_addr, uint16_t reg_data){
	uint16_t verify =0;
	int32_t success = bq_i2c_subcommand_write_word(p_bq->hw, reg_addr, reg_data);
	if(success != 0) return success;
	success = bq_i2c_subcommand_read_word(p_bq->hw, reg_addr, &verify);
	if(success != 0) return success;

//	if(reg_data != verify){
//		return -2;
//	}
	return 0;
}

static void bq_update_cell_voltage_impl(AFE *this){
	BQ769x2 *p_bq = (BQ769x2*) this;

	int32_t success = 1;
	uint32_t i = 0;
	uint32_t block_size = 2*16; //16 cells, each cell need 2 byte for voltage information
	uint16_t voltage[16];
	uint8_t cell_voltage[32];

	success = bq_i2c_read_reg_block(p_bq->hw, CELL_1_VOLTAGE, cell_voltage,
			block_size);
	if(success != 0){
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	for(i =0; i <16; i++)
	{
		voltage[i] = cell_voltage[2*i+1];
		voltage[i] = (voltage[i] <<8) + cell_voltage[2*i];
		this->cell_array->cells[i].voltage = voltage[i];
	}

}

static void bq_update_pack_voltage_impl(AFE *this){
	BQ769x2 *p_bq = (BQ769x2*) this;

	int32_t success = 1;
	uint32_t pack_voltage=0;
	uint32_t i = 0;
	uint32_t j =0;
	uint32_t block_size = 2*16; //16 cells, each cell need 2 byte for voltage information
	uint16_t voltage[16];
	uint8_t cell_voltage[32];

#if 0
	success = bq_i2c_read_reg_word(p_bq->hw, PACK_PIN_VOLTAGE, &pack_voltage);
	if(success != 0){
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
#endif

	success = bq_i2c_read_reg_block(p_bq->hw, CELL_1_VOLTAGE, cell_voltage,
			block_size);
	if(success != 0){
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	for(i =0; i <16; i++)
	{
		voltage[i] = cell_voltage[2*i+1];
		voltage[i] = (voltage[i] <<8) + cell_voltage[2*i];
		pack_voltage += voltage[i];
	}

	this->pack_voltage = pack_voltage;
}

static void bq_update_status_impl(AFE *this){
	BQ769x2 *p_bq = (BQ769x2*) this;
	int success = -1;
	uint8_t data;
	success = bq_i2c_read_reg_byte(p_bq->hw, SAFETY_STATUS_A, &data);
	if (success != 0) {
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	this->status= data;
}

static void bq_update_battery_status_impl(AFE *this){
	BQ769x2 *p_bq = (BQ769x2*) this;
	int success = -1;
	uint16_t data;
	success = bq_i2c_read_reg_word(p_bq->hw, BATTERY_STATUS, &data);
	if (success != 0) {
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	this->battery_status= data;
}

static void bq_update_current_cA_impl(AFE* this){
	BQ769x2 *p_bq = (BQ769x2*) this;
	int success = -1;
	int16_t data;
	success = bq_i2c_read_reg_word_with_sign(p_bq->hw, CC2_CURRENT, &data);
	if (success != 0) {
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	this->current= data;
}

static void bq_update_fet_status_impl(AFE* this){
	BQ769x2 *p_bq = (BQ769x2*) this;
	int success = -1;
	uint8_t data;
	success = bq_i2c_read_reg_byte(p_bq->hw, FET_STATUS, &data);
	if (success != 0) {
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	this->fet_status= data;
}

static void bq_update_safety_alert_status_impl(AFE* this){
	BQ769x2 *p_bq = (BQ769x2*) this;
	int success = -1;
	uint8_t data[6];
	success = bq_i2c_read_reg_block(p_bq->hw, SAFETY_ALERT_A, data, 6);
	if (success != 0) {
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	this->safety.safety_alert_a  = data[0];
	this->safety.safety_status_a = data[1];
	this->safety.safety_alert_b  = data[2];
	this->safety.safety_status_b = data[3];
	this->safety.safety_alert_c  = data[4];
	this->safety.safety_status_c = data[5];
}

static void bq_update_permanent_fail_state_impl(AFE* this){
	BQ769x2 *p_bq = (BQ769x2*) this;
	int success = -1;
	uint8_t data[8];
	success = bq_i2c_read_reg_block(p_bq->hw, PF_ALERT_A, data, 8);
	if (success != 0) {
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	this->permanent_fail.pf_alert_a  = data[0];
	this->permanent_fail.pf_status_a = data[1];
	this->permanent_fail.pf_alert_b  = data[2];
	this->permanent_fail.pf_status_b = data[3];
	this->permanent_fail.pf_alert_c  = data[4];
	this->permanent_fail.pf_status_c = data[5];
	this->permanent_fail.pf_alert_d  = data[6];
	this->permanent_fail.pf_status_d = data[7];
}

static void bq_update_int_temperature_impl(AFE* this){
	BQ769x2 *p_bq = (BQ769x2*) this;
	int success = -1;
	int16_t temp_K;
	int16_t temp_C;
	success = bq_i2c_read_reg_word_with_sign(p_bq->hw, INT_TEMPERATURE, &temp_K);
	if (success != 0) {
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	temp_C =(temp_K/10 - 273);
	this->int_temperature = temp_C;
}

int16_t bq_read_ntc_temperature_TS1(BQ769x2* p_bq){
	int16_t temp_K=0;
	int32_t success=bq_i2c_read_reg_word_with_sign(p_bq->hw, TS1_TEMPERATURE, &temp_K);
	if(success!=0){
		afe_set_error((AFE*)p_bq,AFE_ERROR_COM_LINK);
		return 0;
	}

	return (temp_K/10 - 273); // 0 do C = 273.15 do K
}
int16_t bq_read_ntc_temperature_TS3(BQ769x2* p_bq){
	int16_t temp_K=0;
	int32_t success=bq_i2c_read_reg_word_with_sign(p_bq->hw, TS3_TEMPERATURE, &temp_K);
	if(success!=0){
		afe_set_error((AFE*)p_bq,AFE_ERROR_COM_LINK);
		return 0;
	}

	return (temp_K/10 - 273); // 0 do C = 273.15 do K
}
int16_t bq_read_ntc_temperature_HDQ_TEMP(BQ769x2* p_bq){
	int16_t temp_K=0;
	int32_t success=bq_i2c_read_reg_word_with_sign(p_bq->hw, HDQ_TEMPERATURE, &temp_K);
	if(success!=0){
		afe_set_error((AFE*)p_bq,AFE_ERROR_COM_LINK);
		return 0;
	}

	return (temp_K/10 - 273); // 0 do C = 273.15 do K
}
int16_t bq_read_ntc_temperature_CFETOFF_TEMP(BQ769x2* p_bq){
	int16_t temp_K=0;
	int32_t success=bq_i2c_read_reg_word_with_sign(p_bq->hw, CFETOFF_TEMPERATURE, &temp_K);
	if(success!=0){
		afe_set_error((AFE*)p_bq,AFE_ERROR_COM_LINK);
		return 0;
	}

	return (temp_K/10 - 273); // 0 do C = 273.15 do K
}

int16_t bq_read_cc2_current(BQ769x2* p_bq){
	int16_t current_userA;
	bq_i2c_read_reg_word_with_sign(p_bq->hw, CC2_CURRENT, &current_userA);
	return current_userA;
}

static void bq_config_TS3_pin(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte_verify(p_bq, TS3_CONFIG, data);
}
static void bq_config_CFETOFF_pin(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte_verify(p_bq, CFETOFF_PIN_CONFIG, data);
}
static void bq_config_HDQ_pin(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte_verify(p_bq, HDQ_PIN_CONFIG, data);
}

static void bq_config_ALERT_pin(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte(p_bq->hw, ALERT_PIN_CONFIG, data);
}

static void bq_config_default_alarm_mask(BQ769x2* p_bq, uint16_t data){
	bq_i2c_data_ram_write_word(p_bq->hw, DEFAULT_ALRAM_MASK, data);
}
static void bq_config_enable_protection_A(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte(p_bq->hw, ENABLED_PROTECTION_A, data);
}
static void bq_config_reg12(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte_verify(p_bq, REG12_CONFIG, data);
}

static void bq_config_reg0(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte_verify(p_bq, REG0_CONFIG, data);
}

static void bq_config_mfg_status_init(BQ769x2* p_bq, uint16_t data){
	bq_i2c_data_ram_write_word(p_bq->hw, MFG_STATUS_INIT, data);
}

void bq_enter_config_update_mode(BQ769x2* p_bq){
	uint16_t battery_status;
	bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, SET_CFGUPDATE);
	bq_i2c_read_reg_word(p_bq->hw, BATTERY_STATUS, &battery_status);
}

void bq_exit_config_update_mode(BQ769x2* p_bq){
	uint16_t battery_status;
	bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, EXIT_CFGUPDATE);
	bq_i2c_read_reg_word(p_bq->hw, BATTERY_STATUS, &battery_status);
}

static void bq_clear_alert_bit(BQ769x2* p_bq){
	bq_i2c_write_reg_word(p_bq->hw, ALARM_STATUS, 0xF800);
}

static void bq_config_da_configuration(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte(p_bq->hw, DA_CONFIGURATION, data);
}

static void bq_enable_host_control_fet(BQ769x2* p_bq){
	bq_i2c_write_reg_byte(p_bq->hw, WRITE_ADDR_REG, FET_ENABLE);
}

static void bq_config_fet_options_reg(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte_verify(p_bq, FET_OPTIONS, data);
}
static void bq_config_chg_pump_control_reg(BQ769x2* p_bq, uint8_t data){
	uint8_t chg_pump_control;
	bq_i2c_data_ram_read_byte(p_bq->hw, CHG_PUMP_CONTROL, &chg_pump_control);
	bq_i2c_data_ram_write_byte_verify(p_bq, CHG_PUMP_CONTROL, data);
}

static void bq_config_enable_protection_B(BQ769x2* p_bq, uint8_t data){
	bq_i2c_data_ram_write_byte(p_bq->hw, ENABLED_PROTECTION_B, data);
}

static void bq_set_cov_threshold_mV_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t cov_threshold;
	int success;
	cov_threshold = th/COV_CUV_THRESHOLD_UINT_mV;
	if((cov_threshold >= MIN_COV_THRESHOLD) && (cov_threshold <= MAX_COV_THRESHOLD))
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, COV_THRESHOLD, cov_threshold);
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}
	else
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, COV_THRESHOLD, DEFAULT_COV_THRESHOLD);
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}
}

static void bq_set_cuv_threshold_mV_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t cuv_threshold;
	int success;
	cuv_threshold = th/COV_CUV_THRESHOLD_UINT_mV;
	if((cuv_threshold >= MIN_CUV_THRESHOLD) && (cuv_threshold <= MAX_CUV_THRESHOLD))
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, CUV_THRESHOLD, cuv_threshold);
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}
	else
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, CUV_THRESHOLD, DEFAULT_CUV_THRESHOLD); //
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}

}
static uint32_t scd_threshold_mV[]= {10, 20, 40, 60, 80, 100, 125, 150, 175, 200, 250, 300, 350, 400, 450, 500};
static uint8_t bq_get_scd_from_threshold(uint32_t mA){
	uint32_t shunt_drop_mV = (mA*HAL_CURRENT_SENSE_R_SHUNT_mOhm)/1000;
	uint8_t index =0;
	if(shunt_drop_mV < scd_threshold_mV[0]){
		return 0;
	}
	while((shunt_drop_mV >= scd_threshold_mV[index]) &&(index <15)){
		index++;
	}
	return index - 1;
}
static void bq_set_scd_threshold_mA_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t scd_threshold;
	scd_threshold = bq_get_scd_from_threshold(th); // shunt_drop_mV
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, SCD_THRESHOLD, scd_threshold);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_occ_threshold_mA_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t occ_threshold;
	int success;
	occ_threshold = (th*HAL_CURRENT_SENSE_R_SHUNT_mOhm)/(1000*OCC_THRESHOLD_UNIT_mV);// shunt_drop_mV
	if((occ_threshold >= MIN_OCC_THRESHOLD) && (occ_threshold <= MAX_OCC_THRESHOLD))
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, OCC_THRESHOLD, occ_threshold);
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}
	else
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, OCC_THRESHOLD, DEFAULT_OCC_THRESHOLD);
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}

}
static void bq_set_ocd1_threshold_mA_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t ocd1_threshold;
	int success;
	ocd1_threshold = (th*HAL_CURRENT_SENSE_R_SHUNT_mOhm)/(1000*OCD_THRESHOLD_UINT_mV); // shunt_drop_mV
	if((ocd1_threshold >= MIN_OCD1_THRESHOLD) && (ocd1_threshold <= MAX_OCD1_THRESHOLD))
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, OCD1_THRESHOLD, ocd1_threshold);
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}
	else
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, OCD1_THRESHOLD, DEFAULT_OCD1_THRESHOLD);
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}

}

static void bq_set_ocd2_threshold_mA_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t ocd2_threshold;
	int success;
	ocd2_threshold = (th*HAL_CURRENT_SENSE_R_SHUNT_mOhm)/(1000*OCD_THRESHOLD_UINT_mV);// shunt_drop_mV
	if((ocd2_threshold >= MIN_OCD2_THREHSOLD) && (ocd2_threshold <= MAX_OCD2_THRESHOLD))
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, OCD2_THRESHOLD, ocd2_threshold);
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}
	else
	{
		success=bq_i2c_data_ram_write_byte_verify(p_bq, OCD2_THRESHOLD, DEFAULT_OCD2_THESHOLD);
		if(success!=0)
		{
			afe_set_error(this,AFE_ERROR_COM_LINK);
			return;
		}
	}

}
static void bq_set_cov_delay_ms_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint16_t cov_delay;
	cov_delay = (th-2*COV_CUV_DELAY_UNIT_ms)/COV_CUV_DELAY_UNIT_ms;
	int success=bq_i2c_data_ram_write_word_verify(p_bq, COV_DELAY, cov_delay);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_cuv_delay_ms_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint16_t cuv_delay;
	cuv_delay = (th-2*COV_CUV_DELAY_UNIT_ms)/COV_CUV_DELAY_UNIT_ms;
	int success=bq_i2c_data_ram_write_word_verify(p_bq, CUV_DELAY, cuv_delay);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_scd_delay_us_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t scd_delay;
	scd_delay = th/SCD_DEALY_UNIT_us;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, SCD_DELAY, scd_delay);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_occ_delay_ms_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t occ_delay;
	occ_delay = (th-2*OCC_DELAY_UNIT_ms)/OCC_DELAY_UNIT_ms;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, OCC_DELAY, occ_delay);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_ocd1_delay_ms_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t ocd1_delay;
	ocd1_delay = (th-2*OCD_DELAY_UNIT_ms)/OCD_DELAY_UNIT_ms;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, OCD1_DELAY, ocd1_delay);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_ocd2_delay_mA_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t ocd2_delay;
	ocd2_delay = (th-2*OCD_DELAY_UNIT_ms)/OCD_DELAY_UNIT_ms;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, OCD2_DELAY, ocd2_delay);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_cov_recovery_hysteresis_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t cov_hysteresis;
	cov_hysteresis = th/COV_CUV_RECOVERY_HYSTERESIS_mV;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, COV_RECOVERY_HYSTERESIS, cov_hysteresis);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_cuv_recovery_hysteresis_impl(AFE *this, uint32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t cuv_hysteresis;
	cuv_hysteresis = th/COV_CUV_RECOVERY_HYSTERESIS_mV;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, CUV_RECOVERY_HYSTERESIS, cuv_hysteresis);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_scd_recovery_time_s_impl(AFE *this, uint32_t s){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint8_t scd_time_recovery;
	scd_time_recovery = s;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, CUV_RECOVERY_HYSTERESIS, scd_time_recovery);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_occ_recovery_threshold_mA_impl(AFE *this, int32_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_word_with_sign(p_bq->hw, OCC_RECOVERY_THRESHOLD, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_ocd_recovery_hysteresis_mA_impl(AFE *this, int32_t th) {
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_word_with_sign(p_bq->hw, OCD_RECOVERY_THRESHOLD, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
/*
 * FET option default: 0x0D
 * Charge pump control default : enable 0x01
 */
void bq_turn_off_all_fet(BQ769x2* p_bq){
	int success=1;
	success =bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, ALL_FETS_OFF);
}
void bq_turn_on_all_fet(BQ769x2* p_bq){
	int success=1;
	success=bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, ALL_FETS_ON);
}

void bq_turn_off_fet_DSG_PDSG(BQ769x2* p_bq){
	bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, DSG_PDSG_OFF);
}
void bq_turn_off_fet_CHG_PCHG(BQ769x2* p_bq){
	bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, CHG_PCHG_OFF);
}

uint8_t bq_read_fet_status_reg(BQ769x2* p_bq){
	uint8_t status;
	bq_i2c_read_reg_byte(p_bq->hw, FET_STATUS, &status);
	return status;
}
void bq_config_body_diode_threshold(BQ769x2* p_bq, uint16_t threshold_mA){
	bq_i2c_data_ram_write_word_verify(p_bq, BODY_DIODE_THRESHOLD, threshold_mA);
}

static void bq_set_recovery_time(AFE* this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, RECOVERY_TIME, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_otc_threshold_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, OTC_THRESHOLD, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_otd_threshold_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, OTD_THRESHOLD, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_otf_threshold_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, OTF_THRESHOLD, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_otint_threshold_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, OTINT_THRESHOLD, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_utc_threshold_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, UTC_THRESHOLD, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_utd_threshold_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, UTC_THRESHOLD, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_utint_threshold_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte_verify(p_bq, UTINT_THRESHOLD, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_otc_delay_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, OTC_DELAY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_otd_delay_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, OTD_DELAY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_otf_delay_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, OTF_DELAY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_otint_delay_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, OTINT_DELAY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_utc_delay_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, UTC_DELAY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_utd_delay_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, UTD_DELAY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_utint_delay_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, UTINT_DELAY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_otc_recovery_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, OTC_RECOVERY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_otd_recovery_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, OTD_RECOVERY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_otf_recovery_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, OTF_RECOVERY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_otint_recovery_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, OTINT_RECOVERY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_utc_recovery_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, UTC_RECOVERY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_utd_recovery_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, UTD_RECOVERY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_set_utint_recovery_impl(AFE *this, uint8_t th){
	BQ769x2* p_bq=(BQ769x2*)this;
	int success=bq_i2c_data_ram_write_byte(p_bq->hw, UTINT_RECOVERY, th);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}
static void bq_enable_sleep_mode_impl(AFE *this){
	BQ769x2* p_bq=(BQ769x2*)this;
	int32_t success=bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, SLEEP_ENABLE);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_disable_sleep_mode_impl(AFE *this){
	BQ769x2* p_bq=(BQ769x2*)this;
	int32_t success=bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, SLEEP_DISABLE);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_sleep_hysteresis_time_s(BQ769x2* p_bq, uint8_t time){
	bq_i2c_data_ram_write_byte(p_bq->hw, SLEEP_HYSTERESIS_TIME, time);
}

static void bq_set_sleep_current_mA(BQ769x2* p_bq, uint16_t current_mA){
	bq_i2c_data_ram_write_word(p_bq->hw, SLEEP_CURRENT, current_mA);
}

static void bq_set_sleep_voltage_time_s(BQ769x2* p_bq, uint8_t voltage_time){
	bq_i2c_data_ram_write_byte(p_bq->hw, VOLTAGE_TIME, voltage_time);
}

static void bq_set_sleep_wake_comparator_current_mA(BQ769x2* p_bq, uint16_t current){
	bq_i2c_data_ram_write_word(p_bq->hw, WAKE_COMPARATOR_CURRENT, current);
}

static void bq_set_sleep_charge_pack_tos_delta_mV(BQ769x2* p_bq, uint16_t delta){
	bq_i2c_data_ram_write_word(p_bq->hw, SLEEP_CHARGER_PACK_TOS_DELTA, delta);
}

static void bq_set_sleep_charge_voltage_threshold(BQ769x2* p_bq, uint16_t threshold){
	bq_i2c_data_ram_write_word(p_bq->hw, SLEEP_CHARGER_VOLTAGE_THRESHOLD, threshold);
}

static void bq_enable_deep_sleep_mode_impl(AFE *this){
	BQ769x2* p_bq=(BQ769x2*)this;
	int32_t success;
	success=bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, DEEPSLEEP);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
	success=bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, DEEPSLEEP);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

}
static void bq_disable_deep_sleep_mode_impl(AFE *this){
	BQ769x2* p_bq=(BQ769x2*)this;
	int32_t success=bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, EXIT_DEEPSLEEP);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static int bq_set_bit_power_config_reg(BQ769x2* p_bq, uint16_t bit_mask){
	int success = 0;
	uint16_t control_reg;
	success = bq_i2c_data_ram_read_word(p_bq->hw, POWER_CONFIG, &control_reg);
	if (success != 0) {
		return success;
	}
	control_reg |= bit_mask;
	success = bq_i2c_data_ram_write_word(p_bq->hw, POWER_CONFIG, control_reg);
	return success;
}

static void bq_enable_shutdown_mode_impl(AFE *this){
	BQ769x2* p_bq=(BQ769x2*)this;
	int32_t success;
	success=bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, SHUTDOWN);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
	success=bq_i2c_write_reg_word(p_bq->hw, WRITE_ADDR_REG, SHUTDOWN);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_shutdown_stack_voltage_mV(BQ769x2* p_bq, uint16_t threshold){
	uint16_t reg_data;
	reg_data = threshold/STACK_VOLTAGE_UINT_mV;
	bq_i2c_data_ram_write_word(p_bq->hw, SHUTDOWN_STACK_VOLTAGE, reg_data);
}

static void bq_set_low_v_shutdown_delay_s(BQ769x2* p_bq, uint8_t time){
	bq_i2c_data_ram_write_byte(p_bq->hw, SHUTDOWN_STACK_VOLTAGE, time);
}
static void bq_set_shutdown_fet_off_dealy_ms(BQ769x2* p_bq, uint8_t time){
	uint8_t reg_data;
	reg_data = time/SHUTDOWN_DELAY_ms;
	bq_i2c_data_ram_write_byte(p_bq->hw, FET_OFF_DELAY, reg_data);
}

static void bq_set_shutdown_command_delay_ms(BQ769x2* p_bq, uint8_t time){
	uint8_t reg_data;
	reg_data = time/SHUTDOWN_DELAY_ms;
	bq_i2c_data_ram_write_byte(p_bq->hw, SHUT_DOWN_COMMAND_DELAY, reg_data);
}

static void bq_set_auto_shutdown_time_min(BQ769x2* p_bq, uint8_t time){
	bq_i2c_data_ram_write_byte(p_bq->hw, AUTO_SHUTDOWN_TIME, time);
}

static void bq_set_shutdown_temperature_threshold(BQ769x2* p_bq, uint8_t threshold){
	bq_i2c_data_ram_write_byte(p_bq->hw, SHUTDOWN_TEMPERATURE, threshold);
}

static void bq_set_shutdown_temperature_delay_ms(BQ769x2* p_bq, uint8_t time){
	bq_i2c_data_ram_write_byte(p_bq->hw, SHUTDOWN_TEMPERATURE_DELAY, time);
}

static void bq_set_cell_balance_max_cells(BQ769x2* p_bq, uint8_t number){
	bq_i2c_data_ram_write_byte(p_bq->hw, CELL_BALANCE_MAX_CELLS, number);
}

static void bq_set_level_voltage_threshold(BQ769x2* p_bq, uint16_t threshold){
	bq_i2c_subcommand_write_word(p_bq->hw, CB_SET_LVL, threshold);
}

static void bq_get_time_balancing_active(BQ769x2* p_bq, uint16_t data){
	bq_i2c_subcommand_read_word(p_bq->hw, CBSTATUS1, &data);
}

static void bq_set_min_cell_temp(BQ769x2* p_bq, uint8_t min_temp){
	bq_i2c_data_ram_write_byte(p_bq->hw, MIN_CELL_TEMP, min_temp);
}

static void bq_set_max_cell_temp(BQ769x2* p_bq, uint8_t max_temp){
	bq_i2c_data_ram_write_byte(p_bq->hw, MAX_CELL_TEMP, max_temp);
}

static void bq_set_max_internal_temp(BQ769x2* p_bq, uint8_t max_temp){
	bq_i2c_data_ram_write_byte(p_bq->hw, MAX_INTERNAL_TEMP, max_temp);
}
static void bq_set_cell_balance_interval(BQ769x2* p_bq, uint8_t time_s){
	bq_i2c_data_ram_write_byte(p_bq->hw, CELL_BALANCE_INTERVAL, time_s);
}

// cell balancing aglorithm
static bool cell_need_balancing(const uint32_t target_voltage, const uint32_t cell_vol){
	if((cell_vol > (target_voltage + BALANCING_STOP_DELTA)) && (cell_vol > BALANCING_VOLTAGE_MIN_THRESHOLD) )
		return true;
	return false;
}

static uint16_t cell_array_get_group_max_vol(const Cell_Bank* const p_cells ,const uint8_t len,
		const uint16_t ignore_mask,uint32_t* vol){

	uint32_t max=0;
	uint8_t index=0;
	for(int i=0;i<len;i++){
		if(!(ignore_mask & (1<<i))){
			if(p_cells[i].voltage>max){
				max=p_cells[i].voltage;
				index=i;
			}
		}
	}
	*vol=max;
	return index;
}

static void bq_calculate_balancing_mask(Cell_Bank* cells,uint16_t* buff,const uint32_t target_vol){

	uint16_t max_cell_index=0;
	uint32_t max_vol=0;
	uint16_t checked_mask=0; /* need only 5 last bits for max 5 cell in a group */
	uint16_t adjacent_mask=0;
	uint16_t max_cell_mask=0;
	/* reset mask */
	*buff=0;

	while(checked_mask !=0xFFFF){
		max_cell_index=cell_array_get_group_max_vol(cells, 16,checked_mask,&max_vol);
		if(cell_need_balancing(target_vol, max_vol)){
			max_cell_mask=(1<<max_cell_index);
			*buff|= max_cell_mask;
			/* mask 2 adjacent as checked because bq don't allow to balancing 2 adjacent cells */
			adjacent_mask |= max_cell_mask + (max_cell_mask<<1) + (max_cell_mask>>1);

			checked_mask|= adjacent_mask;

		}else{
			/* if event cell with max voltage don't need balancing so other cell too */
			return;
		}

	}
}

static void bq_host_disable_cell_balancing_impl(AFE *this){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint16_t reg_data =0x00;
	uint16_t verify =0;
	int success=bq_i2c_subcommand_write_word(p_bq->hw, CB_ACTIVE_CELLS, reg_data);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
	success = bq_i2c_subcommand_read_word(p_bq->hw, CB_ACTIVE_CELLS, &verify);
	this->balancing_mask = verify;
}

static void bq_host_enable_cell_balancing_impl(AFE *this, uint32_t voltage_target){
	BQ769x2* p_bq=(BQ769x2*)this;
	uint16_t balancing_mask =0;
	uint16_t verify =0;
	bq_calculate_balancing_mask(this->cell_array->cells, &balancing_mask, voltage_target);
	int success=bq_i2c_subcommand_write_word(p_bq->hw, CB_ACTIVE_CELLS, balancing_mask);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
	success = bq_i2c_subcommand_read_word(p_bq->hw, CB_ACTIVE_CELLS, &verify);
	this->balancing_mask = verify;
}

static void bq_get_calibration_cell_voltage(BQ769x2* p_bq, int16_t* cell_gain){
	int32_t state;
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_1_GAIN, cell_gain);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_2_GAIN, cell_gain+1);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_3_GAIN, cell_gain+2);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_4_GAIN, cell_gain+3);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_5_GAIN, cell_gain+4);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_6_GAIN, cell_gain+5);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_7_GAIN, cell_gain+6);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_8_GAIN, cell_gain+7);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_9_GAIN, cell_gain+8);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_10_GAIN, cell_gain+9);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_11_GAIN, cell_gain+10);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_12_GAIN, cell_gain+11);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_13_GAIN, cell_gain+12);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_14_GAIN, cell_gain+13);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_15_GAIN, cell_gain+14);
	state = bq_i2c_data_ram_read_word_with_sign(p_bq->hw, CELL_16_GAIN, cell_gain+15);
}

static void bq_get_cc_gain(BQ769x2* p_bq, float cc_gain){
	bq_i2c_data_ram_read_reg_with_float(&bq_hw, CC_GAIN, &cc_gain);
}

