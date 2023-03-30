/*
 * test_switch.c
 *
 *  Created on: May 18, 2021
 *      Author: Admin
 */
#include "delay_hw.h"
#include "core_hal.h"
#include "bq_hal.h"
#include "bq769x2.h"
#include "key_init.h"
#include "app_config.h"
#include "inrush_limiter_hal.h"

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
#define BODY_DIOED_THRESH_mA			50
#define STACK_VOLTAGE_UINT_mV			10
#define SHUTDOWN_DELAY_ms				0.25f
#define BALANCING_STOP_DELTA			30
#define BALANCING_VOLTAGE_MIN_THRESHOLD 3500

enum BP_STATE{
	IDLE = 0,
	SOFT_START =1,
	OPERATION = 2
};

typedef enum BP_STATE BP_state;
static void turn_on_all_fet(BQ_Hw* p_hw);
static void turn_off_all_fet(BQ_Hw* p_hw);
BP_state bp_state;
static void bp_set_state(BQ_Hw* p_hw, const BP_state state){
	switch(state)
	{
	case IDLE:
		turn_off_all_fet(p_hw);
		break;
	case SOFT_START:
		inrush_limiter_switch_on(); //on inrush current pin
		break;
	case OPERATION:
		turn_on_all_fet(p_hw);
		inrush_limiter_switch_off();
		break;
	default:
		break;
	}
}

static void test_setup(void){
	global_interrupt_disable();
	core_hw_init();
	afe_hardware_init_1();
	user_key_init();
	key_hw_init();
	inrush_limter_hardware_init();
	global_interrupt_enable();
}

static void config_write_data_ram(BQ769x2* p_bq){
	int32_t state;
	bq_enter_config_update_mode(p_bq);
	state = bq_i2c_data_ram_write_word(p_bq->hw, MFG_STATUS_INIT, 0x0050);
	//state = bq_i2c_data_ram_write_word(p_bq->hw, VCELL_MODE, 0xFFFF);
	state = bq_i2c_data_ram_write_byte(p_bq->hw, ALERT_PIN_CONFIG, 0x2A);
	state = bq_i2c_data_ram_write_byte(p_bq->hw, ENABLED_PROTECTION_A, 0xFC);	 // enable SCD, OCC, OCD1, OCD2 , CUV, COV protection
	state = bq_i2c_data_ram_write_byte(p_bq->hw, FET_OPTIONS, 0x0F); // enable CHG FET on SLEEP Mode
	state = bq_config_da_configuration(p_bq, 0x06);     // configure user-volt:10mV,user-ampe:10mA
    state = bq_config_TS3_pin(p_bq, 0x07);
	state = bq_config_CFETOFF_pin(p_bq, 0x07); 			// configure CFETOFF as temperature sensor pin, use for cell protection
    state = bq_config_HDQ_pin(p_bq, 0x0F);				// configure HDQ as temperature sensor pin, use for fet protection
	bq_exit_config_update_mode(p_bq);
}

static void test_protection(BQ_Hw* p_hw){
	int32_t state;
	// setting cuv
	uint32_t cuv_th =3050;
	uint32_t cuv_delay_th =250;
	uint8_t cuv_threshold;
	uint16_t cuv_delay;
	uint8_t cuv_threshold_level;
	uint8_t cuv_delay_level;
	cuv_threshold = cuv_th/COV_CUV_THRESHOLD_UINT_mV;
	int success=bq_i2c_data_ram_write_byte(p_hw, CUV_THRESHOLD, cuv_threshold);
	if(success ==0)
	{
		state = bq_i2c_data_ram_read_byte(p_hw, CUV_THRESHOLD, &cuv_threshold_level);
	}
	cuv_delay = (cuv_delay_th-2*COV_CUV_DELAY_UNIT_ms)/COV_CUV_DELAY_UNIT_ms;
	success=bq_i2c_data_ram_write_word(p_hw, CUV_DELAY, cuv_delay);
	if(success == 0){
		state = bq_i2c_data_ram_read_byte(p_hw, CUV_DELAY, &cuv_delay_level);
	}
	// setting cov
	uint32_t cov_th =4050;
	uint32_t cov_delay_th =250;
	uint8_t cov_threshold;
	uint16_t cov_delay;
	uint8_t cov_threshold_level;
	uint8_t cov_delay_level;
	cov_threshold = cov_th/COV_CUV_THRESHOLD_UINT_mV;
	success=bq_i2c_data_ram_write_byte(p_hw, COV_THRESHOLD, cov_threshold);
	if(success ==0)
	{
		state = bq_i2c_data_ram_read_byte(p_hw, COV_THRESHOLD, &cov_threshold_level);
	}
	//cov_delay = (cov_delay_th-2*COV_CUV_DELAY_UNIT_ms)/COV_CUV_DELAY_UNIT_ms;
	//success=bq_i2c_data_ram_write_word(p_hw, CUV_DELAY, cov_delay);
	if(success == 0){
		state = bq_i2c_data_ram_read_byte(p_hw, COV_DELAY, &cov_delay_level);
	}
	// setting scd
	uint32_t scd_th =4;
	uint32_t scd_delay_th =1;
	uint8_t scd_threshold;
	uint8_t scd_threshold_level;
	uint8_t scd_delay_level;
	scd_threshold = scd_th;
	success=bq_i2c_data_ram_write_byte(p_hw, SCD_THRESHOLD, scd_threshold);
	if(success ==0)
	{
		state = bq_i2c_data_ram_read_byte(p_hw, SCD_THRESHOLD, &scd_threshold_level);
	}
	success=bq_i2c_data_ram_write_word(p_hw, SCD_DELAY, scd_delay_th);
	if(success == 0){
		state = bq_i2c_data_ram_read_byte(p_hw, SCD_DELAY, &scd_delay_level);
	}
	// setting occ
	uint8_t occ_threshold;
	uint8_t occ_th =12;
	uint8_t occ_threshold_level;
	uint8_t occ_delay_th = 20;
	uint8_t occ_delay;
	uint8_t occ_delay_level;
	occ_threshold = occ_th/OCC_THRESHOLD_UNIT_mV;
	success=bq_i2c_data_ram_write_byte(&bq_hw, OCC_THRESHOLD, occ_threshold);
	if(success == 0){
		state = bq_i2c_data_ram_read_byte(p_hw, OCC_THRESHOLD, &occ_threshold_level);
	}
	occ_delay = (occ_delay_th-2*OCC_DELAY_UNIT_ms)/OCC_DELAY_UNIT_ms;
	success=bq_i2c_data_ram_write_byte(p_hw, OCC_DELAY, occ_delay);
	if(success == 0){
		state = bq_i2c_data_ram_read_byte(p_hw, OCC_DELAY, &occ_delay_level);
	}
	// setting ocd1
	uint8_t ocd1_th =50;
	uint8_t ocd1_threshold;
	uint8_t ocd1_delay;
	uint8_t ocd1_delay_th = 10;
	uint8_t ocd1_threshold_level;
	uint8_t ocd1_delay_level;
	ocd1_threshold = ocd1_th/OCD_THRESHOLD_UINT_mV;
	success=bq_i2c_data_ram_write_byte(p_hw, OCD1_THRESHOLD, ocd1_threshold);
	if(success == 0){
		state = bq_i2c_data_ram_read_byte(p_hw, OCD1_THRESHOLD, &ocd1_threshold_level);
	}
	ocd1_delay = (ocd1_delay_th-2*OCD_DELAY_UNIT_ms)/OCC_DELAY_UNIT_ms;
	success=bq_i2c_data_ram_write_byte(p_hw, OCD1_DELAY, ocd1_delay);
	if(success == 0){
		state = bq_i2c_data_ram_read_byte(p_hw, OCD1_DELAY, &ocd1_delay_level);
	}
	// setting ocd2
	uint8_t ocd2_th =52;
	uint8_t ocd2_threshold;
	uint8_t ocd2_delay;
	uint8_t ocd2_delay_th = 15;
	uint8_t ocd2_threshold_level;
	uint8_t ocd2_delay_level;
	ocd2_threshold = ocd2_th/OCD_THRESHOLD_UINT_mV;
	success=bq_i2c_data_ram_write_byte(p_hw, OCD2_THRESHOLD, ocd2_threshold);
	if(success == 0){
		state = bq_i2c_data_ram_read_byte(p_hw, OCD2_THRESHOLD, &ocd2_threshold_level);
	}
	ocd2_delay = (ocd2_delay_th-2*OCD_DELAY_UNIT_ms)/OCC_DELAY_UNIT_ms;
	success=bq_i2c_data_ram_write_byte(p_hw, OCD2_DELAY, ocd2_delay);
	if(success == 0){
		state = bq_i2c_data_ram_read_byte(p_hw, OCD2_DELAY, &ocd2_delay_level);
	}
}

static void test_read_protect_status(BQ_Hw* p_hw){
		int32_t state;
		uint16_t battery_status =0;
		uint8_t safety_alert[3]={0};
		uint8_t safety_status[3]={0};
		uint8_t pf_alert[4]={0};
		uint8_t pf_status[4]={0};
		uint8_t fet_status=0;
		uint8_t alarm_status[3]={0};

		state = bq_i2c_read_reg_word(p_hw, BATTERY_STATUS, &battery_status);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_ALERT_A, &safety_alert[0]);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_STATUS_A, &safety_status[0]);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_ALERT_B, &safety_alert[1]);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_STATUS_B, &safety_status[1]);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_ALERT_C, &safety_alert[2]);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_STATUS_C, &safety_status[2]);
		state = bq_i2c_read_reg_byte(p_hw, PF_ALERT_A, &pf_alert[0]);
		state = bq_i2c_read_reg_byte(p_hw, PF_STATUS_A, &pf_status[0]);
		state = bq_i2c_read_reg_byte(p_hw, PF_ALERT_B, &pf_alert[1]);
		state = bq_i2c_read_reg_byte(p_hw, PF_STATUS_B, &pf_status[1]);
		state = bq_i2c_read_reg_byte(p_hw, PF_ALERT_C, &pf_alert[2]);
		state = bq_i2c_read_reg_byte(p_hw, PF_STATUS_C, &pf_status[2]);
		state = bq_i2c_read_reg_byte(p_hw, PF_ALERT_D, &pf_alert[3]);
		state = bq_i2c_read_reg_byte(p_hw, PF_STATUS_D, &pf_status[3]);
		state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
		state = bq_i2c_read_reg_byte(p_hw, ALARM_STATUS, &alarm_status[0]);
		state = bq_i2c_read_reg_byte(p_hw, ALARM_RAW_STATUS, &alarm_status[1]);
		state = bq_i2c_read_reg_byte(p_hw, ALARM_ENABLE, &alarm_status[2]);
		hw_delay_ms(10);
}
static void test_on_off_fet(BQ_Hw* p_hw){
	int32_t state=1;
	uint8_t fet_status;
	uint8_t fet_options;
	state = bq_i2c_data_ram_read_byte(p_hw, FET_OPTIONS, &fet_options);
	state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, ALL_FETS_ON);
	hw_delay_ms(10);
	state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
	hw_delay_ms(300);
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, ALL_FETS_OFF);
	hw_delay_ms(10);
	state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
	hw_delay_ms(300);
}

static void turn_on_all_fet(BQ_Hw* p_hw){
	int32_t state =1;
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, ALL_FETS_ON);
}

static void turn_off_all_fet(BQ_Hw* p_hw){
	uint8_t fet_status;
	int32_t state =1;
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, ALL_FETS_OFF);
	hw_delay_ms(10);
	state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
}
int main(void)
{
	int state;
	uint16_t battery_status;
	uint8_t scd_threshold;
	uint8_t enable_protetion;
	hw_delay_ms(100);
	test_setup();
	config_write_data_ram(&bq_hw);
	bp_set_state(&bq_hw, IDLE);
//	test_protection(&bq_hw);
	while(1){
	//	test_on_off_fet(&bq_hw);

	}
	return 0;
}

void SysTick_Handler(void){
	uint8_t current_state = 2 ;
	static volatile uint32_t press_is_hold;
	current_state = key_read();
	switch(bp_state){
	case IDLE:
		   if(current_state == 0){
			   press_is_hold += APP_STATE_MACHINE_UPDATE_TICK_mS;
			   if(press_is_hold == 1000){
		       bp_set_state(&bq_hw, SOFT_START);
		       bp_state = SOFT_START;
		       press_is_hold = 0;
			   }
		   }
		   break;
	case SOFT_START:
			hw_delay_ms(1000);
			bp_set_state(&bq_hw, OPERATION);
		    bp_state = OPERATION;
			break;
	case OPERATION:
		   if(current_state == 0){
			   press_is_hold += APP_STATE_MACHINE_UPDATE_TICK_mS;
			   if(press_is_hold == 1000){
		       bp_set_state(&bq_hw, IDLE);
		       bp_state = IDLE;
		       press_is_hold = 0;
			   }
		   }
		   break;
	}
}

void TIM7_IRQHandler(void) {
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) {
	}
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
 }

