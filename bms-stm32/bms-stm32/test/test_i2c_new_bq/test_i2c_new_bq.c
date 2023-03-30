/*
 * test_i2c_new_bq.c
 *
 *  Created on: May 6, 2021
 *      Author: Admin
 */


#include "debug_com_port_hal.h"
#include "delay_hw.h"
#include "core_hal.h"
#include "bq_hal.h"
#include "bq769x2.h"
#include "inrush_limiter_hal.h"

static void bq_i2c_init_100kHz(void);
static void test_setup(void){
	core_hw_init();
	afe_hardware_init_1();
}

//helper function
static void test_config(BQ_Hw* p_hw);
static void config_write_data_ram(BQ_Hw* p_hw);
static void test_bq_i2c_link(BQ_Hw* p_hw);
static void test_read_voltage_current(BQ_Hw* p_hw, int16_t* voltage_cell_pin);
static void test_read_temperature(BQ_Hw* p_hw);
static void test_read_bq_status(BQ_Hw* p_hw);
static void test_comm_type(BQ_Hw* p_hw);
static void test_read_fw_verison(BQ_Hw* p_hw);
static void test_config_reg1(BQ_Hw* p_hw);
static void test_on_off_fet(BQ_Hw* p_hw);
static void test_soft_start(BQ_Hw* p_hw);
static void test_read_balancing_bitmask(BQ_Hw* p_hw);
static void read_calib_cell_volt(BQ_Hw* p_hw, int16_t* cell_gain);

static void config_write_data_ram(BQ_Hw* p_hw){
	int32_t state;
	uint16_t battery_status;
	uint16_t cell_16_gain;
	uint16_t cell_16_voltage;
	bq_i2c_subcommand_write_word(p_hw, WRITE_ADDR_REG, SET_CFGUPDATE);
	hw_delay_ms(2);
	state = bq_i2c_read_reg_word(p_hw, BATTERY_STATUS, &battery_status);
	state = bq_i2c_data_ram_read_word(&bq_hw, CELL_16_GAIN, &cell_16_gain);
	//state = bq_i2c_data_ram_write_word(&bq_hw, CELL_16_GAIN, 12409);
	bq_i2c_subcommand_write_word(p_hw, WRITE_ADDR_REG, EXIT_CFGUPDATE);
	state = bq_i2c_data_ram_read_word(&bq_hw, CELL_16_GAIN, &cell_16_gain);
	state = bq_i2c_read_reg_word(&bq_hw, CELL_16_VOLTAGE, &cell_16_voltage);
	hw_delay_ms(1);
}

static void test_bq_i2c_link(BQ_Hw* p_hw){
		int32_t state=0;
		uint16_t data_voltage=0;
		uint16_t data_subcommand=0;
		uint8_t data_read_data_ram=0;
		uint8_t write_data=0x8C;
		uint8_t data_read;
        // test read write with direct command
		state =	bq_i2c_read_reg_word(p_hw, 0x14 , &data_voltage);

		state =	bq_i2c_write_reg_word(p_hw, 0x66, 0xF082);
		//test read write with sub command
		state = bq_i2c_subcommand_read_word(p_hw, 0x0001, &data_subcommand);// read device number

		state = bq_i2c_write_reg_word(p_hw, 0x3E, 0x0022); // write fet enable

		//test read write with data ram

		state = bq_i2c_data_ram_read_byte(p_hw, 0x9261, &data_read); // read data ram 0x9261: Enable protection A

		state= bq_i2c_data_ram_write_byte(p_hw, 0x9261, 0x8C); //write 0x8C to 0x9261:Enable protection A

		state= bq_i2c_data_ram_write_word(p_hw, CUV_DELAY, 3);

}
static void test_read_voltage_current(BQ_Hw* p_hw, int16_t* voltage_cell_pin){
		uint32_t i = 0;
		int32_t state;
		uint32_t block_size = 2*16; //16 cells, each cell need 2 byte for voltage information
		uint8_t cell_voltage[32];
		uint16_t voltage[16];
		int16_t stack_voltage;
		int16_t pack_pin_voltage;
		int16_t ld_pin_voltage;
		int32_t success =1;
		int16_t cc2_current;
		uint8_t da_config;

		success = bq_i2c_read_reg_block(p_hw, CELL_1_VOLTAGE, cell_voltage,
				block_size);
		for(i =0; i <16; i++)
		{
			voltage[i] = cell_voltage[2*i+1];
			voltage[i] = (voltage[i] <<8) + cell_voltage[2*i];
		}
		uint16_t vcell_offset =11;
		state = bq_i2c_data_ram_read_word(p_hw, VCELL_OFFSET, &vcell_offset);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_1_VOLTAGE, voltage_cell_pin);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_2_VOLTAGE, voltage_cell_pin+1);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_3_VOLTAGE, voltage_cell_pin+2);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_4_VOLTAGE, voltage_cell_pin+3);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_5_VOLTAGE, voltage_cell_pin+4);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_6_VOLTAGE, voltage_cell_pin+5);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_7_VOLTAGE, voltage_cell_pin+6);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_8_VOLTAGE, voltage_cell_pin+7);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_9_VOLTAGE, voltage_cell_pin+8);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_10_VOLTAGE, voltage_cell_pin+9);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_11_VOLTAGE, voltage_cell_pin+10);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_12_VOLTAGE, voltage_cell_pin+11);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_13_VOLTAGE, voltage_cell_pin+12);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_14_VOLTAGE, voltage_cell_pin+13);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_15_VOLTAGE, voltage_cell_pin+14);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CELL_16_VOLTAGE, voltage_cell_pin+15);
		state = bq_i2c_read_reg_word_with_sign(p_hw, STACK_VOLTAGE, &stack_voltage);
		state = bq_i2c_read_reg_word_with_sign(p_hw, PACK_PIN_VOLTAGE, &pack_pin_voltage);

		int sum=0;
		for(i =0; i <16; i++){
			sum = sum+voltage_cell_pin[i];
		}
		state = bq_i2c_read_reg_word_with_sign(p_hw, LD_PIN_VOLTAGE, &ld_pin_voltage);
		state = bq_i2c_data_ram_read_byte(p_hw, DA_CONFIGURATION, &da_config);
		state = bq_i2c_data_ram_write_byte(p_hw, DA_CONFIGURATION, 0x06);
		state = bq_i2c_data_ram_read_byte(p_hw, DA_CONFIGURATION, &da_config);
		state = bq_i2c_read_reg_word_with_sign(p_hw, CC2_CURRENT, &cc2_current);
		hw_delay_ms(10);
}
static void test_read_temperature(BQ_Hw* p_hw){
		int32_t state;
		uint16_t int_temperature;
		uint16_t temperature_die;
		uint16_t ts1_temperature_K;
		uint16_t ts1_temperature_C;
		uint8_t cfet_off =0x0B;
		uint16_t cfet_off_temperature_K;
		uint16_t cfet_off_temperature_C;
		state = bq_i2c_read_reg_word(&bq_hw, INT_TEMPERATURE, &int_temperature);
		temperature_die = int_temperature/10 - 273;
		state = bq_i2c_read_reg_word(&bq_hw, TS1_TEMPERATURE, &ts1_temperature_K);
		ts1_temperature_C = ts1_temperature_K/10 - 273;
		state = bq_i2c_data_ram_write_byte(&bq_hw, CFETOFF_PIN_CONFIG, 0x0B);
		state = bq_i2c_read_reg_word(&bq_hw, CFETOFF_TEMPERATURE, &cfet_off_temperature_K);
		cfet_off_temperature_C = cfet_off_temperature_K/10 - 273;
		hw_delay_ms(10);
}
static void test_read_bq_status(BQ_Hw* p_hw){
		int32_t state;
		uint16_t battery_status =0;
		uint8_t safety_alert_a;
		uint8_t safety_alert_b;
		uint8_t safety_alert_c;
		uint8_t safety_status_a;
		uint8_t safety_status_b;
		uint8_t safety_status_c;
		uint8_t pf_alert_a;
		uint8_t pf_alert_b;
		uint8_t pf_alert_c;
		uint8_t pf_alert_d;
		uint8_t pf_status_a;
		uint8_t pf_status_b;
		uint8_t pf_status_c;
		uint8_t pf_status_d;
		uint8_t fet_status=0;
		uint8_t alarm_status;
		uint8_t alarm_raw_status;
		uint8_t alarm_enable;

		state = bq_i2c_read_reg_word(p_hw, BATTERY_STATUS, &battery_status);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_ALERT_A, &safety_alert_a);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_STATUS_A, &safety_status_a);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_ALERT_B, &safety_alert_b);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_STATUS_B, &safety_status_b);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_ALERT_C, &safety_alert_c);
		state = bq_i2c_read_reg_byte(p_hw, SAFETY_STATUS_C, &safety_status_c);
		state = bq_i2c_read_reg_byte(p_hw, PF_ALERT_A, &pf_alert_a);
		state = bq_i2c_read_reg_byte(p_hw, PF_STATUS_A, &pf_status_a);
		state = bq_i2c_read_reg_byte(p_hw, PF_ALERT_B, &pf_alert_b);
		state = bq_i2c_read_reg_byte(p_hw, PF_STATUS_B, &pf_status_b);
		state = bq_i2c_read_reg_byte(p_hw, PF_ALERT_C, &pf_alert_c);
		state = bq_i2c_read_reg_byte(p_hw, PF_STATUS_C, &pf_status_c);
		state = bq_i2c_read_reg_byte(p_hw, PF_ALERT_D, &pf_alert_d);
		state = bq_i2c_read_reg_byte(p_hw, PF_STATUS_D, &pf_status_d);
		state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
		state = bq_i2c_read_reg_byte(p_hw, ALARM_STATUS, &alarm_status);
		state = bq_i2c_read_reg_byte(p_hw, ALARM_RAW_STATUS, &alarm_raw_status);
		state = bq_i2c_read_reg_byte(p_hw, ALARM_ENABLE, &alarm_enable);
		hw_delay_ms(10);
}

static void test_comm_type(BQ_Hw* p_hw){
		int32_t state;
		uint8_t comm_type =0;
		uint16_t data_subcommand=1;
		state = bq_i2c_data_ram_read_byte(p_hw, COMM_TYPE, &comm_type);
		state = bq_i2c_data_ram_write_byte(p_hw, COMM_TYPE, 0x07); //set i2c 100kz without crc
		bq_i2c_init_100kHz();
		state = bq_i2c_subcommand_read_word(p_hw, DEVICE_NUMBER, &data_subcommand);
		state = bq_i2c_data_ram_read_byte(p_hw, COMM_TYPE, &comm_type);
		hw_delay_ms(10);
}

static void test_read_fw_verison(BQ_Hw* p_hw){
		int32_t state;
		uint8_t fw_version[6]={0};
		uint16_t device_number;
		state = bq_i2c_subcommand_read_word(p_hw, DEVICE_NUMBER, &device_number);// read device number
		state = bq_i2c_subcommand_read_block(p_hw, FW_VERSION, fw_version, 6);
		hw_delay_ms(10);

}

static void test_config(BQ_Hw* p_hw){
		int32_t state;
		uint16_t vcell_mode;
		uint8_t enable_protection_a =0;
		uint8_t enable_protection_b=0;
		uint8_t write_reg =0xFC;
		uint16_t default_alarm_mask;
		state = bq_i2c_data_ram_write_word(p_hw, VCELL_MODE, 0xFFFF);
		state = bq_i2c_data_ram_read_word(p_hw, VCELL_MODE, &vcell_mode);

		state = bq_i2c_data_ram_read_byte(p_hw, ENABLED_PROTECTION_A
				, &enable_protection_a);
	//	state = bq_i2c_data_ram_write(p_hw, ENABLED_PROTECTION_A, &write_reg, 1);
		state = bq_i2c_data_ram_write_byte(p_hw, ENABLED_PROTECTION_A, 0x88);
		state = bq_i2c_data_ram_read_byte(p_hw, ENABLED_PROTECTION_A
				, &enable_protection_a);
		state = bq_i2c_data_ram_read_word(p_hw, DEFAULT_ALRAM_MASK, &default_alarm_mask);
		hw_delay_ms(10);
}

static void test_config_reg1(BQ_Hw* p_hw){
	int32_t state;
	uint8_t reg12_config;
	uint8_t reg0_config;
	state = bq_i2c_data_ram_write_byte(p_hw, REG0_CONFIG, 0x01);
	state = bq_i2c_data_ram_read_byte(p_hw, REG0_CONFIG, &reg0_config);
	state = bq_i2c_data_ram_read_byte(p_hw, REG12_CONFIG, &reg12_config);
	state = bq_i2c_data_ram_write_byte(p_hw, REG12_CONFIG, 0x9D);
	state = bq_i2c_data_ram_read_byte(p_hw, REG12_CONFIG, &reg12_config);
	//state = bq_i2c_subcommand_write_byte(p_hw, REG12_CONTROL, 0x0D);

}

static void test_on_off_fet(BQ_Hw* p_hw){
	int32_t state;
	uint8_t fet_status;
	uint16_t battery_status;

	state = bq_i2c_read_reg_word(p_hw, BATTERY_STATUS, &battery_status);
//	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, FET_ENABLE);
	state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, ALL_FETS_OFF); // turn on all fet
	hw_delay_ms(10);
	state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
	inrush_limiter_switch_on();
//	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, FET_ENABLE);
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, ALL_FETS_ON);
	state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
	hw_delay_ms(100);
	inrush_limiter_switch_off();
	hw_delay_ms(100);
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, ALL_FETS_OFF); // turn on all fet
	hw_delay_ms(100);

}

static void test_soft_start(BQ_Hw* p_hw){
	int32_t state;
	uint8_t fet_status;
	uint16_t battery_status;
	state = bq_i2c_read_reg_word(p_hw, BATTERY_STATUS, &battery_status);
	inrush_limiter_switch_off(); //on inrush current pin
//	state = bq_i2c_write_reg_word(pq_hw, WRITE_ADDR_REG, FET_ENABLE);
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, ALL_FETS_ON); // turn on all fet
	state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status); //off inrush current
	hw_delay_ms(400);
	inrush_limiter_switch_on(); //off inrush current pin
	hw_delay_ms(500);
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, ALL_FETS_OFF);
	state = bq_i2c_read_reg_byte(p_hw, FET_STATUS, &fet_status);
	hw_delay_ms(500);

}
static void test_read_balancing_bitmask(BQ_Hw* p_hw){
	int32_t state;
	uint8_t balancing_bit =100;
	state = bq_i2c_data_ram_read_byte(p_hw, BALANCING_CONFIGURATION, &balancing_bit);
	hw_delay_ms(10);
}

static void read_calib_cell_volt(BQ_Hw* p_hw, int16_t* cell_gain){
	int32_t state;
	int16_t adc_in;
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_1_GAIN, cell_gain);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_2_GAIN, cell_gain+1);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_3_GAIN, cell_gain+2);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_4_GAIN, cell_gain+3);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_5_GAIN, cell_gain+4);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_6_GAIN, cell_gain+5);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_7_GAIN, cell_gain+6);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_8_GAIN, cell_gain+7);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_9_GAIN, cell_gain+8);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_10_GAIN, cell_gain+9);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_11_GAIN, cell_gain+10);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_12_GAIN, cell_gain+11);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_13_GAIN, cell_gain+12);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_14_GAIN, cell_gain+13);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_15_GAIN, cell_gain+14);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, CELL_16_GAIN, cell_gain+15);
	state = bq_i2c_data_ram_read_word_with_sign(p_hw, ADC_GAIN, &adc_in);
	hw_delay_ms(10);
}

static void calibration(BQ_Hw* p_hw){
	int32_t state;
	int32_t cell_voltage_count[8];
	int8_t dastatus1[32];
	state = bq_i2c_subcommand_read_block_with_sign(p_hw, DASTATUS1, dastatus1, 32);
	cell_voltage_count[0] = ((dastatus1[2]<<16) + (dastatus1[1]<<8) + (dastatus1[0]));
	cell_voltage_count[2] = ((dastatus1[10]<<16) + (dastatus1[9]<<8) + (dastatus1[8]));
	cell_voltage_count[4] =  ((dastatus1[18]<<16) + (dastatus1[17]<<8) + (dastatus1[16]));
	cell_voltage_count[6] = ((dastatus1[26]<<16) + (dastatus1[25]<<8) + (dastatus1[24]));
}
static void bq_i2c_init_100kHz(void) {
	I2C_InitTypeDef I2C_InitStructure;

	/* I2C configuration */
	/* sEE_I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0xAB;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStructure.I2C_Timing =0x00201D2B; /* for HSI clock source */
	I2C_InitStructure.I2C_Timing =  0x10805E89; /* for SYSCLK 48MHz  0x10805E89*/
	I2C_Init(AFE_I2C_DEVICE, &I2C_InitStructure);

	/* sEE_I2C Peripheral Enable */
	I2C_Cmd(AFE_I2C_DEVICE, ENABLE);
}

static void test_toggle_fet(BQ_Hw* p_hw){
	uint16_t mfg_status;
	int state=0;
	state = bq_i2c_subcommand_read_word(p_hw, MANUFACTURING_STATUS, &mfg_status);
	state = bq_i2c_data_ram_write_word(p_hw, MFG_STATUS_INIT, 0x0050); // enable autonomous control mode
	state = bq_i2c_subcommand_read_word(p_hw, MANUFACTURING_STATUS, &mfg_status);
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, FET_ENABLE);
	state = bq_i2c_subcommand_read_word(p_hw, MANUFACTURING_STATUS, &mfg_status);
	state = bq_i2c_write_reg_word(p_hw, WRITE_ADDR_REG, FET_ENABLE);
	state = bq_i2c_subcommand_read_word(p_hw, MANUFACTURING_STATUS, &mfg_status);
}
int main(void)
{
	int state=0;
	uint8_t enable_protetion;
	uint8_t fet_status;
	uint16_t battery_status;
	uint8_t scd_threshold;
	uint8_t vol_cur_count[32];
	int16_t cell_gain[16];
	int16_t voltage_cell_pin[16];
	uint8_t dsg_fet_protection_a=0;
	uint8_t ocd2_threshold;
	uint8_t ocd1_threshold=0;
	uint8_t security_setting;
	uint8_t safety_status_a =4;
	uint16_t mfg_status;

	hw_delay_ms(100);
	test_setup();
	inrush_limter_hardware_init();
	//state = bq_i2c_data_ram_write_word(&bq_hw, VCELL_MODE, 0xFFFF);
//	state = bq_i2c_data_ram_write_word(&bq_hw, POWER_CONFIG, 0x2982);
	state = bq_i2c_data_ram_write_byte(&bq_hw, ENABLED_PROTECTION_A, 0x8C); // all protection enable
	state = bq_i2c_data_ram_read_byte(&bq_hw, ENABLED_PROTECTION_A, &enable_protetion);
	state = bq_i2c_data_ram_write_byte(&bq_hw, CHG_FET_PROTECTIONS_A, 0x98);
	state = bq_i2c_data_ram_write_byte(&bq_hw, DSG_FET_PROTECTIONS_A, 0x80);
	state = bq_i2c_data_ram_write_word(&bq_hw, MFG_STATUS_INIT, 0x0050); // enable autonomous control mode
	state = bq_i2c_data_ram_write_byte(&bq_hw, FET_OPTIONS, 0x0F); // enable CHG FET on SLEEP Mode
//	state = bq_i2c_data_ram_write_word(&bq_hw, DEFAULT_ALRAM_MASK, 0xF800); //enable FUSE
	state = bq_i2c_read_reg_byte(&bq_hw, FET_STATUS, &fet_status);
	state =bq_i2c_data_ram_write_byte(&bq_hw, SCD_THRESHOLD, 0x04);
	state = bq_i2c_data_ram_write_byte(&bq_hw, OCD1_THRESHOLD, 0x04);
	state = bq_i2c_data_ram_write_byte(&bq_hw, CUV_THRESHOLD, 70);


	state = bq_i2c_read_reg_word(&bq_hw, BATTERY_STATUS, &battery_status);
	state = bq_i2c_read_reg_byte(&bq_hw, SAFETY_STATUS_A, &safety_status_a);
	test_read_bq_status(&bq_hw);
	//state = bq_i2c_data_ram_write_byte(&bq_hw, DSG_FET_PROTECTIONS_A, 0xE4);
	state = bq_i2c_data_ram_read_byte(&bq_hw, OCD1_THRESHOLD, &ocd1_threshold);
	state = bq_i2c_data_ram_read_byte(&bq_hw, SCD_THRESHOLD, &scd_threshold);
	state = bq_i2c_data_ram_read_byte(&bq_hw, DSG_FET_PROTECTIONS_A, &dsg_fet_protection_a);
	state = bq_i2c_data_ram_read_byte(&bq_hw, OCD2_THRESHOLD, &ocd2_threshold);
	state = bq_i2c_data_ram_read_byte(&bq_hw, SECURITY_SETTINGS, &security_setting);
	while(1){
//		int16_t cc2_current;
//		bq_i2c_read_reg_word_with_sign(&bq_hw, CC2_CURRENT, &cc2_current);
//		test_on_off_fet(&bq_hw);
//		test_read_bq_status(&bq_hw);
//		test_read_voltage_current(&bq_hw, voltage_cell_pin);
	//		read_calib_cell_volt(&bq_hw, cell_gain);
	//		config_write_data_ram(&bq_hw);
		test_toggle_fet(&bq_hw);
		hw_delay_ms(100);
	}
	return 0;
}


void SysTick_Handler(void){

}

void TIM7_IRQHandler(void) {
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) {
	}
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
 }

