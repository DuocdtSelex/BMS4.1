#if 0
/*
 * bq769x0.c
 *
 *  Created on: Aug 19, 2020
 *      Author: quangnd
 */
#include "bq769x0.h"
#include "afe.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "current_sense_hal.h"

static void bq_disable_cell_balancing_impl(AFE *this);
static void bq_enable_cell_balancing_impl(AFE *this,const uint32_t target_vol);
static void bq_set_oc_threshold_mA_impl(AFE *this, uint32_t th);
static void bq_set_sc_threshold_mA_impl(AFE *this, uint32_t th);
static void bq_set_uv_threshold_mV_impl(AFE *this, uint32_t th);
static void bq_set_ov_threshold_mV_impl(AFE *this, uint32_t th);
static void bq_set_oc_delay_ms_impl(AFE *this, uint32_t th);
static void bq_set_sc_delay_us_impl(AFE *this, uint32_t th);
static void bq_set_uv_delay_impl(AFE *this, uint32_t th);
static void bq_set_ov_delay_impl(AFE *this, uint32_t th);
static void bq_update_cell_voltage_impl(AFE *this);
static void bq_update_pack_voltage_impl(AFE *this);
static void bq_update_status_impl(AFE *this);
static void bq_reset_error_impl(AFE *this);
static void bq_set_ship_mode_impl(AFE *this);

static AFE_Interface bq_interface = {
		bq_disable_cell_balancing_impl,
		bq_enable_cell_balancing_impl,
		bq_set_oc_threshold_mA_impl,
		bq_set_sc_threshold_mA_impl,
		bq_set_uv_threshold_mV_impl,
		bq_set_ov_threshold_mV_impl,
		bq_set_oc_delay_ms_impl,
		bq_set_sc_delay_us_impl,
		bq_set_uv_delay_impl,
		bq_set_ov_delay_impl,
		bq_update_cell_voltage_impl,
		bq_update_pack_voltage_impl,
		bq_update_status_impl,
		bq_reset_error_impl,
		bq_set_ship_mode_impl
};

/**
 * Helper functions
 */

static int32_t bq_reg_write_verify(BQ769x0* p_bq,uint8_t reg_addr,uint8_t content);
static uint8_t bq_get_oc_reg_from_threshold(uint32_t mA,uint8_t level);
static uint8_t bq_get_oc_delay_reg_from_threshold(uint32_t mS);
static uint8_t bq_get_sc_reg_from_threshold(uint32_t mA,uint8_t level);
static uint8_t bq_get_sc_delay_reg_from_threshold(uint32_t uS);
static uint8_t bq_get_uv_reg_from_threshold(uint32_t mV,uint32_t offset,uint32_t gain);
static uint8_t bq_get_ov_reg_from_threshold(uint32_t mV,uint32_t offset,uint32_t gain);
static uint8_t bq_get_uv_delay_reg_from_threshold(uint32_t s);
static uint8_t bq_get_ov_delay_reg_from_threshold(uint32_t s);

static void bq_update_adc_gain(BQ769x0 *p_bq);
static void bq_update_adc_offset(BQ769x0 *p_bq);
static void bq_set_temp_monitor_source(BQ769x0 *p_bq,
		const BQ_TEMP_MONITOR_SRC src);
static void bq_enable_adc(BQ769x0* p_bq);

void bq_init(BQ769x0 *p_bq) {
	bq_set_interface(p_bq, &bq_interface);
	bq_update_adc_gain(p_bq);
	bq_update_adc_offset(p_bq);
	bq_set_temp_monitor_source(p_bq, EXT_THERMISTOR);
	bq_enable_adc(p_bq);
	bq_set_ocd_protection_level(p_bq,1);
	/* For optimal performance,
	 *  these bits should be programmed to 0x19 upon device startup
	 *  */
	bq_reg_write_verify(p_bq,CC_CFG,0x19);
	bq_set_cc_en(p_bq);
}


void bq_turn_on_charge(BQ769x0 *p_bq) {
	bq_set_register_bit(p_bq, SYS_CTRL2, CHG_ON_BIT);
}

void bq_turn_off_charge(BQ769x0 *p_bq) {

	bq_clear_register_bit(p_bq, SYS_CTRL2, CHG_ON_BIT);
}

void bq_turn_on_discharge(BQ769x0 *p_bq) {

	bq_set_register_bit(p_bq, SYS_CTRL2, DSG_ON_BIT);
}

void bq_turn_off_discharge(BQ769x0 *p_bq) {
	bq_clear_register_bit(p_bq, SYS_CTRL2, DSG_ON_BIT);
}

uint8_t bq_is_discharge_sw_on(BQ769x0* p_bq){

        uint8_t reg_data=0;
        int32_t rc;
        rc=bq_read_reg_byte_with_crc(p_bq->hw,SYS_CTRL2,&reg_data);
        if(rc<0){
                afe_set_error((AFE*)p_bq,AFE_ERROR_COM_LINK);
                return 0;
        }

        if(reg_data & DSG_ON_BIT) return 1;
        return 0;
}

uint8_t bq_is_charge_sw_on(BQ769x0* p_bq){

        uint8_t reg_data=0;
        int32_t rc;
        rc=bq_read_reg_byte_with_crc(p_bq->hw,SYS_CTRL2,&reg_data);
        if(rc<0){
                afe_set_error((AFE*)p_bq,AFE_ERROR_COM_LINK);
                return 0;
        }

        if(reg_data & CHG_ON_BIT) return 1;
        return 0;
}

int32_t bq_read_current_mA(const BQ769x0* const p_bq){

	int16_t adc;
	int success=0;
	success= bq_read_reg_word_with_crc(p_bq->hw,CC_HI_BYTE,(uint16_t*)&adc);
	if(success!=0){
		afe_set_error((AFE*) p_bq,AFE_ERROR_COM_LINK);
		return 0;
	}
	return ((int32_t)adc) * 8440/(1000*CURRENT_SENSE_R_SHUNT_mOhm);
}

void bq_set_cc_en(BQ769x0* p_bq){

	bq_set_register_bit(p_bq,SYS_CTRL2,CC_EN_BIT);
	bq_clear_register_bit(p_bq,SYS_CTRL2,CC_ONESHOT_BIT);
}

void bq_clear_cc_flag(BQ769x0* p_bq){
	bq_set_register_bit(p_bq,SYS_STAT,BQ_STAT_CCREADY);
}

void bq_set_oneshot_en(BQ769x0* p_bq){

	bq_clear_register_bit(p_bq,SYS_CTRL2,CC_EN_BIT);
	bq_set_register_bit(p_bq,SYS_CTRL2,CC_ONESHOT_BIT);
}

static void bq_update_adc_gain(BQ769x0 *p_bq) {
	int32_t result = -1;
	result = bq_read_reg_byte_with_crc(p_bq->hw, ADCGAIN1,
			&(p_bq->bq_registers.ADCGain1.ADCGain1Byte));
	result += bq_read_reg_byte_with_crc(p_bq->hw, ADCGAIN2,
			&(p_bq->bq_registers.ADCGain2.ADCGain2Byte));
	result += bq_read_reg_byte_with_crc(p_bq->hw, ADCOFFSET,
			&(p_bq->bq_registers.ADCOffset));
	if (result != 0) {
		afe_set_error((AFE*) p_bq, AFE_ERROR_COM_LINK);
		return;
	}
	p_bq->adc_gain = 365
			+ ((p_bq->bq_registers.ADCGain1.ADCGain1Byte & 0x0C) << 1)
			+ ((p_bq->bq_registers.ADCGain2.ADCGain2Byte & 0xE0) >> 5);
}

static void bq_update_adc_offset(BQ769x0 *p_bq) {
	int32_t result = -1;
	result = bq_read_reg_byte_with_crc(p_bq->hw, ADCOFFSET,
			&(p_bq->bq_registers.ADCOffset));
	if (result != 0) {
		afe_set_error((AFE*) p_bq, AFE_ERROR_COM_LINK);
		return;
	}
	p_bq->adc_offset = (uint32_t) p_bq->bq_registers.ADCOffset;
}

static void bq_set_temp_monitor_source(BQ769x0 *p_bq,
		const BQ_TEMP_MONITOR_SRC src) {
	int success = -1;
	switch (src) {
	case DIE_TEMP:
		success = bq_clear_register_bit(p_bq, SYS_CTRL1, TEMP_SEL_BIT);
		break;
	case EXT_THERMISTOR:
		success = bq_set_register_bit(p_bq, SYS_CTRL1, TEMP_SEL_BIT);
		break;
	default:
		return;
	}

	if (success != 0) {
		afe_set_error((AFE*) p_bq, AFE_ERROR_COM_LINK);
		return;
	}
	p_bq->temp_monitor_source = src;
}

void bq_set_ship_mode_impl(AFE *this){
	BQ769x0 *p_bq = (BQ769x0*) this;
	// Default SYS_CTRL1 = 0x00
	//SHIP mode
	uint8_t data =0;
	bq_read_reg_byte_with_crc(p_bq->hw, SYS_CTRL1, &data);
	data&=0x18; //Keep ADC_EN, TEMP_SEL, clear other bit
	bq_write_reg_byte_with_crc(p_bq->hw, SYS_CTRL1, data);
	bq_write_reg_byte_with_crc(p_bq->hw, SYS_CTRL1, SHUT_B_BIT);
	bq_write_reg_byte_with_crc(p_bq->hw, SYS_CTRL1, SHUT_A_BIT);

}
#if 0
void bq_set_protect_params(BQ769x0* p_bq){

	int result=0;
	p_bq->bq_registers.Protect1.Protect1Bit.SCD_DELAY = SCDDelay;
	p_bq->bq_registers.Protect1.Protect1Bit.SCD_THRESH = SCDThresh;
	p_bq->bq_registers.Protect2.Protect2Bit.OCD_DELAY = OCDDelay;
	p_bq->bq_registers.Protect2.Protect2Bit.OCD_THRESH = OCDThresh;
	p_bq->bq_registers.Protect3.Protect3Bit.OV_DELAY = OVDelay;
	p_bq->bq_registers.Protect3.Protect3Bit.UV_DELAY = UVDelay;

	Registers2.Protect1.Protect1Bit.SCD_DELAY = SCDDelay;
	Registers2.Protect1.Protect1Bit.SCD_THRESH = SCDThresh;
	Registers2.Protect2.Protect2Bit.OCD_DELAY = OCDDelay;
	Registers2.Protect2.Protect2Bit.OCD_THRESH = OCDThresh;
	Registers2.Protect3.Protect3Bit.OV_DELAY = OVDelay;
	Registers2.Protect3.Protect3Bit.UV_DELAY = UVDelay;

	result = GetADCGainOffset();
	if(result !=0){
		afe_set_error((AFE*) p_bq,AFE_ERROR_COM_LINK);
		return;
	}

	Gain1 = (365 + ((p_bq->bq_registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((p_bq->bq_registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5)) / 1000.0;
	iGain1 = 365 + ((p_bq->bq_registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((p_bq->bq_registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5);
	p_bq->bq_registers.OVTrip = (unsigned char)((((unsigned short)((OVPThreshold - p_bq->bq_registers.ADCOffset)/Gain1 + 0.5) - OV_THRESH_BASE) >> 4) & 0xFF);
	p_bq->bq_registers.UVTrip = (unsigned char)((((unsigned short)((UVPThreshold - p_bq->bq_registers.ADCOffset)/Gain1 + 0.5) - UV_THRESH_BASE) >> 4) & 0xFF);

}
#endif

void bq_set_ocd_protection_level(BQ769x0 *p_bq, const uint8_t level) {
	int32_t success = -1;
	if (level == 0) {
		success = bq_clear_register_bit(p_bq, PROTECT1, SNRS_BIT);
	} else {
		success = bq_set_register_bit(p_bq, PROTECT1, SNRS_BIT);
	}
	if (success != 0) {
		afe_set_error((AFE*) p_bq, AFE_ERROR_COM_LINK);
		return;
	}
	p_bq->protect_level=level;
}

void bq_set_cell_group(BQ769x0* p_bq,const uint8_t group){

	p_bq->cell_group=group;
}

/**
 * VTSX = (ADC in Decimal) x 382 µV/LSB
 * RTS = (10,000 × VTSX) ÷ (3.3 – VTSX)
*/
uint16_t bq_read_ntc_impedance(BQ769x0* p_bq,const uint8_t id){

	uint16_t adc=0;
	int32_t success=bq_read_reg_word_with_crc(p_bq->hw,TS1_HI_BYTE+2*(id-1),&adc);
	if(success!=0){
		afe_set_error((AFE*)p_bq,AFE_ERROR_COM_LINK);
		return 0;
	}

	adc&= 0x3FFF;
	uint32_t v_tsx_uV=382*(uint32_t)adc/1000;
	uint32_t v_ref_uV=3300;
	if(v_tsx_uV==v_ref_uV){
		return 0;
	}
	return (10000*v_tsx_uV)/(v_ref_uV-v_tsx_uV);
}

int bq_set_register_bit(BQ769x0 *p_bq, uint8_t reg, const uint8_t bit_mask) {

	int success = 0;
	uint8_t reg_data = 0;
	success = bq_read_reg_byte_with_crc(p_bq->hw, reg, &reg_data);
	if (success != 0) {
		return success;
	}
	reg_data |= bit_mask;
	success = bq_write_reg_byte_with_crc(p_bq->hw, reg, reg_data);
	return success;
}

int bq_clear_register_bit(BQ769x0 *p_bq, uint8_t reg, const uint8_t bit_mask) {

	int success = 0;
	uint8_t control_reg = 0;
	success = bq_read_reg_byte_with_crc(p_bq->hw, reg, &control_reg);
	if (success != 0) {
		return success;
	}
	control_reg &= ~bit_mask;
	success = bq_write_reg_byte_with_crc(p_bq->hw, reg, control_reg);
	return success;
}

static void bq_update_status_impl(AFE *this) {

	BQ769x0 *p_bq = (BQ769x0*) this;
	int success = -1;
	uint8_t data;
	success = bq_read_reg_byte_with_crc(p_bq->hw, SYS_STAT,&data);
	if (success != 0) {
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	success=bq_write_reg_byte_with_crc(p_bq->hw, SYS_STAT, data);
	if (success != 0) {
		afe_set_error(this, AFE_ERROR_COM_LINK);
		return;
	}
	this->status= data & 0x7F;

}

static void bq_disable_cell_balancing_impl(AFE *this) {

	BQ769x0 *p_bq = (BQ769x0*) this;
	int result=0;
	uint8_t buffer[MAX_CELL_GROUP]={0};
	uint8_t verify_buffer[MAX_CELL_GROUP]={0};
	result+= bq_write_reg_block_with_crc(p_bq->hw,CELLBAL1,buffer,p_bq->cell_group);
	result+= bq_read_reg_block_with_crc(p_bq->hw,CELLBAL1,verify_buffer,p_bq->cell_group);
	if(result!=0){
		afe_set_error((AFE*)p_bq,AFE_ERROR_COM_LINK);
		return;
	}

	for(int i=0;i<p_bq->cell_group;i++){
		if(buffer[i] != verify_buffer[i]){
			afe_set_error((AFE*)p_bq,AFE_ERROR_READ_MISMATCH);
			return;
		}
	}
}

static bool cell_need_balancing(const uint32_t target_voltage, const uint32_t cell_vol){
	if((cell_vol > (target_voltage + BALANCING_OFFSET)) && (cell_vol > BALANCING_VOLTAGE_THRESHOLD) )
		return true;
	return false;
}

static uint8_t cell_array_get_group_max_vol(const Cell_Bank* const p_cells,const uint8_t len,
		const uint8_t ignore_mask,uint32_t* vol){

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

static void bq_calculate_group_balancing_mask(Cell_Bank* cells,uint8_t* buff,const uint32_t target_vol){

	uint8_t max_cell_index=0;
	uint32_t max_vol=0;
	uint8_t checked_mask=0b11100000; /* need only 5 last bits for max 5 cell in a group */
	uint8_t adjacent_mask=0;
	uint8_t max_cell_mask=0;
	uint8_t short_cell_mask=0;
	/* reset mask */
	*buff=0;
	for(int i=0;i<5;i++){
		if(cells[i].is_short==1){
			short_cell_mask|= (1<<i);
		}
	}
	while(checked_mask !=0xFF){
		max_cell_index=cell_array_get_group_max_vol(cells, 5,checked_mask,&max_vol);
		if(cell_need_balancing(target_vol, max_vol)){
			max_cell_mask=(1<<max_cell_index);
			*buff|= max_cell_mask;
			/* mask 2 adjacent as checked because bq don't allow to balancing 2 adjacent cells */
			adjacent_mask |= max_cell_mask + (max_cell_mask<<1) + (max_cell_mask>>1);
			/* process shorted cell */
			if(max_cell_mask & (short_cell_mask>>1)){
				adjacent_mask |= (max_cell_mask<<2);
			}
			if(max_cell_mask & (short_cell_mask<<1)){
				adjacent_mask |= max_cell_mask>>2;
			}
			checked_mask|= adjacent_mask;

		}else{
			/* if event cell with max voltage don't need balancing so other cell too */
			return;
		}

	}

#if 0
	/* balancing only max cel */
		max_cell_index=cell_array_get_group_max_vol(cells, 5,checked_mask,&max_vol);
		if(cell_need_balancing(target_vol, max_vol)){
			adjacent_mask=1<<max_cell_index;
			*buff|= adjacent_mask;
			/* mask 2 adjacent as checked because bq don't allow to balancing 2 adjacent cells */
			checked_mask|= adjacent_mask + (adjacent_mask<<1) + (adjacent_mask>>1);
		}else{
			/* if event cell with max voltage don't need balancing so other cell too */
			return;
		}
#endif
}

static void bq_enable_cell_balancing_impl(AFE *this,const uint32_t target_vol) {

	BQ769x0 *p_bq = (BQ769x0*) this;
	uint8_t balancing_mask[CELL_GROUP_NUM]={0};

	for(int i=0;i<p_bq->cell_group;i++){
		bq_calculate_group_balancing_mask(&this->cell_array->cells[5*i],&balancing_mask[i], target_vol);
	}

	int result =0;
	uint8_t reg_state[CELL_GROUP_NUM] = { 0 };
	result +=bq_write_reg_block_with_crc(p_bq->hw, CELLBAL1,&balancing_mask[0], p_bq->cell_group);
	result += bq_read_reg_block_with_crc(p_bq->hw, CELLBAL1, reg_state, p_bq->cell_group);

	if(result != 0){
		afe_set_error((AFE*)p_bq,AFE_ERROR_COM_LINK);
		return;
	}

	for(int i=0;i<p_bq->cell_group;i++){
		if(balancing_mask[i] != reg_state[i]){
			afe_set_error((AFE*) p_bq,AFE_ERROR_READ_MISMATCH);
			return;
		}
	}

#if 0
	int result=0;
	uint8_t reg_state[CELL_GROUP_NUM]={0};
	while((reg_state[0]!=balancing_bits[0])||
			(reg_state[1]!=balancing_bits[1])){

		result= I2CWriteBlockWithCRC(AFE1_I2C_DEVICE,AFE_I2C_ADDRESS,CELLBAL1,balancing_bits,CELL_GROUP_NUM-2);
		result = I2CReadBlockWithCRC(AFE1_I2C_DEVICE,AFE_I2C_ADDRESS,CELLBAL1,reg_state,CELL_GROUP_NUM-2);
	}
	while((reg_state[2]!=balancing_bits[2])||
			(reg_state[3]!=balancing_bits[3])){

		result= I2CWriteBlockWithCRC(AFE2_I2C_DEVICE,AFE_I2C_ADDRESS,CELLBAL1,balancing_bits + 2,CELL_GROUP_NUM-2);
		result = I2CReadBlockWithCRC(AFE2_I2C_DEVICE,AFE_I2C_ADDRESS,CELLBAL1,reg_state + 2,CELL_GROUP_NUM-2);
	}
	return result;
#endif
}

static void bq_set_oc_threshold_mA_impl(AFE *this, uint32_t th) {

	BQ769x0* p_bq=(BQ769x0*)this;
	int success=0;
	uint8_t reg_value=0;
	uint8_t oc_reg=bq_get_oc_reg_from_threshold(th,p_bq->protect_level);

	success=bq_read_reg_byte_with_crc(p_bq->hw,PROTECT2,&reg_value);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

	reg_value &=0xF0; /* keep other bit unchange, keep only OC threshold bits */
	oc_reg&=0x0F;
	reg_value+=oc_reg;
	success=bq_reg_write_verify(p_bq,PROTECT2,reg_value);

	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_sc_threshold_mA_impl(AFE *this, uint32_t th){

	BQ769x0* p_bq=(BQ769x0*)this;
	int success=0;
	uint8_t reg_value=0;
	uint8_t oc_reg=bq_get_sc_reg_from_threshold(th,p_bq->protect_level);

	success=bq_read_reg_byte_with_crc(p_bq->hw,PROTECT1,&reg_value);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

	reg_value &=0xE0; /* keep other bit unchange, keep only SC threshold bits */
	oc_reg&=0x07;
	reg_value+=oc_reg;
	success=bq_reg_write_verify(p_bq,PROTECT1,reg_value);

	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

}

static void bq_set_uv_threshold_mV_impl(AFE *this, uint32_t th) {

	BQ769x0* p_bq=(BQ769x0*)this;
	uint8_t reg_value=bq_get_uv_reg_from_threshold(th,p_bq->adc_offset,p_bq->adc_gain);
	int success=bq_reg_write_verify(p_bq,UV_TRIP,reg_value);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_ov_threshold_mV_impl(AFE *this, uint32_t th) {

	BQ769x0* p_bq=(BQ769x0*)this;
	uint8_t reg_value=bq_get_ov_reg_from_threshold(th,p_bq->adc_offset,p_bq->adc_gain);
	int success=bq_reg_write_verify(p_bq,OV_TRIP,reg_value);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

}

static void bq_set_sc_delay_us_impl(AFE *this, uint32_t th){

	BQ769x0* p_bq=(BQ769x0*)this;
	uint8_t sc_reg=bq_get_sc_delay_reg_from_threshold(th);
	uint8_t reg_value=0;
	int32_t success=0;
	success=bq_read_reg_byte_with_crc(p_bq->hw,PROTECT1,&reg_value);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

	reg_value &=0xE7; /* keep other bit unchange, keep only SC threshold bits */
	sc_reg<<=3;
	reg_value+=sc_reg;
	success=bq_reg_write_verify(p_bq,PROTECT1,reg_value);

	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_oc_delay_ms_impl(AFE *this, uint32_t th){

	BQ769x0* p_bq=(BQ769x0*)this;
	uint8_t oc_reg=bq_get_oc_delay_reg_from_threshold(th);
	uint8_t reg_value=0;
	int32_t success=0;
	success=bq_read_reg_byte_with_crc(p_bq->hw,PROTECT2,&reg_value);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

	reg_value &=0xE7; /* keep other bit unchange, keep only SC threshold bits */
	oc_reg<<=4;
	reg_value+=oc_reg;
	success=bq_reg_write_verify(p_bq,PROTECT2,reg_value);

	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_uv_delay_impl(AFE *this, uint32_t th){

	BQ769x0* p_bq=(BQ769x0*)this;
	uint8_t uvd_reg=bq_get_uv_delay_reg_from_threshold(th);
	uint8_t reg_value=0;
	int32_t success=0;
	success=bq_read_reg_byte_with_crc(p_bq->hw,PROTECT3,&reg_value);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

	reg_value &=0x3F; /* keep other bit unchange, keep only SC threshold bits */
	uvd_reg<<=6;
	reg_value+=uvd_reg;
	success=bq_reg_write_verify(p_bq,PROTECT3,reg_value);

	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_set_ov_delay_impl(AFE *this, uint32_t th){

	BQ769x0* p_bq=(BQ769x0*)this;
	uint8_t ovd_reg=bq_get_ov_delay_reg_from_threshold(th);
	uint8_t reg_value=0;
	int32_t success=0;
	success=bq_read_reg_byte_with_crc(p_bq->hw,PROTECT3,&reg_value);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

	reg_value &=0xCF; /* keep other bit unchange, keep only SC threshold bits */
	ovd_reg<<=4;
	reg_value+=ovd_reg;
	success=bq_reg_write_verify(p_bq,PROTECT3,reg_value);

	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
}

static void bq_update_cell_voltage_impl(AFE *this) {

	BQ769x0 *p_bq = (BQ769x0*) this;

	int32_t success = 1;
	int32_t i = 0;
	uint8_t *pRawADCData1 = NULL;
	uint16_t iTemp = 0;
	uint32_t lTemp = 0;

	/* every group of cell has 5 cell, with 2byte for voltage information */
	uint32_t block_size=2*p_bq->cell_group*5;

	success = bq_read_reg_block_with_crc(p_bq->hw,VC1_HI_BYTE, &(p_bq->bq_registers.VCell1.VCell1Byte.VC1_HI),
			block_size);
	if(success!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}

	pRawADCData1 = &p_bq->bq_registers.VCell1.VCell1Byte.VC1_HI;
	for (i = 0; i < p_bq->base.cell_array->serial_cells; i++) {
			iTemp = (uint16_t) (*pRawADCData1 << 8) + *(pRawADCData1 + 1);
			lTemp = ((uint32_t) iTemp * p_bq->adc_gain) / 1000;
			lTemp += p_bq->adc_offset;
			this->cell_array->cells[i].voltage = lTemp;
			pRawADCData1 += 2;
	}
}
;

static void bq_update_pack_voltage_impl(AFE *this) {

	BQ769x0 *p_bq = (BQ769x0*) this;
	uint16_t adc = 0;
	int result = 1;
	result = bq_read_reg_word_with_crc(p_bq->hw, BAT_HI_BYTE, &adc);
	if(result!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
	/* 4 x GAIN x ADC(cell) + (#Cells x OFFSET) */
	this->pack_voltage = 4*p_bq->adc_gain * (uint32_t) adc / 1000;
	this->pack_voltage +=(uint32_t) p_bq->unshort_cell*p_bq->adc_offset;
}

static void bq_reset_error_impl(AFE *this){

	BQ769x0 *p_bq = (BQ769x0*) this;
	int result = 1;
	uint8_t data=0;
	result=bq_read_reg_byte_with_crc(p_bq->hw,SYS_STAT,&data);
	if(result!=0){
	        afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
	result = bq_write_reg_byte_with_crc(p_bq->hw,SYS_STAT,data);
	if(result!=0){
		afe_set_error(this,AFE_ERROR_COM_LINK);
		return;
	}
	this->error=AFE_ERROR_NO;
}

static void bq_enable_adc(BQ769x0* p_bq){

	bq_set_register_bit(p_bq,SYS_CTRL1,ADC_EN_BIT);
}


static int32_t bq_reg_write_verify(BQ769x0* p_bq,uint8_t reg_addr,uint8_t content){

	uint8_t verify=0;
	int32_t success=bq_write_reg_byte_with_crc(p_bq->hw,reg_addr,content);
	if(success!=0) return success;
	success=bq_read_reg_byte_with_crc(p_bq->hw,reg_addr,&verify);
	if(success!=0) return success;

	if(content != verify){
		return -2;
	}
	return 0;
}

static uint32_t oc_level0_mV[]={8,11,14,17,19,22,25,28,31,33,36,39,42,44,47,50};
static uint32_t oc_level1_mV[]={17,22,28,33,39,44,50,56,61,67,72,78,83,89,94,100};
/**
 * @brief Get OC register value correspond to Ampaire rating
 */
static uint8_t bq_get_oc_reg_from_threshold(uint32_t mA,uint8_t level){

	uint32_t shunt_drop_mV= mA*HAL_CURRENT_SENSE_R_SHUNT_mOhm/1000;
	uint32_t* oc_level_mV;
	if(level==0){
		oc_level_mV=oc_level0_mV;
	}else if(level==1){
		oc_level_mV=oc_level1_mV;
	};

	uint8_t index=0;
	if(shunt_drop_mV < oc_level_mV[0])
		return 0;
	while((shunt_drop_mV>=oc_level_mV[index]) && (index<15)){
		index++;
	}
	return index -1;
};

/*
 * 0x0 8
0x1 20
0x2 40
0x3 80
0x4 160
0x5 320
0x6 640
0x7 1280
 */

static uint32_t oc_delay_ms[]={8,20,40,80,160,320,640,1280};

static uint8_t bq_get_oc_delay_reg_from_threshold(uint32_t ms){

	uint8_t reg_value=0;
	while((reg_value<7)&&(ms>oc_delay_ms[reg_value])){
		reg_value++;
	}

	return reg_value;
}

static uint8_t sc_level0_mV[]={22,33,44,56,67,78,89,100};
static uint8_t sc_level1_mV[]={44,67,89,111,133,155,178,200};
/**
 * @brief Get OC register value correspond to Ampaire rating
 */
static uint8_t bq_get_sc_reg_from_threshold(uint32_t mA,uint8_t level){

	uint32_t shunt_drop_mV= mA*HAL_CURRENT_SENSE_R_SHUNT_mOhm/1000;
	uint8_t* sc_level_mV;
	if(level==0){
		sc_level_mV=sc_level0_mV;
	}else if(level==1){
		sc_level_mV=sc_level1_mV;
	};

	uint8_t index=0;
	while((shunt_drop_mV>sc_level_mV[index]) && (index<7)){
		index++;
	}
	return index;
};

static uint32_t sc_delay_us[]={70,100,200,400};
static uint8_t bq_get_sc_delay_reg_from_threshold(uint32_t us){

	uint8_t reg_value=0;
	while((reg_value<4)&&(us>sc_delay_us[reg_value])){
		reg_value++;
	}

	return reg_value;
}

static uint8_t bq_get_uv_reg_from_threshold(uint32_t mV,uint32_t offset,uint32_t gain){

	uint32_t adc_value=1000*(mV-offset)/gain;
	adc_value-=UV_THRESH_BASE;
	adc_value>>=4;
	adc_value&=0xFF;
	return (uint8_t) adc_value;
};

static uint8_t bq_get_ov_reg_from_threshold(uint32_t mV,uint32_t offset,uint32_t gain){

	uint32_t adc_value=1000*(mV-offset)/gain;
	adc_value-=OV_THRESH_BASE;
	adc_value>>=4;
	adc_value&=0xFF;
	return (uint8_t) adc_value;
};

static uint8_t uv_delay_s[]={1,4,8,16};
static uint8_t bq_get_uv_delay_reg_from_threshold(uint32_t s){

	uint8_t reg_value=0;
	while((reg_value<4)&&(s>uv_delay_s[reg_value])){
		reg_value++;
	}

	return reg_value;

}

static uint8_t ov_delay_s[]={1,2,4,8};
static uint8_t bq_get_ov_delay_reg_from_threshold(uint32_t s){

	uint8_t reg_value=0;
	while((reg_value<4)&&(s>ov_delay_s[reg_value])){
		reg_value++;
	}

	return reg_value;
};

#endif
