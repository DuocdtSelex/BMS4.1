/*
 * adc_sensor.c
 *
 *  Created on: Aug 22, 2020
 *      Author: quangnd
 */

#include "adc_sensor.h"

static void adc_sensor_update_result_default_impl(ADC_Sensor* p_ss);

void adc_sensor_init(ADC_Sensor* p_ss){
	p_ss->update_result=adc_sensor_update_result_default_impl;
}

static void adc_sensor_update_result_default_impl(ADC_Sensor* p_ss){
	p_ss->update_adc(p_ss);
	p_ss->update_offset(p_ss);
	p_ss->result=(p_ss->adc-p_ss->offset)* p_ss->gain;
}



