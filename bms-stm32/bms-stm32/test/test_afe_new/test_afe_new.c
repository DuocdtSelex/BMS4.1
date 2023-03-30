/*
 * test_afe_new.c
 *
 *  Created on: May 6, 2021
 *      Author: Admin
 */

#include "bq769x2.h"
#include "core_hw.h"
#include "debug_com_port_hardware.h"
#include "stdlib.h"
#include "delay_hw.h"
#include "string_util.h"
#include "afe_config.h"

BQ769x2 bq;
AFE app_afe;
Cell_Array pack_cell_array;
Cell_Array cell_array;
Cell_Bank cells[PACK_MAX_CELL];
static uint8_t short_cell_mask[PACK_MAX_CELL]={
		0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0
};
static void print_cell_voltage(Cell_Array* p_array,const uint8_t num);
static void print_bq_status(BQ769x2* p_bq);

static void bq_setup(void){
	bq.hw=&bq_hw;
	bq.hw =&bq_hw;
	bq_init_1(&bq);
	//bq_set_cell_group(&bq, 1);

	pack_cell_array.cells=&cells[0];
	pack_cell_array.serial_cells=PACK_MAX_CELL;
	for(int i=0;i<pack_cell_array.serial_cells;i++){
		pack_cell_array.cells[i].capacity=CELL_CAPACITY;
		pack_cell_array.cells[i].voltage=3700UL;
		pack_cell_array.cells[i].is_short=short_cell_mask[i];
	}
	app_afe.cell_array=&pack_cell_array;

	bq.unshort_cell =16;

	cell_array.cells=&cells[0];
	bq.base.cell_array=&cell_array;
	bq.base.cell_array->serial_cells=BQ_MAX_CELL;

	afe_set_oc_threshold_mA(&app_afe,OCD1_THRESHOLD_mA);
	afe_set_oc_delay_ms(&app_afe,OCD1_DELAY_mS);

	afe_set_sc_threshold_mA(&app_afe,SCD_THRESHOLD_mA);
	afe_set_sc_delay_us(&app_afe,SCD_DELAY_uS);

	afe_set_ov_threshold_mV(&app_afe,COV_THRESHOLD_mV);
	afe_set_ov_delay(&app_afe,COV_DELAY);

	afe_set_uv_threshold_mV(&app_afe,CUV_THRESHOLD_mV);
	afe_set_uv_delay(&app_afe,CUV_DELAY);


}

static void test_setup(void){
	core_hw_init();
	debug_com_hw_init();
	afe_hardware_init_1();
	bq_setup();
	afe_set_ov_threshold_mV((AFE*)&bq,4000);
	afe_set_ov_threshold_mV((AFE*)&bq,4000);
}

static void test_read_cell_voltage(void){
		bq_update_cell_voltage(&bq);
		debug_sends(&debug_port,(uint8_t*)"\n");
}

static void test_read_ntc(void){

	uint16_t ntc[4];
	uint8_t buffer[20];

	ntc[0]=bq_read_ntc_temperature_CFETOFF_TEMP(&bq);
	ntc[1]=bq_read_ntc_temperature_HDQ_TEMP(&bq);
	ntc[2]=bq_read_ntc_temperature_TS1(&bq);
	ntc[3]=bq_read_ntc_temperature_TS3(&bq);
	for(int i=0;i<4;i++){
		long_to_string(ntc[i],buffer);
		debug_sends(&debug_port,buffer);
		debug_sends(&debug_port,(uint8_t*)",");
	}
	debug_sends(&debug_port,(uint8_t*)"\n");
}

static void test_set_uv_threshold(BQ769x2* p_bq){

	static uint32_t th=2500;
	uint8_t buffer[20]={0};
	afe_set_uv_threshold_mV((AFE*)p_bq,th);
	afe_update_status((AFE*)p_bq);

	long_to_string(th,buffer);
	debug_sends(&debug_port,(uint8_t*)"UV Threshold: ");
	debug_sends(&debug_port,buffer);
	debug_sends(&debug_port,(uint8_t*)" State: ");
	print_bq_status(p_bq);
	debug_sends(&debug_port,(uint8_t*)"\n");
	th+=100;
	if(th>4300) th=2000;
}

static void bq_sw_off(void){
	bq_turn_off_all_fet(&bq);
}

static void bq_sw_on(void){
	bq_turn_on_all_fet(&bq);
}


static void test_on_off(void){

	static uint8_t state=0;
	state=~state;
	debug_sends(&debug_port,(uint8_t*)"TEST SWITCH: ");
	if(state){

		debug_sends(&debug_port,(uint8_t*)"OFF");
		bq_sw_off();
	}
	else{
		bq_sw_on();
		debug_sends(&debug_port,(uint8_t*)"ON");
	}
	debug_sends(&debug_port,(uint8_t*)"\n");
}

static void test_read_current(void){
}

static void test_oc_protect(void){

	static uint8_t on=0;
	if(!on){
		bq_sw_on();
		on=~on;
	}
	afe_set_oc_delay_ms((AFE*)&bq,10);
	afe_set_oc_threshold_mA((AFE*)&bq,2000);
	print_bq_status(&bq);
	debug_sends(&debug_port,(uint8_t*)"\n");
	test_read_current();
	test_read_cell_voltage();
}


static void test_set_ov_threshold(BQ769x2* p_bq){

	static uint32_t th=4300;
	uint8_t buffer[20]={0};
	afe_set_ov_threshold_mV((AFE*)p_bq,th);
	afe_set_ov_delay((AFE*)p_bq,1);
	afe_update_status((AFE*)p_bq);

	long_to_string(th,buffer);
	debug_sends(&debug_port,(uint8_t*)"OV Threshold: ");
	debug_sends(&debug_port,buffer);
	debug_sends(&debug_port,(uint8_t*)" State: ");
	print_bq_status(p_bq);
	debug_sends(&debug_port,(uint8_t*)"\n");
	th-=100;
	if(th <100) th=4300;
}


int main(void){
	test_setup();
	while(1){
		//test_read_cell_voltage();
		//test_set_uv_threshold(&bq);
		//test_set_ov_threshold(&bq);
		//test_read_current();
		//test_on_off();
		//test_oc_protect();
		test_read_ntc();
		hw_delay_ms(2000);
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


static void print_cell_voltage(Cell_Array* p_array,const uint8_t num){

	uint8_t buffer[20]={0};
	for(int i=0;i<num;i++){
		long_to_string(p_array->cells[i].voltage,buffer);
		debug_sends(&debug_port,buffer);
		debug_sends(&debug_port,(uint8_t*)",");
	}
}

static void print_bq_status(BQ769x2* p_bq){
	uint32_t status=afe_get_status((AFE*)p_bq);
}
