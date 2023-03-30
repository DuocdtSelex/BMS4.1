/*
 * test_AFE.c
 *
 *  Created on: Sep 22, 2020
 *      Author: quangnd
 */
#include "bq769x0.h"
#include "core_hw.h"
#include "debug_com_port_hardware.h"
#include "stdlib.h"
#include "delay_hw.h"
#include "string_util.h"

BQ769x0 upper_bq;
BQ769x0 lower_bq;
AFE app_afe;
Cell_Array pack_cell_array;
Cell_Array lower_cell_array;
Cell_Array upper_cell_array;
Cell_Bank cells[20];
static uint8_t short_cell_mask[20]={
		0,0,0,1,0,0,0,0,1,0, /* for lower half */
		0,0,0,1,0,0,0,0,1,0 /* for higer half */
};

static void print_cell_voltage(Cell_Array* p_array,const uint8_t num);
static void print_bq_status(BQ769x0* p_bq);

static void bq_setup(void){
	lower_bq.hw=&lower_bq_hw;
	upper_bq.hw=&upper_bq_hw;
	bq_init(&lower_bq);
	bq_init(&upper_bq);
	bq_set_cell_group(&lower_bq,2);
	bq_set_cell_group(&upper_bq,2);

	pack_cell_array.cells=&cells[0];
	pack_cell_array.serial_cells=20;
	for(int i=0;i<pack_cell_array.serial_cells;i++){
		pack_cell_array.cells[i].capacity=3000;
		pack_cell_array.cells[i].voltage=3700UL;
		pack_cell_array.cells[i].is_short=short_cell_mask[i];
	}

	lower_bq.unshort_cell=8;
	upper_bq.unshort_cell=8;

	lower_cell_array.cells=&cells[0];
	lower_bq.base.cell_array=&lower_cell_array;
	lower_bq.base.cell_array->serial_cells=10;

	upper_cell_array.cells=&(cells[10]);
	upper_bq.base.cell_array=&upper_cell_array ;
	upper_bq.base.cell_array->serial_cells=10;


}


static void test_setup(void){
	core_hw_init();
	debug_com_hw_init();
	afe_hardware_init();
	bq_setup();
	afe_set_ov_threshold_mV((AFE*)&upper_bq,4300);
	afe_set_ov_threshold_mV((AFE*)&lower_bq,4300);
}

static void test_read_cell_voltage(void){
		bq_update_cell_voltage(&upper_bq);
		bq_update_cell_voltage(&lower_bq);

		print_cell_voltage(lower_bq.base.cell_array,10);
		print_cell_voltage(upper_bq.base.cell_array,10);
		debug_sends(&debug_port,(uint8_t*)"\n");
}

static void test_read_ntc(void){

	uint16_t ntc[4];
	uint8_t buffer[20];

	ntc[0]=bq_read_ntc_impedance(&upper_bq,1);
	ntc[1]=bq_read_ntc_impedance(&upper_bq,2);
	ntc[2]=bq_read_ntc_impedance(&lower_bq,1);
	ntc[3]=bq_read_ntc_impedance(&lower_bq,2);
	for(int i=0;i<4;i++){
		long_to_string(ntc[i],buffer);
		debug_sends(&debug_port,buffer);
		debug_sends(&debug_port,(uint8_t*)",");
	}
	debug_sends(&debug_port,(uint8_t*)"\n");
}

static void test_set_uv_threshold(BQ769x0* p_bq){

	static uint32_t th=2000;
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

	bq_turn_off_charge(&lower_bq);
	bq_turn_off_discharge(&lower_bq);
	bq_turn_off_charge(&upper_bq);
	bq_turn_off_discharge(&upper_bq);
}

static void bq_sw_on(void){
	bq_turn_on_charge(&lower_bq);
	bq_turn_on_discharge(&lower_bq);
	bq_turn_on_charge(&upper_bq);
	bq_turn_on_discharge(&upper_bq);
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

	uint8_t buffer[20];
	int32_t current=bq_read_current_mA(&lower_bq);
	bq_clear_cc_flag(&lower_bq);
	slong_to_string(current,buffer);
	debug_sends(&debug_port,buffer);
	debug_sends(&debug_port,(uint8_t*)"\n");
}

static void test_oc_protect(void){

	static uint8_t on=0;
	if(!on){
		bq_sw_on();
		on=~on;
	}
	afe_set_oc_delay_ms((AFE*)&lower_bq,10);
	afe_set_oc_threshold_mA((AFE*)&lower_bq,2000);
	print_bq_status(&lower_bq);
	debug_sends(&debug_port,(uint8_t*)"\n");
	test_read_current();
	test_read_cell_voltage();
}


static void test_set_ov_threshold(BQ769x0* p_bq){

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
		//test_set_uv_threshold(&lower_bq);
		//test_set_ov_threshold(&lower_bq);
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

static void print_bq_status(BQ769x0* p_bq){

	uint32_t status=afe_get_status((AFE*)p_bq);

	if(status !=0){
		if(status & BQ_STAT_UV){
			debug_sends(&debug_port,(uint8_t*)"UV-");
		}

		if(status & BQ_STAT_OV){
			debug_sends(&debug_port,(uint8_t*)"OV-");
		}

		if(status & BQ_STAT_SCD){
			debug_sends(&debug_port,(uint8_t*)"SCD-");
		}

		if(status & BQ_STAT_OCD){
			debug_sends(&debug_port,(uint8_t*)"OCD-");
		}

		if(status & BQ_STAT_XREADY){
			debug_sends(&debug_port,(uint8_t*)"XREADY-");
		}

		afe_reset_error((AFE*)p_bq);
	}
	else{
		debug_sends(&debug_port,(uint8_t*)"OK");
	}
}
