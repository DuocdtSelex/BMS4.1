/*
 * test_UART.c
 *
 *  Created on: Sep 9, 2020
 *      Author: son19
 */

/*		Description
 * 	Gui du lieu thong qua UART voi chu ki 250 1 lan
 * 	Du lieu duoc gui la 0x0A
 *	hw_delay_ms(250) va sua doi trong hw_delay_ms : hw_delay_us(80 -> 8)
 */


#include "hc05_hw.h"
#include "delay_hw.h"
#include "core_hal.h"

const char* test_str="Selex BMS\n";

static void hc05_receive_impl(const char c);

static void test_setup(void){
	core_hw_init();
	hc05_hw_init();
	hc05_set_receive_handle(hc05_receive_impl);
	global_interrupt_enable();
}

int main(void)
{
	hw_delay_ms(500);
	test_setup();
	while(1){
		hc05_hw_sends(&hc05_port,(uint8_t*)test_str);
		hw_delay_ms(1000);
	}
	return 0;
}

static void hc05_receive_impl(const char c){
        hc05_send(&hc05_port,c);
}

void SysTick_Handler(void){

}
