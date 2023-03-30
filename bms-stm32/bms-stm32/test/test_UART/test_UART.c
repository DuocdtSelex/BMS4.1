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


#include "debug_com_port_hal.h"
#include "delay_hw.h"
#include "core_hal.h"

const char* test_str="Selex BMS\n";

static void test_setup(void){
	core_hw_init();
	debug_com_hw_init();
}

int main(void)
{
	hw_delay_ms(500);
	test_setup();
	while(1){
		debug_sends(&debug_port,(uint8_t*)test_str);
		hw_delay_ms(1000);
	}
	return 0;
}

void SysTick_Handler(void){

}
