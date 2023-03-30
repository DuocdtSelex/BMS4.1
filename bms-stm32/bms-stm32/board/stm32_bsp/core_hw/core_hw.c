/*
 * core_hw.c
 *
 *  Created on: Sep 16, 2020
 *      Author: quangnd
 */

#include "stm32f0xx.h"
#include "core_hw.h"
#include "string_util.h"

#define DEV_ID_BASE_ADDR                        0x1FFFF7AC

void core_hw_init(void){

	SystemInit();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/SYSTICK_FREQ_Hz);
}

int32_t core_read_id(char* id){
        char buff[12]={0};

        char* base=(char*)(DEV_ID_BASE_ADDR);
        for(int i=0;i<12;i++){
                buff[i]=*(base+i);
        }

        for(int i=0;i<5;i++){
                byte_to_hex_ascii(buff[i],id+2*i);
        }
        for(int i=0;i<7;i++){
                id[10+i]=buff[5+i];
        }
        return 17;
}
