#include "board.h"
#include "stm32f0xx.h"
#include "can_hardware.h"
#include "stm32f0xx_it.h"
#include "bq_hardware.h"
#include "inrush_limiter_hardware.h"
#include "hc05_hw.h"
#include "core_hw.h"
#include "rgb_hw.h"
#include "key_hw.h"
#include "node_id_hw.h"
#include "soc_hw.h"
#include "bq_hw.h"

void board_init() {
        core_hw_init();
        afe_hardware_init_1();
        hc05_hw_init();
        temp_sense_hardware_init();
        inrush_limter_hardware_init();
        can_hardware_init();
        key_hw_init();
        rgb_hw_init();
        node_id_hw_init();
        soc_hw_init();
}

void global_interrupt_enable() {
        __enable_irq();
}
void global_interrupt_disable(){
	__disable_irq();
}

