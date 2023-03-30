#ifndef _BOARD_H_
#define _BOARD_H_
#include "bq_hal.h"
#include "current_sense_hal.h"
#include "interrupt_hal.h"
#include "hc05_hal.h"
#include "soc_hal.h"

void board_init(void) WEAK;

void hw_delay_us(uint32_t us) WEAK;
void hw_delay_ms(uint32_t ms) WEAK;

#endif
