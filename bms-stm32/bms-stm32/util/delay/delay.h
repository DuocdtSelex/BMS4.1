/*
 * delay.h
 *
 *  Created on: Aug 21, 2020
 *      Author: quangnd
 */

#ifndef UTIL_DELAY_DELAY_H_
#define UTIL_DELAY_DELAY_H_
#include "board.h"

static inline void delay_us(uint32_t us){
	hw_delay_us(us);
}

static inline void delay_ms(uint32_t ms){
	hw_delay_ms(ms);
}
#endif /* UTIL_DELAY_DELAY_H_ */
