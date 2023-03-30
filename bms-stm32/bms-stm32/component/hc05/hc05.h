/*
 * hc05.h
 *
 *  Created on: Nov 3, 2020
 *      Author: quangnd
 */

#ifndef COMPONENT_HC05_HC05_H_
#define COMPONENT_HC05_HC05_H_
#include "hc05_hal.h"

typedef struct Hc05_t Hc05;

struct Hc05_t{
        HC05_HW* p_hw;
};

void hc05_init(Hc05* p_hc05, HC05_HW* p_hw);
static inline void hc05_sends(Hc05* p_hc,const uint8_t* s){
        hc05_hw_sends(p_hc->p_hw,s);
}

#endif /* COMPONENT_HC05_HC05_H_ */
