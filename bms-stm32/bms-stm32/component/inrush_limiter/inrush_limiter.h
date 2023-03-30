/*
 * inrush_limiter.h
 *
 *  Created on: Aug 25, 2020
 *      Author: quangnd
 */

#ifndef COMPONENT_INRUSH_LIMITER_INRUSH_LIMITER_H_
#define COMPONENT_INRUSH_LIMITER_INRUSH_LIMITER_H_
#include "switch.h"
#include "stdint.h"

typedef struct Inrush_Limiter_t Inrush_Limiter;

struct Inrush_Limiter_t{

	Switch* p_switch;
	uint8_t is_bypass;
	void (*active)(Inrush_Limiter* p_lmt);
	void (*stop)(Inrush_Limiter* p_lmt);
};

Inrush_Limiter* inrush_limiter_create(void);

void inrush_limiter_set_switch(Inrush_Limiter* p_lmt,Switch* p_sw);

static inline void inrush_limiter_active(Inrush_Limiter* p_lmt){
	p_lmt->active(p_lmt);
}

static inline void inrush_limiter_stop(Inrush_Limiter* p_lmt){
	p_lmt->stop(p_lmt);
}

#endif /* COMPONENT_INRUSH_LIMITER_INRUSH_LIMITER_H_ */
