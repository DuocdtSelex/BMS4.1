/*
 * inrush_limiter_init.c
 *
 *  Created on: Aug 25, 2020
 *      Author: quangnd
 */

#include "inrush_limiter_hal.h"
#include "inrush_limiter.h"

Inrush_Limiter* inrush_limiter;

static void inrush_sw_on_impl(Switch* p_sw);
static void inrush_sw_off_impl(Switch* p_sw);

static void inrush_limiter_active_impl(Inrush_Limiter* p_lmt);
static void inrush_limiter_stop_impl(Inrush_Limiter* p_lmt);

static Switch ic_sw;

void inrush_limiter_init(void){
	inrush_limiter=inrush_limiter_create();
	inrush_limiter->active=inrush_limiter_active_impl;
	inrush_limiter->stop=inrush_limiter_stop_impl;
	ic_sw.sw_on=inrush_sw_on_impl;
	ic_sw.sw_off=inrush_sw_off_impl;
	inrush_limiter->p_switch=&ic_sw;
}

static void inrush_sw_on_impl(Switch* p_sw){
	p_sw->state=SW_ST_ON;
	inrush_limiter_switch_on();
}

static void inrush_sw_off_impl(Switch* p_sw){
	p_sw->state=SW_ST_OFF;
	inrush_limiter_switch_off();
}

static void inrush_limiter_active_impl(Inrush_Limiter* p_lmt){

	sw_on(p_lmt->p_switch);
}

static void inrush_limiter_stop_impl(Inrush_Limiter* p_lmt){
	sw_off(p_lmt->p_switch);
}
