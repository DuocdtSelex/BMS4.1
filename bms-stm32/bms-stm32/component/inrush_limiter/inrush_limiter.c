/*
 * inrush_limiter.c
 *
 *  Created on: Aug 25, 2020
 *      Author: quangnd
 */
#include "inrush_limiter.h"
#include "stdlib.h"

Inrush_Limiter* inrush_limiter_create(void){

	Inrush_Limiter* this=(Inrush_Limiter*)malloc(sizeof(Inrush_Limiter));
	while(this==NULL){};
	return this;
}

void inrush_limiter_set_switch(Inrush_Limiter* p_lmt,Switch* p_sw){

	p_lmt->p_switch=p_sw;
}


