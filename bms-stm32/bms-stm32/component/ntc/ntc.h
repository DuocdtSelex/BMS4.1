/*
 * ntc.h
 *
 *  Created on: Sep 30, 2020
 *      Author: quangnd
 */

#ifndef COMPONENT_NTC_NTC_H_
#define COMPONENT_NTC_NTC_H_

#include "stdint.h"

typedef struct NTC_t NTC;

struct NTC_t{
	const uint16_t* lut;
	uint16_t lut_size;
	int16_t min_temp;
	int16_t max_temp;
	int16_t temp;
	uint16_t impedance;
	void (*update_impedance)(NTC* p_ntc);
	void (*update_temperature)(NTC* p_ntc);
};

static inline void ntc_update_impedance(NTC* p_ntc){
	p_ntc->update_impedance(p_ntc);
}

static inline void ntc_update_temperature(NTC* p_ntc){
	p_ntc->update_temperature(p_ntc);
}

static inline uint16_t ntc_get_impedance(const NTC* const p_ntc){
	return p_ntc->impedance;
}

static inline void ntc_set_lut(NTC* p_ntc,const uint16_t* const p_lut){
	p_ntc->lut=p_lut;
}

static inline int16_t ntc_get_temp(const NTC* const p_ntc){
	return p_ntc->temp;
}


void ntc_update_temp(NTC* p_ntc);
void ntc_update_temp_from_bq(NTC* p_ntc);

#endif /* COMPONENT_NTC_NTC_H_ */
