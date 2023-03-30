/*
 * switch.h
 *
 *  Created on: Aug 21, 2020
 *      Author: quangnd
 */

#ifndef COMPONENT_SWITCH_SWITCH_H_
#define COMPONENT_SWITCH_SWITCH_H_

typedef enum SW_STATE
{
	SW_ST_ON,
	SW_ST_OFF
}SW_STATE;

typedef struct Switch_t Switch;
struct Switch_t{
	SW_STATE state;
	void (*sw_on)(Switch* p_sw);
	void (*sw_off)(Switch* p_sw);
	void (*sw_init)(Switch* p_sw);
};

static inline void sw_on(Switch* p_sw){
	p_sw->sw_on(p_sw);
	p_sw->state=SW_ST_ON;
}

static inline void sw_off(Switch* p_sw){
	p_sw->sw_off(p_sw);
	p_sw->state=SW_ST_OFF;
}

static inline SW_STATE sw_get_state(Switch* p_sw){
	return p_sw->state;
}

#endif /* COMPONENT_SWITCH_SWITCH_H_ */
