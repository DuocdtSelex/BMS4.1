/*
 * keypad.h
 *
 *  Created on: Oct 20, 2020
 *      Author: quangnd
 */

#ifndef COMPONENT_KEYPAD_KEYPAD_H_
#define COMPONENT_KEYPAD_KEYPAD_H_
#include "key_hal.h"

typedef enum KEY_STATE_t{
        KEY_ST_PRESSED=0,
        KEY_ST_RELEASED=1,
        KEY_ST_HOLD=2
}KEY_STATE;
typedef struct Keypad_t Keypad;

struct Keypad_t{
        KEY_STATE state;
        KEY_STATE (*read)(Keypad* p_key);
        uint32_t hold_threshold;
        uint32_t hold_time;
        uint32_t debounce_counter;
};

static inline void key_init(Keypad* p_key){
        /* normal open button */
        p_key->state=KEY_ST_RELEASED;
        p_key->debounce_counter=0;
}

void key_update_state(Keypad* p_key);
static inline KEY_STATE keypad_get_state(Keypad* p_key){
        return p_key->state;
}

static inline void keypad_reset_hold_state(Keypad* p_key){
        p_key->hold_time=0;
}

static inline uint8_t keypad_is_hold(Keypad* p_key){
        return (p_key->hold_time>=p_key->hold_threshold);
}


#endif /* COMPONENT_KEYPAD_KEYPAD_H_ */
