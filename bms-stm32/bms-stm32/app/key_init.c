/*
 * key_init.c
 *
 *  Created on: Oct 20, 2020
 *      Author: quangnd
 */

#include "keypad.h"
#include "key_init.h"
#include "app_config.h"
#define KEY_HOLD_THRESHOLD_mS                           2000

Keypad user_key;

static KEY_STATE user_key_read_impl(Keypad* p_key);
void user_key_init(void){
        key_init(&user_key);
        user_key.read=user_key_read_impl;
        user_key.hold_threshold=KEY_HOLD_THRESHOLD_mS/APP_STATE_MACHINE_UPDATE_TICK_mS;
}


static KEY_STATE user_key_read_impl(Keypad* p_key){

        uint8_t current_state=key_read();
        if(current_state)
                return KEY_ST_PRESSED;

        return KEY_ST_RELEASED;
}
