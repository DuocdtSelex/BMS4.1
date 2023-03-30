/*
 * keypad.c
 *
 *  Created on: Oct 20, 2020
 *      Author: quangnd
 */

#include "keypad.h"
void key_update_state(Keypad* p_key){

        KEY_STATE state=p_key->read(p_key);
        p_key->debounce_counter= (p_key->debounce_counter<<1) + (uint32_t) state;
        /*  if the key state is stable during 32 reads ( all bit of debounce counter is 1 */
        if(p_key->debounce_counter==0xFFFFFFFF){
                /* if old state is pressed */
                if(p_key->state==KEY_ST_PRESSED){
                        /* key hold event */
                        p_key->hold_time++;
                }
                else{
                        /* key press event */
                        p_key->hold_time=0;
                        p_key->state=KEY_ST_PRESSED;
                }
        }

        if(p_key->debounce_counter==0x00000000){
                p_key->hold_time=0;
                p_key->state=KEY_ST_RELEASED;
        }
}


