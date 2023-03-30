/*
 * led_init.c
 *
 *  Created on: Nov 16, 2020
 *      Author: quangnd
 */
#include "rgb_led.h"
#include "rgb_led_hal.h"
RGB_Led indicator_led;
static void rgb_led_on_impl(RGB_Led* p_led,RGB_Color color);
static void rgb_led_off_impl(RGB_Led* p_led);
static void rgb_led_blink_impl(RGB_Led* p_led, const uint32_t interval_ms);

void indicator_led_init(void){
        indicator_led.led_blink=rgb_led_blink_impl;
        indicator_led.led_off=rgb_led_off_impl;
        indicator_led.led_on=rgb_led_on_impl;
}

static void rgb_led_on_impl(RGB_Led* p_led,RGB_Color color){
        p_led->color.r_luminance=color.r_luminance;
        p_led->color.g_luminance=color.g_luminance;
        p_led->color.b_luminance=color.b_luminance;
        rgb_set_color(color.r_luminance,color.g_luminance,color.b_luminance);
}

static void rgb_led_off_impl(RGB_Led* p_led){
        p_led->color.r_luminance=0;
        p_led->color.g_luminance=0;
        p_led->color.b_luminance=0;
        rgb_set_color(0,0,0);
}

static void rgb_led_blink_impl(RGB_Led* p_led, const uint32_t interval_ms){
        p_led->blink_interval=interval_ms;
}
