/*
 * rgb_led.h
 *
 *  Created on: Aug 22, 2020
 *      Author: quangnd
 */

#ifndef COMPONENT_RGB_LED_RGB_LED_H_
#define COMPONENT_RGB_LED_RGB_LED_H_
#include "rgb_led_hal.h"
#include "stdint.h"

typedef struct RGB_Led_t RGB_Led;
typedef struct RGB_Color_t RGB_Color;
struct RGB_Color_t{
	uint8_t r_luminance;
	uint8_t g_luminance;
	uint8_t b_luminance;

};

struct RGB_Led_t{
	RGB_Color color;
	void (*led_on)(RGB_Led* p_led, RGB_Color color);
	void (*led_off)(RGB_Led* p_led);
	void (*led_blink)(RGB_Led* p_led,const uint32_t interval_ms);
};

static inline void rgb_led_on(RGB_Led* p_led,RGB_Color color){
	p_led->led_on(p_led,color);
}

static inline void rgb_led_off(RGB_Led* p_led){
	p_led->led_off(p_led);
}

static inline void rgb_led_blink(RGB_Led* p_led, const uint32_t interval_ms){
	p_led->led_blink(p_led,interval_ms);
}

#endif /* COMPONENT_RGB_LED_RGB_LED_H_ */
