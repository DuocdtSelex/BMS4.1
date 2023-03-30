

#ifndef COMPONENT_ADC_ADC_H_
#define COMPONENT_ADC_ADC_H_

#include "current_sense_hal.h"
#include "ntc.h"

int32_t* adc_get_coulombcounter(void);
void adc_init(void);
int32_t* adc_get_current();
uint8_t adc_get_temperature();

#endif /* COMPONENT_ADC_ADC_H_ */
