#ifndef BOARD_ADC_HARDWARE_H_
#define BOARD_ADC_HARDWARE_H_

#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"
#include "stm32f0xx_tim.h"
#include "string.h"
#include "math.h"

#define ADC_NUM_CHANNEL					2
#define ADC_TS5_ID						0
#define ADC_TS6_ID						1
//#define ADC_TEMPERATURE_ID				2


extern __IO uint32_t filtered_adc_values[ADC_NUM_CHANNEL];

#define ADC_VREF_mV						2500UL
#define CURRENT_SENSE_ADC_GAIN			1UL
#define CURRENT_SENSE_ADC_OFFSET		0UL
#define CURRENT_SENSE_R_SHUNT_mOhm			1

#if 0
int32_t current_sense_read_adc(void);
int32_t current_sense_read_offset(void);
int32_t current_sense_calculate_gain(void);
void current_sense_hardware_init(void);
#endif

int32_t ts5_temp_sense_read_adc(void);
int32_t ts6_temp_sense_read_adc(void);
uint16_t ts5_ntc_read_impedance(void);
uint16_t ts6_ntc_read_impedance(void);
void temp_sense_hardware_init(void);

#endif /* BOARD_ADC_HARDWARE_H_ */
