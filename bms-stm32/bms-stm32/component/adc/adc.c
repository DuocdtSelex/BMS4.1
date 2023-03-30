#include "adc.h"

#include "../../service/bms/bms.h"
#include "../../board/stm32_bsp/current_sense_hardware/current_sense_hardware.h"


int32_t adc_coulombcounter = 86400000;
int32_t current_value = 10;
static uint32_t adc_convert_vol_to_mvol(float vol);

float vref = 0.0,current = 0.0,ntc = 0.0;
float adc_voltage[ADC_NUM_CHANNEL] = {0.0};
static const float R_Shunt = 0.001;
static const float vcc = 3.3;
static const float adc_resoloution = 4095.0;
static const uint8_t gain = 20;
static const float read_adc_fre  = 0.016; // 32 lan ngat 0.5 ms TIM7
static const float ee_gain_charge = 0.995;
static const float ee_gain_discharge = 1.0;
float ee_gain = 0.995;
static void adc_read_value(uint32_t* adc_values);
static float adc_get_vref(void);
static int32_t adc_caculate_current(void);
static volatile uint8_t channel = 0;


void adc_init(void){
	adc_set_update_handle(adc_read_value);
}

int32_t* adc_get_coulombcounter(void){
	return (&adc_coulombcounter);
}

int32_t* adc_get_current(void){
  return (&current_value);
}

static uint32_t adc_convert_vol_to_mvol(float vol){
	return (uint32_t)(vol*1000);
}

static void adc_read_value(uint32_t* adc_values) {
	while (channel < ADC_NUM_CHANNEL) {
#if FFT
		adc_voltage[channel] = (float) sqrt((adc_values[channel] >> adc_shift_values))* vcc
		/ adc_resoloution;
#endif
#if !FFT
		adc_voltage[channel] = (float) ((float) adc_values[channel]
				/ ADC_NUM_FILTERS) * vcc / adc_resoloution;
#endif
		adc_values[channel] = 0;
		channel++;
	}
	channel = 0;
	current_value =  adc_caculate_current();
	if (current_value > 0) {
		ee_gain = ee_gain_charge;
	} else {
		ee_gain = ee_gain_discharge;
	}
	//adc_coulombcounter += (int32_t) (adc_convert_vol_to_mvol(adc_voltage[ADC_CURRENT_ID]) * read_adc_fre);
	adc_coulombcounter += (int32_t) current_value*read_adc_fre;
}
static float adc_get_vref(void){
     return(adc_voltage[ADC_VOLTAGE_ID]);
}

static int32_t adc_caculate_current(void){
	return ((int32_t)1000*(adc_get_vref()-adc_voltage[ADC_TS5_ID])/R_Shunt/gain);
}
uint8_t adc_caculate_temperature(){
	uint8_t id = 0;
     uint16_t ntc = (uint16_t)1000*(10*(adc_voltage[ADC_TEMPERATURE_ID])/(vcc-adc_voltage[ADC_TEMPERATURE_ID]));
     while(ntc < mf58_lookups[id++]){
     }
	return(id+NTC_LOOKUPS_MIN_TEMPERATURE);
}

