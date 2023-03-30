#include "stm32f0xx.h"
#include "adc.h"
#include "debug_com_port_hardware.h"

char* buffer[10];

typedef struct Current_t Current_t;

struct Current_t
{
	int32_t adc;
	int32_t result;
	int32_t offset;
	int32_t gain;
	void(*update_adc)(Current_t* p_cur);
	void (*update_offset)(Current_t* p_cur);
	void (*update_result)(void);

};

Current_t current;

void ADC_sensor_update(void)
{
	current.update_adc(&current);
	current.update_offset(&current);
	current.result = (current.adc - current.offset)*current.gain;
}

void read_adc(Current_t* p_cur)
{
	p_cur->adc = current_sense_read_adc();
}

void read_offset(Current_t* p_cur)
{
	p_cur->offset = current_sense_read_offset();
}

void ADC_sensor_init(void)
{
	current.update_adc = read_adc;
	current.gain = current_sense_calculate_gain;
	current.update_offset = read_offset;
	current.update_result = ADC_sensor_update;
}

void System_Init(void)
{
	current_sense_hardware_init();
	debug_com_hw_init();
	ADC_sensor_init();
}

int main()
{
	SystemInit();

	while (1){
		debug_sends(&debug_port,"CURRENT: ");
		itoa(current.result, buffer, 10);
		debug_sends(&debug_port,buffer);
		debug_sends(&debug_port,"\n\r ");


	}
	return 1;
}
