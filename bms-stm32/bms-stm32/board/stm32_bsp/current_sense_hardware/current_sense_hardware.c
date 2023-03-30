#include "current_sense_hardware.h"
#define ADC1_DR_Address    				0x40012440              //ADC1->DR
#define ADC_TS6_PIN						GPIO_Pin_1
#define ADC_TS6_PORT					GPIOA
#define ADC_TS6_CHANNEL					ADC_Channel_1
#define ADC_TS6_CLK						RCC_AHBPeriph_GPIOA

#define ADC_TS5_PIN						GPIO_Pin_0
#define ADC_TS5_PORT					GPIOA
#define ADC_TS5_CHANNEL					ADC_Channel_0
#define ADC_TS5_CLK						RCC_AHBPeriph_GPIOA

//#define ADC_TEMP_PIN					GPIO_Pin_0
//#define ADC_TEMP_PORT					GPIOB
//#define ADC_TEMP_CHANNEL				ADC_Channel_8
//#define ADC_TEMP_CLK					RCC_AHBPeriph_GPIOB

#define ADC_DEVICE						ADC1
#define ADC_DEVICE_CLK					RCC_APB2Periph_ADC1

#define DMA_DEVICE						DMA1_Channel1
#define DMA_DEVICE_CLK					RCC_AHBPeriph_DMA1

#define ADC_TIM_DEV						TIM2
#define TIM_PRESCALER					23
#define TIM_PERIOD						999
#define ADC_TIM_DEV_CLK					RCC_APB1Periph_TIM2
#define ADC_TIM_INTERRUPT_PRIORITY		        1
#define ADC_NUM_FILTERS					32

#define ADC_UPDATE_HANDLE				TIM2_IRQHandler

#define    						VREF_ADR         ((uint16_t*)((uint32_t)0x1FFFF7BA))
#define 						FFT			0

#define ADC_VREF_uV							3300000
#define ADC_RESOLUTION						12
#define CURRENT_SENSE_OPAM_GAIN				20

static __IO uint16_t current_adc_sample[ADC_NUM_CHANNEL] = {0};
__IO uint32_t filtered_adc_values[ADC_NUM_CHANNEL] = {0};

volatile uint8_t filter_counter = 0;
uint16_t adc_factor_value = 0;
static uint8_t channel = 0;

static void adc_hardware_clock_init(void);
static void adc_hardware_gpio_init(void);
static void adc_hardware_dma_init(void);
static void adc_hardware_adc_init(void);
static void adc_hardware_timer_init(void);

void temp_sense_hardware_init(void){
	adc_hardware_clock_init();
	adc_hardware_gpio_init();
	adc_hardware_dma_init();
	adc_hardware_adc_init();
	adc_hardware_timer_init();
}

int32_t ts5_temp_sense_read_adc(void){
	return (int32_t)filtered_adc_values[ADC_TS5_ID];
}

int32_t ts5_temp_sense_read_offset(void){
	return 0;
}

int32_t ts6_temp_sense_read_adc(void){
	return (int32_t)filtered_adc_values[ADC_TS6_ID];
}

int32_t ts6_temp_sense_read_offset(void){
	return 0;
}

uint16_t ts5_ntc_read_impedance(void){
    int32_t temp_sense_adc = 0;
    temp_sense_adc = filtered_adc_values[ADC_TS5_ID];
	uint32_t v_temp_fet_uV=805*(uint32_t)temp_sense_adc/1000; // 1 LSB approximate 805uV
	uint32_t v_ref_uV=3300;
	if(v_temp_fet_uV==v_ref_uV){
		return 0;
	}
	return (10000*v_temp_fet_uV)/(v_ref_uV-v_temp_fet_uV);
}

uint16_t ts6_ntc_read_impedance(void){
    int32_t temp_sense_adc = 0;
    temp_sense_adc = filtered_adc_values[ADC_TS6_ID];
	uint32_t v_temp_fet_uV=805*(uint32_t)temp_sense_adc/1000; // 1 LSB approximate 805uV
	uint32_t v_ref_uV=3300;
	if(v_temp_fet_uV==v_ref_uV){
		return 0;
	}
	return (10000*v_temp_fet_uV)/(v_ref_uV-v_temp_fet_uV);
}
#if 0
int32_t current_sense_calculate_gain(void){

	int32_t adc_div_vol=(ADC_VREF_uV>>ADC_RESOLUTION);
	return (adc_div_vol/(CURRENT_SENSE_OPAM_GAIN*CURRENT_SENSE_R_SHUNT_mOhm));
}
#endif

void ADC_UPDATE_HANDLE(void) {

	if(TIM_GetITStatus(ADC_TIM_DEV, TIM_IT_Update) != RESET) {
			while (channel < ADC_NUM_CHANNEL) {
				filtered_adc_values[channel] = (uint32_t)current_adc_sample[channel];
				channel++;
			}
			channel = 0;
			TIM_ClearFlag(ADC_TIM_DEV, TIM_FLAG_Update);
	}
}

static void adc_hardware_clock_init(void) {
	RCC_APB2PeriphClockCmd(ADC_DEVICE_CLK | RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(ADC_TS6_CLK | ADC_TS5_CLK| DMA_DEVICE_CLK,ENABLE);
	RCC_APB1PeriphClockCmd(ADC_TIM_DEV_CLK,ENABLE);
}

static void adc_hardware_gpio_init(void){


	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = ADC_TS6_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(ADC_TS6_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ADC_TS5_PIN;
	GPIO_Init(ADC_TS5_PORT, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = ADC_TEMP_PIN;
//    GPIO_Init(ADC_TEMP_PORT, &GPIO_InitStructure);
}

static void adc_hardware_adc_init(void){
	ADC_InitTypeDef     ADC_InitStructure;

	/* ADCs DeInit */
	ADC_DeInit(ADC_DEVICE);

	ADC_StructInit(&ADC_InitStructure);
	/* Initialize ADC structure */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC_DEVICE,&ADC_InitStructure);

    ADC_ChannelConfig(ADC_DEVICE,ADC_TS6_CHANNEL,ADC_SampleTime_1_5Cycles);
	ADC_ChannelConfig(ADC_DEVICE,ADC_TS5_CHANNEL, ADC_SampleTime_1_5Cycles);
//	ADC_ChannelConfig(ADC_DEVICE,ADC_TEMP_CHANNEL, ADC_SampleTime_1_5Cycles);

	ADC_GetCalibrationFactor(ADC_DEVICE);

    ADC_DMARequestModeConfig(ADC_DEVICE,ADC_DMAMode_Circular);
    ADC_DMACmd(ADC_DEVICE,ENABLE);
	  /* Enable the ADC peripheral */
	ADC_Cmd(ADC_DEVICE, ENABLE);
	  /* Wait the ADRDY flag */
	while(!ADC_GetFlagStatus(ADC_DEVICE, ADC_FLAG_ADRDY));
	ADC_StartOfConversion(ADC_DEVICE);
}


static void adc_hardware_dma_init(void){

	  DMA_InitTypeDef   DMA_InitStructure;

	  /* DMA1 clock enable */
	  RCC_AHBPeriphClockCmd(DMA_DEVICE_CLK , ENABLE);

	  /* DMA1 Channel1 Config */
	  DMA_DeInit(DMA_DEVICE);
	  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&ADC_DEVICE->DR);   //(uint32_t) ADC1_DR_Address;
	  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)current_adc_sample;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	  DMA_InitStructure.DMA_BufferSize = ADC_NUM_CHANNEL;
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	  DMA_Init(DMA_DEVICE, &DMA_InitStructure);
	  /* DMA1 Channel1 enable */
	  DMA_Cmd(DMA_DEVICE,ENABLE);

}

static void adc_hardware_timer_init(void){

	TIM_TimeBaseInitTypeDef TimBase_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	TimBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimBase_InitStructure.TIM_Prescaler  = TIM_PRESCALER;
	TimBase_InitStructure.TIM_Period = TIM_PERIOD;
	TimBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimBase_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(ADC_TIM_DEV,&TimBase_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPriority = ADC_TIM_INTERRUPT_PRIORITY;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearFlag(ADC_TIM_DEV,TIM_FLAG_Update);
	TIM_ITConfig(ADC_TIM_DEV,TIM_IT_Update,ENABLE);
	TIM_Cmd(ADC_TIM_DEV,ENABLE);
}
