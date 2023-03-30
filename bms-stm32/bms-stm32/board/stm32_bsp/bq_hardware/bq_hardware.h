#if 0
#ifndef AFE_HARDWARE_H_
#define AFE_HARDWARE_H_

#include "stm32f0xx_exti.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx.h"


typedef struct BQ_Hw_t BQ_Hw;
typedef void (*BQ_Interrupt_Handle)(BQ_Hw* p_bq);

struct BQ_Hw_t{
	I2C_TypeDef* i2c_module;
	uint8_t i2c_address;
	void (*reset_i2c_bus)(void);
	BQ_Interrupt_Handle handle;
};

extern struct BQ_Hw_t upper_bq_hw;
extern struct BQ_Hw_t lower_bq_hw;

int afe_hardware_init(void);

static inline void bq_set_interrupt_handle(BQ_Hw* p_hw,BQ_Interrupt_Handle handle){
	p_hw->handle=handle;
}
int32_t bq_read_reg_block_with_crc(BQ_Hw* p_bq,uint8_t reg, uint8_t *buffer, uint8_t len);
int32_t bq_read_reg_byte_with_crc(BQ_Hw* p_bq,uint8_t reg, uint8_t *data);
int32_t bq_read_reg_word_with_crc(BQ_Hw* p_bq,uint8_t reg, uint16_t *data);
int32_t bq_write_reg_block_with_crc(BQ_Hw* p_bq,uint8_t start, uint8_t *buffer, uint8_t len);
int32_t bq_write_reg_word_with_crc(BQ_Hw* p_bq,uint8_t reg, uint32_t data);
int32_t bq_write_reg_byte_with_crc(BQ_Hw* p_bq,uint8_t reg, uint8_t data);



#define AFE_I2C_ADDRESS					(0x08<<1)

#define AFE1_I2C_CLK                   	RCC_APB1Periph_I2C1

#define AFE1_I2C_SCL_PIN               	GPIO_Pin_6
#define AFE1_I2C_SCL_PORT         		GPIOB
#define AFE1_I2C_SCL_CLK          		RCC_AHBPeriph_GPIOB
#define AFE1_I2C_SCL_SOURCE            	GPIO_PinSource6
#define AFE1_I2C_SCL_AF                	GPIO_AF_1

#define AFE1_I2C_SDA_PIN              	GPIO_Pin_7
#define AFE1_I2C_SDA_PORT        		GPIOB
#define AFE1_I2C_SDA_CLK         		RCC_AHBPeriph_GPIOB
#define AFE1_I2C_SDA_SOURCE           	GPIO_PinSource7
#define AFE1_I2C_SDA_AF               	GPIO_AF_1

#define AFE1_ALERT_PORT					GPIOB
#define AFE1_ALERT_PIN					GPIO_Pin_5
#define AFE1_ALERT_CLK					RCC_AHBPeriph_GPIOB

#define AFE1_I2C_DEVICE 				I2C1
#define AFE1_INTERRUPT_CHANNEL			5
#define AFE1_INTERRUPT_VECTOR			EXTI4_15_IRQn
#define AFE1_INTERRUPT_LINE				EXTI_Line5
#define AFE1_INTERRUPT_PRIORITY			        3


#define AFE2_I2C_CLK                   	RCC_APB1Periph_I2C2

#define AFE2_I2C_SCL_PIN               	GPIO_Pin_10
#define AFE2_I2C_SCL_PORT         		GPIOB
#define AFE2_I2C_SCL_CLK          		RCC_AHBPeriph_GPIOB
#define AFE2_I2C_SCL_SOURCE            	GPIO_PinSource10
#define AFE2_I2C_SCL_AF                	GPIO_AF_1

#define AFE2_I2C_SDA_PIN              	GPIO_Pin_11
#define AFE2_I2C_SDA_PORT        		GPIOB
#define AFE2_I2C_SDA_CLK         		RCC_AHBPeriph_GPIOB
#define AFE2_I2C_SDA_SOURCE           	GPIO_PinSource11
#define AFE2_I2C_SDA_AF               	GPIO_AF_1


#define AFE2_ALERT_PORT					GPIOA
#define AFE2_ALERT_PIN					GPIO_Pin_11
#define AFE2_ALERT_CLK					RCC_AHBPeriph_GPIOA

#define AFE2_I2C_DEVICE 				I2C2
#define AFE2_INTERRUPT_CHANNEL			11
#define AFE2_INTERRUPT_VECTOR			EXTI4_15_IRQn
#define AFE2_INTERRUPT_LINE				EXTI_Line11
#define AFE2_INTERRUPT_PRIORITY			3


#define PERCENT_LED_BLUE_PIN			GPIO_Pin_2
#define PERCENT_LED_BLUE_PORT			GPIOB
#define PERCENT_LED_BLUE_CLK			RCC_AHBPeriph_GPIOB

#define PERCENT_LED_RED_PIN				GPIO_Pin_3
#define PERCENT_LED_RED_PORT			GPIOB
#define PERCENT_LED_RED_CLK				RCC_AHBPeriph_GPIOB

#define PERCENT_LED_GREEN_PIN			GPIO_Pin_12
#define PERCENT_LED_GREEN_PORT			GPIOA
#define PERCENT_LED_GREEN_CLK			RCC_AHBPeriph_GPIOA

#define PUSH_PIN               			GPIO_Pin_10
#define PUSH_PORT         		    	GPIOA
#define PUSH_CLK          		    	RCC_AHBPeriph_GPIOA

#define TEST_SW_PIN						GPIO_Pin_13
#define TEST_SW_PORT					GPIOC
#define TEST_SW_CLK						RCC_AHBPeriph_GPIOC

#define AFE_TIMER_DEVICE				TIM7

typedef enum BTN_STATE BTN_STATE;

enum BTN_STATE {
	BTN_ON = 0x00, BTN_OFF = 0x01
};

#endif
#endif
