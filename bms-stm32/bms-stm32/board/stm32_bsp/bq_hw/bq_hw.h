/*
 * bq_hw.h
 *
 *  Created on: Apr 13, 2021
 *      Author: Admin
 */

#ifndef BOARD_STM32_BSP_BQ_HW_BQ_HW_H_
#define BOARD_STM32_BSP_BQ_HW_BQ_HW_H_

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

extern struct BQ_Hw_t bq_hw;


int afe_hardware_init_1(void);

static inline void bq_set_interrupt_handle_1(BQ_Hw* p_hw,BQ_Interrupt_Handle handle){
	p_hw->handle=handle;
}

/* read/write data for bq769x2
 * read function
*/
int32_t bq_i2c_read_reg_byte(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer);
int32_t bq_i2c_read_reg_word(BQ_Hw* p_bq, uint8_t reg_addr, uint16_t* buffer);
int32_t bq_i2c_read_reg_word_with_sign(BQ_Hw* p_bq, uint8_t reg_addr, int16_t* buffer);
int32_t bq_i2c_read_reg_block(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length);
int32_t bq_i2c_read_reg_block_with_sign(BQ_Hw* p_bq, uint8_t reg_addr, int8_t* buffer, uint8_t length);
int32_t bq_i2c_read_reg_block_with_crc(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length);
int32_t bq_i2c_data_ram_read(BQ_Hw* p_bq,  uint16_t reg_addr, uint8_t* buffer, uint8_t length);
int32_t bq_i2c_subcommand_read_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t *buffer);
int32_t bq_i2c_subcommand_read_block(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t* buffer, uint8_t length);
int32_t bq_i2c_subcommand_read_block_with_sign(BQ_Hw* p_bq, uint16_t reg_addr, int8_t* buffer, uint8_t length);
int32_t bq_i2c_data_ram_read_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t* buffer);
int32_t bq_i2c_data_ram_read_byte(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t* buffer);
int32_t bq_i2c_data_ram_read_word_with_sign(BQ_Hw* p_bq, uint16_t reg_addr, int16_t *buffer);
int32_t bq_i2c_data_ram_read_reg_with_float(BQ_Hw* p_bq, uint16_t reg_addr, float *buffer);
// write function
int32_t bq_i2c_write_reg_byte(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t reg_data);
int32_t bq_i2c_write_reg_word(BQ_Hw* p_bq, uint8_t reg_addr, uint16_t reg_data);
int32_t bq_i2c_write_reg_block(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length);
int32_t bq_i2c_wirte_reg_block_with_crc(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length);
int32_t bq_i2c_data_ram_write(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t* buffer, uint8_t length);
int32_t bq_i2c_data_ram_write_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t reg_data);
int32_t bq_i2c_data_ram_write_word_with_sign(BQ_Hw* p_bq, uint16_t reg_addr, int16_t reg_data);
int32_t bq_i2c_data_ram_write_byte(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t reg_data);
int32_t bq_i2c_subcommand_write_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t reg_data);
int32_t bq_i2c_subcommand_write_byte(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t reg_data);

#define AFE_I2C_ADDRESS					0x10

#define AFE_I2C_CLK                   	RCC_APB1Periph_I2C1

#define AFE_I2C_SCL_PIN               	GPIO_Pin_6
#define AFE_I2C_SCL_PORT         		GPIOB
#define AFE_I2C_SCL_CLK          		RCC_AHBPeriph_GPIOB
#define AFE_I2C_SCL_SOURCE            	GPIO_PinSource6
#define AFE_I2C_SCL_AF                	GPIO_AF_1

#define AFE_I2C_SDA_PIN              	GPIO_Pin_7
#define AFE_I2C_SDA_PORT        		GPIOB
#define AFE_I2C_SDA_CLK         		RCC_AHBPeriph_GPIOB
#define AFE_I2C_SDA_SOURCE           	GPIO_PinSource7
#define AFE_I2C_SDA_AF               	GPIO_AF_1

#define AFE_ALERT_PORT					GPIOB
#define AFE_ALERT_PIN					GPIO_Pin_5
#define AFE_ALERT_CLK					RCC_AHBPeriph_GPIOB

#define AFE_I2C_DEVICE 				  	I2C1
#define AFE_INTERRUPT_CHANNEL			5
#define AFE_INTERRUPT_VECTOR			EXTI4_15_IRQn
#define AFE_INTERRUPT_LINE				EXTI_Line5
#define AFE_INTERRUPT_PRIORITY		    3

#define AFE_TIMER_DEVICE				TIM7
#define CRC_MODE 						0
#define WRITE_ADDR_REG					0x3E
#define READ_ADDR_REG					0x40
#define WRITE_RAM_ADDR_REG_CHECKSUM		0x60

#endif /* BOARD_STM32_BSP_BQ_HW_BQ_HW_H_ */
