/*
 * afe_hal.h
 *
 *  Created on: Aug 19, 2020
 *      Author: quangnd
 */

#ifndef BOARD_BQ_HAL_H_
#define BOARD_BQ_HAL_H_

#include "compiler_optimize.h"
//#include "bq_hardware.h"
#include "bq_hw.h"

extern BQ_Hw bq_hw;

#define HAL_BQ1_ON_STATE_CHANGE				BQ1_ON_STATE_CHANGE
#define HAL_BQ2_ON_STATE_CHANGE				BQ2_ON_STATE_CHANGE
#define HAL_BQ_INT_CLEAR_FLAG				BQ_INT_CLEAR_FLAG

int32_t bq_read_reg_block_with_crc_1(BQ_Hw* p_bq,uint8_t reg, uint8_t *buffer, uint8_t len)WEAK;
int32_t bq_read_reg_byte_with_crc_1(BQ_Hw* p_bq,uint8_t reg, uint8_t *data)WEAK;
int32_t bq_read_reg_word_with_crc_1(BQ_Hw* p_bq,uint8_t reg, uint16_t *data)WEAK;
int32_t bq_write_reg_block_with_crc_1(BQ_Hw* p_bq,uint8_t start, uint8_t *buffer, uint8_t len)WEAK;
int32_t bq_write_reg_word_with_crc_1(BQ_Hw* p_bq,uint8_t reg, uint32_t data)WEAK;
int32_t bq_write_reg_byte_with_crc_1(BQ_Hw* p_bq,uint8_t reg, uint8_t data)WEAK;
/* read/write data for bq769x2
 * read function
*/
int32_t bq_i2c_read_reg_byte(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer)WEAK;
int32_t bq_i2c_read_reg_word(BQ_Hw* p_bq, uint8_t reg_addr, uint16_t* buffer)WEAK;
int32_t bq_i2c_read_reg_word_with_sign(BQ_Hw* p_bq, uint8_t reg_addr, int16_t* buffer)WEAK;
int32_t bq_i2c_read_reg_block(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length)WEAK;
int32_t bq_i2c_read_reg_block_with_sign(BQ_Hw* p_bq, uint8_t reg_addr, int8_t* buffer, uint8_t length)WEAK;
int32_t bq_i2c_read_reg_block_with_crc(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length)WEAK;
int32_t bq_i2c_data_ram_read(BQ_Hw* p_bq,  uint16_t reg_addr, uint8_t* buffer, uint8_t length)WEAK;
int32_t bq_i2c_subcommand_read_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t *buffer)WEAK;
int32_t bq_i2c_subcommand_read_block(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t* buffer, uint8_t length)WEAK;
int32_t bq_i2c_subcommand_read_block_with_sign(BQ_Hw* p_bq, uint16_t reg_addr, int8_t* buffer, uint8_t length)WEAK;
int32_t bq_i2c_data_ram_read_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t * buffer)WEAK;
int32_t bq_i2c_data_ram_read_word_with_sign(BQ_Hw* p_bq, uint16_t reg_addr, int16_t *buffer)WEAK;
int32_t bq_i2c_data_ram_read_reg_with_float(BQ_Hw* p_bq, uint16_t reg_addr, float *buffer)WEAK;

// write function
int32_t bq_i2c_write_reg_byte(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t reg_data)WEAK;
int32_t bq_i2c_write_reg_word(BQ_Hw* p_bq, uint8_t reg_addr, uint16_t reg_data)WEAK;
int32_t bq_i2c_write_reg_block(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length)WEAK;
int32_t bq_i2c_wirte_reg_block_with_crc(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length)WEAK;
int32_t bq_i2c_data_ram_write(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t* buffer, uint8_t length)WEAK;
int32_t bq_i2c_data_ram_write_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t reg_data)WEAK;
int32_t bq_i2c_data_ram_write_word_with_sign(BQ_Hw* p_bq, uint16_t reg_addr, int16_t reg_data)WEAK;
int32_t bq_i2c_data_ram_write_byte(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t reg_data)WEAK;
int32_t bq_i2c_subcommand_write_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t reg_data)WEAK;
int32_t bq_i2c_subcommand_write_byte(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t reg_data)WEAK;

#endif /* BOARD_BQ_HAL_H_ */
