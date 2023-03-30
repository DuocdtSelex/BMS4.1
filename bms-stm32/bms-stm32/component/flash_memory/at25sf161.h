/*
 * at25sf161.h
 *
 *  Created on: Nov 4, 2021
 *      Author: Admin
 */

#ifndef COMPONENT_FLASH_MEMORY_AT25SF161_H_
#define COMPONENT_FLASH_MEMORY_AT25SF161_H_

#include "stdint.h"
#include "flash_memory.h"
#include "stdbool.h"
#include "ext_flash_hardware_hal.h"

/* Definitions for command and address AT25SF161*/

#define CMD_READ_ARRAY    								0x0B
#define CMD_READ_ARRAY_SLOW								0x03 /* no dummy byte, but limited to lower clock rate */
#define CMD_DUAL_OUTPUT_READ 							0x3B
#define CMD_DUAL_INPUT_OUTPUT_READ 						0xBB
#define CMD_QUAD_OUTPUT_READ     						0x6B
#define CMD_QUAD_INPUT_OUTPUT_READ 						0xEB
#define CMD_CONTINUOUS_READ_MODE_RESET_DUAL				0xFFFF
#define CMD_CONTINUOUS_READ_MODE_RESET_QUAD				0xFF

#define CMD_BLOCK_ERASE 								0x20 /* 4K bytes */
#define CMD_BLOCK_ERASE_LARGE							0x52 /* 32K bytes */
#define CMD_BLOCK_ERASE_LARGER							0x8D /* 64K bytes */
#define CMD_CHIP_ERASE									0x60
#define CMD_CHIP_ERASE_2								0xC7 /* not used, exactly the same as CMD_CHIP_ERASE */
#define CMD_BYTE_PAGE_PROGRAM 							0x02
#define CMD_PROGRAM_ERASE_SUSPEND 						0x75
#define CMD_PROGRAM_ERASE_RESUME 						0x7A

#define CMD_WRITE_ENABLE 								0x06
#define CMD_WRITE_DISABLE 								0x40

#define CMD_ERASE_SECURITY_REGISTERS_PAGE  				0x44
#define CMD_PROGRAM_SECURITY_REGISTER_PAGE				0x42
#define CMD_READ_SECURITY_REGISTER_PAGE	  				0x48

#define CMD_READ_STATUS_REGISTER_BYTE_1					0x05
#define CMD_READ_STATUS_REGISTER_BYTE_2 				0x35
#define CMD_WRITE_STATUS_REGISTER 						0x01
#define CMD_WRITE_ENABLE_FOR_VOLATILE_STATUS_REG 		0x50

#define CMD_READ_MANUFACTURER_AND_DEVICE_ID 			0x9F
#define CMD_READ_ID    									0x90
#define CMD_DEEP_POWER_DOWN 							0xB9
#define CMD_RESUME_FROM_DEEP_POWER_DOWN 				0xAB
#define CMD_RESUME_FROM_DEEP_POWER_DOWN_AND_READ_ID		0xAB

// SPI timeout
#define FLAHS_SPI_TIMEOUT 	1000

// Device memory
#define FLASH_BLOCK_SIZE			0x10000			/* 64KB */
#define FLASH_SECTOR_SIZE			0x1000			/* 4KB */
#define FLASH_PAGE_SIZE				0x100			/* 256B */

#define STATUS_BUSY_BIT                 (1<<0)
#define STATUS_WEL_BIT                  (1<<1)
#define STATUS_BP0_BIT                  (1<<2)
#define STATUS_BP1_BIT                  (1<<3)
#define STATUS_BP2_BIT                  (1<<4)
#define STATUS_TB_BIT                   (1<<5)
#define STATUS_SEC_BIT                  (1<<6)
#define STATUS_SRP0_BIT                 (1<<7)

struct flash_memory_t flash_memory;

struct flash_memory_t{
	uint32_t address_device;
	uint32_t opcode;
	uint32_t length_opcode;
	uint32_t address_byte;
	uint32_t dummy_byte;
	uint32_t data_byte;
	uint32_t length;
};

typedef enum BLOCK_SIZE{
    BLOCK_4K_SIZE       =0x00,
    BLOCK_32K_SIZE      =0x01,
    BLOCK_64K_SIZE      =0x02,
}BLOCK_SIZE;

// read flash memory
int flash_read_array(const uint32_t address, uint8_t* p_data, const uint16_t len);
int flash_dual_output_read_array(const uint32_t address, uint8_t* p_data, uint16_t const len);
void flash_dual_input_output_read_array(void);
void flash_quad_output_read_array(void);
void flash_quad_input_output_read_array(void);
void flash_continuous_read_mode_reset(void);

//program and erase commands
int flash_byte_page_program(const uint32_t address, uint8_t* p_data, uint8_t length);
int flash_block_erase(const uint32_t block, const BLOCK_SIZE block_size);
int flash_chip_erase(void);
int flash_program_erase_suspend(void);
int flash_program_erase_resume(void);

// protection commands and features

int flash_write_enable(void);
int flash_write_disable(void);

// security register commands
int flash_erase_security_registers(const uint32_t address);
int flash_program_security_registers(const uint32_t address, uint8_t* p_data, uint8_t length);
int flash_read_security_registers(const uint32_t address, uint8_t* p_data, uint8_t length);

// status register commands
int ext_flash_read_status1_reg(uint8_t* p_data);
int ext_flash_read_status2_reg(uint8_t* p_data);
int ext_flash_read_status_reg(uint16_t* p_data);
int flash_write_status_register(uint8_t write_status1, uint8_t write_status2);
int flash_write_enable_for_volatile_status_register(void);

// other command and functions
int flash_read_manufacturer_and_devicec_id(uint8_t* rx_buff, uint8_t rx_length);
int flash_read_id_leagacy_command(uint8_t* p_data);
int flash_deep_power_down(void);
int flash_resume_from_deep_power_down(void);
int flash_resume_from_deep_power_down_and_read_id(uint8_t* p_data);
void flash_hold_function(void);

#endif /* COMPONENT_FLASH_MEMORY_AT25SF161_H_ */
