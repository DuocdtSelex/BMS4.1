/*
 * at25sf161.c
 *
 *  Created on: Nov 4, 2021
 *      Author: Admin
 */

#ifndef COMPONENT_FLASH_MEMORY_AT25SF161_C_
#define COMPONENT_FLASH_MEMORY_AT25SF161_C_

#include "at25sf161.h"

void ext_flash_write_array_impl(Flash_Memory* this);
void ext_flash_read_array_impl(Flash_Memory* this);
void ext_flash_erase_array_impl(Flash_Memory* this);

static Flash_Memory_Interface at_interface = {
		ext_flash_write_array_impl,
		ext_flash_read_array_impl,
		ext_flash_erase_array_impl,
};

// helper function

inline void address_to_bytes(const uint32_t address, uint8_t* address_bytes) {

	uint8_t byte_index = 0;
	for (byte_index = 0; byte_index < 3; byte_index++) {
//		address_bytes[byte_index] = (uint8_t) ((address
//				<< (24 - 8 * byte_index)) >> 24);
		address_bytes[2 - byte_index] = (uint8_t) ((address
						<< (24 - 8 * byte_index)) >> 24);
	}
}

void ext_flash_write_array_impl(Flash_Memory* this){

}
void ext_flash_read_array_impl(Flash_Memory* this){

}
void ext_flash_erase_array_impl(Flash_Memory* this){

}

// helper function

static bool addr_aligned(uint32_t addr, uint8_t alignment_size);
static int flash_read_reg(const uint8_t reg, uint8_t* p_data);
static int flash_verify_program_data(const uint8_t* data_send, uint8_t* data_read,
		const uint16_t len);
/***************************************************************************//**
 * @brief
 *   Checks if address is aligned on a power of 2
 * @note
 * 		Determines if address is aligned to a multiple of alignment_size which
 * 		is a power of 2.  Alignment_size is read from Device table in spiflash.c
 * @param[in] addr
 * 		Address to check Power of 2 alignment
 * @param[in] alignment_size
 * 		Alignment_size used to validate if Power of 2
 * @return
 * 		TRUE if aligned on Power of 2, False if not.
 *
 ******************************************************************************/
static bool addr_aligned(uint32_t addr, uint8_t alignment_size)
{
	return (addr & (alignment_size-1)) == 0;
}

static int flash_read_reg(const uint8_t reg, uint8_t* p_data) {
	int result = -1;
	external_flash_chip_select_active();
	result = ext_flash_send_bytes(&reg, 1);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	result = ext_flash_get_bytes(p_data, 1);
	external_flash_chip_select_deactive();
	return result;
}

static int flash_verify_program_data(const uint8_t* data_send, uint8_t* data_read,
		const uint16_t len) {
	uint8_t index_byte = 0;
	while (index_byte < len) {
		if (data_send[index_byte] != data_read[index_byte])
			return -1;
		index_byte++;
	}
	return 0;
}

// read flash memory
int flash_read_array(const uint32_t address, uint8_t* p_data, const uint16_t len){
	uint8_t buffer[4] = { 0 };
	int result = 0;
	buffer[0] = CMD_READ_ARRAY;
	address_to_bytes(address, buffer + 1);
	external_flash_chip_select_active();
	result = ext_flash_send_bytes(buffer, 4);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	result = ext_flash_get_bytes(p_data, len);
	external_flash_chip_select_deactive();
	ext_flash_send_dummy_byte();
	return result;
}
int flash_dual_output_read_array(const uint32_t address, uint8_t* p_data, uint16_t const len){
	uint8_t buffer[5] ={0};
	int result = 0;
	buffer[0] = CMD_DUAL_OUTPUT_READ;
	address_to_bytes(address, buffer+1);
	buffer[4] = EXT_FLASH_DUMMY_BYTE;
	external_flash_chip_select_active();

	result = ext_flash_get_bytes(p_data, len);
	external_flash_chip_select_deactive();
	return result;
}
void flash_dual_input_output_read_array(void){
	;
}
void flash_quad_output_read_array(void){
	;
}
void flash_quad_input_output_read_array(void){
	;
}
void flash_continuous_read_mode_reset(void){
	;
}

//program and erase commands
int flash_byte_page_program(const uint32_t address, uint8_t* p_data, uint8_t length){
	int result = 0;
	uint8_t buffer[4] = {0};
	uint8_t data_read[255] ={0};
	flash_write_enable();
	buffer[0] = CMD_BYTE_PAGE_PROGRAM;
	address_to_bytes(address, buffer+1);

	do{
		external_flash_chip_select_active();
		result = ext_flash_send_bytes(buffer, 4);
		if(result != FLASH_SUCESS)
		{
			external_flash_chip_select_deactive();
			flash_write_disable();
			return result;
		}
		result = ext_flash_send_bytes(p_data, length);
		if(result != FLASH_SUCESS)
		{
			external_flash_chip_select_deactive();
			flash_write_disable();
			return result;
		}
		external_flash_chip_select_deactive();
		ext_flash_send_dummy_byte();
		flash_read_array(address, data_read, length);
	} while (flash_verify_program_data(p_data, data_read, length) != 0);

	flash_write_disable();
	return result;
}
int flash_block_erase(const uint32_t block, const BLOCK_SIZE block_size){
	uint8_t opcode = 0;
	int result = 0;
	flash_write_enable();
	uint8_t buffer[4] = { 0 };
	switch (block_size) {
	case BLOCK_4K_SIZE:
		opcode = CMD_BLOCK_ERASE;
		break;
	case BLOCK_32K_SIZE:
		opcode = CMD_BLOCK_ERASE_LARGE;
		break;
	case BLOCK_64K_SIZE:
		opcode = CMD_BLOCK_ERASE_LARGER;
		break;
	default:
		return FLASH_FAIL;
	}

	buffer[0] = opcode;
	address_to_bytes(block, buffer + 1);
	external_flash_chip_select_active();
	result = ext_flash_send_bytes(buffer, 4);
	if(result != FLASH_SUCESS)
	{
		external_flash_chip_select_deactive();
		flash_write_disable();
		return result;
	}
	external_flash_chip_select_deactive();
	ext_flash_send_dummy_byte();
	flash_write_disable();
	return result;
}
int flash_chip_erase(void){
	uint8_t opcode = CMD_CHIP_ERASE;
	uint8_t result = 0;
	if (flash_write_enable() != 0){
		external_flash_chip_select_deactive();
		return FLASH_FAIL;
	}
	external_flash_chip_select_active();
	result = ext_flash_send_bytes(&opcode, 1);
	if(result != FLASH_SUCESS)
	{
		external_flash_chip_select_deactive();
		return result;
	}

	external_flash_chip_select_deactive();
	ext_flash_send_dummy_byte();
	flash_write_disable();
	return result;
}
int flash_program_erase_suspend(void){
	int result = 0;
	uint8_t opcode = CMD_PROGRAM_ERASE_SUSPEND;
	external_flash_chip_select_active();
	result = ext_flash_send_bytes(&opcode, 1);
	if(result != FLASH_SUCESS)
	{
		external_flash_chip_select_deactive();
		return result;
	}
	external_flash_chip_select_deactive();
	ext_flash_send_dummy_byte();
	return result;
}
int flash_program_erase_resume(void){
	int result = 0;
	uint8_t opcode = CMD_PROGRAM_ERASE_RESUME;
	external_flash_chip_select_active();
	result = ext_flash_send_bytes(&opcode, 1);
	if(result != FLASH_SUCESS)
	{
		external_flash_chip_select_deactive();
		return result;
	}
	external_flash_chip_select_deactive();
	ext_flash_send_dummy_byte();
	return result;
}

// protection commands and features

int flash_write_enable(void){
	uint8_t status1 = 0;
	uint8_t result = 0;
	do {
		uint8_t opcode = CMD_WRITE_ENABLE;
		external_flash_chip_select_active();
		result = ext_flash_send_bytes(&opcode, 1);
		external_flash_chip_select_deactive();
		ext_flash_send_dummy_byte();
		ext_flash_read_status1_reg(&status1);
	} while ((status1 & STATUS_WEL_BIT) == 0);
	return result;
}
int flash_write_disable(void){
	uint8_t result = 0;
	uint8_t status1 = 0;
	do {
		uint8_t opcode = CMD_WRITE_DISABLE;
		external_flash_chip_select_active();
		result = ext_flash_send_bytes(&opcode, 1);
		external_flash_chip_select_deactive();
		ext_flash_send_dummy_byte();
		ext_flash_read_status1_reg(&status1);
	} while ((status1 & STATUS_WEL_BIT) == 1);
	return result;
}

// security register commands
int flash_erase_security_registers(const uint32_t address){
	int result = 0;
	uint8_t buffer[4] = {0};
	buffer[0] = CMD_ERASE_SECURITY_REGISTERS_PAGE;
	flash_write_enable();
	external_flash_chip_select_active();
	address_to_bytes(address, buffer + 1);
	result = ext_flash_send_bytes(buffer, 4);
	if(result != FLASH_SUCESS)
	{
		external_flash_chip_select_deactive();
		flash_write_disable();
		return result;
	}
	external_flash_chip_select_deactive();
	ext_flash_send_dummy_byte();
	flash_write_disable();
	return result;
}
int flash_program_security_registers(const uint32_t address, uint8_t* p_data, uint8_t length){
	int result = 0;
	uint8_t buffer[4] = {0};
	uint8_t data_read[255] ={0};
	flash_write_enable();
	buffer[0] = CMD_PROGRAM_SECURITY_REGISTER_PAGE;
	address_to_bytes(address, buffer+1);

	do{
		external_flash_chip_select_active();
		result = ext_flash_send_bytes(buffer, 4);
		if(result != FLASH_SUCESS)
		{
			external_flash_chip_select_deactive();
			flash_write_disable();
			return result;
		}
		result = ext_flash_send_bytes(p_data, length);
		if(result != FLASH_SUCESS)
		{
			external_flash_chip_select_deactive();
			flash_write_disable();
			return result;
		}
		external_flash_chip_select_deactive();
		ext_flash_send_dummy_byte();
		flash_read_array(address, data_read, length);
	} while (flash_verify_program_data(p_data, data_read, length) != 0);

	flash_write_disable();
	return result;
}
int flash_read_security_registers(const uint32_t address, uint8_t* p_data, uint8_t length){
	uint8_t buffer[5] = { 0 };
	int result = 0;
	buffer[0] = CMD_READ_SECURITY_REGISTER_PAGE;
	address_to_bytes(address, buffer + 1);
	buffer[4] = EXT_FLASH_DUMMY_BYTE;
	external_flash_chip_select_active();
	result = ext_flash_send_bytes(buffer, 5);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	result = ext_flash_get_bytes(p_data, length);
	external_flash_chip_select_deactive();
	ext_flash_send_dummy_byte();
	return result;
}

// status register commands
int ext_flash_read_status1_reg(uint8_t* p_data) {
	return flash_read_reg(CMD_READ_STATUS_REGISTER_BYTE_1, p_data);
}

int ext_flash_read_status2_reg(uint8_t* p_data) {
	return flash_read_reg(CMD_READ_STATUS_REGISTER_BYTE_2, p_data);
}

int ext_flash_read_status_reg(uint16_t* p_data) {
	uint8_t status = 0;
	if (flash_read_reg(CMD_READ_STATUS_REGISTER_BYTE_2, &status) != 0)
		return -1;
	(*p_data) = status;
	(*p_data) <<= 8;
	if (flash_read_reg(CMD_READ_STATUS_REGISTER_BYTE_1, &status) != 0)
		return -1;
	(*p_data) |= (status & 0x00FF);
	return 0;
}
int flash_write_status_register(uint8_t write_status1, uint8_t write_status2){
	uint8_t result;
	uint8_t buffer[3] ={0};

	external_flash_chip_select_active();
	buffer[0] = CMD_WRITE_STATUS_REGISTER;
	buffer[1] = write_status1;
	buffer[2] = write_status2;
	result = ext_flash_send_bytes(buffer,3);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	external_flash_chip_select_deactive();
	return result;
}
int flash_write_enable_for_volatile_status_register(void){
	uint8_t result;
	uint8_t opcode;
	external_flash_chip_select_active();
	opcode = CMD_WRITE_ENABLE_FOR_VOLATILE_STATUS_REG;
	result = ext_flash_send_bytes(&opcode,1);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	external_flash_chip_select_deactive();
	return result;
}

// other command and functions
int flash_read_manufacturer_and_devicec_id(uint8_t* rx_buff, uint8_t rx_length){
	uint8_t result = 0;
	uint8_t opcode = 0;
	external_flash_chip_select_active();
	opcode = CMD_READ_MANUFACTURER_AND_DEVICE_ID;
	result = ext_flash_send_bytes(&opcode, 1);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	result = ext_flash_get_bytes(rx_buff, rx_length);
	external_flash_chip_select_deactive();
	return result;
}
int flash_read_id_leagacy_command(uint8_t* p_data){
	uint8_t result = 0;
	uint8_t buffer[4] = {0};
	external_flash_chip_select_active();
	buffer[0] = CMD_READ_ID;
	buffer[1] = EXT_FLASH_DUMMY_BYTE;
	buffer[2] = EXT_FLASH_DUMMY_BYTE;
	buffer[3] = EXT_FLASH_DUMMY_BYTE;
	result = ext_flash_send_bytes(buffer, 4);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	result = ext_flash_get_bytes(p_data, 2);
	external_flash_chip_select_deactive();
	ext_flash_send_dummy_byte();
	return result;
}
int flash_deep_power_down(void){
	uint8_t result = 0;
	uint8_t opcode = CMD_DEEP_POWER_DOWN;
	external_flash_chip_select_active();
	result = ext_flash_send_bytes(&opcode, 1);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	external_flash_chip_select_deactive();
	return result;
}
int flash_resume_from_deep_power_down(void){
	uint8_t result = 0;
	uint8_t opcode = 0;
	external_flash_chip_select_active();
	opcode = CMD_RESUME_FROM_DEEP_POWER_DOWN;
	result = ext_flash_send_bytes(&opcode, 1);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	external_flash_chip_select_deactive();
	return result;
}
int flash_resume_from_deep_power_down_and_read_id(uint8_t* p_data){
	uint8_t result = 0;
	uint8_t buffer[4] = {0};
	external_flash_chip_select_active();
	buffer[0] = CMD_RESUME_FROM_DEEP_POWER_DOWN;
	buffer[1] = EXT_FLASH_DUMMY_BYTE;
	result = ext_flash_send_bytes(buffer, 4);
	if (result != FLASH_SUCESS) {
		external_flash_chip_select_deactive();
		return result;
	}
	result = ext_flash_get_bytes(p_data, 2);
	external_flash_chip_select_deactive();
	return result;
}
void flash_hold_function(void){
	;
}

#endif /* COMPONENT_FLASH_MEMORY_AT25SF161_C_ */
