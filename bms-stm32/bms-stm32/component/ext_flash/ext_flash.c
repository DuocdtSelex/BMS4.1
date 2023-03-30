#include "ext_flash.h"
#include "board.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_spi.h"
#include "spi_driver.h"
#include "timer.h"

#ifndef EXT_FLASH_USE_LOW_FREQUENCY
#define EXT_FLASH_USE_LOW_FREQUENCY
#endif

#define FLASH_TIMEOUT       70000

uint32_t g_timeout = 0;

static void ext_flash_cs_active();
static void ext_flash_cs_deactive();
static bool flash_wait_for_flag(uint16_t flag, uint32_t status);
static int ext_flash_send_bytes(const uint8_t* p_data, const uint8_t len);
static int ext_flash_get_bytes(uint8_t* p_data, const uint8_t len);
static void ext_flash_flush_rx_buffer();
static void ext_flash_send_dummy_byte();
extern inline void address_to_bytes(const uint32_t address,
		uint8_t* address_bytes);
static int ext_flash_confirm_data(const uint8_t* data_send, uint8_t* data_read,
		const uint16_t len);
static bool ext_flash_is_ready(void);
void ext_flash_init() {

}

int ext_flash_read_bytes(const uint32_t address, uint8_t* p_data,
		const uint16_t len) {

	uint8_t buffer[4] = { 0 };
	int result = 0;
	buffer[0] = READ_ARRAY_CMD;
	address_to_bytes(address, buffer + 1);
	ext_flash_cs_active();
	result = ext_flash_send_bytes(buffer, 4);
	if (result != FLASH_SUCESS) {
		ext_flash_cs_deactive();
		return result;
	}
	result = ext_flash_get_bytes(p_data, len);
	ext_flash_cs_deactive();
	ext_flash_send_dummy_byte();
	return result;
}

int ext_flash_read_page(const uint32_t page, uint8_t* p_data) {
	return(ext_flash_read_bytess(page,p_data,EXT_FLASH_PAGE_SIZE));
}

int ext_flash_read_all(uint8_t* p_data) {
	return 0;
}

#if 1
int ext_flash_write_bytes(const uint32_t address, const uint8_t* p_data,
		const uint16_t len) {
	while(ext_flash_is_ready() == false);
	uint8_t buffer[4] = { 0 };
	uint8_t data_read[255] = { 0 };
	int result = 0;
	do {
		ext_flash_write_enable();
		buffer[0] = BYTE_PAGE_WRITE_CMD;
		address_to_bytes(address, buffer + 1);
		ext_flash_cs_active();
		result = ext_flash_send_bytes(buffer, 4);
		if (result != FLASH_SUCESS) {
			ext_flash_cs_deactive();
			return result;
		}
		result = ext_flash_send_bytes(p_data, len);
		if (result != FLASH_SUCESS) {
			ext_flash_cs_deactive();
			return result;
		}
		ext_flash_cs_deactive();
		ext_flash_send_dummy_byte();
		ext_flash_read_bytes(address, data_read, len);
	} while (ext_flash_confirm_data(p_data, data_read, len) != 0);
	return result;
}

#endif

int ext_flash_write_bytess(const uint32_t address, const uint8_t* p_data,
		const uint16_t len) {
	int result = 0;
	if (len < 101) {
		result = ext_flash_write_bytes(address, p_data, len);
	} else if (len < 201) {
		result = ext_flash_write_bytes(address, p_data, 100);
		result = ext_flash_write_bytes(address + 100, p_data + 100,
				(len - 100));
	} else {
		result = ext_flash_write_bytes(address, p_data, 100);
		result = ext_flash_write_bytes(address + 100, p_data + 100, 100);
		result = ext_flash_write_bytes(address + 200, p_data + 200,
				(len - 200));
	}
	return result;
}

int ext_flash_read_bytess(const uint32_t address, uint8_t* p_data,
		const uint16_t len) {
	uint8_t result = -1;
	if (len < 101) {
		result = ext_flash_read_bytes(address, p_data, len);
	} else if (len < 201) {
		result = ext_flash_read_bytes(address, p_data, 100);
		result = ext_flash_read_bytes(address + 100, p_data + 100, (len - 100));
	} else {
		result = ext_flash_read_bytes(address, p_data, 100);
		result = ext_flash_read_bytes(address + 100, p_data + 100, 100);
		result = ext_flash_read_bytes(address + 200, p_data + 200, (len - 200));
	}
	return result;
}

static int ext_flash_confirm_data(const uint8_t* data_send, uint8_t* data_read,
		const uint16_t len) {
	uint8_t index_byte = 0;
	while (index_byte < len) {
		if (data_send[index_byte] != data_read[index_byte])
			return -1;
		index_byte++;
	}
	return 0;
}

int ext_flash_write_page(const uint32_t page, const uint8_t* p_data) {
	return (ext_flash_write_bytess(page, p_data, EXT_FLASH_PAGE_SIZE));
}

int ext_flash_erase_all() {
	uint8_t opcode = CHIP_ERASE_CMD;
	uint8_t result = 0;
	if (ext_flash_write_enable() != 0)
		return -1;
	ext_flash_cs_active();
	result = ext_flash_send_bytes(&opcode, 1);
	ext_flash_cs_deactive();
	ext_flash_send_dummy_byte();
	return result;

}

int ext_flash_erase_block(const uint32_t block, const BLOCK_SIZE block_size) {
	uint8_t opcode = 0;
	int result = 0;
	ext_flash_write_enable();
	uint8_t buffer[4] = { 0 };
	switch (block_size) {
	case BLOCK_4K_SIZE:
		opcode = BLOCK_4K_ERASE_CMD;
		break;
	case BLOCK_32K_SIZE:
		opcode = BLOCK_32K_ERASE_CMD;
		break;
	case BLOCK_64K_SIZE:
		opcode = BLOCK_64K_ERASE_CMD;
		break;
	default:
		return FLASH_FAIL;
	}

	buffer[0] = opcode;
	address_to_bytes(block, buffer + 1);
	ext_flash_cs_active();
	result = ext_flash_send_bytes(buffer, 4);
	ext_flash_cs_deactive();
	ext_flash_send_dummy_byte();
	return result;
}

int ext_flash_write_enable() {
	uint8_t status1 = 0;
	uint8_t result = 0;
	do {
		uint8_t opcode = WRITE_ENABLE_CMD;
		ext_flash_cs_active();
		result = ext_flash_send_bytes(&opcode, 1);
		ext_flash_cs_deactive();
		ext_flash_send_dummy_byte();
		ext_flash_read_status1(&status1);
	} while ((status1 & STATUS_WEL_BIT) == 0);
	return result;
}

int ext_flash_write_sector(const uint32_t sector_add,const uint8_t* p_data){
     ext_flash_erase_block(sector_add,BLOCK_64K_SIZE);
     uint32_t add = sector_add;
     uint8_t result = 0;
     while(add <(sector_add + EXT_FLASH_SECTOR_SIZE)){
        result = ext_flash_write_bytess(add,p_data+add,EXT_FLASH_PAGE_SIZE);
        if(result != 0)
        	return -1;
        add += EXT_FLASH_PAGE_SIZE;
     }
     return 0;
}

int ext_flash_write_disable() {
	uint8_t result = 0;
	uint8_t status1 = 0;
	do {
		uint8_t opcode = WRITE_DISABLE_CMD;
		ext_flash_cs_active();
		result = ext_flash_send_bytes(&opcode, 1);
		ext_flash_cs_deactive();
		ext_flash_send_dummy_byte();
		ext_flash_read_status1(&status1);
	} while ((status1 & STATUS_WEL_BIT) == 1);
	return result;
}

int ext_flash_read_status1(uint8_t* p_data) {
	return ext_flash_read_reg(READ_STATUS_1_CMD, p_data);
}

int ext_flash_read_status2(uint8_t* p_data) {
	return (ext_flash_read_reg(READ_STATUS_2_CMD, p_data));
}

int ext_flash_read_status(uint16_t* p_data) {
	uint16_t status = 0;
	if (ext_flash_read_reg(READ_STATUS_2_CMD, &status) != 0)
		return -1;
	(*p_data) = status;
	(*p_data) <<= 8;
	if (ext_flash_read_reg(READ_STATUS_1_CMD, &status) != 0)
		return -1;
	(*p_data) |= (status & 0x00FF);
	return 0;
}

int ext_flash_write_status(const uint8_t data) {
	return FLASH_SUCESS;
}

int ext_flash_read_manufacture_id(uint8_t* p_data) {
	int result = 0;
	uint8_t buffer = READ_MANUFACTURE_ID;
	ext_flash_flush_rx_buffer();
	ext_flash_cs_active();
	result = ext_flash_send_bytes(&buffer, 1);
	if (result != FLASH_SUCESS) {
		ext_flash_cs_deactive();
		return result;
	}

	result = ext_flash_get_bytes(p_data, 3);
	ext_flash_cs_deactive();
	ext_flash_send_dummy_byte();
	return result;
#if 0
	ext_flash_cs_active();
	ext_flash_write_read(READ_MANUFACTURE_ID,p_data,3);
	ext_flash_cs_deactive();
#endif
}

int ext_flash_read_serial_number(uint8_t* p_data) {
	return FLASH_SUCESS;
}

int ext_flash_set_deep_power_down() {
	return FLASH_SUCESS;
}

int ext_flash_resume_from_deep_power_down() {
	return FLASH_SUCESS;
}

static void ext_flash_cs_active() {
	GPIO_ResetBits(EXT_FLASH_SPI_CS_PORT, EXT_FLASH_SPI_CS_PIN);
}

static void ext_flash_cs_deactive() {
	GPIO_SetBits(EXT_FLASH_SPI_CS_PORT, EXT_FLASH_SPI_CS_PIN);
}

static bool flash_wait_for_flag(uint16_t flag, uint32_t status) {
	g_timeout = 0;
	while (SPI_I2S_GetFlagStatus(EXT_FLASH_SPI_DEV, flag) == status) {
		if (g_timeout++ > FLASH_TIMEOUT)
			return false;
	}
	return true;
}

int ext_flash_read_reg(const uint8_t reg, uint8_t* p_data) {
	int result = -1;
	ext_flash_cs_active();
	result = ext_flash_send_bytes(&reg, 1);
	if (result != FLASH_SUCESS) {
		ext_flash_cs_deactive();
		return result;
	}
	result = ext_flash_get_bytes(p_data, 1);
	ext_flash_cs_deactive();
	return result;
}

int ext_flash_write_command(const uint8_t command) {
	int result = 0;
	ext_flash_cs_active();
	result = ext_flash_send_bytes(command, 1);
	ext_flash_cs_deactive();
	return result;
}

int ext_flash_write_reg(const uint8_t reg, const uint8_t data) {
	uint8_t buffer[2] = { 0 };
	int result = 0;
	ext_flash_write_enable();
	buffer[0] = reg;
	buffer[1] = data;
	ext_flash_cs_active();
	result = ext_flash_send_bytes(buffer, 2);
	ext_flash_cs_deactive();
	return result;
}

static int ext_flash_send_bytes(const uint8_t* p_data, const uint8_t len) {

	uint8_t data = 0;
	uint8_t index = 0;
	while (index < len) {
		/* Wait until the transmit buffer is empty */
		if (!flash_wait_for_flag(SPI_I2S_FLAG_TXE, RESET))
			return FLASH_FAIL;

		/* Send the byte */
		SPI_SendData8(EXT_FLASH_SPI_DEV, p_data[index]);
		/* Wait to receive a byte*/
		if (!flash_wait_for_flag(SPI_I2S_FLAG_RXNE, RESET))
			return FLASH_FAIL;
		/* Return the byte read from the SPI bus */
		data = SPI_ReceiveData8(EXT_FLASH_SPI_DEV);
		index++;
	}
	return 0;
}

static int ext_flash_get_bytes(uint8_t* p_data, const uint8_t len) {

	uint8_t index = 0;
	while (index < len) {
		/* Wait until the transmit buffer is empty */
		if (!flash_wait_for_flag(SPI_I2S_FLAG_TXE, RESET))
			return FLASH_FAIL;
		/* Send the byte */
		SPI_SendData8(EXT_FLASH_SPI_DEV, EXT_FLASH_DUMMY_BYTE);

		/* Wait until a data is received */
		if (!flash_wait_for_flag(SPI_I2S_FLAG_RXNE, RESET))
			return FLASH_FAIL;

		/* Return the shifted data */
		p_data[index] = SPI_ReceiveData8(EXT_FLASH_SPI_DEV);
		index++;
	}
	return 0;
}

static void ext_flash_flush_rx_buffer() {
	uint8_t data = 0;
	/* flush rx buffer */
	while (SPI_GetReceptionFIFOStatus(EXT_FLASH_SPI_DEV)
			!= SPI_ReceptionFIFOStatus_Empty) {
		data = SPI_ReceiveData8(EXT_FLASH_SPI_DEV);
	}

}

static void ext_flash_send_dummy_byte() {

	uint8_t dummy_data = 0;
	/* Wait until the transmit buffer is empty */
	if (!flash_wait_for_flag(SPI_I2S_FLAG_TXE, RESET))
		return;
	/* Send the byte */
	SPI_SendData8(EXT_FLASH_SPI_DEV, EXT_FLASH_DUMMY_BYTE);

	/* Wait until a data is received */
	if (!flash_wait_for_flag(SPI_I2S_FLAG_RXNE, RESET))
		return;

	/* Return the shifted data */
	dummy_data = SPI_ReceiveData8(EXT_FLASH_SPI_DEV);
}
;

inline void address_to_bytes(const uint32_t address, uint8_t* address_bytes) {

	uint8_t byte_index = 0;
	for (byte_index = 0; byte_index < 3; byte_index++) {
//		address_bytes[byte_index] = (uint8_t) ((address
//				<< (24 - 8 * byte_index)) >> 24);
		address_bytes[2 - byte_index] = (uint8_t) ((address
						<< (24 - 8 * byte_index)) >> 24);
	}
}

static bool ext_flash_is_ready(void) {
	uint8_t status1 = 0;
	ext_flash_read_status1(&status1);
	if ((status1 & STATUS_BUSY_BIT))
		return false;
	else
		return true;
}
