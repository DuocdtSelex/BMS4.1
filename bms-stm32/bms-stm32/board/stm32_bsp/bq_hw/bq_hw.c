/*
 * bq_hw.c
 *
 *  Created on: Apr 13, 2021
 *      Author: Admin
 */

#include "bq_hw.h"
#include "stm32f0xx.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"
#include "stdbool.h"
#include "delay_hw.h"
#include "delay_hw.h"

#define CRC_KEY 7
#define I2C_TIMEOUT		 				10000
#define I2C_READ_BLOCK_TIMEOUT		 	1000

#define LOW_BYTE(Data)			(uint8_t)(0xff & Data)
#define HIGH_BYTE(Data)			(uint8_t)(0xff & (Data >> 8))

static void bq_gpio_init_1(void);
static void bq_clock_init_1(void);
static void bq_interrupt_init_1(void);
static void bq_i2c_init_1(void);
static void bq_config_timer_1(void);

static void bq_reset_i2c_bus_1(void);

static int I2CSendBytes(I2C_TypeDef *I2C_DEV, unsigned char I2CSlaveAddress,
		unsigned char *DataBuffer, unsigned int ByteCount,
		unsigned int *SentByte);
static int I2CReadBytes(I2C_TypeDef *I2C_DEV, unsigned char I2CSlaveAddress,
		unsigned char *DataBuffer, unsigned int ExpectedByteNumber,
		unsigned int *NumberOfReceivedBytes);
static uint8_t CRC8(uint8_t *ptr, uint8_t len, uint8_t key);
static uint8_t check_sum(uint8_t *ptr, uint8_t len);

static bool i2c_wait_for_flag_status(I2C_TypeDef *I2C_DEV, uint32_t flag,
		FlagStatus status, uint32_t timeout_us);

static void bq_hw_init_1(void);

struct BQ_Hw_t bq_hw;

static void bq_hw_init_1(void) {

	bq_hw.i2c_module = AFE_I2C_DEVICE;
	bq_hw.i2c_address = AFE_I2C_ADDRESS;
	bq_hw.reset_i2c_bus = bq_reset_i2c_bus_1;
}

int32_t bq_read_reg_block_with_crc_1(BQ_Hw* p_bq,uint8_t reg, uint8_t *buffer, uint8_t len){

	unsigned char TargetRegister = reg;
	uint8_t StartData[255];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char new_CRC = 0;
	uint8_t *ReadData = StartData;
	int ReadStatus = 0;
	int i;

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_BUSY, RESET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}
	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(p_bq->i2c_module, p_bq->i2c_address, 1,
			I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TXIS, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}

	/* Send Register address */
	I2C_SendData(p_bq->i2c_module, TargetRegister);
	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TC, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}

	ReadStatus = I2CReadBytes(p_bq->i2c_module, p_bq->i2c_address, ReadData,
			2 * len, &ReadDataCount);

	if (ReadStatus != 0) {
		return -1;
	}

	CRCInput[0] = (p_bq->i2c_address) + 1;
	CRCInput[1] = *ReadData;

	new_CRC = CRC8(CRCInput, 2, CRC_KEY);

	ReadData++;
	if (new_CRC != *ReadData) {
		return -1;
	} else
		*buffer = *(ReadData - 1);

	for (i = 1; i < len; i++) {
		ReadData++;
		new_CRC = CRC8(ReadData, 1, CRC_KEY);
		ReadData++;
		buffer++;

		if (new_CRC != *ReadData) {
			return -1;
		} else
			*buffer = *(ReadData - 1);
	}
	return 0;
}

int32_t bq_read_reg_byte_with_crc_1(BQ_Hw *p_bq, uint8_t reg, uint8_t *data) {
	unsigned char TargetRegister = reg;
	unsigned char ReadData[2];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char new_CRC = 0;
	int ReadStatus = 0;

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_BUSY, RESET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}
	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(p_bq->i2c_module, p_bq->i2c_address, 1,
			I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TXIS, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}

	/* Send Register address */
	I2C_SendData(p_bq->i2c_module, TargetRegister);

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TC, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}

	ReadStatus = I2CReadBytes(p_bq->i2c_module, p_bq->i2c_address, ReadData, 2,
			&ReadDataCount);

	if (ReadStatus != 0) {
		return -1;
	}
	CRCInput[0] = (p_bq->i2c_address) + 1;
	CRCInput[1] = ReadData[0];

	new_CRC = CRC8(CRCInput, 2, CRC_KEY);

	if (new_CRC != ReadData[1])
		return -1;

	*data = ReadData[0];
	return 0;
}

int32_t bq_read_reg_word_with_crc_1(BQ_Hw *p_bq, uint8_t reg, uint16_t *data) {
	unsigned char TargetRegister = reg;
	unsigned char ReadData[4];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char new_CRC = 0;
	int ReadStatus = 0;
	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_BUSY, RESET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(p_bq->i2c_module, p_bq->i2c_address, 1,
			I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TXIS, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}

	/* Send Register address */
	I2C_SendData(p_bq->i2c_module, TargetRegister);
	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TC, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}

	ReadStatus = I2CReadBytes(p_bq->i2c_module, p_bq->i2c_address, ReadData, 4,
			&ReadDataCount);

	if (ReadStatus != 0) {
		return -1;
	}

	CRCInput[0] = (p_bq->i2c_address) + 1;
	CRCInput[1] = ReadData[0];

	new_CRC = CRC8(CRCInput, 2, CRC_KEY);

	if (new_CRC != ReadData[1])
		return -1;

	new_CRC = CRC8(ReadData + 2, 1, CRC_KEY);

	if (new_CRC != ReadData[3])
		return -1;

	*data = ReadData[0];

	*data = (*data << 8) + ReadData[2];

	return 0;
}

int32_t bq_write_reg_block_with_crc_1(BQ_Hw *p_bq, uint8_t start, uint8_t *buffer,
		uint8_t len) {

	unsigned char BufferCRC[255] = { 0 };
	uint8_t *Pointer = BufferCRC;
	int i;
	unsigned int SentByte = 0;
	int result;

	Pointer = BufferCRC;
	*Pointer = p_bq->i2c_address;
	Pointer++;
	*Pointer = start;
	Pointer++;
	*Pointer = *buffer;
	Pointer++;
	*Pointer = CRC8(BufferCRC, 3, CRC_KEY);

	for (i = 1; i < len; i++) {
		Pointer++;
		buffer++;
		*Pointer = *buffer;
		*(Pointer + 1) = CRC8(Pointer, 1, CRC_KEY);
		Pointer++;
	}
	result = I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, BufferCRC + 1,
			2 * len + 1, &SentByte);
	return result;

}

int32_t bq_write_reg_word_with_crc_1(BQ_Hw *p_bq, uint8_t reg, uint32_t data) {
	unsigned char DataBuffer[6];
	unsigned int SentByte = 0;

	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = reg;
	DataBuffer[2] = LOW_BYTE(data);
	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);
	DataBuffer[4] = HIGH_BYTE(data);
	DataBuffer[5] = CRC8(DataBuffer + 4, 1, CRC_KEY);

	return (I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1, 5,
			&SentByte));
}

int32_t bq_write_reg_byte_with_crc_1(BQ_Hw *p_bq, uint8_t reg, uint8_t data) {

	unsigned char DataBuffer[4];
	unsigned int SentByte = 0;

	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = reg;
	DataBuffer[2] = data;
	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);

	return (I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1, 3,
			&SentByte));
}

/*
 *  function for bq 76952
 *  i2c read function: byte, word, block, block with crc, read data ram
*/
int32_t bq_i2c_read_reg_byte(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer){
	unsigned char TargetRegister = reg_addr;
	unsigned char ReadData[1];
	unsigned int ReadDataCount = 0;
	int ReadStatus = 0;

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_BUSY, RESET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}
	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(p_bq->i2c_module, p_bq->i2c_address, 1,
			I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TXIS, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -2;
	}

	/* Send Register address */
	I2C_SendData(p_bq->i2c_module, TargetRegister);

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TC, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -3;
	}

	ReadStatus = I2CReadBytes(p_bq->i2c_module, p_bq->i2c_address, ReadData, 1,
			&ReadDataCount);

	if (ReadStatus != 0) {
		return -4;
	}

	*buffer = ReadData[0];
	return 0;
}

int32_t bq_i2c_read_reg_word(BQ_Hw* p_bq, uint8_t reg_addr, uint16_t* buffer){
		unsigned char TargetRegister = reg_addr;
		unsigned char ReadData[2];
		unsigned int ReadDataCount = 0;
		int ReadStatus = 0;
		if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_BUSY, RESET,
				I2C_TIMEOUT) == false) {
			p_bq->reset_i2c_bus();
			return -1;
		}

		/* Configure slave address, nbytes, reload, end mode and start or stop generation */
		I2C_TransferHandling(p_bq->i2c_module, p_bq->i2c_address, 1,
				I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TXIS, SET,
				I2C_TIMEOUT) == false) {
			p_bq->reset_i2c_bus();
			return -2;
		}

		/* Send Register address */
		I2C_SendData(p_bq->i2c_module, TargetRegister);
		if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TC, SET,
				I2C_TIMEOUT) == false) {
			p_bq->reset_i2c_bus();
			return -3;
		}

		ReadStatus = I2CReadBytes(p_bq->i2c_module, p_bq->i2c_address, ReadData, 2,
				&ReadDataCount);

		if (ReadStatus != 0) {
			return -4;
		}

		*buffer = ReadData[1];

		*buffer = (*buffer << 8) + ReadData[0];

		return 0;
}

int32_t bq_i2c_read_reg_word_with_sign(BQ_Hw* p_bq, uint8_t reg_addr, int16_t* buffer){
	unsigned char TargetRegister = reg_addr;
	unsigned char ReadData[2];
	unsigned int ReadDataCount = 0;
	int ReadStatus = 0;
	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_BUSY, RESET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(p_bq->i2c_module, p_bq->i2c_address, 1,
			I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TXIS, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -2;
	}

	/* Send Register address */
	I2C_SendData(p_bq->i2c_module, TargetRegister);
	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TC, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -3;
	}

	ReadStatus = I2CReadBytes(p_bq->i2c_module, p_bq->i2c_address, ReadData, 2,
			&ReadDataCount);

	if (ReadStatus != 0) {
		return -4;
	}

	*buffer = ReadData[1];

	*buffer = (*buffer << 8) + ReadData[0];

	return 0;
}

int32_t bq_i2c_read_reg_block(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length){

		unsigned char TargetRegister = reg_addr;
		uint8_t StartData[255];
		unsigned int ReadDataCount = 0;
		uint8_t *ReadData = StartData;
		int ReadStatus = 0;
		int i;

		if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_BUSY, RESET,
				I2C_TIMEOUT) == false) {
			p_bq->reset_i2c_bus();
			return -1;
		}
		/* Configure slave address, nbytes, reload, end mode and start or stop generation */
		I2C_TransferHandling(p_bq->i2c_module, p_bq->i2c_address, 1,
				I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TXIS, SET,
				I2C_TIMEOUT) == false) {
			p_bq->reset_i2c_bus();
			return -2;
		}

		/* Send Register address */
		I2C_SendData(p_bq->i2c_module, TargetRegister);
		if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TC, SET,
				I2C_TIMEOUT) == false) {
			p_bq->reset_i2c_bus();
			return -3;
		}

		ReadStatus = I2CReadBytes(p_bq->i2c_module, p_bq->i2c_address, ReadData,
				length, &ReadDataCount);

		if (ReadStatus != 0) {
			return -4;
		}

		for (i = 1; i <= length; i++) {
			*buffer = *ReadData;
			ReadData++;
			buffer++;
		}
		return 0;
}

int32_t bq_i2c_read_reg_block_with_sign(BQ_Hw* p_bq, uint8_t reg_addr, int8_t* buffer, uint8_t length){

	unsigned char TargetRegister = reg_addr;
	uint8_t StartData[255];
	unsigned int ReadDataCount = 0;
	uint8_t *ReadData = StartData;
	int ReadStatus = 0;
	int i;

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_BUSY, RESET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}
	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(p_bq->i2c_module, p_bq->i2c_address, 1,
			I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TXIS, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -2;
	}

	/* Send Register address */
	I2C_SendData(p_bq->i2c_module, TargetRegister);
	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TC, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -3;
	}

	ReadStatus = I2CReadBytes(p_bq->i2c_module, p_bq->i2c_address, ReadData,
			length, &ReadDataCount);

	if (ReadStatus != 0) {
		return -4;
	}

	for (i = 1; i <= length; i++) {
		*buffer = *ReadData;
		ReadData++;
		buffer++;
	}
	return 0;
}

int32_t bq_i2c_read_reg_block_with_crc(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length){
	unsigned char TargetRegister = reg_addr;
	uint8_t StartData[255];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[4];
	unsigned char new_CRC = 0;
	uint8_t *ReadData = StartData;
	int ReadStatus = 0;
	int i;

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_BUSY, RESET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -1;
	}
	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(p_bq->i2c_module, p_bq->i2c_address, 1,
			I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TXIS, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -2;
	}

	/* Send Register address */
	I2C_SendData(p_bq->i2c_module, TargetRegister);
	if (i2c_wait_for_flag_status(p_bq->i2c_module, I2C_FLAG_TC, SET,
			I2C_TIMEOUT) == false) {
		p_bq->reset_i2c_bus();
		return -3;
	}

	ReadStatus = I2CReadBytes(p_bq->i2c_module, p_bq->i2c_address, ReadData,
			2 * length, &ReadDataCount);

	if (ReadStatus != 0) {
		return -4;
	}
	CRCInput[0] = p_bq->i2c_address;
	CRCInput[1] = TargetRegister;
	CRCInput[2] = (p_bq->i2c_address) + 1;
	CRCInput[3] = *ReadData;

	new_CRC = CRC8(CRCInput, 4, CRC_KEY);

	ReadData++;
	if (new_CRC != *ReadData) {
		return -5;
	} else
		*buffer = *(ReadData - 1);

	for (i = 1; i < length; i++) {
		ReadData++;
		new_CRC = CRC8(ReadData, 1, CRC_KEY);
		ReadData++;
		buffer++;

		if (new_CRC != *ReadData) {
			return -6;
		} else
			*buffer = *(ReadData - 1);
	}
	return 0;
}

int32_t bq_i2c_data_ram_read(BQ_Hw* p_bq,  uint16_t reg_addr, uint8_t* buffer, uint8_t length){
	int32_t check_write, check_read;
	check_write = bq_i2c_write_reg_word(p_bq , WRITE_ADDR_REG , reg_addr);
	if(check_write != 0){
		return -1;
	}

	hw_delay_ms(2);

	check_read = bq_i2c_read_reg_block(p_bq, READ_ADDR_REG, buffer, length);
	if(check_read !=0){
		return -2;
	}
	return 0;
}

int32_t bq_i2c_subcommand_read_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t *buffer){
	int32_t check_write,  check_read;
	check_write = bq_i2c_write_reg_word(p_bq , WRITE_ADDR_REG , reg_addr);
	if(check_write != 0){
		return -1;
	}
	hw_delay_ms(2);

	check_read = bq_i2c_read_reg_word(p_bq, READ_ADDR_REG, buffer);
	if(check_read !=0){
		return -2;
	}
	return 0;
}
int32_t bq_i2c_subcommand_read_block(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t* buffer, uint8_t length){
	int32_t check_write, check_read;
	check_write = bq_i2c_write_reg_word(p_bq , WRITE_ADDR_REG , reg_addr);
	if(check_write != 0){
		return -1;
	}
	hw_delay_ms(2);

	check_read = bq_i2c_read_reg_block(p_bq, READ_ADDR_REG, buffer, length);
	if(check_read !=0){
		return -2;
	}
	return 0;
}

int32_t bq_i2c_subcommand_read_block_with_sign(BQ_Hw* p_bq, uint16_t reg_addr, int8_t* buffer, uint8_t length){
	int32_t check_write, check_read;
	check_write = bq_i2c_write_reg_word(p_bq , WRITE_ADDR_REG , reg_addr);
	if(check_write != 0){
		return -1;
	}
	hw_delay_ms(2);

	check_read = bq_i2c_read_reg_block_with_sign(p_bq, READ_ADDR_REG, buffer, length);
	if(check_read !=0){
		return -2;
	}
	return 0;
}

int32_t bq_i2c_data_ram_read_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t * buffer){
	int32_t check_write, check_read;
	check_write = bq_i2c_write_reg_word(p_bq , WRITE_ADDR_REG , reg_addr);
	if(check_write != 0){
		return -1;
	}

	hw_delay_ms(2);

	check_read = bq_i2c_read_reg_word(&bq_hw, READ_ADDR_REG, buffer);
	if(check_read !=0){
		return -2;
	}
	return 0;
}
int32_t bq_i2c_data_ram_read_byte(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t* buffer){
	int32_t check_write, check_read;
	check_write = bq_i2c_write_reg_word(p_bq , WRITE_ADDR_REG , reg_addr);
	if(check_write != 0){
		return -1;
	}

	hw_delay_ms(2);

	check_read = bq_i2c_read_reg_byte(p_bq, READ_ADDR_REG, buffer);
	if(check_read !=0){
		return -2;
	}
	return 0;
}

int32_t bq_i2c_data_ram_read_word_with_sign(BQ_Hw* p_bq, uint16_t reg_addr, int16_t *buffer){
	int32_t check_write, check_read;
	check_write = bq_i2c_write_reg_word(p_bq , WRITE_ADDR_REG , reg_addr);
	if(check_write != 0){
		return -1;
	}

	hw_delay_ms(2);

	check_read = bq_i2c_read_reg_word_with_sign(&bq_hw, READ_ADDR_REG, buffer);
	if(check_read !=0){
		return -2;
	}
	return 0;
}
int32_t bq_i2c_data_ram_read_reg_with_float(BQ_Hw* p_bq, uint16_t reg_addr, float *buffer){
	int32_t check_write, check_read;
	uint8_t data[4];
	check_write = bq_i2c_write_reg_word(p_bq , WRITE_ADDR_REG , reg_addr);
	if(check_write != 0){
		return -1;
	}

	hw_delay_ms(2);

	check_read = bq_i2c_read_reg_block(&bq_hw, READ_ADDR_REG, data, 4);
	if(check_read !=0){
		return -2;
	}
	*buffer = (data[3] << 24);
	*buffer += (data[2]<<16);
	*buffer += (data[1]<<8);
	*buffer += data[0];
	return 0;
}
/*
 * i2c write function: byte, word, block, block with crc, write data ram
 */
int32_t bq_i2c_write_reg_byte(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t reg_data){
	unsigned char DataBuffer[3];
	unsigned int SentByte = 0;

	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = reg_addr;
	DataBuffer[2] = reg_data;
//	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);

	return (I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1, 2,
			&SentByte));
}
int32_t bq_i2c_write_reg_word(BQ_Hw* p_bq, uint8_t reg_addr, uint16_t reg_data){
	unsigned char DataBuffer[4];
	unsigned int SentByte = 0;

	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = reg_addr;
	DataBuffer[2] = LOW_BYTE(reg_data);
	DataBuffer[3] = HIGH_BYTE(reg_data);

	return (I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1, 3,
			&SentByte));
}
int32_t bq_i2c_write_reg_block(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length){
	unsigned char DataBuffer[255] = { 0 };
	uint8_t *Pointer = DataBuffer;
	int i;
	unsigned int SentByte = 0;
	int result;

	Pointer = DataBuffer;
	*Pointer = p_bq->i2c_address;
	Pointer++;
	*Pointer = reg_addr;
	Pointer++;
	*Pointer = *buffer;

	for (i = 1; i < length; i++) {
		Pointer++;
		buffer++;
		*Pointer = *buffer;
	}
	result = I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1,
			length + 1, &SentByte);
	return result;
}
int32_t bq_i2c_wirte_reg_block_with_crc(BQ_Hw* p_bq, uint8_t reg_addr, uint8_t* buffer, uint8_t length){
	unsigned char BufferCRC[255] = { 0 };
	uint8_t *Pointer = BufferCRC;
	int i;
	unsigned int SentByte = 0;
	int result;

	Pointer = BufferCRC;
	*Pointer = p_bq->i2c_address;
	Pointer++;
	*Pointer = reg_addr;
	Pointer++;
	*Pointer = *buffer;
	Pointer++;
	*Pointer = CRC8(BufferCRC, 3, CRC_KEY);

	for (i = 1; i < length; i++) {
		Pointer++;
		buffer++;
		*Pointer = *buffer;
		*(Pointer + 1) = CRC8(Pointer, 1, CRC_KEY);
		Pointer++;
	}
	result = I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, BufferCRC + 1,
			2 * length + 1, &SentByte);
	return result;
}
int32_t bq_i2c_data_ram_write(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t* buffer, uint8_t length){
	// Write register address 0x3E flow by data
	unsigned char DataBuffer[255] = { 0 };
	uint8_t BufferChecksum[20] ={ 0 };
	uint8_t *Pointer = DataBuffer;
	uint8_t *p_checksum = BufferChecksum;
	uint16_t TX_2Byte;
	int i;
	unsigned int SentByte = 0;
	int result1, result2;

	Pointer = DataBuffer;
	*Pointer = p_bq->i2c_address;
	Pointer++;
	*Pointer = WRITE_ADDR_REG;
	Pointer++;
	*Pointer = LOW_BYTE(reg_addr);
	Pointer++;
	*Pointer = HIGH_BYTE(reg_addr);
	Pointer++;
	*Pointer = *buffer;

	for (i = 1; i < length; i++) {
		Pointer++;
		buffer++;
		*Pointer = *buffer;
	}
	result1 = I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1,
			length + 3, &SentByte);

	if(result1 != 0){
		return -1;
	}
	hw_delay_ms(2);
	// write to register 0x60 with checksum and length
	p_checksum = BufferChecksum;
	*p_checksum = LOW_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = HIGH_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = *buffer;

	for (i = 1; i < length; i++) {
		p_checksum++;
		buffer++;
		*p_checksum = *buffer;
	}
	TX_2Byte = check_sum(BufferChecksum, length+2);
	TX_2Byte = ((length+4)<<8)+ TX_2Byte;
	result2 = bq_i2c_write_reg_word(p_bq, WRITE_RAM_ADDR_REG_CHECKSUM, TX_2Byte);

	if(result2 != 0){
		return -1;
	}
	return 0;
}
int32_t bq_i2c_data_ram_write_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t reg_data){
	// Write register address 0x3E flow by data
	unsigned char DataBuffer[6] = { 0 };
	uint8_t BufferChecksum[20] ={ 0 };
	uint8_t *p_checksum = BufferChecksum;
	uint16_t TX_2Byte;
	unsigned int SentByte = 0;
	int result1, result2;


	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = WRITE_ADDR_REG;
	DataBuffer[2] = LOW_BYTE(reg_addr);
	DataBuffer[3] = HIGH_BYTE(reg_addr);
	DataBuffer[4] = LOW_BYTE(reg_data);
	DataBuffer[5] = HIGH_BYTE(reg_data);

	result1 = I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1,
			5, &SentByte);

	if(result1 != 0){
		return -1;
	}
	hw_delay_ms(2);
	// write to register 0x60 with checksum and length
	p_checksum = BufferChecksum;
	*p_checksum = LOW_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = HIGH_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = LOW_BYTE(reg_data);
	p_checksum++;
	*p_checksum = HIGH_BYTE(reg_data);

	TX_2Byte = check_sum(BufferChecksum, 4);
	TX_2Byte = (6<<8)+ TX_2Byte;
	result2 = bq_i2c_write_reg_word(p_bq, WRITE_RAM_ADDR_REG_CHECKSUM, TX_2Byte);

	if(result2 != 0){
		return -1;
	}
	return 0;
}

int32_t bq_i2c_data_ram_write_word_with_sign(BQ_Hw* p_bq, uint16_t reg_addr, int16_t reg_data){
	// Write register address 0x3E flow by data
	unsigned char DataBuffer[6] = { 0 };
	uint8_t BufferChecksum[20] ={ 0 };
	uint8_t *p_checksum = BufferChecksum;
	uint16_t TX_2Byte;
	unsigned int SentByte = 0;
	int result1, result2;


	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = WRITE_ADDR_REG;
	DataBuffer[2] = LOW_BYTE(reg_addr);
	DataBuffer[3] = HIGH_BYTE(reg_addr);
	DataBuffer[4] = LOW_BYTE(reg_data);
	DataBuffer[5] = HIGH_BYTE(reg_data);

	result1 = I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1,
			5, &SentByte);

	if(result1 != 0){
		return -1;
	}
	hw_delay_ms(2);
	// write to register 0x60 with checksum and length
	p_checksum = BufferChecksum;
	*p_checksum = LOW_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = HIGH_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = LOW_BYTE(reg_data);
	p_checksum++;
	*p_checksum = HIGH_BYTE(reg_data);

	TX_2Byte = check_sum(BufferChecksum, 4);
	TX_2Byte = (6<<8)+ TX_2Byte;
	result2 = bq_i2c_write_reg_word(p_bq, WRITE_RAM_ADDR_REG_CHECKSUM, TX_2Byte);

	if(result2 != 0){
		return -1;
	}
	return 0;
}
int32_t bq_i2c_data_ram_write_byte(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t reg_data){
		// Write register address 0x3E flow by data
	unsigned char DataBuffer[5] = { 0 };
	uint8_t BufferChecksum[20] ={ 0 };
	uint8_t *p_checksum = BufferChecksum;
	uint16_t TX_2Byte;
	unsigned int SentByte = 0;
	int result1, result2;


	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = WRITE_ADDR_REG;
	DataBuffer[2] = LOW_BYTE(reg_addr);
	DataBuffer[3] = HIGH_BYTE(reg_addr);
	DataBuffer[4] = reg_data;

	result1 = I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1,
			4, &SentByte);

	if(result1 != 0){
		return -1;
	}
	hw_delay_ms(2);
	// write to register 0x60 with checksum and length
	p_checksum = BufferChecksum;
	*p_checksum = LOW_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = HIGH_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = reg_data;

	TX_2Byte = check_sum(BufferChecksum, 3);
	TX_2Byte = (5<<8)+ TX_2Byte;
	result2 = bq_i2c_write_reg_word(p_bq, WRITE_RAM_ADDR_REG_CHECKSUM, TX_2Byte);

	if(result2 != 0){
		return -1;
	}
	return 0;
}
int32_t bq_i2c_subcommand_write_word(BQ_Hw* p_bq, uint16_t reg_addr, uint16_t reg_data){
	// Write register address 0x3E flow by data
	unsigned char DataBuffer[6] = { 0 };
	uint8_t BufferChecksum[20] ={ 0 };
	uint8_t *p_checksum = BufferChecksum;
	uint16_t TX_2Byte;
	unsigned int SentByte = 0;
	int result1, result2;

	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = WRITE_ADDR_REG;
	DataBuffer[2] = LOW_BYTE(reg_addr);
	DataBuffer[3] = HIGH_BYTE(reg_addr);
	DataBuffer[4] = LOW_BYTE(reg_data);
	DataBuffer[5] = HIGH_BYTE(reg_data);

	result1 = I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1,
			5, &SentByte);

	if(result1 != 0){
		return -1;
	}
	hw_delay_ms(2);
	// write to register 0x60 with checksum and length
	p_checksum = BufferChecksum;
	*p_checksum = LOW_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = HIGH_BYTE(reg_addr);
	p_checksum++;
	*p_checksum = LOW_BYTE(reg_data);
	p_checksum++;
	*p_checksum = HIGH_BYTE(reg_data);

	TX_2Byte = check_sum(BufferChecksum, 4);
	TX_2Byte = (6<<8)+ TX_2Byte;
	result2 = bq_i2c_write_reg_word(p_bq, WRITE_RAM_ADDR_REG_CHECKSUM, TX_2Byte);

	if(result2 != 0){
		return -1;
	}
	return 0;
}

int32_t bq_i2c_subcommand_write_byte(BQ_Hw* p_bq, uint16_t reg_addr, uint8_t reg_data){
	// Write register address 0x3E flow by data
	unsigned char DataBuffer[5] = { 0 };
	unsigned int SentByte = 0;
	int result;

	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = WRITE_ADDR_REG;
	DataBuffer[2] = LOW_BYTE(reg_addr);
	DataBuffer[3] = HIGH_BYTE(reg_addr);
	DataBuffer[4] = reg_data;

	result = I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1,
			4, &SentByte);

	if(result != 0){
		return -1;
	}
	return result;
}
int afe_hardware_init_1(void) {
	bq_clock_init_1();
	bq_gpio_init_1();
	bq_interrupt_init_1();
	bq_i2c_init_1();
	bq_config_timer_1();
	bq_hw_init_1();
	return 0;
}

static void bq_gpio_init_1(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = AFE_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(AFE_I2C_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AFE_I2C_SDA_PIN;
	GPIO_Init(AFE_I2C_SDA_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(AFE_I2C_SCL_PORT, AFE_I2C_SCL_SOURCE, AFE_I2C_SCL_AF);
	GPIO_PinAFConfig(AFE_I2C_SDA_PORT, AFE_I2C_SDA_SOURCE, AFE_I2C_SDA_AF);

	GPIO_InitStructure.GPIO_Pin = AFE_ALERT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AFE_ALERT_PORT, &GPIO_InitStructure);

	/* GPIOB Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
#if !GPIO_SOFT_START

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* GPIOA, GPIOB and GPIOE Clocks enable */
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);

	/* GPIOA Configuration: Channel 1 as alternate function push-pull */
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);

	/* GPIOB Configuration: Channel 1 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_0);

	uint16_t TimerPrescaler = (SystemCoreClock / 10000) - 1;

	/* TIM1 clock enable */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TimerPrescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	uint16_t pulse_length = ((TimerPrescaler + 1) * 70) / 100 - 1;
	/* Channel 1 PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = pulse_length;
	//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //low voltage in active mode
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	//TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	//TIM_OC1Init(TIM1, &TIM_OCInitStructure); //TIM_OC1Init for enable the channel 1
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);
	/* TIM1 counter enable */
	//TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM14, ENABLE);
	/* TIM1 Main Output Enable */
	//TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM14, ENABLE);
#endif

}

static void bq_clock_init_1(void) {
	//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
	/* Configure the I2C clock source. The clock is derived from the HSI */
//	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

	/* sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
	RCC_AHBPeriphClockCmd(AFE_I2C_SCL_CLK | AFE_I2C_SDA_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(AFE_ALERT_CLK, ENABLE);

	/* sEE_I2C Periph clock enable */
	RCC_APB1PeriphClockCmd(AFE_I2C_CLK, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
}
static void bq_interrupt_init_1(void) {
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// interrupt 1 for bq
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);

	/* Configure EXTI5 line */
	EXTI_InitStructure.EXTI_Line = AFE_INTERRUPT_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI5_15 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = AFE_INTERRUPT_VECTOR;
	NVIC_InitStructure.NVIC_IRQChannelPriority = AFE_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void bq_i2c_init_1(void) {
	I2C_InitTypeDef I2C_InitStructure;

	/* I2C configuration */
	/* sEE_I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0xAB;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStructure.I2C_Timing =0x00201D2B; /* for HSI clock source */
	I2C_InitStructure.I2C_Timing = 0x00901850; /* for SYSCLK 48MHz  0x10805E89*/
//	I2C_InitStructure.I2C_Timing =0x1080D6FF; /* sysclock 48mhz, i2c freq: 50kgz
	I2C_Init(AFE_I2C_DEVICE, &I2C_InitStructure);

	/* sEE_I2C Peripheral Enable */
	I2C_Cmd(AFE_I2C_DEVICE, ENABLE);
}

static void bq_reset_i2c_bus_1(void) {
//	I2C_Cmd(AFE_I2C_DEVICE, DISABLE);
//
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//	GPIO_InitStructure.GPIO_Pin = AFE_I2C_SCL_PIN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_Init(AFE_I2C_SCL_PORT, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = AFE_I2C_SDA_PIN;
//	GPIO_Init(AFE_I2C_SDA_PORT, &GPIO_InitStructure);
//
//	GPIO_SetBits(AFE_I2C_SCL_PORT, AFE_I2C_SCL_PIN);
//	GPIO_SetBits(AFE_I2C_SDA_PORT, AFE_I2C_SDA_PIN);
//
//	GPIO_InitStructure.GPIO_Pin = AFE_I2C_SCL_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(AFE_I2C_SCL_PORT, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = AFE_I2C_SDA_PIN;
//	GPIO_Init(AFE_I2C_SDA_PORT, &GPIO_InitStructure);
//
//	GPIO_PinAFConfig(AFE_I2C_SCL_PORT, AFE_I2C_SCL_SOURCE, AFE_I2C_SCL_AF);
//	GPIO_PinAFConfig(AFE_I2C_SDA_PORT, AFE_I2C_SDA_SOURCE, AFE_I2C_SDA_AF);
//
//	I2C_Cmd(AFE_I2C_DEVICE, ENABLE);

}

static void bq_config_timer_1(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseInitTypeDef TimBase_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TimBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimBase_InitStructure.TIM_Prescaler = 4799;
	TimBase_InitStructure.TIM_Period = 499;   // 0.05 second
	TimBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimBase_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(AFE_TIMER_DEVICE, &TimBase_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearFlag(AFE_TIMER_DEVICE, TIM_FLAG_Update);
	TIM_ITConfig(AFE_TIMER_DEVICE, TIM_IT_Update, ENABLE);
	TIM_Cmd(AFE_TIMER_DEVICE, ENABLE);
	//Configure timer 14 for CAP
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	TIM_TimeBaseInitTypeDef TimBase_InitStruc;

	uint16_t TimerPrescaler = (SystemCoreClock / 1000) - 1;
	TimBase_InitStruc.TIM_ClockDivision = TIM_CKD_DIV1;
	TimBase_InitStruc.TIM_Prescaler = 0;
	TimBase_InitStruc.TIM_Period = TimerPrescaler;
	TimBase_InitStruc.TIM_CounterMode = TIM_CounterMode_Up;
	TimBase_InitStruc.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM14, &TimBase_InitStruc);
	/* Channel 14 PWM mode */
	TIM_OCInitTypeDef TIM_OCInitStructure;

	uint16_t pulse_length = ((TimerPrescaler + 1) * 70) / 100 - 1;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = pulse_length; //duty 30%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM14, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM14, ENABLE);
}

static uint8_t CRC8(uint8_t *ptr, uint8_t len, uint8_t key) {
	uint8_t i;
	uint8_t crc = 0;
	while (len-- != 0) {
		crc ^= *ptr;
		for (i = 0x80; i != 0; i /= 2) {
			if ((crc & 0x80) != 0) {
				crc *= 2;
				crc ^= key;
			} else
				crc *= 2;
		}
		ptr++;
	}
	return (crc);
}

static bool i2c_wait_for_flag_status(I2C_TypeDef *I2C_DEV, uint32_t flag,
		FlagStatus status, uint32_t timeout_us) {

	uint32_t tick = 0;
	while (I2C_GetFlagStatus(I2C_DEV, flag) != status) {
		tick++;
		hw_delay_us(1);
		if (tick == timeout_us) {
			return false;
		}
	}
	return true;
}

static int I2CSendBytes(I2C_TypeDef *I2C_DEV, unsigned char I2CSlaveAddress,
		unsigned char *DataBuffer, unsigned int ByteCount,
		unsigned int *SentByte) {
	uint32_t DataNum = 0;
	if (i2c_wait_for_flag_status(I2C_DEV, I2C_FLAG_BUSY,RESET,I2C_TIMEOUT) == false)
		return -1;

	/* Configure slave address, nbytes, reload and generate start */
	I2C_TransferHandling(I2C_DEV, I2CSlaveAddress, 1, I2C_Reload_Mode,
			I2C_Generate_Start_Write);

	if (i2c_wait_for_flag_status(I2C_DEV, I2C_FLAG_TXIS,SET,I2C_TIMEOUT) == false)
		return -2;
	/* Send MSB of memory address */

	I2C_SendData(I2C_DEV, DataBuffer[0]);

	if (i2c_wait_for_flag_status(I2C_DEV, I2C_FLAG_TCR,SET,I2C_TIMEOUT) == false)
		return -3;

	/* Update CR2 : set Slave Address , set write request, generate Start and set end mode */
	I2C_TransferHandling(I2C_DEV, I2CSlaveAddress, ByteCount - 1,
			I2C_AutoEnd_Mode, I2C_No_StartStop);
	DataNum = 1;
	while (DataNum != ByteCount) {
		/* Wait until TXIS flag is set */
		if (i2c_wait_for_flag_status(I2C_DEV, I2C_FLAG_TXIS,SET,I2C_TIMEOUT) == false){
			return -4;
		}
		/* Write data to TXDR */
		I2C_SendData(I2C_DEV, DataBuffer[DataNum]);
		/* Update number of transmitted data */
		DataNum++;
	}

	if (i2c_wait_for_flag_status(I2C_DEV, I2C_FLAG_STOPF,SET,I2C_TIMEOUT) == false)
		return -1;
	/* Clear STOPF flag */
	I2C_ClearFlag(I2C_DEV, I2C_ICR_STOPCF);

	/* If all operations OK, return sEE_OK (0) */
	*SentByte = ByteCount;
	return 0;
}

static int I2CReadBytes(I2C_TypeDef *I2C_DEV, unsigned char I2CSlaveAddress,
		unsigned char *DataBuffer, unsigned int ExpectedByteNumber,
		unsigned int *NumberOfReceivedBytes) { /* Configure slave address, nbytes, reload, end mode and start or stop generation */

	uint32_t data_num = 0;
	I2C_TransferHandling(I2C_DEV, I2CSlaveAddress, ExpectedByteNumber,
			I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	/* Reset local variable */
	data_num = 0;
	/* Wait until all data are received */
	while (data_num != ExpectedByteNumber) {
		if (i2c_wait_for_flag_status(I2C_DEV, I2C_FLAG_RXNE,SET,I2C_TIMEOUT) == false)
			return -1;

		/* Read data from RXDR */
		DataBuffer[data_num] = I2C_ReceiveData(I2C_DEV);
		/* Update number of received data */
		data_num++;
	}

	if (i2c_wait_for_flag_status(I2C_DEV, I2C_FLAG_STOPF,SET,I2C_TIMEOUT) == false)
		return -1;

	I2C_ClearFlag(I2C_DEV, I2C_FLAG_STOPF);
	*NumberOfReceivedBytes = ExpectedByteNumber;
	return 0;
}

static uint8_t check_sum(uint8_t *ptr, uint8_t len)
{
	uint8_t i;
	uint8_t checksum = 0;

	for(i=0; i<len; i++)
		checksum += ptr[i];

	checksum = 0xff & ~checksum;

	return(checksum);
}
void EXTI4_15_IRQHandler(void){

	if(EXTI_GetITStatus(EXTI_Line5)==SET){
		if(bq_hw.handle){
			bq_hw.handle(&bq_hw);
		}
		EXTI_ClearFlag(EXTI_Line5);
	}

}
