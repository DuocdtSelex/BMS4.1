#if 0
#include "bq_hardware.h"
#include "stm32f0xx.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"
#include "stdbool.h"
#include "delay_hw.h"

#define CRC_KEY 7
#define I2C_TIMEOUT		 				10000
#define I2C_READ_BLOCK_TIMEOUT		 	1000

#define LOW_BYTE(Data)			(uint8_t)(0xff & Data)
#define HIGH_BYTE(Data)			(uint8_t)(0xff & (Data >> 8))

static void bq_gpio_init(void);
static void bq_clock_init(void);
static void bq_interrupt_init(void);
static void bq_i2c_init(void);
static void bq_config_timer(void);

static void bq1_reset_i2c_bus(void);
static void bq2_reset_i2c_bus(void);

static int I2CSendBytes(I2C_TypeDef *I2C_DEV, unsigned char I2CSlaveAddress,
		unsigned char *DataBuffer, unsigned int ByteCount,
		unsigned int *SentByte);
static int I2CReadBytes(I2C_TypeDef *I2C_DEV, unsigned char I2CSlaveAddress,
		unsigned char *DataBuffer, unsigned int ExpectedByteNumber,
		unsigned int *NumberOfReceivedBytes);
static uint8_t CRC8(uint8_t *ptr, uint8_t len, uint8_t key);

static bool i2c_wait_for_flag_status(I2C_TypeDef *I2C_DEV, uint32_t flag,
		FlagStatus status, uint32_t timeout_us);

static void bq1_hw_init(void);
static void bq2_hw_init(void);

struct BQ_Hw_t upper_bq_hw;
struct BQ_Hw_t lower_bq_hw;

static void bq1_hw_init(void) {

	lower_bq_hw.i2c_module = AFE1_I2C_DEVICE;
	lower_bq_hw.i2c_address = AFE_I2C_ADDRESS;
	lower_bq_hw.reset_i2c_bus = bq1_reset_i2c_bus;
}

static void bq2_hw_init(void) {
	upper_bq_hw.i2c_module = AFE2_I2C_DEVICE;
	upper_bq_hw.i2c_address = AFE_I2C_ADDRESS;
	upper_bq_hw.reset_i2c_bus = bq2_reset_i2c_bus;
}

int32_t bq_read_reg_block_with_crc(BQ_Hw *p_bq, uint8_t reg, uint8_t *buffer,
		uint8_t len) {

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

int32_t bq_read_reg_byte_with_crc(BQ_Hw *p_bq, uint8_t reg, uint8_t *data) {
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

int32_t bq_read_reg_word_with_crc(BQ_Hw *p_bq, uint8_t reg, uint16_t *data) {
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

int32_t bq_write_reg_block_with_crc(BQ_Hw *p_bq, uint8_t start, uint8_t *buffer,
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

int32_t bq_write_reg_word_with_crc(BQ_Hw *p_bq, uint8_t reg, uint32_t data) {
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

int32_t bq_write_reg_byte_with_crc(BQ_Hw *p_bq, uint8_t reg, uint8_t data) {

	unsigned char DataBuffer[4];
	unsigned int SentByte = 0;

	DataBuffer[0] = p_bq->i2c_address;
	DataBuffer[1] = reg;
	DataBuffer[2] = data;
	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);

	return (I2CSendBytes(p_bq->i2c_module, p_bq->i2c_address, DataBuffer + 1, 3,
			&SentByte));
}

#if 0

#define MAX_OCD_THRESH		16
const unsigned int OVPThreshold = 4250;
//const unsigned int OVPThreshold = 3250;//test
const unsigned int UVPThreshold = 2800;
//const unsigned int UVPThreshold = 5000;
const unsigned char SCDDelay = SCD_DELAY_70us;

//const unsigned char SCDThresh = SCD_THRESH_44mV_22mV;
//const unsigned char SCDThresh = SCD_THRESH_67mV_33mV;
//const unsigned char SCDThresh = SCD_THRESH_89mV_44mV;
//const unsigned char SCDThresh = SCD_THRESH_111mV_56mV;
//const unsigned char SCDThresh = SCD_THRESH_133mV_67mV;
//const unsigned char SCDThresh = SCD_THRESH_178mV_89mV;
//const unsigned char SCDThresh = SCD_THRESH_200mV_100mV;


const unsigned char OCDDelay = OCD_DELAY_8ms;
const int32_t OCD_Value[MAX_OCD_THRESH] = {8000,11000,14000,17000,19000,22000,25000,28000,31000,33000,36000,39000,42000,44000,47000,50000};
const unsigned char OCP_LEVEL = 0;
const unsigned char OCDThresh = OCD_THRESH_89mV_44mV;
//const unsigned char OCDThresh = 0x09;
//const unsigned char SCDThresh = 2;
const unsigned char SCDThresh = SCD_THRESH_111mV_56mV;

//const unsigned char OCP_LEVEL = 0;
//const unsigned char OCDThresh =OCD_THRESH_17mV_8mV;
//const unsigned char OCDThresh =OCD_THRESH_22mV_11mV;
//const unsigned char OCDThresh =OCD_THRESH_28mV_14mV;
//const unsigned char OCDThresh =OCD_THRESH_33mV_17mV;
//const unsigned char OCDThresh =OCD_THRESH_39mV_19mV;
//const unsigned char OCDThresh =OCD_THRESH_44mV_22mV;
//const unsigned char OCDThresh =OCD_THRESH_50mV_25mV;
//const unsigned char OCDThresh =OCD_THRESH_56mV_28MV;
//const unsigned char OCDThresh =OCD_THRESH_61mV_31mV;
//const unsigned char OCDThresh =OCD_THRESH_67mV_33mV;
//const unsigned char OCDThresh =OCD_THRESH_72mV_36mV;
//const unsigned char OCDThresh =OCD_THRESH_78mV_39mV;
//const unsigned char OCDThresh =OCD_THRESH_83mV_42mV;
//const unsigned char OCDThresh =OCD_THRESH_89mV_44mV;
//const unsigned char OCDThresh =OCD_THRESH_94mV_47mV;
//const unsigned char OCDThresh =OCD_THRESH_100mV_50mV;

const unsigned char OVDelay = OV_DELAY_1s;
const unsigned char UVDelay = UV_DELAY_1s;

uint16_t* ntc_lookups=NULL;
uint16_t CellVoltage[CELL_NUM];
int32_t afe_current_adc = 0;
int32_t ocp_threshold = 0;
uint32_t p_vol = 0;
volatile uint16_t p_vol1;
volatile uint16_t p_vol2;
uint8_t balancing_bits[CELL_GROUP_NUM];
float Gain1 = 0,Gain2 = 0;
int iGain1 = 0,iGain2 = 0;
static uint8_t ntc_buffer[12]={0};

int16_t thermistor_values[NTC_NUM];

/** Private helper functions *************************************************/


static bool is_check_balancing_complete(const uint8_t group,const uint8_t* ignore_check_cell);
static void get_min_cell(uint16_t* voltage, uint8_t *min_cell_index);
static void set_group_balancing_bits(const uint8_t group, const uint16_t min_voltage, uint8_t min_cell_index,uint8_t* ignore_check_max_voltage);
static void search_group_max_cell(const uint8_t start_index, const uint8_t finish_index,const uint8_t* ignore_check_cell,
	uint8_t* max_cell);
static void search_group_min_cell(const uint8_t start_index, const uint8_t finish_index, uint8_t* min_cell);
static void set_balancing_bit(uint8_t* balancing_byte,const uint8_t bit);
static void clear_balancing_bit(uint8_t* balancing_byte,const uint8_t bit);
static bool is_need_balancing(const uint16_t min_voltage,
		const uint8_t index);

static int bq_set_protect_params(void);
static void get_max_cell(uint16_t* voltage);



int afe_setup(void) {
	int result = 0;
	uint8_t state = 0;

	ntc_lookups = (uint16_t*) mf58_lookups;
	result = bq_set_protect_params();
	if (result != 0)
		goto init_fault_handle;

	result = config_afe();
	if (result != 0)
		goto init_fault_handle;

	set_ocd_protection_level(AFE1_I2C_DEVICE, OCP_LEVEL);
	set_ocd_protection_level(AFE2_I2C_DEVICE, OCP_LEVEL);

	result = enable_adc(AFE1_I2C_DEVICE);
	if (result != 0)
		goto init_fault_handle;

	result = enable_adc(AFE2_I2C_DEVICE);
	if (result != 0)
		goto init_fault_handle;

	result = enalbe_coulump_counter(AFE1_I2C_DEVICE, ALWAYS_ON);
	if (result != 0)
		goto init_fault_handle;

	result = enalbe_coulump_counter(AFE2_I2C_DEVICE, ALWAYS_ON);
	if (result != 0)
		goto init_fault_handle;

	afe_stop_balancing(AFE1_I2C_DEVICE);
	afe_stop_balancing(AFE2_I2C_DEVICE);
	//set_debug_fault_delay();
	read_status_flags(AFE1_I2C_DEVICE, &state);
	while (state != 0) {
		clear_status_flags(AFE1_I2C_DEVICE, 0xff);
		read_status_flags(AFE1_I2C_DEVICE, &state);
	}

	state = 0;
	read_status_flags(AFE2_I2C_DEVICE, &state);
	while (state != 0) {
		clear_status_flags(AFE2_I2C_DEVICE, 0xff);
		read_status_flags(AFE2_I2C_DEVICE, &state);
	}
	delay_ms(100);
	global_interrupt_enable();
	return 0;
	init_fault_handle: return result;
}

uint16_t* afe_get_cell_voltates(){
	return CellVoltage;
}
uint8_t* afe_get_balancing_mask(){
	return balancing_bits;
}

void afe_read_current_adc(I2C_TypeDef* i2c_dev){
	int16_t adc=0;
    int32_t cur=0;
	int success=-1;
	while(success !=0){
		success= I2CReadRegisterWordWithCRC(i2c_dev,AFE_I2C_ADDRESS,CC_HI_BYTE,&adc);
	//	USART_Write_String("afe read current adc");
	}
	afe_current_adc=(int32_t)adc;
    afe_current_adc*=AFE_CURRENT_FACTOR;
}

void set_debug_fault_delay(I2C_TypeDef* i2c_dev){
	int success=-1;
	while(success !=0){
		success=set_register_bit(i2c_dev,SYS_CTRL2,DELAY_DIS_BIT);
	}
}

int enable_adc(I2C_TypeDef* i2c_dev){
	return set_register_bit(i2c_dev,SYS_CTRL1,ADC_EN_BIT);
}
int disable_adc(I2C_TypeDef* i2c_dev){
    return 0;
}
int enalbe_coulump_counter(I2C_TypeDef* i2c_dev,CURRENT_CONVERT_MODE current_convert_mode){
	int success=0;
	uint8_t control_reg=0;
	success = I2CReadRegisterByteWithCRC(i2c_dev,AFE_I2C_ADDRESS,SYS_CTRL2,&control_reg);
	if(success !=0) return success;

	if(current_convert_mode){
		control_reg |= CC_ONESHOT_BIT;
		control_reg &= ~CC_EN_BIT;
	}
	else{
		control_reg |= CC_EN_BIT;
	}
	success= I2CWriteRegisterByteWithCRC(i2c_dev,AFE_I2C_ADDRESS,SYS_CTRL2,control_reg);
	return success;
}

 int afe_read_balancing_status(I2C_TypeDef* i2c_dev,uint8_t balancing_bits[]){
	int success=-1;
	if(i2c_dev == AFE1_I2C_DEVICE){
		success = I2CReadBlockWithCRC(i2c_dev,AFE_I2C_ADDRESS,CELLBAL1,balancing_bits,CELL_GROUP_NUM);
	}else if (i2c_dev == AFE2_I2C_DEVICE){
		success = I2CReadBlockWithCRC(i2c_dev,AFE_I2C_ADDRESS,CELLBAL1,balancing_bits + 2,CELL_GROUP_NUM);
	}
	return success;
}


void afe_calculate_balancing(){
	uint16_t min_cell_voltage;
	uint8_t cell_group_index = 0;
	uint8_t ignore_check_max_voltage[16] = { 0 };

	uint8_t min_cell_index;
	get_min_cell(&min_cell_voltage, &min_cell_index);
	for (cell_group_index = 0; cell_group_index < CELL_GROUP_NUM; cell_group_index++) {
		balancing_bits[cell_group_index] = 0;
		set_group_balancing_bits(cell_group_index, min_cell_voltage, min_cell_index, ignore_check_max_voltage);
	}
}

int GetADCGainOffset() {
	int result;

	result = I2CReadRegisterByteWithCRC(AFE1_I2C_DEVICE, AFE_I2C_ADDRESS, ADCGAIN1, &(Registers1.ADCGain1.ADCGain1Byte));
	result = I2CReadRegisterByteWithCRC(AFE1_I2C_DEVICE, AFE_I2C_ADDRESS, ADCGAIN2, &(Registers1.ADCGain2.ADCGain2Byte));
	result = I2CReadRegisterByteWithCRC(AFE1_I2C_DEVICE, AFE_I2C_ADDRESS, ADCOFFSET, &(Registers1.ADCOffset));

	result = I2CReadRegisterByteWithCRC(AFE2_I2C_DEVICE, AFE_I2C_ADDRESS, ADCGAIN1, &(Registers2.ADCGain1.ADCGain1Byte));
	result = I2CReadRegisterByteWithCRC(AFE2_I2C_DEVICE, AFE_I2C_ADDRESS, ADCGAIN2, &(Registers2.ADCGain2.ADCGain2Byte));
	result = I2CReadRegisterByteWithCRC(AFE2_I2C_DEVICE, AFE_I2C_ADDRESS, ADCOFFSET, &(Registers2.ADCOffset));

	return result;
}

int config_afe() {
	int result = 0;
	unsigned char bms_protection_config1[5], bms_protection_config2[5];
	result = -1;
	while (result != 0) {
		result = I2CWriteRegisterByteWithCRC(AFE1_I2C_DEVICE, AFE_I2C_ADDRESS, CC_CFG, 0x19);
		result = I2CWriteRegisterByteWithCRC(AFE2_I2C_DEVICE, AFE_I2C_ADDRESS, CC_CFG, 0x19);
	}

	result = I2CWriteBlockWithCRC(AFE1_I2C_DEVICE, AFE_I2C_ADDRESS, PROTECT1, &(Registers1.Protect1.Protect1Byte), 5);
	result = I2CWriteBlockWithCRC(AFE2_I2C_DEVICE, AFE_I2C_ADDRESS, PROTECT1, &(Registers2.Protect1.Protect1Byte), 5);
	if (result != 0)
		return result;

	result = I2CReadBlockWithCRC(AFE1_I2C_DEVICE, AFE_I2C_ADDRESS, PROTECT1, bms_protection_config1, 5);
	result = I2CReadBlockWithCRC(AFE2_I2C_DEVICE, AFE_I2C_ADDRESS, PROTECT1, bms_protection_config2, 5);

	if (bms_protection_config1[0] != Registers1.Protect1.Protect1Byte
			|| bms_protection_config1[1] != Registers1.Protect2.Protect2Byte
			|| bms_protection_config1[2] != Registers1.Protect3.Protect3Byte
			|| bms_protection_config1[3] != Registers1.OVTrip
			|| bms_protection_config1[4] != Registers1.UVTrip) {
		result = -1;
	}

	if (bms_protection_config2[0] != Registers2.Protect1.Protect1Byte
			|| bms_protection_config2[1] != Registers2.Protect2.Protect2Byte
			|| bms_protection_config2[2] != Registers2.Protect3.Protect3Byte
			|| bms_protection_config2[3] != Registers2.OVTrip
			|| bms_protection_config2[4] != Registers2.UVTrip) {
		result = -1;
	}

	return result;
}

 int32_t* afe_get_ocp_threshold(void){
	ocp_threshold = (int32_t)OCD_Value[OCDThresh]*(1+ OCP_LEVEL);
	return (&ocp_threshold);
}
static bool is_check_balancing_complete(const uint8_t group,const uint8_t* ignore_check_cell){
	uint8_t start_index = 4 * group;
	uint8_t finish_index = start_index + 4;
	uint8_t cell_id = start_index;
	bool check = true;
	while (cell_id <= finish_index){
		if (!ignore_check_cell[cell_id])
		{
			check = false;
			break;
		}
		cell_id++;
	}
	return check;
}

static void get_min_cell(uint16_t* voltage, uint8_t *min_cell_index){
	uint8_t cell_index;
	uint16_t min_voltage=65535;
	for(cell_index=0;cell_index<CELL_NUM;cell_index++){
		if(CellVoltage[cell_index] < min_voltage){
			min_voltage=CellVoltage[cell_index];
			*min_cell_index = cell_index;
		}
	}
	*voltage = min_voltage;

}

static void get_max_cell(uint16_t* voltage){
	uint8_t cell_index;
	uint16_t max_voltage=0;
	for(cell_index=0;cell_index<CELL_NUM;cell_index++){
		if(CellVoltage[cell_index] > max_voltage){
			max_voltage=CellVoltage[cell_index];
		}
	}
	*voltage = max_voltage;
}

uint8_t afe_is_balancing_complete(void){
	uint16_t min_voltage = 0,max_voltage = 0;
	uint8_t min_volt_index;
	get_min_cell(&min_voltage, &min_volt_index);
	get_max_cell(&max_voltage);
	if(max_voltage < BALANCING_VOLTAGE_THRESHOLD)
		return true;
	if(max_voltage < (min_voltage + BALANCING_OFFSET))
		return true;
	return false;
}


static void set_balancing_bit(uint8_t* balancing_byte,const uint8_t bit){
	*balancing_byte |= (1<<bit);
}
static void clear_balancing_bit(uint8_t* balancing_byte,const uint8_t bit){
	*balancing_byte &= ~(1<<bit);
}
static bool is_need_balancing(const uint16_t min_voltage,const uint8_t index){
/*
	uint16_t CellVoltage_temp[20];
	uint8_t j = 0;
	for (int i = 0; i < 20; i++) {
		CellVoltage_temp[i] = CellVoltage[j];
		j += 1;
		if (i == 3 || i == 7 || i == 11 || i == 15)
			j -= 1;
	}
*/
	if(CellVoltage[index] > (min_voltage + BALANCING_OFFSET ) && CellVoltage[index] > BALANCING_VOLTAGE_THRESHOLD)
		return true;
	return false;
}
static void search_group_max_cell(const uint8_t start_index, const uint8_t finish_index,const uint8_t* ignore_check_cell,uint8_t* max_cell){

	uint8_t cell_index = start_index;
	uint16_t max_group_voltage = 0;
/*
	uint16_t CellVoltage_temp[20];
	uint8_t j=0;
		for (int i =0; i< 20; i++){
			CellVoltage_temp[i] = CellVoltage[j];
			j+=1;
			if (i==3 || i==7 || i==11 || i == 15)
				j-=1;
		}
*/
	while(ignore_check_cell[cell_index++]!=0);
	max_group_voltage = CellVoltage[--cell_index];
	*max_cell = cell_index;
	for (cell_index = start_index + 1; cell_index < finish_index; cell_index++){
		if (ignore_check_cell[cell_index] == 0){
			if (CellVoltage[cell_index] > max_group_voltage){
				max_group_voltage = CellVoltage[cell_index];
				*max_cell = cell_index;
			}
		}
}
}

static void search_group_min_cell(const uint8_t start_index,
		const uint8_t finish_index, uint8_t *min_cell) {

	uint8_t cell_index = start_index;
	uint16_t min_group_voltage = 0;
	min_group_voltage = CellVoltage[cell_index];
	*min_cell = cell_index;
	for (cell_index = start_index + 1; cell_index < finish_index;
			cell_index++) {
		if (CellVoltage[cell_index] < min_group_voltage) {
			min_group_voltage = CellVoltage[cell_index];
			*min_cell = cell_index;
		}
	}
}


static void set_group_balancing_bits(const uint8_t group, const uint16_t min_voltage, uint8_t min_cell_index,uint8_t* ignore_check_max_voltage){
#if 0
	uint8_t start_cell_index = 0;
	uint8_t finish_cell_index = 0;
	uint8_t group_max_cell_index;
	uint8_t group_min_cell_index;
	uint8_t balancing_pattern = 0;
	start_cell_index = 4 * group;
	finish_cell_index = start_cell_index + 4;

	while (is_check_balancing_complete(group,ignore_check_max_voltage)  == false){
		search_group_max_cell(start_cell_index, finish_cell_index, ignore_check_max_voltage, &group_max_cell_index);
		search_group_min_cell(start_cell_index, finish_cell_index, &group_min_cell_index);
		ignore_check_max_voltage[group_min_cell_index] = 1;
		ignore_check_max_voltage[group_max_cell_index] = 1;
		if (is_need_balancing(min_voltage, group_max_cell_index)) {

			//if (finish_cell_index <= 4) {
				if ((group_max_cell_index - start_cell_index) == 3) {
					set_balancing_bit(&balancing_pattern, group_max_cell_index - start_cell_index);
					group_max_cell_index++;
				}
/*
			} else if (finish_cell_index <= 8) {
				if ((group_max_cell_index - start_cell_index) == 3) {
					set_balancing_bit(&balancing_pattern, group_max_cell_index - start_cell_index);
					group_max_cell_index++;
				}
			}

			else if (finish_cell_index <= 12) {
				if ((group_max_cell_index - start_cell_index) == 3) {
					set_balancing_bit(&balancing_pattern, group_max_cell_index - start_cell_index);
					group_max_cell_index++;
				}

			} else if (finish_cell_index <= 16) {
				if ((group_max_cell_index - start_cell_index) == 3) {
					set_balancing_bit(&balancing_pattern, group_max_cell_index - start_cell_index);
					group_max_cell_index++;
				}
			}
*/
		}
		set_balancing_bit(&balancing_pattern, group_max_cell_index - start_cell_index);
		//if (group_max_cell_index < finish_cell_index)
		//	ignore_check_max_voltage[group_max_cell_index + 1] = 1;
		//if (group_max_cell_index > start_cell_index)
		//	ignore_check_max_voltage[group_max_cell_index - 1] = 1;
	}
	balancing_bits[group] = balancing_pattern;
#endif

	uint8_t start_cell_index = 0;
	uint8_t finish_cell_index = 0;
	uint8_t group_max_cell_index;
	uint8_t balancing_pattern = 0;
	start_cell_index = 4 * group;
	finish_cell_index = start_cell_index + 4;
	while (is_check_balancing_complete(group,ignore_check_max_voltage)  == false){
		search_group_max_cell(start_cell_index, finish_cell_index, ignore_check_max_voltage, &group_max_cell_index);
		if (is_need_balancing(min_voltage, group_max_cell_index))
		{
			if ((group_max_cell_index - start_cell_index) == 3) {
							set_balancing_bit(&balancing_pattern, group_max_cell_index - start_cell_index);
							group_max_cell_index++;
						}
		}
			set_balancing_bit(&balancing_pattern,
					group_max_cell_index - start_cell_index);
		ignore_check_max_voltage[group_max_cell_index] = 1;
		if (group_max_cell_index < finish_cell_index)
			ignore_check_max_voltage[group_max_cell_index + 1] = 1;
		if (group_max_cell_index > start_cell_index)
			ignore_check_max_voltage[group_max_cell_index - 1] = 1;
	}
	balancing_bits[group] = balancing_pattern;
}

static int16_t ntc_get_temp(const uint16_t adc_value ){
    uint32_t ntc_resistant;
    uint32_t ntc_voltage=0;
    int16_t lookup_index;

    ntc_voltage=((uint32_t)adc_value * NTC_ADC_LSB_VOLTAGE)/1000; /* in mili volt unit */
    ntc_resistant=10000*ntc_voltage / (NTC_ADC_REF_VOLTAGE-ntc_voltage); /* in ohm unit */

    lookup_index=0;
    while((lookup_index <(NTC_LOOKUPS_TABLE_SIZE-1)) && (ntc_resistant<=ntc_lookups[lookup_index])){
        lookup_index++;
    }
    return lookup_index+ NTC_LOOKUPS_MIN_TEMPERATURE;
}

int afe_read_ntc(I2C_TypeDef* i2c_dev){
	    int success=-1;
	    // each AFE use two temperature sensors
	    if(i2c_dev == AFE1_I2C_DEVICE){
	    	success=I2CReadBlockWithCRC(AFE1_I2C_DEVICE,AFE_I2C_ADDRESS,TS1_HI_BYTE,ntc_buffer,4);
	    }else if(i2c_dev == AFE2_I2C_DEVICE){
	    	success=I2CReadBlockWithCRC(AFE2_I2C_DEVICE,AFE_I2C_ADDRESS,TS1_HI_BYTE,ntc_buffer + 4,4);
	    }
	    if(success !=0) return success;
}

void afe_read_temperature(void){
	uint16_t ntc_adc_value = 0;
	uint8_t ntc_index = 0;
	for (ntc_index = 0; ntc_index < NTC_NUM; ntc_index++) {
		ntc_adc_value = ((uint16_t) ntc_buffer[2 * ntc_index] << 8)
				+ (uint16_t) ntc_buffer[2 * ntc_index + 1];
		thermistor_values[ntc_index] = ntc_get_temp(ntc_adc_value);
	}
}


int16_t* afe_get_thermistor_values(){
    return thermistor_values;
}

void afe_read_pack_voltage(I2C_TypeDef* i2c_dev) {
#if 1
	uint32_t adc = 0;
	int result = 1;
	if (i2c_dev == AFE1_I2C_DEVICE) {
		while (result != 0) {
			result = I2CReadRegisterWordWithCRC(AFE1_I2C_DEVICE,
			AFE_I2C_ADDRESS,
			BAT_HI_BYTE, &adc);
		}

		/* 4 x GAIN x ADC(cell) + (#Cells x OFFSET) */
		if (adc != 0) {
			p_vol1 = 4 * iGain1 * adc / 1000;
			p_vol1 += CELL_NUM * Registers1.ADCOffset;
		}
	}

	result = 1;
	if (i2c_dev == AFE2_I2C_DEVICE) {
		while (result != 0) {
			result = I2CReadRegisterWordWithCRC(AFE2_I2C_DEVICE,
					AFE_I2C_ADDRESS,
					BAT_HI_BYTE, &adc);
		}
		if (adc != 0) {
			p_vol2 = 4 * iGain2 * adc / 1000;
			p_vol2 += CELL_NUM * Registers2.ADCOffset;
		}
	}
	p_vol = p_vol1+p_vol2;
#endif
#if 0
	p_vol = 0;
	for (uint8_t i= 0; i<CELL_NUM; i ++)
	{
		p_vol +=  CellVoltage[i];
	}
#endif
}

int32_t afe_get_current_adc(void){
	return afe_current_adc;
}

void afe_enter_ship_mode(I2C_TypeDef* i2c_dev) {
	int8_t result = -1;
	uint8_t sys_ctrl1_reg_value, write1, write2, temp1 = 0, temp2 = 0;
	while (result != 0) {
		result = I2CReadRegisterByte(i2c_dev,AFE_I2C_ADDRESS, SYS_CTRL1,
				&sys_ctrl1_reg_value);
		led_percent_display(90);
	}
	write1 = sys_ctrl1_reg_value | SHUT_B_BIT;
	write1 = write1 & (~SHUT_A_BIT);
	write2 = sys_ctrl1_reg_value | SHUT_A_BIT;
	write2 = write2 & (~SHUT_B_BIT);
	sys_ctrl1_reg_value &= 0xFC;
	I2CWriteRegisterByteWithCRC(i2c_dev,AFE_I2C_ADDRESS, SYS_CTRL1,
			sys_ctrl1_reg_value);
	while (temp1 != write1) {
	        led_percent_display(90);
		I2CWriteRegisterByteWithCRC(i2c_dev,AFE_I2C_ADDRESS, SYS_CTRL1, write1);
		  result = I2CReadRegisterByte(i2c_dev,AFE_I2C_ADDRESS, SYS_CTRL1,&temp1);
		 // USART_Write_String("enter shipmode step 1");
	}
	while (temp2 != write2) {
	        led_percent_display(90);
		I2CWriteRegisterByteWithCRC(i2c_dev,AFE_I2C_ADDRESS, SYS_CTRL1, write2);
		 result = I2CReadRegisterByte(i2c_dev,AFE_I2C_ADDRESS, SYS_CTRL1,&temp2);
		// USART_Write_String("enter shipmode step 2");
	}
}

void afe_exit_ship_mode(I2C_TypeDef* i2c_dev){
	int8_t result = -1;
	uint8_t sys_ctrl1_reg_value;
	while (result != 0) {
		result = I2CReadRegisterByte(i2c_dev,AFE_I2C_ADDRESS, SYS_CTRL1,
				&sys_ctrl1_reg_value);
		//USART_Write_String("exit shipmode");
	}
	sys_ctrl1_reg_value &= 0xFC;
	I2CWriteRegisterByteWithCRC(i2c_dev,AFE_I2C_ADDRESS, SYS_CTRL1,
			sys_ctrl1_reg_value);
}

#endif

int afe_hardware_init(void) {
	bq_clock_init();
	bq_gpio_init();
	bq_interrupt_init();
	bq_i2c_init();
	bq_config_timer();
	bq1_hw_init();
	bq2_hw_init();
	return 0;
}

static void bq_gpio_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = AFE1_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(AFE1_I2C_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AFE1_I2C_SDA_PIN;
	GPIO_Init(AFE1_I2C_SDA_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AFE2_I2C_SCL_PIN;
	GPIO_Init(AFE2_I2C_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AFE2_I2C_SDA_PIN;
	GPIO_Init(AFE2_I2C_SDA_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(AFE1_I2C_SCL_PORT, AFE1_I2C_SCL_SOURCE, AFE1_I2C_SCL_AF);
	GPIO_PinAFConfig(AFE1_I2C_SDA_PORT, AFE1_I2C_SDA_SOURCE, AFE1_I2C_SDA_AF);

	GPIO_PinAFConfig(AFE2_I2C_SCL_PORT, AFE2_I2C_SCL_SOURCE, AFE2_I2C_SCL_AF);
	GPIO_PinAFConfig(AFE2_I2C_SDA_PORT, AFE2_I2C_SDA_SOURCE, AFE2_I2C_SDA_AF);

	GPIO_InitStructure.GPIO_Pin = AFE1_ALERT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AFE1_ALERT_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AFE2_ALERT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AFE2_ALERT_PORT, &GPIO_InitStructure);

	/* Configure NFC_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = PERCENT_LED_BLUE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(PERCENT_LED_BLUE_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PERCENT_LED_RED_PIN;
	GPIO_Init(PERCENT_LED_RED_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PERCENT_LED_GREEN_PIN;
	GPIO_Init(PERCENT_LED_GREEN_PORT, &GPIO_InitStructure);

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

	GPIO_InitStructure.GPIO_Pin = PUSH_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PUSH_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = TEST_SW_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TEST_SW_PORT, &GPIO_InitStructure);

}

static void bq_clock_init(void) {
	//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
	/* Configure the I2C clock source. The clock is derived from the HSI */
//	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

	/* sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
	RCC_AHBPeriphClockCmd(AFE1_I2C_SCL_CLK | AFE1_I2C_SDA_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(AFE2_I2C_SCL_CLK | AFE2_I2C_SDA_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(AFE1_ALERT_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(AFE2_ALERT_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(PUSH_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(PERCENT_LED_BLUE_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(PERCENT_LED_GREEN_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(PERCENT_LED_RED_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(TEST_SW_CLK, ENABLE);

	/* sEE_I2C Periph clock enable */
	RCC_APB1PeriphClockCmd(AFE1_I2C_CLK | AFE2_I2C_CLK, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
}
static void bq_interrupt_init(void) {
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// interrupt 1 for bq1
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);

	/* Configure EXTI5 line */
	EXTI_InitStructure.EXTI_Line = AFE1_INTERRUPT_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI5_15 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = AFE1_INTERRUPT_VECTOR;
	NVIC_InitStructure.NVIC_IRQChannelPriority = AFE1_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// interrupt 2 for bq2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource11);

	/* Configure EXTI11 line */
	EXTI_InitStructure.EXTI_Line = AFE2_INTERRUPT_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//hight -> low
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI5_15 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = AFE2_INTERRUPT_VECTOR;
	NVIC_InitStructure.NVIC_IRQChannelPriority = AFE2_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

static void bq_i2c_init(void) {
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
	I2C_InitStructure.I2C_Timing = 0x10805E89; /* for SYSCLK 48MHz */
	I2C_Init(AFE1_I2C_DEVICE, &I2C_InitStructure);
	I2C_Init(AFE2_I2C_DEVICE, &I2C_InitStructure);

	/* sEE_I2C Peripheral Enable */
	I2C_Cmd(AFE1_I2C_DEVICE, ENABLE);
	I2C_Cmd(AFE2_I2C_DEVICE, ENABLE);
}

static void bq1_reset_i2c_bus(void) {
	I2C_Cmd(AFE1_I2C_DEVICE, DISABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = AFE1_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(AFE1_I2C_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AFE1_I2C_SDA_PIN;
	GPIO_Init(AFE1_I2C_SDA_PORT, &GPIO_InitStructure);

	GPIO_SetBits(AFE1_I2C_SCL_PORT, AFE1_I2C_SCL_PIN);
	GPIO_SetBits(AFE1_I2C_SDA_PORT, AFE1_I2C_SDA_PIN);

	GPIO_InitStructure.GPIO_Pin = AFE1_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(AFE1_I2C_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AFE1_I2C_SDA_PIN;
	GPIO_Init(AFE1_I2C_SDA_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(AFE1_I2C_SCL_PORT, AFE1_I2C_SCL_SOURCE, AFE1_I2C_SCL_AF);
	GPIO_PinAFConfig(AFE1_I2C_SDA_PORT, AFE1_I2C_SDA_SOURCE, AFE1_I2C_SDA_AF);

	I2C_Cmd(AFE1_I2C_DEVICE, ENABLE);

}

static void bq2_reset_i2c_bus(void) {
	I2C_Cmd(AFE2_I2C_DEVICE, DISABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = AFE2_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(AFE2_I2C_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AFE2_I2C_SDA_PIN;
	GPIO_Init(AFE2_I2C_SDA_PORT, &GPIO_InitStructure);

	GPIO_SetBits(AFE2_I2C_SCL_PORT, AFE2_I2C_SCL_PIN);
	GPIO_SetBits(AFE2_I2C_SDA_PORT, AFE2_I2C_SDA_PIN);

	GPIO_InitStructure.GPIO_Pin = AFE2_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(AFE2_I2C_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AFE2_I2C_SDA_PIN;
	GPIO_Init(AFE2_I2C_SDA_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(AFE2_I2C_SCL_PORT, AFE2_I2C_SCL_SOURCE, AFE2_I2C_SCL_AF);
	GPIO_PinAFConfig(AFE2_I2C_SDA_PORT, AFE2_I2C_SDA_SOURCE, AFE2_I2C_SDA_AF);

	I2C_Cmd(AFE2_I2C_DEVICE, ENABLE);

}

static void bq_config_timer(void) {
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
		return -1;
	/* Send MSB of memory address */

	I2C_SendData(I2C_DEV, DataBuffer[0]);

	if (i2c_wait_for_flag_status(I2C_DEV, I2C_FLAG_TCR,SET,I2C_TIMEOUT) == false)
		return -1;

	/* Update CR2 : set Slave Address , set write request, generate Start and set end mode */
	I2C_TransferHandling(I2C_DEV, I2CSlaveAddress, ByteCount - 1,
			I2C_AutoEnd_Mode, I2C_No_StartStop);
	DataNum = 1;
	while (DataNum != ByteCount) {
		/* Wait until TXIS flag is set */
		if (i2c_wait_for_flag_status(I2C_DEV, I2C_FLAG_TXIS,SET,I2C_TIMEOUT) == false){
			return -1;
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

void EXTI4_15_IRQHandler(void){

	if(EXTI_GetITStatus(EXTI_Line5)==SET){
		if(lower_bq_hw.handle){
			lower_bq_hw.handle(&lower_bq_hw);
		}
		EXTI_ClearFlag(EXTI_Line5);
	}

	if(EXTI_GetITStatus(EXTI_Line11)==SET){
		if(upper_bq_hw.handle){
			upper_bq_hw.handle(&upper_bq_hw);
		}
		EXTI_ClearFlag(EXTI_Line11);
	}

}
#endif
