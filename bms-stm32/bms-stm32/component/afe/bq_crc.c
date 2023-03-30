#include "bq_crc.h"
#include "stdlib.h"
#include "stm32f0xx.h"
#include "board.h"
#include "afe_hardware.h"
#include "stm32f0xx_adc.h"
#include "timer.h"
#include "ntc.h"
#include "i2c_driver.h"
#include "stdint.h"

RegisterGroup Registers;

#define MAX_OCD_THRESH		16

const unsigned char SCDDelay = SCD_DELAY_70us;

//const unsigned char SCDThresh = SCD_THRESH_44mV_22mV;
//const unsigned char SCDThresh = SCD_THRESH_67mV_33mV;
//const unsigned char SCDThresh = SCD_THRESH_89mV_44mV;
//const unsigned char SCDThresh = SCD_THRESH_111mV_56mV;
//const unsigned char SCDThresh = SCD_THRESH_133mV_67mV;
//const unsigned char SCDThresh = SCD_THRESH_178mV_89mV;
//const unsigned char SCDThresh = SCD_THRESH_200mV_100mV;


const unsigned char OCDDelay = OCD_DELAY_20ms;


const unsigned char OCP_LEVEL = 1;
#if PACK_CHANGE
unsigned char OCDThresh =OCD_THRESH_33mV_17mV;
const unsigned char SCDThresh = SCD_THRESH_89mV_44mV;
#endif

#if !PACK_CHANGE
unsigned char OCDThresh = OCD_THRESH_50mV_25mV;
const unsigned char SCDThresh = SCD_THRESH_89mV_44mV;
#endif
const int32_t OCD_Value[MAX_OCD_THRESH] = {8000,11000,14000,17000,19000,22000,25000,28000,31000,33000,36000,39000,42000,44000,47000,50000};


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
uint16_t CellVoltage[15];
int32_t afe_current_adc = 0;
int32_t ocp_threshold = 0;
uint16_t p_vol = 0;
uint8_t balancing_bits[CELL_GROUP_NUM];
float Gain = 0;
int iGain = 0;
static uint8_t ntc_buffer[12]={0};

int16_t thermistor_values[NTC_NUM];

/** Private helper functions *************************************************/


static bool is_check_balancing_complete(const uint8_t group,const uint8_t* ignore_check_cell);
static void get_min_cell(uint16_t* voltage);
static void set_group_balancing_bits(const uint8_t group, const uint16_t min_voltage,uint8_t* ignore_check_max_voltage);
static void search_group_max_cell(const uint8_t start_index, const uint8_t finish_index,const uint8_t* ignore_check_cell,
	uint8_t* max_cell);

static void set_balancing_bit(uint8_t* balancing_byte,const uint8_t bit);
static void clear_balancing_bit(uint8_t* balancing_byte,const uint8_t bit);
static bool is_need_balancing(const uint16_t min_voltage,
		const uint8_t index);

static void set_ocd_protection_level(const uint8_t level);


static int I2CReadBlockWithCRC(unsigned char I2CSlaveAddress,
	       	unsigned char Register, unsigned char *Buffer, unsigned char Length);
static int I2CReadRegisterByteWithCRC(unsigned char I2CSlaveAddress,
		unsigned char Register, unsigned char *Data);
static int I2CReadRegisterWordWithCRC(unsigned char I2CSlaveAddress,
		unsigned char Register, uint16_t *Data);
static int I2CWriteBlockWithCRC(unsigned char I2CSlaveAddress,
	       	unsigned char StartAddress, unsigned char *Buffer, unsigned char Length);
static int I2CWriteRegisterWordWithCRC(unsigned char I2CSlaveAddress,
		unsigned char Register, unsigned int Data);
static int I2CWriteRegisterByteWithCRC(unsigned char I2CSlaveAddress,
		unsigned char Register, unsigned char Data);
static unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
static int afe_set_protect_params();
static void get_max_cell(uint16_t* voltage);


int afe_setup()
{
	int result = 0;
	uint8_t state=0;

    ntc_lookups=(uint16_t*)mf58_lookups;
	result=afe_set_protect_params();
	if(result !=0) goto init_fault_handle;

    	result = config_afe();
	if(result !=0) goto init_fault_handle;

	set_ocd_protection_level(OCP_LEVEL);

	result=enable_adc();
	if(result !=0) goto init_fault_handle;

	result=enalbe_coulump_counter(ALWAYS_ON);
	if(result !=0) goto init_fault_handle;

	stop_balancing();
	//set_debug_fault_delay();
	read_status_flags(&state);
	while(state !=0){
		clear_status_flags(0xff);
		read_status_flags(&state);
	}
	delay_ms(100);
	global_interrupt_enable();
	return 0;
init_fault_handle:
	return result;
}

uint16_t* afe_get_cell_voltates(){
	return CellVoltage;
}
uint8_t* afe_get_balancing_mask(){
	return balancing_bits;
}

static int afe_set_protect_params(){
	int result=0;
	Registers.Protect1.Protect1Bit.SCD_DELAY = SCDDelay;
	Registers.Protect1.Protect1Bit.SCD_THRESH = SCDThresh;
	Registers.Protect2.Protect2Bit.OCD_DELAY = OCDDelay;
	Registers.Protect2.Protect2Bit.OCD_THRESH = OCDThresh;
	Registers.Protect3.Protect3Bit.OV_DELAY = OVDelay;
	Registers.Protect3.Protect3Bit.UV_DELAY = UVDelay;

	result = GetADCGainOffset();
	if(result !=0) return result;

	Gain = (365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) +
		       	((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5)) / 1000.0;
	iGain = 365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) +
			((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5);
    	Registers.OVTrip = (unsigned char)((((unsigned short)((OVPThreshold - Registers.ADCOffset)/Gain + 0.5)
				    - OV_THRESH_BASE) >> 4) & 0xFF);
    	Registers.UVTrip = (unsigned char)((((unsigned short)((UVPThreshold - Registers.ADCOffset)/Gain + 0.5)
				    - UV_THRESH_BASE) >> 4) & 0xFF);
	return 0;
}

static void set_ocd_protection_level(const uint8_t level){
	uint8_t success=-1;
	while(success!=0){
		if(level==0){
			success=clear_register_bit(PROTECT1,SNRS_BIT);
		}
		else{
			success=set_register_bit(PROTECT1,SNRS_BIT);
		}
	}
}

void afe_read_current_adc(void){
	int16_t adc=0;
    int32_t cur=0;
	int success=-1;
	while(success !=0){
		success= I2CReadRegisterWordWithCRC(AFE_I2C_ADDRESS,CC_HI_BYTE,&adc);
	}
	 afe_current_adc=(int32_t)adc;
    afe_current_adc*=8440;
}

void clear_status_flags(const uint8_t mask ){
	int success=-1;
	while(success !=0){
		/* clear SYS_STAT to pulldown ALERT */
		success=I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS,SYS_STAT,mask);
	}
}

void read_status_flags(uint8_t* p_flags){
	int success=-1;
	while(success!=0){
		success = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS,SYS_STAT,p_flags);
	}
}
void set_debug_fault_delay(){
	int success=-1;
	while(success !=0){
		success=set_register_bit(SYS_CTRL2,DELAY_DIS_BIT);
	}
}

int enable_adc(){
	return set_register_bit(SYS_CTRL1,ADC_EN_BIT);
}
int disable_adc(){
    return 0;
}
int enalbe_coulump_counter(CURRENT_CONVERT_MODE current_convert_mode){
	int success=0;
	uint8_t control_reg=0;
	success = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS,SYS_CTRL2,&control_reg);
	if(success !=0) return success;

	if(current_convert_mode){
		control_reg |= CC_ONESHOT_BIT;
		control_reg &= ~CC_EN_BIT;
	}
	else{
		control_reg |= CC_EN_BIT;
	}
	success= I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS,SYS_CTRL2,control_reg);
	return success;
}
int set_register_bit(uint8_t reg, const uint8_t bit_mask) {
        int success = 0;
        uint8_t control_reg = 0;
        success = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS, reg,
                        &control_reg);
        if (success != 0)
                return success;
        control_reg |= bit_mask;
        uint8_t reg_val = ~control_reg;
        uint8_t sys_sta = 0;
//        while (reg_val != control_reg) {
//                afe_reset_i2c_bus();
                success = I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS, reg,
                                control_reg);
                success = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS, reg,
                                &reg_val);
                I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS, SYS_STAT,
                                                &sys_sta);
//        }
        return success;
}
int clear_register_bit(uint8_t reg, const uint8_t bit_mask) {
        int success = 0;
        uint8_t control_reg = 0;
        success = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS, reg,
                        &control_reg);
        if (success != 0)
                return success;
        control_reg &= ~bit_mask;
        uint8_t reg_val = ~control_reg;
        while (reg_val != control_reg) {
                afe_reset_i2c_bus();
                success = I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS, reg,
                                control_reg);
                success = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS, reg,
                                &reg_val);
        }
        return success;
}

 int afe_turn_on_discharge(void){
	return set_register_bit(SYS_CTRL2,DSG_ON_BIT);
}

 int afe_turn_off_discharge(void){
	return clear_register_bit(SYS_CTRL2,DSG_ON_BIT);
}

 int afe_turn_on_charge(void){
	return set_register_bit(SYS_CTRL2,CHG_ON_BIT);
}

 int afe_turn_off_charge(void){
	return clear_register_bit(SYS_CTRL2,CHG_ON_BIT);
}

 /*
int is_load_present(bool *present){
	int success=-1;
	uint8_t reg_value=0;
	success=I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS,SYS_CTRL1,&reg_value);
	if(success !=0) return success;
	if( reg_value & LOAD_PRESENT_BIT){
		*present= true;
	}else{
		*present =false;
	}
	return 0;
}
*/
int read_balancing_status(uint8_t balancing_bits[]){
	int success=-1;
	success = I2CReadBlockWithCRC(AFE_I2C_ADDRESS,CELLBAL1,balancing_bits,3);
	return success;
}


int active_balancing(){
	int result=0;
	result= I2CWriteBlockWithCRC(AFE_I2C_ADDRESS,CELLBAL1,balancing_bits,3);
	return result;
}
int stop_balancing(){
	uint8_t balancing_bits[CELL_GROUP_NUM]={0};
	int result=0;
	uint8_t group_index=0;
	result= I2CWriteBlockWithCRC(AFE_I2C_ADDRESS,CELLBAL1,balancing_bits,3);
	if(result != 0)  return result;
	result = I2CReadBlockWithCRC(AFE_I2C_ADDRESS,CELLBAL1,balancing_bits,3);
	if(result !=0) return result;
	for(group_index=0;group_index < CELL_GROUP_NUM;group_index++){
		/* check all lower 5 bit of cell balancing mask is clear */
		if(balancing_bits[group_index] & 0xE0) return -1;
	}
	return 0;

}
int afe_read_cell_voltage()
{
	int Result = 0, i = 0;
	uint8_t *pRawADCData = NULL;
	uint16_t iTemp = 0;
	uint32_t lTemp = 0;

	Result = I2CReadBlockWithCRC(AFE_I2C_ADDRESS,VC1_HI_BYTE,
			&(Registers.VCell1.VCell1Byte.VC1_HI),30);
	if(Result !=0){
		return Result;
	}
	pRawADCData = &Registers.VCell1.VCell1Byte.VC1_HI;
	for (i = 0; i < 15; i++)
	{
		iTemp = (uint16_t)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = ((uint32_t)iTemp * iGain)/1000;
		lTemp += Registers.ADCOffset;

		CellVoltage[i] = lTemp;
		pRawADCData += 2;
	}
	return Result;
}

void calculate_balancing(){
	uint16_t min_cell_voltage;
	uint8_t cell_group_index = 0;
	uint8_t ignore_check_max_voltage[15] = { 0 };

	get_min_cell(&min_cell_voltage);
	for (cell_group_index = 0; cell_group_index < CELL_GROUP_NUM;
			cell_group_index++) {
		balancing_bits[cell_group_index] = 0;
		set_group_balancing_bits(cell_group_index, min_cell_voltage,
				ignore_check_max_voltage);
	}
}

int GetADCGainOffset()
{
	int result;

	result = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS, ADCGAIN1, &(Registers.ADCGain1.ADCGain1Byte));
	result = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS, ADCGAIN2, &(Registers.ADCGain2.ADCGain2Byte));
	result = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS, ADCOFFSET, &(Registers.ADCOffset));

	return result;
}

int config_afe()
{
	int result = 0;
	unsigned char bms_protection_config[5];
	result=-1;
	while(result != 0){
		result= I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS,CC_CFG,0x19);
	}

	result = I2CWriteBlockWithCRC(AFE_I2C_ADDRESS, PROTECT1, &(Registers.Protect1.Protect1Byte), 5);
	if(result !=0) return result;

	result = I2CReadBlockWithCRC(AFE_I2C_ADDRESS, PROTECT1, bms_protection_config, 5);

	if(bms_protection_config[0] != Registers.Protect1.Protect1Byte
			|| bms_protection_config[1] != Registers.Protect2.Protect2Byte
			|| bms_protection_config[2] != Registers.Protect3.Protect3Byte
			|| bms_protection_config[3] != Registers.OVTrip
			|| bms_protection_config[4] != Registers.UVTrip)
	{
		result = -1;
	}

	return result;
}
#if  !ENA_VER2
int main_switch_on(){
	int success=0;
	uint8_t control_reg=0;
	success = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS,SYS_CTRL2,&control_reg);
	if(success !=0) return success;
	control_reg |= (DSG_ON_BIT | CHG_ON_BIT);
	success= I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS,SYS_CTRL2,control_reg);
	return success;
	/*
	int success=0;
	success=turn_off_charge();
	if(success !=0)	return success;
	success==turn_on_discharge();
	return success;
	*/
}
int main_switch_off(){
	int success=0;
	uint8_t control_reg=0;
	success = I2CReadRegisterByteWithCRC(AFE_I2C_ADDRESS,SYS_CTRL2,&control_reg);
	if(success !=0) return success;
	control_reg &= ~(DSG_ON_BIT | CHG_ON_BIT);
	success= I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS,SYS_CTRL2,control_reg);
	return success;
/*
	int success=0;
	success=turn_on_charge();
	if(success !=0)	return success;
	success==turn_off_discharge();
	return success;
	*/

}
#endif



static int I2CWriteRegisterByteWithCRC(unsigned char I2CSlaveAddress, unsigned char Register, unsigned char Data)
{
	unsigned char DataBuffer[4];
	unsigned int SentByte = 0;

    	DataBuffer[0] = I2CSlaveAddress;
	DataBuffer[1] = Register;
	DataBuffer[2] = Data;
	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);

	return(I2CSendBytes(AFE_I2C_DEVICE,I2CSlaveAddress, DataBuffer + 1, 3, &SentByte));
}

static int I2CWriteRegisterWordWithCRC(unsigned char I2CSlaveAddress, unsigned char Register, unsigned int Data)
{
	unsigned char DataBuffer[6];
	unsigned int SentByte = 0;

    	DataBuffer[0] = I2CSlaveAddress;
	DataBuffer[1] = Register;
	DataBuffer[2] = LOW_BYTE(Data);
	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);
	DataBuffer[4] = HIGH_BYTE(Data);
	DataBuffer[5] = CRC8(DataBuffer + 4, 1, CRC_KEY);

	return(I2CSendBytes(AFE_I2C_DEVICE,I2CSlaveAddress, DataBuffer + 1, 5, &SentByte));
}

static int I2CWriteBlockWithCRC(unsigned char I2CSlaveAddress, unsigned char StartAddress, unsigned char *Buffer, unsigned char Length)
{
	unsigned char BufferCRC[255]={0};
       	uint8_t*Pointer=BufferCRC;
	int i;
	unsigned int SentByte = 0;
	int result;

	Pointer = BufferCRC;
	*Pointer = I2CSlaveAddress;
	Pointer++;
	*Pointer = StartAddress;
	Pointer++;
	*Pointer = *Buffer;
	Pointer++;
	*Pointer = CRC8(BufferCRC, 3, CRC_KEY);

	for(i = 1; i < Length; i++)
	{
        Pointer++;
        Buffer++;
        *Pointer = *Buffer;
		*(Pointer + 1) = CRC8(Pointer, 1, CRC_KEY);
		Pointer++;
	}
	result = I2CSendBytes(AFE_I2C_DEVICE,I2CSlaveAddress, BufferCRC + 1, 2*Length + 1, &SentByte);
	return result;
}



static unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key)
{
	unsigned char i;
	unsigned char crc = 0;
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
#if 0
	unsigned char i;
	unsigned char crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= key;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= key;
		}
		ptr++;
	}
	return(crc);
#endif
}

static int I2CReadRegisterByteWithCRC(unsigned char I2CSlaveAddress,
		unsigned char Register, unsigned char *Data)
{
	unsigned char TargetRegister = Register;
	unsigned char ReadData[2];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char new_CRC = 0;
    	int ReadStatus = 0;

	if(wait_for_flag_reset(AFE_I2C_DEVICE,I2C_FLAG_BUSY)==false){
	        debug_print("i2c busy flag",15);
		afe_reset_i2c_bus();
		return -1;
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(AFE_I2C_DEVICE,I2CSlaveAddress
			, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if(wait_for_flag_set(AFE_I2C_DEVICE,I2C_FLAG_TXIS)==false){
	        debug_print("i2c txis flag",15);
		afe_reset_i2c_bus();
		return -1;
	}

	/* Send Register address */
	I2C_SendData(AFE_I2C_DEVICE, TargetRegister);

	if(wait_for_flag_set(AFE_I2C_DEVICE,I2C_FLAG_TC)==false) {
		afe_reset_i2c_bus();
		 debug_print("i2c tc flag",15);
		return -1;
	}

	ReadStatus = I2CReadBytes(AFE_I2C_DEVICE,I2CSlaveAddress, ReadData, 2, &ReadDataCount);

	if (ReadStatus != 0)
	{
		return -1;
	}
	CRCInput[0] = (I2CSlaveAddress) + 1;
	CRCInput[1] = ReadData[0];

	new_CRC = CRC8(CRCInput, 2, CRC_KEY);

	if (new_CRC != ReadData[1])
		return -1;

	*Data = ReadData[0];
	return 0;
}

static int I2CReadRegisterWordWithCRC(unsigned char I2CSlaveAddress,
		unsigned char Register, uint16_t *Data)
{
	unsigned char TargetRegister = Register;
	unsigned char ReadData[4];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char new_CRC = 0;
    	int ReadStatus = 0;

	if(wait_for_flag_reset(AFE_I2C_DEVICE,I2C_FLAG_BUSY)==false) {
	        debug_print("i2c busy flag",15);
		afe_reset_i2c_bus();
		led_percent_display(90);
		return -1;
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(AFE_I2C_DEVICE,I2CSlaveAddress
			, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if(wait_for_flag_set(AFE_I2C_DEVICE,I2C_FLAG_TXIS)==false) {
	        led_percent_display(90);
	        debug_print("i2c txis flag",15);
		afe_reset_i2c_bus();
		return -1;
	}

	/* Send Register address */
	I2C_SendData(AFE_I2C_DEVICE, TargetRegister);

	if(wait_for_flag_set(AFE_I2C_DEVICE,I2C_FLAG_TC)==false) {
	        led_percent_display(90);
	        debug_print("i2c tc flag",15);
		afe_reset_i2c_bus();
		return -1;
	}

	ReadStatus = I2CReadBytes(AFE_I2C_DEVICE,I2CSlaveAddress, ReadData, 4, &ReadDataCount);

	if (ReadStatus != 0)
	{
		return -1;
	}

	CRCInput[0] = (I2CSlaveAddress ) + 1;
	CRCInput[1] = ReadData[0];

	new_CRC = CRC8(CRCInput, 2, CRC_KEY);

	if (new_CRC != ReadData[1])
		return -1;

	new_CRC = CRC8(ReadData + 2, 1, CRC_KEY);

	if (new_CRC != ReadData[3])
		return -1;

	*Data = ReadData[0];

	*Data = (*Data << 8) + ReadData[2];

	return 0;
}

static int I2CReadBlockWithCRC(unsigned char I2CSlaveAddress,
	       	unsigned char Register, unsigned char *Buffer, unsigned char Length)
{
	unsigned char TargetRegister = Register;
	uint8_t StartData[255];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char new_CRC = 0;
	uint8_t* ReadData=StartData;
   	int ReadStatus = 0;
   	int i;


	if(wait_for_flag_reset(AFE_I2C_DEVICE,I2C_FLAG_BUSY)==false) {
	        led_percent_display(90);
	        debug_print("i2c busy flag",15);
		afe_reset_i2c_bus();
		return -1;
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(AFE_I2C_DEVICE,I2CSlaveAddress
			, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	if(wait_for_flag_set(AFE_I2C_DEVICE,I2C_FLAG_TXIS)==false) {
	        led_percent_display(90);
	        debug_print("i2c txis flag",15);
	        afe_reset_i2c_bus();
		return -1;
	}

	/* Send Register address */
	I2C_SendData(AFE_I2C_DEVICE, TargetRegister);

	if(wait_for_flag_set(AFE_I2C_DEVICE,I2C_FLAG_TC)==false) {
	        led_percent_display(90);
	        debug_print("i2c tc flag",15);
	        afe_reset_i2c_bus();
		return -1;
	}

	ReadStatus = I2CReadBytes(AFE_I2C_DEVICE,I2CSlaveAddress, ReadData, 2 * Length, &ReadDataCount);

	if (ReadStatus != 0)
	{
		return -1;
	}

	CRCInput[0] = (I2CSlaveAddress ) + 1;
	CRCInput[1] = *ReadData;

	new_CRC = CRC8(CRCInput, 2, CRC_KEY);

	ReadData++;
	if (new_CRC != *ReadData)
	{
		return -1;
	}
	else
		*Buffer = *(ReadData - 1);

	for(i = 1; i < Length; i++)
	{
		ReadData++;
		new_CRC = CRC8(ReadData, 1, CRC_KEY);
		ReadData++;
		Buffer++;

		if (new_CRC != *ReadData)
		{
			return -1;
		}
		else
			*Buffer = *(ReadData - 1);
	}
	return 0;
}

 int32_t* afe_get_ocp_threshold(void){
	ocp_threshold = (int32_t)OCD_Value[OCDThresh]*(1+ OCP_LEVEL);
	return (&ocp_threshold);
}
static bool is_check_balancing_complete(const uint8_t group,const uint8_t* ignore_check_cell){
	uint8_t start_index = 5 * group;
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

static void get_min_cell(uint16_t* voltage){
	uint8_t cell_index;
	uint16_t min_voltage=65535;
	for(cell_index=0;cell_index<CELL_NUM;cell_index++){
		if(CellVoltage[cell_index] < min_voltage){
			min_voltage=CellVoltage[cell_index];
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
	get_min_cell(&min_voltage);
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
	if(CellVoltage[index] > (min_voltage + BALANCING_OFFSET) && CellVoltage[index] > BALANCING_VOLTAGE_THRESHOLD)
		return true;
	return false;
}
static void search_group_max_cell(const uint8_t start_index, const uint8_t finish_index,const uint8_t* ignore_check_cell,
	uint8_t* max_cell){
	uint8_t cell_index = start_index;
	uint16_t max_group_voltage = 0;
	while(ignore_check_cell[cell_index++]!=0);
	max_group_voltage = CellVoltage[--cell_index];
	*max_cell = cell_index;
	for (cell_index = start_index + 1; cell_index <= finish_index; cell_index++){
		if (ignore_check_cell[cell_index] == 0){
			if (CellVoltage[cell_index] > max_group_voltage){
				max_group_voltage = CellVoltage[cell_index];
				*max_cell = cell_index;
			}
		}
	}
}

static void set_group_balancing_bits(const uint8_t group, const uint16_t min_voltage,uint8_t* ignore_check_max_voltage){
	uint8_t start_cell_index = 0;
	uint8_t finish_cell_index = 0;
	uint8_t group_max_cell_index;
	uint8_t balancing_pattern = 0;
	start_cell_index = 5 * group;
	finish_cell_index = start_cell_index + 4;
	while (is_check_balancing_complete(group,ignore_check_max_voltage)  == false){
		search_group_max_cell(start_cell_index, finish_cell_index, ignore_check_max_voltage, &group_max_cell_index);
		if (is_need_balancing(min_voltage, group_max_cell_index))
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

bool is_afe_fault(const uint8_t state){
	uint8_t is_fault=0;
	if(state & (DEVICE_XREADY_BIT)) is_fault=1;
	if(state & (AFE_STS_OV)) is_fault=1;
	if(state & (AFE_STS_UV)) is_fault=1;
	if(state & (AFE_STS_OCD)) is_fault=1;
	if(state & (AFE_STS_SCD)) is_fault=1;
	if(is_fault==1) return true;
	return false;
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

int afe_read_ntc(void){
	    int success=-1;
	    success=I2CReadBlockWithCRC(AFE_I2C_ADDRESS,TS1_HI_BYTE,ntc_buffer,6);
	    if(success !=0) return success;
}

void afe_read_temperature(){
	uint16_t ntc_adc_value = 0;
	uint8_t ntc_index = 0;
	for (ntc_index = 0; ntc_index < 3; ntc_index++) {
		ntc_adc_value = ((uint16_t) ntc_buffer[2 * ntc_index] << 8)
				+ (uint16_t) ntc_buffer[2 * ntc_index + 1];
		thermistor_values[ntc_index] = ntc_get_temp(ntc_adc_value);
	}
}


int16_t* afe_get_thermistor_values(){
    return thermistor_values;
}




void afe_read_pack_voltage(void){
    uint16_t adc=0;
    int result=0;
	//while(result !=0){
		result = I2CReadRegisterWordWithCRC(AFE_I2C_ADDRESS,BAT_HI_BYTE,&adc);
	//}
    /* 4 x GAIN x ADC(cell) + (#Cells x OFFSET) */
    p_vol= 4*iGain * adc/1000;
    p_vol+= CELL_NUM * Registers.ADCOffset;
}

int32_t afe_get_current_adc(void){
	return afe_current_adc;
}


void afe_enter_ship_mode(void) {
	int8_t result = -1;
	uint8_t sys_ctrl1_reg_value, write1, write2, temp1 = 0, temp2 = 0;
	while (result != 0) {
		result = I2CReadRegisterByte(AFE_I2C_DEVICE,AFE_I2C_ADDRESS, SYS_CTRL1,
				&sys_ctrl1_reg_value);
		led_percent_display(90);
	}
	write1 = sys_ctrl1_reg_value | SHUT_B_BIT;
	write1 = write1 & (~SHUT_A_BIT);
	write2 = sys_ctrl1_reg_value | SHUT_A_BIT;
	write2 = write2 & (~SHUT_B_BIT);
	sys_ctrl1_reg_value &= 0xFC;
	I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS, SYS_CTRL1,
			sys_ctrl1_reg_value);
	while (temp1 != write1) {
	        led_percent_display(90);
		I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS, SYS_CTRL1, write1);
		  result = I2CReadRegisterByte(AFE_I2C_DEVICE,AFE_I2C_ADDRESS, SYS_CTRL1,&temp1);
	}
	while (temp2 != write2) {
	        led_percent_display(90);
		I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS, SYS_CTRL1, write2);
		 result = I2CReadRegisterByte(AFE_I2C_DEVICE,AFE_I2C_ADDRESS, SYS_CTRL1,&temp2);
	}
}

void afe_exit_ship_mode(void){
	int8_t result = -1;
	uint8_t sys_ctrl1_reg_value;
	while (result != 0) {
		result = I2CReadRegisterByte(AFE_I2C_DEVICE,AFE_I2C_ADDRESS, SYS_CTRL1,
				&sys_ctrl1_reg_value);
	}
	sys_ctrl1_reg_value &= 0xFC;
	I2CWriteRegisterByteWithCRC(AFE_I2C_ADDRESS, SYS_CTRL1,
			sys_ctrl1_reg_value);
}
