#ifndef BQ79640_H_
#define BQ79640_H_
#include "board.h"


#define CRC_KEY 7
static const unsigned int OVPThreshold = 4100;
static const unsigned int UVPThreshold = 2800;

#define DELAY_LIMIT 0xffff

#define SYS_STAT                    0
#define CELLBAL1                    1
#define CELLBAL2                    2
#define CELLBAL3                    3
#define SYS_CTRL1                   4
#define SYS_CTRL2                   5
#define PROTECT1                    0x06
#define PROTECT2                    0x07
#define PROTECT3                    0x08
#define OV_TRIP                     0x09
#define UV_TRIP                     0x0A
#define CC_CFG		                0x0B
#define VC1_HI_BYTE                 0x0C
#define BAT_HI_BYTE                 0x2A
#define BAT_LOW_BYTE                 0x2A
#define CC_HI_BYTE		            0x32
#define TS1_HI_BYTE		            0x2C

#define ADCGAIN1                    0x50
#define ADCOFFSET                   0x51
#define ADCGAIN2                    0x59

#define SCD_DELAY_70us		        0x0
#define SCD_DELAY_100us		        0x1
#define SCD_DEALY_200us		        0x2
#define SCD_DELAY_400us		        0x3

#define SCD_THRESH_44mV_22mV	    0
#define SCD_THRESH_67mV_33mV	    1
#define SCD_THRESH_89mV_44mV	    2
#define SCD_THRESH_111mV_56mV	    3
#define SCD_THRESH_133mV_67mV	    4
#define SCD_TRHESH_155mV_68mV	    5
#define SCD_THRESH_178mV_89mV	    6
#define SCD_THRESH_200mV_100mV	    7

#define OCD_DEALY_10ms		        0x0
#define OCD_DELAY_20ms		        0x1
#define OCD_DELAY_40ms		        0x2
#define OCD_DELAY_80ms		        0x3
#define OCD_DELAY_160ms		        0x4
#define OCD_DELAY_320ms		        0x5
#define OCD_DELAY_640ms		        0x6
#define OCD_DELAY_1280ms	        0x7

#define OCD_THRESH_17mV_8mV	        0
#define OCD_THRESH_22mV_11mV	    1
#define OCD_THRESH_28mV_14mV	    2
#define OCD_THRESH_33mV_17mV	    3
#define OCD_THRESH_39mV_19mV	    4
#define OCD_THRESH_44mV_22mV	    5
#define OCD_THRESH_50mV_25mV	    6
#define OCD_THRESH_56mV_28MV	    7
#define OCD_THRESH_61mV_31mV	    8
#define OCD_THRESH_67mV_33mV	    9
#define OCD_THRESH_72mV_36mV	    0xA
#define OCD_THRESH_78mV_39mV	    0xB
#define OCD_THRESH_83mV_42mV	    0xC
#define OCD_THRESH_89mV_44mV	    0xD
#define OCD_THRESH_94mV_47mV	    0xE
#define OCD_THRESH_100mV_50mV	    0xF

#define UV_DELAY_1s		            0
#define UV_DELAY_4s		            1
#define UV_DELAY_8s		            2
#define UV_DELAY_16s		        3

#define OV_DELAY_1s		            0
#define OV_DELAY_2s		            1
#define OV_DELAY_4s		            2
#define OV_DELAY_8s		            3

#define OV_THRESH_BASE			    0x2008
#define UV_THRESH_BASE			    0x1000
#define OV_STEP					    0x10
#define UV_STEP					    0x10

#define ADCGAIN_BASE			    365

//SYS_STAT bit masks
#define CC_READY_BIT 		    (1<<7)
#define DEVICE_XREADY_BIT 	    (1<<5)
#define OVRD_ALERT_BIT 		    (1<<4)
#define AFE_STS_UV 			        (1<<3)
#define AFE_STS_OV 			        (1<<2)
#define AFE_STS_SCD 		        (1<<1)
#define AFE_STS_OCD 		        (1<<0)

//SYS_CTRL1 bit masks
#define LOAD_PRESENT_BIT 	    (1<<7)
#define ADC_EN_BIT  		    (1<<4)
#define TEMP_SEL_BIT  		    (1<<3)
#define SHUT_A_BIT  		    (1<<1)
#define SHUT_B_BIT  		    (1<<0)
//SYS_CTRL2 bit masks                    )
#define DELAY_DIS_BIT  		    (1<<7)
#define CC_EN_BIT  		        (1<<6)
#define CC_ONESHOT_BIT  	    (1<<5)
#define DSG_ON_BIT  		    (1<<1)
#define CHG_ON_BIT  		    (1<<0)
#define SNRS_BIT  		        (1<<7)


#define LOW_BYTE(Data)			(unsigned char)(0xff & Data)
#define HIGH_BYTE(Data)			(unsigned char)(0xff & (Data >> 8))

#define CELL_GROUP_NUM		3
#define CELL_NUM			15
#define CELL_MAX_CAP		4000

#define BALANCING_OFFSET 	40
#define CHARGE_DETECT_OFFSET	250

#define R_SHUNT			1000

#define NTC_ADC_LSB_VOLTAGE         382 /* in micro vol */
#define NTC_INTERNAL_PULL_UP        10000 /* in ohm */
#define NTC_ADC_REF_VOLTAGE         3300 /* in milivolt */
#define NTC_NUM                     3
#define BALANCING_VOLTAGE_THRESHOLD		3500
#define ENA_VER2					1
#define VARIED_PACK_PIN					0

typedef enum CURRENT_CONVERT_MODE{
	ALWAYS_ON	 = 0x00,
	ONE_SHOT	 = 0x01
}CURRENT_CONVERT_MODE;


typedef enum PACK_OPERATING_STATE{
	PACK_CHARGING		=0x00,
	PACK_DISCHARGING	=0x01,
	PACK_STANDBY		=0x02
}PACK_OPERATING_STATE;

extern uint16_t CellVoltage[15];

typedef struct _Register_Group
{
	union
	{
		struct
		{
			unsigned char OCD			:1;
			unsigned char SCD			:1;
			unsigned char OV			:1;
			unsigned char UV			:1;
			unsigned char OVRD_ALERT	:1;
			unsigned char DEVICE_XREADY	:1;
			unsigned char WAKE			:1;
			unsigned char CC_READY		:1;
		}StatusBit;
		unsigned char StatusByte;
	}SysStatus;

	union
	{
		struct
		{
			unsigned char RSVD			:3;
			unsigned char CB5			:1;
			unsigned char CB4			:1;
			unsigned char CB3			:1;
			unsigned char CB2			:1;
			unsigned char CB1			:1;
		}CellBal1Bit;
		unsigned char CellBal1Byte;
	}CellBal1;

	union
	{
		struct
		{
			unsigned char RSVD			:3;
			unsigned char CB10			:1;
			unsigned char CB9			:1;
			unsigned char CB8			:1;
			unsigned char CB7			:1;
			unsigned char CB6			:1;
		}CellBal2Bit;
		unsigned char CellBal2Byte;
	}CellBal2;

	union
	{
		struct
		{
			unsigned char RSVD			:3;
			unsigned char CB15			:1;
			unsigned char CB14			:1;
			unsigned char CB13			:1;
			unsigned char CB12			:1;
			unsigned char CB11			:1;
		}CellBal3Bit;
		unsigned char CellBal3Byte;
	}CellBal3;

	union
	{
		struct
		{
			unsigned char SHUT_B		:1;
			unsigned char SHUT_A		:1;
			unsigned char RSVD1			:1;
			unsigned char TEMP_SEL		:1;
			unsigned char ADC_EN		:1;
			unsigned char RSVD2			:2;
			unsigned char LOAD_PRESENT	:1;
		}SysCtrl1Bit;
		unsigned char SysCtrl1Byte;
	}SysCtrl1;

	union
	{
		struct
		{
			unsigned char CHG_ON		:1;
			unsigned char DSG_ON		:1;
			unsigned char WAKE_T		:2;
			unsigned char WAKE_EN		:1;
			unsigned char CC_ONESHOT	:1;
			unsigned char CC_EN			:1;
			unsigned char DELAY_DIS		:1;
		}SysCtrl2Bit;
		unsigned char SysCtrl2Byte;
	}SysCtrl2;

	union
	{
		struct
		{
			unsigned char SCD_THRESH	:3;
			unsigned char SCD_DELAY		:2;
			unsigned char RSVD			:2;
			unsigned char RSNS			:1;
		}Protect1Bit;
		unsigned char Protect1Byte;
	}Protect1;

	union
	{
		struct
		{
			unsigned char OCD_THRESH	:4;
			unsigned char OCD_DELAY		:3;
			unsigned char RSVD			:1;
		}Protect2Bit;
		unsigned char Protect2Byte;
	}Protect2;

	union
	{
		struct
		{
			unsigned char RSVD			:4;
			unsigned char OV_DELAY		:2;
			unsigned char UV_DELAY		:2;
		}Protect3Bit;
		unsigned char Protect3Byte;
	}Protect3;

	unsigned char OVTrip;
	unsigned char UVTrip;
	unsigned char CCCfg;			//must be 0x19

	union
	{
		struct
		{
			unsigned char VC1_HI;
			unsigned char VC1_LO;
		}VCell1Byte;
		unsigned short VCell1Word;
	}VCell1;

	union
	{
		struct
		{
			unsigned char VC2_HI;
			unsigned char VC2_LO;
		}VCell2Byte;
		unsigned short VCell2Word;
	}VCell2;

	union
	{
		struct
		{
			unsigned char VC3_HI;
			unsigned char VC3_LO;
		}VCell3Byte;
		unsigned short VCell3Word;
	}VCell3;

	union
	{
		struct
		{
			unsigned char VC4_HI;
			unsigned char VC4_LO;
		}VCell4Byte;
		unsigned short VCell4Word;
	}VCell4;

	union
	{
		struct
		{
			unsigned char VC5_HI;
			unsigned char VC5_LO;
		}VCell5Byte;
		unsigned short VCell5Word;
	}VCell5;

	union
	{
		struct
		{
			unsigned char VC6_HI;
			unsigned char VC6_LO;
		}VCell6Byte;
		unsigned short VCell6Word;
	}VCell6;

	union
	{
		struct
		{
			unsigned char VC7_HI;
			unsigned char VC7_LO;
		}VCell7Byte;
		unsigned short VCell7Word;
	}VCell7;

	union
	{
		struct
		{
			unsigned char VC8_HI;
			unsigned char VC8_LO;
		}VCell8Byte;
		unsigned short VCell8Word;
	}VCell8;

	union
	{
		struct
		{
			unsigned char VC9_HI;
			unsigned char VC9_LO;
		}VCell9Byte;
		unsigned short VCell9Word;
	}VCell9;

	union
	{
		struct
		{
			unsigned char VC10_HI;
			unsigned char VC10_LO;
		}VCell10Byte;
		unsigned short VCell10Word;
	}VCell10;

	union
	{
		struct
		{
			unsigned char VC11_HI;
			unsigned char VC11_LO;
		}VCell11Byte;
		unsigned short VCell11Word;
	}VCell11;

	union
	{
		struct
		{
			unsigned char VC12_HI;
			unsigned char VC12_LO;
		}VCell12Byte;
		unsigned short VCell12Word;
	}VCell12;
	union
	{
		struct
		{
			unsigned char VC13_HI;
			unsigned char VC13_LO;
		}VCell13Byte;
		unsigned short VCell13Word;
	}VCell13;

	union
	{
		struct
		{
			unsigned char VC14_HI;
			unsigned char VC14_LO;
		}VCell14Byte;
		unsigned short VCell14Word;
	}VCell14;

	union
	{
		struct
		{
			unsigned char VC15_HI;
			unsigned char VC15_LO;
		}VCell15Byte;
		unsigned short VCell15Word;
	}VCell15;

	union
	{
		struct
		{
			unsigned char BAT_HI;
			unsigned char BAT_LO;
		}VBatByte;
		unsigned short VBatWord;
	}VBat;

	union
	{
		struct
		{
			unsigned char TS1_HI;
			unsigned char TS1_LO;
		}TS1Byte;
		unsigned short TS1Word;
	}TS1;

	union
	{
		struct
		{
			unsigned char TS2_HI;
			unsigned char TS2_LO;
		}TS2Byte;
		unsigned short TS2Word;
	}TS2;

	union
	{
		struct
		{
			unsigned char TS3_HI;
			unsigned char TS3_LO;
		}TS3Byte;
		unsigned short TS3Word;
	}TS3;

	union
	{
		struct
		{
			unsigned char CC_HI;
			unsigned char CC_LO;
		}CCByte;
		unsigned short CCWord;
	}CC;

	union
	{
		struct
		{
			unsigned char RSVD1			:2;
			unsigned char ADCGAIN_4_3	:2;
			unsigned char RSVD2			:4;
		}ADCGain1Bit;
		unsigned char ADCGain1Byte;
	}ADCGain1;

	unsigned char ADCOffset;

	union
	{
		struct
		{
			unsigned char RSVD			:5;
			unsigned char ADCGAIN_2_0	:3;
		}ADCGain2Bit;
		unsigned char ADCGain2Byte;
	}ADCGain2;

}RegisterGroup;

extern uint16_t p_vol;

//bool is_afe_fault(const uint8_t state);
void afe_read_current_adc(void);
void calculate_balancing();
int GetADCGainOffset();
int active_balancing();
int stop_balancing();
int read_balancing_status(uint8_t balancing_bits[]);
int afe_setup();
int config_afe();
int afe_read_cell_voltage();
void afe_check_status(void);
void clear_status_flags(const uint8_t mask);
void read_status_flags(uint8_t* p_flags);
void set_debug_fault_delay();
int clear_register_bit(uint8_t reg,const uint8_t bit_mask);
void clear_status_flags(const uint8_t mask );
int set_register_bit(uint8_t reg,const uint8_t bit_mask);
int afe_read_ntc(void);
int enable_adc();
int disable_adc();
int enalbe_coulump_counter(CURRENT_CONVERT_MODE current_convert_mode);
int disalbe_coulump_counter();
int read_temperature();
#if !ENA_VER2
int main_switch_on();
int main_switch_off();
#endif
//int is_load_present(bool *present);
int32_t afe_get_current_adc(void);
uint16_t* afe_get_cell_voltates();
uint8_t* afe_get_balancing_mask();
void afe_read_pack_voltage(void);
void afe_read_temperature(void);
int16_t* afe_get_thermistor_values();
PACK_OPERATING_STATE get_pack_operating_state();
void afe_enter_ship_mode(void);
void afe_exit_ship_mode(void);
int afe_turn_on_discharge(void);
int afe_turn_off_discharge(void);
int afe_turn_on_charge(void);
int afe_turn_off_charge(void);
uint8_t afe_is_balancing_complete(void);
int32_t* afe_get_ocp_threshold(void);

#endif



