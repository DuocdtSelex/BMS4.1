/*
 * bq769x2.h
 *
 *  Created on: Apr 13, 2021
 *      Author: Admin
 */

#ifndef COMPONENT_AFE_BQ769X2_H_
#define COMPONENT_AFE_BQ769X2_H_

#include "bq_hw.h"
#include "afe.h"
/*
This configuration for bq 76952
*/

// Direct Commands Table
#define CONTROL_STATUS  			0x00
#define SAFETY_ALERT_A  			0x02
#define SAFETY_STATUS_A 			0x03
#define SAFETY_ALERT_B	 			0x04
#define SAFETY_STATUS_B				0x05
#define SAFETY_ALERT_C   			0x06
#define SAFETY_STATUS_C	 			0x07
#define PF_ALERT_A					0x0A
#define PF_STATUS_A					0x0B
#define PF_ALERT_B					0x0C
#define PF_STATUS_B 				0x0D
#define PF_ALERT_C					0x0E
#define PF_STATUS_C					0x0F
#define PF_ALERT_D					0x10
#define PF_STATUS_D					0x11
#define BATTERY_STATUS				0x12
#define CELL_1_VOLTAGE				0x14
#define CELL_2_VOLTAGE				0x16
#define CELL_3_VOLTAGE				0x18
#define CELL_4_VOLTAGE 				0x1A
#define CELL_5_VOLTAGE 				0x1C
#define CELL_6_VOLTAGE				0x1E
#define CELL_7_VOLTAGE 				0x20
#define CELL_8_VOLTAGE				0x22
#define CELL_9_VOLTAGE				0x24
#define CELL_10_VOLTAGE				0x26
#define CELL_11_VOLTAGE				0x28
#define CELL_12_VOLTAGE				0x2A
#define CELL_13_VOLTAGE				0x2C
#define CELL_14_VOLTAGE				0x2E
#define CELL_15_VOLTAGE				0x30
#define CELL_16_VOLTAGE				0x32
#define STACK_VOLTAGE				0x34
#define PACK_PIN_VOLTAGE			0x36
#define LD_PIN_VOLTAGE				0x38
#define CC2_CURRENT					0x3A
#define ALARM_STATUS				0x62
#define ALARM_RAW_STATUS			0x64
#define ALARM_ENABLE				0x66
#define INT_TEMPERATURE				0x68
#define CFETOFF_TEMPERATURE			0x6A
#define DFETOFF_TEMPERATURE			0x6C
#define ALERT_TEMPERATURE			0x6E
#define TS1_TEMPERATURE				0x70
#define TS2_TEMPERATURE				0x72
#define TS3_TEMPERATURE				0x74
#define HDQ_TEMPERATURE				0x76
#define DCHG_TEMPERATURE			0x78
#define DDSG_TEMPERATURE			0x7A
#define FET_STATUS					0x7F

// Control Status Register bit masks
// Safety Alert A bit masks
// Safety Status A bit masks
// Safety Alert B bit masks
// Safety Status B bit masks
// Safety Alert C bit masks
// Safety Status C bit masks
// PF Alert A bit masks
// PF Status A bit masks
// PF Alert B bit masks
// PF Status B bit masks
// PF Alert C bit masks
// PF Status C bit masks
// PF ALert D bit masks
// PF Status D bit masks
// Battery State bit mask
// Alarm Status bit masks
// Alarm Raw Status bit masks
// Alarm Enable bit masks
// FET Status bit masks

// Command-only Subcommands
#define EXIT_DEEPSLEEP					0x000E
#define DEEPSLEEP 						0x000F
#define SHUTDOWN						0x0010
#define RESET 							0x0012
#define PDSGTEST						0x001C
#define FUSE_TOGGLE						0x001D
#define PCHG_TEST						0x001E
#define CHGTEST							0x001F
#define DSGTEST							0x0020
#define FET_ENABLE						0x0022
#define PF_ENABLE						0x0024
#define PF_RESET						0x0029
#define SEAL 							0x0030
#define RESET_PASSQ						0x0082
#define PTO_RECOVER						0x008A
#define SET_CFGUPDATE					0x0090
#define EXIT_CFGUPDATE					0x0092
#define DSG_PDSG_OFF					0x0093
#define CHG_PCHG_OFF					0x0094
#define ALL_FETS_OFF					0x0095
#define ALL_FETS_ON						0x0096
#define SLEEP_ENABLE					0x0099
#define SLEEP_DISABLE					0x009A
#define OCDL_RECOVER					0x009B
#define SCDL_RECOVER 					0x009C
#define LOAD_DECTECT_RESTART			0x009D
#define LOAD_DECTECT_ON					0x009E
#define LOAD_DECTECT_OFF				0x009F
#define CFETOFF_LO						0x2800
#define DFETOFF_LO						0x2801
#define ALERT_LO						0x2802
#define HDQ_LO							0x2806
#define DCHG_LO							0x2807
#define DDSG_LO							0x2808
#define CFETOFF_HI			 			0x2810
#define DFETOFF_HI						0x2811
#define ALERT_HI						0x2812
#define HDQ_HI							0x2816
#define DCHG_HI							0x2817
#define DDSG_HI							0x2818
#define PF_FORCE_A						0x2857
#define PF_FORCE_B						0x29A3
#define SWAP_COMM_MODE					0x29BC
#define SWAP_TO_I2C						0x29E7
#define SWAP_TO_SPI						0x7C35
#define SWAP_TO_HDQ						0x7C40

// Subcommand with Data
#define DEVICE_NUMBER 					0x0001
#define FW_VERSION						0x0002
#define HW_VERSION						0x0003
#define IROM_SIG						0x0004
#define STATIC_CFG_SIG					0x0005
#define PREV_MACWRITE					0x0007
#define DROM_SIG						0x0009
#define SECURITY_KEYS 					0x0035
#define SAVED_PF_STATUS					0x0035
#define MANUFACTURING_STATUS			0x0057
#define MANU_DATA						0x0070
#define DASTATUS1						0x0071
#define DASTATUS2						0x0072
#define DASTATUS3						0x0073
#define DASTATUS4						0x0074
#define DASTATUS5						0x0075
#define DASTATUS6						0x0076
#define DASTATUS7						0x0077
#define CUV_SNAPSHOT					0x0080
#define COV_SNAPSHOT					0x0081
#define CB_ACTIVE_CELLS					0x0083
#define CB_SET_LVL						0x0084
#define CBSTATUS1						0x0085
#define CBSTATUS2						0x0086
#define CBSTATUS3						0x0087
#define FET_CONTROL						0x0097
#define REG12_CONTROL					0x0098
#define OTP_WR_CHECK					0x00A0
#define OTP_WRITE						0x00A1
#define READ_CAL_1						0xF081
#define CAL_CUV							0XF090
#define CAL_COV							0xF091

// Data Memory Table (RAM register)
//Calibration
#define CELL_1_GAIN 					0x9180
#define CELL_2_GAIN						0x9182
#define CELL_3_GAIN						0x9184
#define CELL_4_GAIN						0x9186
#define CELL_5_GAIN 					0x9188
#define CELL_6_GAIN						0x918A
#define CELL_7_GAIN						0x918C
#define CELL_8_GAIN						0x918E
#define CELL_9_GAIN 					0x9190
#define CELL_10_GAIN					0x9192
#define CELL_11_GAIN					0x9194
#define CELL_12_GAIN					0x9196
#define CELL_13_GAIN 					0x9198
#define CELL_14_GAIN					0x919A
#define CELL_15_GAIN					0x919C
#define CELL_16_GAIN					0x919E
#define PACK_GAIN						0x91A0
#define TOS_GAIN						0x01A2
#define LD_GAIN 						0x91A4
#define ADC_GAIN						0x91A6
#define CC_GAIN							0x91A8
#define CAPACITY_GAIN 					0x91AC
#define VCELL_OFFSET					0x91B0
#define VDIV_OFFSET						0x91B2
#define CC_OFFSET_SAMPLES				0x91C6
#define BOARD_OFFSET					0x91C8
#define INTERNAL_TEMP_OFFSET			0x91CA
#define CFETOFF_TEMP_OFFSET				0x91CB
#define DFETOFF_TEMP_OFFSET				0x91CC
#define ALERT_TEMP_OFFSET				0x91CD
#define TS1_TEMP_OFFSET					0x91CE
#define TS2_TEMP_OFFSET 				0x91CF
#define TS3_TEMP_OFFSET					0x91D0
#define HDQ_TEMP_OFFSET					0x91D1
#define DCHG_TEMP_OFFSET				0x91D2
#define DDSG_TEMP_OFFSET				0x91D3
#define INT_GAIN						0x91E2
#define INT_BASE_OFFSET					0x91E4
#define INT_MAXIMUM_AD					0x91E6
#define INT_MAXIMUM_TEMP				0x91E8
#define COEFF_A1_18K					0x91EA
#define COEFF_A2_18K					0x91EC
#define COEFF_A3_18K					0x91EE
#define COEFF_A4_18K					0x91F0
#define COEFF_A5_18K					0x91F2
#define COEFF_B1_18K					0x91F4
#define COEFF_B2_18K					0x91F6
#define COEFF_B3_18K					0x91F8
#define COEFF_B4_18K					0x91FA
#define ADC0_18K						0x91FE
#define COEFF_A1_180K					0x9200
#define COEFF_A2_180K					0x9202
#define COEFF_A3_180K					0x9204
#define COEFF_A4_180K					0x9206
#define COEFF_A5_180K					0x9208
#define COEFF_B1_180K					0x920A
#define COEFF_B2_180K					0x920C
#define COEFF_B3_180K					0x920E
#define COEFF_B4_180K					0x9210
#define ADC0_180K						0x9214
#define COEFF_A1_CUSTOM					0x9216
#define COEFF_A2_CUSTOM					0x9218
#define COEFF_A3_CUSTOM					0x921A
#define COEFF_A4_CUSTOM					0x921C
#define COEFF_A5_CUSTOM					0x921E
#define COEFF_B1_CUSTOM					0x9220
#define COEFF_B2_CUSTOM					0x9222
#define COEFF_B3_CUSTOM					0x9224
#define COEFF_B4_CUSTOM					0x9226
#define RC0_CUSTOM 						0x9228
#define ADC0_CUSTOM						0x922A
#define CC_DEADBAND						0x922D
#define CUV_THRESHOLD_OVERRIDE			0x91D4
#define COV_THRESHOLD_OVERRIDE			0x91D6
// Settings
#define MIN_BLOW_FUSE_VOLTAGE			0x9231
#define POWER_CONFIG					0x9234
#define REG12_CONFIG					0x9236
#define REG0_CONFIG						0x9237
#define HWD_REGULATOR_OPTIONS			0x9238
#define COMM_TYPE						0x9239
#define I2C_ADDRESS						0x293A
#define SPI_CONFIGURATION 				0x923C
#define COMM_IDEL_TIME					0x923D
#define CFETOFF_PIN_CONFIG				0x92FA
#define DFETOFF_PIN_CONFIG 				0x92FB
#define ALERT_PIN_CONFIG 				0x92FC
#define TS1_CONFIG 						0x92FD
#define TS2_CONFIG						0x92FE
#define TS3_CONFIG						0x92FF
#define HDQ_PIN_CONFIG					0x9300
#define DCHG_PIN_CONFIG					0x9301
#define DDSG_PIN_CONFIG					0x9302
#define DA_CONFIGURATION				0x9303
#define VCELL_MODE 						0x9304
#define CC3_SAMPLES						0x9307
#define PROTECTION_CONFIGURATION		0x925F
#define ENABLED_PROTECTION_A			0x9261
#define ENABLED_PROTECTION_B			0x9262
#define ENABLED_PROTECTION_C			0x9263
#define CHG_FET_PROTECTIONS_A			0x9265
#define CHG_FET_PROTECTIONS_B 			0x9266
#define CHG_FET_PROTECTIONS_C			0x9267
#define DSG_FET_PROTECTIONS_A 			0x9269
#define DSG_FET_PROTECTIONS_B 			0x926A
#define DSG_FET_PROTECTIONS_C 			0x926B
#define BODY_DIODE_THRESHOLD			0x9273
#define DEFAULT_ALRAM_MASK 				0x926D
#define SF_ALERT_MASK_A 				0x926F
#define SF_ALERT_MASK_B 				0x9270
#define SF_ALERT_MASK_C 				0x9271
#define PF_ALERT_MASK_A 				0x92C4
#define PF_ALERT_MASK_B 				0x92C5
#define PF_ALERT_MASK_C 				0x92C6
#define SF_ALERT_MASK_D 				0x92C7
#define ENABLED_PF_A 					0x92C0
#define ENABLED_PF_B					0x92C1
#define ENABLED_PF_C 					0x92C2
#define ENABLED_PF_D					0x92C3
#define FET_OPTIONS 					0x9308
#define CHG_PUMP_CONTROL				0x9309
#define PRECHARGE_START_VOLTAGE			0x930A
#define PRECHANGE_STOP_VOLTAGE			0x930C
#define PREDISCHARGE_TIMEOUT 			0x930E
#define PREDISCHARGE_STOP_DELTA			0x930F
#define DSG_CURRENT_THRESHOLD			0x9310
#define CHG_CURRENT_THRESHOLD			0x9312
#define CHECK_TIME 						0x9314
#define CELL_1_INTERCONNECT				0x9315
#define CELL_2_INTERCONNECT				0x9317
#define CELL_3_INTERCONNECT				0x9319
#define CELL_4_INTERCONNECT				0x931B
#define CELL_5_INTERCONNECT				0x931D
#define CELL_6_INTERCONNECT				0x931F
#define CELL_7_INTERCONNECT				0x9321
#define CELL_8_INTERCONNECT				0x9323
#define CELL_9_INTERCONNECT				0x9325
#define CELL_10_INTERCONNECT			0x9327
#define CELL_11_INTERCONNECT			0x9329
#define CELL_12_INTERCONNECT			0x932B
#define CELL_13_INTERCONNECT			0x932D
#define CELL_14_INTERCONNECT			0x932F
#define CELL_15_INTERCONNECT			0x9331
#define CELL_16_INTERCONNECT			0x9333
#define MFG_STATUS_INIT					0x9343
#define BALANCING_CONFIGURATION			0x9335
#define MIN_CELL_TEMP					0x9336
#define MAX_CELL_TEMP					0x9337
#define MAX_INTERNAL_TEMP				0x9338
#define CELL_BALANCE_INTERVAL 			0x9339
#define CELL_BALANCE_MAX_CELLS			0x933A
#define CELL_BALANCE_MIN_CELL_V_CHARGE	0x933B
#define CELL_BALANCE_MIN_DELTA_CHARGE	0x933D
#define CELL_BALANCE_STOP_DELTA_CHARGE	0x933E
#define CELL_BALANCE_MIN_CELL_V_RELAX 	0x933F
#define CELL_BALANCE_MIN_DELTA_RELAX 	0x9341
#define CELL_BALANCE_STOP_DELTA_RELAX 	0x9342
// Power
#define SHUTDOWN_CELL_VOLTAGE			0x923F
#define SHUTDOWN_STACK_VOLTAGE			0x9241
#define LOW_V_SHUTDOWN_DELAY 			0x9243
#define SHUTDOWN_TEMPERATURE 			0x9244
#define SHUTDOWN_TEMPERATURE_DELAY 		0x9245
#define FET_OFF_DELAY 					0x9252
#define SHUT_DOWN_COMMAND_DELAY 		0x9253
#define AUTO_SHUTDOWN_TIME 				0x9254
#define RAM_FAIL_SHUTDOWN_TIME 			0x9255
#define SLEEP_CURRENT 					0x9248
#define VOLTAGE_TIME 					0x924A
#define WAKE_COMPARATOR_CURRENT 		0x924B
#define SLEEP_HYSTERESIS_TIME			0x924D
#define SLEEP_CHARGER_VOLTAGE_THRESHOLD	0x924E
#define SLEEP_CHARGER_PACK_TOS_DELTA	0x9250
// System Data
#define CONFIG_RAM_SIGNATURE			0x91E0
// Protection
#define CUV_THRESHOLD 					0x9275
#define CUV_DELAY 						0x9276
#define CUV_RECOVERY_HYSTERESIS			0x927B
#define COV_THRESHOLD 					0x9278
#define COV_DELAY 						0x9279
#define COV_RECOVERY_HYSTERESIS			0x927C
#define COVL_LATCH_LIMIT 				0x927D
#define COVL_COUNTER_DEC_DELAY 			0x927E
#define COVL_RECOVERY_TIME 				0x927F
#define OCC_THRESHOLD 					0x9280
#define OCC_DELAY 						0x9281
#define OCC_RECOVERY_THRESHOLD			0x9288
#define PACK_TOS_DELTA 					0x92B0
#define OCD1_THRESHOLD 					0x9282
#define OCD1_DELAY 						0x9383
#define OCD2_THRESHOLD 					0x9284
#define OCD2_DELAY 						0x9285
#define SCD_THRESHOLD					0x9286
#define SCD_DELAY 						0x9287
#define SCD_RECOVERY_TIME 				0x9284
#define OCD3_THRESHOLD 					0x928A
#define OCD3_DELAY 						0x928C
#define OCD_RECOVERY_THRESHOLD 			0x928D
#define OCDL_LATCH_LIMIT				0x928F
#define OCDL_COUNTER_DEC_DELAY  		0x9290
#define OCDL_RECOVERY_TIME 				0x9291
#define OCDL_RECOVERY_THRESHOLD 		0x9292
#define SCDL_LATCH_LIMIT 				0x9295
#define SCDL_COUNTER_DEC_DELAY 			0x9296
#define SCDL_RECOVERY_TIME 				0x9297
#define SCDL_RECOVERY_THRESHOLD 		0x9298
#define OTC_THRESHOLD					0x929A
#define OTC_DELAY 						0x929B
#define OTC_RECOVERY 					0x929C
#define OTD_THRESHOLD 					0x929D
#define OTD_DELAY 						0x929E
#define OTD_RECOVERY 					0x929F
#define OTF_THRESHOLD 					0x92A0
#define OTF_DELAY 						0x92A1
#define OTF_RECOVERY 					0x92A2
#define OTINT_THRESHOLD 				0x92A3
#define OTINT_DELAY 					0x92A4
#define OTINT_RECOVERY 					0x92A5
#define UTC_THRESHOLD 					0x92A6
#define UTC_DELAY 						0x92A7
#define UTC_RECOVERY					0x92A8
#define UTD_THRESHOLD 					0x92A9
#define UTD_DELAY 						0x92AA
#define UTD_RECOVERY 					0x92AB
#define UTINT_THRESHOLD 				0x92AC
#define UTINT_DELAY 					0x92AD
#define UTINT_RECOVERY 					0x92AE
#define RECOVERY_TIME 					0x92AF
#define HWD_DELAY 						0x92B2
#define LOAD_DETECT_ACTIVE_TIME			0x92B4
#define LOAD_DETECT_RETRY_DELAY 		0x92B5
#define LOAD_DETECT_TIMEOUT 			0x92B6
#define PTO_CHARGE_THRESHOLD 			0x92BA
#define PTO_DELAY						0x92BC
#define PTO_RESET 						0x92BE
//Permanent Fail
#define CUDEP_THRESHOLD					0x92C8
#define CUDEP_DELAY 					0x92CA
#define SUV_THRESHOLD					0x92CB
#define SUV_DELAY 						0x92CD
#define SOV_THRESHOLD 					0x92CE
#define SOV_DELAY 						0x92D0
#define TOS_THRESHOLD					0x92D1
#define TOS_DELAY 						0x92D3
#define SOCC_THRESHOLD	 				0x92D4
#define SOCC_DELAY 						0x92D6
#define SOCD_THRESHOLD 					0x92D7
#define SOCD_DELAY 						0x92D9
#define SOT_THRESHOLD 					0x92DA
#define SOT_DELAY 						0x92DB
#define SOTF_THRESHOLD 					0x92DC
#define SOTF_DELAY 						0x92DD
#define VIMR_CHECK_VOLTAGE 				0x92DE
#define VIMR_MAX_RELAX_CURRENT 			0x92E0
#define VIMR_TRESHOLD					0x92E2
#define VIMR_DELAY 						0x92E4
#define VIMR_RELAX_MIN_DUARATION		0x92E5
#define VIMA_CHECK_VOLTAGE 				0x92E7
#define VIMA_MIN_ACTIVE_CURRENT 		0x92E9
#define VIMA_THRESHOLD 					0x92EB
#define VIMA_DELAY 						0x92ED
#define CFETF_OFF_THRESHOLD 			0x92EE
#define CFET_OFF_DELAY 					0x92F0
#define DFETF_OFF_THRESHOLD 			0x92F1
#define DEFTF_OFF_DELAY 				0x92F3
#define VSSF_FAIL_THRESHOLD 			0x92F4
#define DELAY_2LVL 						0x92F7
#define LFOF_DELAY 						0x92F8
#define HWMX_DELAY 						0x92F9
// Security
#define SECURITY_SETTINGS				0x9256
#define UNSEAL_KEY_STEP_1  				0x9257
#define UNSEAL_KEY_STEP_2				0x9259
#define FULL_ACCESS_KEY_STEP_1			0x925B
#define FULL_ACCESS_KEY_STEP_2 			0x925D

/*
 * sub command bit mask
 */

// FET control register bit mask
#define PCHG_OFF 						(1<<3)
#define CHG_OFF							(1<<2)
#define PDSG_OFF						(1<<1)
#define DSG_OFF							(1<<0)

/*
 * Data RAM bit mask
 */
// FET_OPTION register
#define FET_INIT_OFF 					(1<<5)
#define PDSG_EN							(1<<4)
#define FET_CTRL_EN						(1<<3)
#define HOST_FET_EN						(1<<2)
#define SLEEPCHG						(1<<1)
#define SFET							(1<<0)
// POWER_CONFIG register
#define DPSLD_OT						(1<<13)
#define SHUT_TS2						(1<<12)
#define DPSLP_PD						(1<<11)
#define DPSLP_LDO						(1<<10)
#define DPSLP_LFO						(1<<9)
#define SLEEP_REG						(1<<8)
#define OTSD							(1<<7)
#define FASTADC							(1<<6)
#define CB_LOOP_SLOW_1					(1<<5)
#define CB_LOOP_SLOW_0					(1<<4)
#define LOOP_SLOW_1						(1<<3)
#define LOOP_SLOW_0						(1<<2)

typedef struct _Register_Group BQ_Register_Group;
struct _Register_Group{
	union
	{
		struct
		{
		unsigned short RSVD			:13;
		unsigned char DEEP_SLEEP	:1;
		unsigned char LD_TIMEOUT	:1;
		unsigned char LD_ON 		:1;
		}ControlStatusBit;
		unsigned short ControlStatusByte;
	}ControlStatus;

	union
	{
		struct
		{
			unsigned char SCD 		:1;
			unsigned char OCD2 		:1;
			unsigned char OCD1		:1;
			unsigned char OCC 		:1;
			unsigned char COV 		:1;
			unsigned char CUV 		:1;
			unsigned char RSVD 		:2;
		}SafetyAlertABit;
		unsigned char SatetyAlertAByte;
	}SafetyAlertA;

	union
	{
		struct
		{
			unsigned char SCD 		:1;
			unsigned char OCD2 		:1;
			unsigned char OCD1 		:1;
			unsigned char OCC 		:1;
			unsigned char COV 		:1;
			unsigned char CUV 		:1;
			unsigned char RSVD 		:2;
		}SafetyStatusABit;
		unsigned char SafetyStatusAByte;
	}SafetyStatusA;

	union
	{
		struct
		{
			unsigned char OTF 		:1;
			unsigned char OTINT		:1;
			unsigned char OTD 		:1;
			unsigned char OTC 		:1;
			unsigned char RSVD		:1;
			unsigned char UTINT 	:1;
			unsigned char UTD 		:1;
			unsigned char UTC 		:1;
		}SafetyAlertBBit;
		unsigned char SafetyAlertBByte;
	}SafetyAlertB;

	union
	{
		struct
		{
			unsigned char OTF		:1;
			unsigned char OTINT		:1;
			unsigned char OTD 		:1;
			unsigned char OTC 		:1;
			unsigned char RSVD		:1;
			unsigned char UTINT 	:1;
			unsigned char UTD 		:1;
			unsigned char UTC 		:1;
		}SafetyStatusBBit;
		unsigned char SafetyStatusBByte;
	}SafetyStatusB;

	union
	{
		struct
		{
			unsigned char OCD3  	:1;
			unsigned char SCDL		:1;
			unsigned char OCDL		:1;
			unsigned char COVL 		:1;
			unsigned char PTOS 		:1;
			unsigned char RSVD  	:3;
		}SafetyAlertCBit;
		unsigned char SafetyAlertCByte;
	}SafetyAlertC;

	union
	{
		struct
		{
			unsigned char OCD3 		:1;
			unsigned char SCDL		:1;
			unsigned char OCDL		:1;
			unsigned char COVL 		:1;
			unsigned char RSVD1		:1;
			unsigned char PTO 		:1;
			unsigned char HWDF		:1;
			unsigned char RSVD2 	:1;
		}SafetyStatusCBit;
		unsigned char SafetyStatusCByte;
	}SafetyStatusC;

	union
	{
		struct
		{
			unsigned char CUDEP		:1;
			unsigned char SOTF		:1;
			unsigned char RSVD		:1;
			unsigned char SOT 		:1;
			unsigned char SOCD		:1;
			unsigned char SOCC		:1;
			unsigned char SOV 		:1;
			unsigned char SUV		:1;
		}PFAlertABit;
		unsigned char PFAlertAByte;
	}PFAlertA;

	union
	{
		struct
		{
			unsigned char CUDEP		:1;
			unsigned char SOTF		:1;
			unsigned char RSVD 		:1;
			unsigned char SOT 		:1;
			unsigned char SOCD		:1;
			unsigned char SOCC 		:1;
			unsigned char SOV 		:1;
			unsigned char SUV 		:1;
		}PFStatusABit;
		unsigned char PFStatusAByte;
	}PFStatusA;

	union
	{
		struct
		{
			unsigned char SCDL  	:1;
			unsigned char RSVD		:2;
			unsigned char VIMA		:1;
			unsigned char VIMR		:1;
			unsigned char SLVL		:1;
			unsigned char DFETF 	:1;
			unsigned char CFETF 	:1;
		}PFAlertBBit;
		unsigned char PFAlertBByte;
	}PFAlertB;

	union
	{
		struct
		{
			unsigned char SCDL  	:1;
			unsigned char RSVD		:2;
			unsigned char VIMA		:1;
			unsigned char VIMR		:1;
			unsigned char SLVL		:1;
			unsigned char DFETF 	:1;
			unsigned char CFETF 	:1;
		}PFStatusBBit;
		unsigned char PFStatusBByte;
	}PFStatusB;

	union
	{
		struct
		{
			unsigned char RSVD1 	:1;
			unsigned char HWMX		:1;
			unsigned char VSSF		:1;
			unsigned char VREF		:1;
			unsigned char LFOF		:1;
			unsigned char RSVD2 	:3;
		}PFAlertCBit;
		unsigned char PFAlertCByte;
	}PFAlertC;

	union
	{
		struct
		{
			unsigned char CMDF 		:1;
			unsigned char HWMX		:1;
			unsigned char VSSF		:1;
			unsigned char VREF		:1;
			unsigned char LFOF		:1;
			unsigned char IRMF		:1;
			unsigned char DRMF		:1;
			unsigned char OTPF		:1;
		}PFStatusCBit;
		unsigned char PFStatusCByte;
	}PFStatusC;

	union
	{
		struct
		{
			unsigned char RSVD 		:7;
			unsigned char TOSF		:1;
		}PFAlertDBit;
		unsigned char PFAlertDByte;
	}PFAlertD;

	union
	{
		struct
		{
			unsigned char RSVD 		:7;
			unsigned char TOSF		:1;
		}PFStatusDBit;
		unsigned char PFStatusDByte;
	}PFStatusD;

	union
	{
		struct
		{
			unsigned char SLEEP 	:1;
			unsigned char RSVD 		:1;
			unsigned char SD_CMD	:1;
			unsigned char PF 		:1;
			unsigned char SS 		:1;
			unsigned char FUSE 		:1;
			unsigned char SEC1		:1;
			unsigned char SEC0 		:1;
			unsigned char OTPB		:1;
			unsigned char OTPW		:1;
			unsigned char COW_CHK	:1;
			unsigned char WD 		:1;
			unsigned char POR		:1;
			unsigned char SLEEP_EN	:1;
			unsigned char PCHG_MODE	:1;
			unsigned char CFGUPDATE	:1;
		}BatteryStatusBit;
		unsigned short BatteryStatusByte;
	}BateryStatus;

	union
	{
		struct
		{
			unsigned char SSBC 			:1;
			unsigned char SSA 			:1;
			unsigned char PF 			:1;
			unsigned char MSK_SFALERT 	:1;
			unsigned char MSK_PFALERT	:1;
			unsigned char INITSTART		:1;
			unsigned char INITCOMP		:1;
			unsigned char RSVD 			:1;
			unsigned char FULLSCAN		:1;
			unsigned char XCHG			:1;
			unsigned char XDSG 			:1;
			unsigned char SHUTV			:1;
			unsigned char FUSE			:1;
			unsigned char CB 			:1;
			unsigned char ADSCAN 		:1;
			unsigned char WAKE			:1;
		}AlarmStatusBit;
		unsigned short AlarmStatusByte;
	}AlarmStatus;

	union
	{
		struct
		{
			unsigned char SSBC 			:1;
			unsigned char SSA 			:1;
			unsigned char PF 			:1;
			unsigned char MSK_SFALERT	:1;
			unsigned char MSK_PFALERT	:1;
			unsigned char INITSTART		:1;
			unsigned char INITCOMP 		:1;
			unsigned char RSVD 			:1;
			unsigned char FULLSCAN 		:1;
			unsigned char XCHG 			:1;
			unsigned char CDSG			:1;
			unsigned char SHUTV 		:1;
			unsigned char FUSE 			:1;
			unsigned char CB 			:1;
			unsigned char ADSCAN 		:1;
			unsigned char WAKE 			:1;
		}AlarmRawStatusBit;
		unsigned short AlarmRawStatusByte;
	}AlarmRawStatus;

	union
	{
		struct
		{
			unsigned char SSBC  		:1;
			unsigned char SSA 			:1;
			unsigned char PF 			:1;
			unsigned char MSK_SFALERT	:1;
			unsigned char MSK_PFALERT 	:1;
			unsigned char INITSTART 	:1;
			unsigned char INITCOMP		:1;
			unsigned char RSVD 			:1;
			unsigned char FULLSCAN 		:1;
			unsigned char XCHG 			:1;
			unsigned char XDSG 			:1;
			unsigned char SHUTV 		:1;
			unsigned char FUSE 			:1;
			unsigned char CB 			:1;
			unsigned char ADSCAN		:1;
			unsigned char WAKE 			:1;
		}AlarmEnableBit;
		unsigned short AlarmEnableByte;
	}AlarmEnable;

	union
	{
		struct
		{
			unsigned char RSVD 			:1;
			unsigned char ALRT_PIN 		:1;
			unsigned char DDSG_PIN 		:1;
			unsigned char PDSG_PIN 		:1;
			unsigned char PDSG_FET		:1;
			unsigned char DSG_FET 		:1;
			unsigned char PCHG_FET		:1;
			unsigned char CHG_FET		:1;
		}FETStatusBit;
		unsigned char FETStatusByte;
	}FETStatus;
};

typedef struct BQ769x2_t BQ769x2;

struct BQ769x2_t{
	AFE base;
	BQ_Hw* hw;
	BQ_Register_Group bq_registers;
	uint32_t adc_gain;
	uint32_t adc_offset;
	//BQ_TEMP_MONITOR_SRC temp_monitor_source;
	uint8_t cell_group;
	uint8_t protect_level;
	uint8_t unshort_cell;
};

void bq_init_1(BQ769x2 *p_bq);
void bq_set_cell_group(BQ769x2* p_bq,const uint8_t group);

void bq_turn_off_all_fet(BQ769x2* p_bq);
void bq_turn_on_all_fet(BQ769x2* p_bq);
void bq_turn_off_fet_DSG_PDSG(BQ769x2* p_bq);
void bq_turn_off_fet_CHG_PCHG(BQ769x2* p_bq);
uint8_t bq_read_fet_status_reg(BQ769x2* p_bq);
/*
 *  configure body diode threshold to avoid current damage FETs
 */
void bq_config_body_diode_threshold(BQ769x2* p_bq, uint16_t threshold_mA);
/*
 *  set bit and clear bit for byte or word with direct command
 */
int bq_set_byte_register_bit(BQ769x2* p_bq,uint8_t reg, const uint8_t bit_mask);
int bq_clear_byte_register_bit(BQ769x2* p_bq,uint8_t reg, const uint8_t bit_mask);
int bq_set_word_register_bit(BQ769x2* p_bq,uint16_t reg, const uint16_t bit_mask);
int bq_clear_word_register_bit(BQ769x2* p_bq,uint16_t reg, const uint16_t bit_mask);

int16_t bq_read_ntc_temperature_TS1(BQ769x2* p_bq);
int16_t bq_read_ntc_temperature_TS3(BQ769x2* p_bq);
int16_t bq_read_ntc_temperature_HDQ_TEMP(BQ769x2* p_bq);
int16_t bq_read_ntc_temperature_CFETOFF_TEMP(BQ769x2* p_bq);

int16_t bq_read_cc2_current(BQ769x2* p_bq);
void bq_enter_config_update_mode(BQ769x2* p_bq);
void bq_exit_config_update_mode(BQ769x2* p_bq);
static void bq_clear_alert_bit(BQ769x2* p_bq);

static inline void bq_set_interface(BQ769x2* const p_bq,const AFE_Interface* const p_inf){
	afe_set_interface((AFE*) p_bq,p_inf);
}
static inline uint32_t bq_get_pack_voltage(const BQ769x2* const p_bq){
	return afe_get_pack_voltage((AFE*)p_bq);
}

static inline uint32_t bq_get_cell_voltage(const BQ769x2* const p_bq,const uint8_t cell_id){
	return afe_get_cell_voltage((AFE*)p_bq,cell_id);
}

static inline void bq_update_cell_voltage(const BQ769x2* const p_bq){
	p_bq->base.interface->update_cell_voltage((AFE*)p_bq);
}

#endif /* COMPONENT_AFE_BQ769X2_H_ */
