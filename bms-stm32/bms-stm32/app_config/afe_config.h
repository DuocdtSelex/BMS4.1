/*
 * afe_config.h
 *
 *  Created on: Aug 21, 2020
 *      Author: quangnd
 */

#ifndef APP_CONFIG_AFE_CONFIG_H_
#define APP_CONFIG_AFE_CONFIG_H_

#define BQ_CELL_GROUP						1
#define BQ_MAX_CELL							(16*BQ_CELL_GROUP)
#define PACK_MAX_CELL						BQ_MAX_CELL
#define CELL_CAPACITY						(4*4000UL) /* 4 parallel cell */

#define SCD_THRESHOLD_mA  					80000
#define SCD_DELAY_uS  						15
#define SCD_RECOVERY_TIME_s					5

#define OCC_THRESHOLD_mA					12000
#define OCC_DELAY_mS						160
#define OCC_HYSTERESIS_mA					-200

#define OCD1_THRESHOLD_mA  					50000
#define OCD1_DELAY_mS  						10

#define OCD2_THRESHOLD_mA					50000
#define OCD2_DELAY_mS						30
#define OCD_RECOVERY_THRESHOLD_mA			-10

#define COV_THRESHOLD_mV					4352
#define CUV_THRESHOLD_mV					2500
#define COV_DELAY_mS						255
#define CUV_DELAY_mS						255
#define COV_HYSTERESIS_mV					110
#define CUV_HYSTERESIS_mV					110

#define OTD_THRESHOLD_DEG_C					60
#define OTC_THRESHOLD_DEG_C					60
#define OTINT_THRESHOLD_DEG_C				85
#define OTF_THRESHOLD_DEG_C					80

#define OTC_RECOVERY_C								55

#define UTD_THRESHOLD_DEG_C					0
#define UTC_THRESHOLD_DEG_C					0
#define UTINT_THRESHOLD_DEG_C				0


#define OT_THRESHOLD_DEG						70

#endif /* APP_CONFIG_AFE_CONFIG_H_ */
