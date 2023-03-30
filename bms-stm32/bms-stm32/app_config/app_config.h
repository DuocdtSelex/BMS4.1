/*
 * app_config.h
 *
 *  Created on: Oct 20, 2020
 *      Author: quangnd
 */

#ifndef APP_CONFIG_APP_CONFIG_H_
#define APP_CONFIG_APP_CONFIG_H_
#include "core_hal.h"

//add from BMA RA
#define cnt_sys_tick                        					(10)
#define CNT_5_MINUTE_mS                      					30000
#define CNT_20_MINUTE_10mS                      				120000
#define CNT_10_SEC_mS                      						10000
#define CNT_150_mS                        						150
//
//
#define CNT_1_5_MINUTE_mS										9000
#define CNT_1_MINUTE_mS                        					6000
//
//#define CNT_20_MINUTE_100mS                      				12000
//#define CNT_10_SEC_100mS                      					100
//#define CNT_10_SEC_100mS                      					100
//#define CNT_10_SEC_100mS                      					100
//
//#define BMS_OBJECT_INDEX										0x2003
//#define BMS_SERIAL_NUMBER_OBJECT_SUB_INDEX						0x00
//#define BMS_MAINSWITCH_SUB_INDEX								0x01
//#define SLAVE_ID_NUMBER_OBJECT_SUB_INDEX						0x02
//
//#define MATING_INDEX											0x2004
//#define BP_MATING_STATE_SN_OBJECT_SUB_INDEX						0x00
//#define VEHICLE_SN_OBJECT_SUB_INDEX								0x01
//
//#define NEW_FIRMWARE_REQUEST_INDEX								0x2001

#define APP_STATE_MACHINE_UPDATE_TICK_mS                        (1000/(HAL_SYSTICK_FREQ_Hz))

#endif /* APP_CONFIG_APP_CONFIG_H_ */
