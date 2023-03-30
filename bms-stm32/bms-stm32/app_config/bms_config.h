/*
 * bms_config.h
 *
 *  Created on: Aug 18, 2020
 *      Author: quangnd
 */

#ifndef APP_CONFIG_BMS_CONFIG_H_
#define APP_CONFIG_BMS_CONFIG_H_

#define APP_FAULT_RECOVER_TIMEOUT_mS                    7000
#define INRUSH_LIM_TIMEOUT_mS                           2000
#define INRUSH_CURRENT_EXIT_THRESHOLD_mA                50L
#define BMS_STANDBY_TO_SHIPMODE_TIME_MIN                30
#define BMS_SYSTEM_BOOST_UP_mS							3000

#define BMS_BALANCING_ENABLE							1
#define CUR_EXIT_BOOT_UP_CentiA							15
//#define OVER_TEMP_FET_PROTECT							100 /*100oC*/
//#define UNDER_TEMP_FET_PROTECT							-10 /*-10oC*/
//#define UNDER_TEMP_FET_PROTECT_RECOVERY					0 /*0oC*/
//#define OVER_TEMP_FET_PROTECT_RECOVERY					90 /*90oC*/
//#define BMS_COV_HANLDE_VOL								4250
//#define BMS_CUV_HANLDE_VOL								3000

#define BMS_BALANCING_ENABLE							1

#define BMS_NODE_ID_PIN_DEBOUNCE_TIMEOUT_mS										(4000UL)

#endif /* APP_CONFIG_BMS_CONFIG_H_ */
