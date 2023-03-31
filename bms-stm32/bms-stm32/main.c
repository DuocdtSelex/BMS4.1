/*
 * main.c
 *
 *  Created on: Aug 18, 2020
 *      Author: quangnd
 */
#include "board.h"
#include "afe_init.h"
#include "bms_init.h"
#include "key_init.h"
#include "afe.h"
#include "delay.h"
#include "can_hal.h"
#include "string_util.h"
#include "ntc.h"
#include "app_config.h"
#include "bms_config.h"
#include "keypad.h"
#include "CO.h"
#include "canopen_init.h"
#include "rgb_led_hal.h"
#include "hc05_hal.h"
#include "app_config.h"
#include "bms_config.h"

extern BMS selex_bms;
extern Keypad user_key;
static uint32_t app_fault_recover_timeout_ms;

//add from BMS RA
uint32_t cnt_on_off_test = 0;
static uint32_t shutdown_update_counter = 0;
static uint32_t softstart_fail_cnt = 0;
static uint8_t  pack_data_buffer[512];
static uint32_t nodeID_debounce = 0;
static uint32_t only_dischar_time_existence = 0;

CanTxMsg tx_msg;
uint32_t sys_timestamp=0;

static void app_init(void){
	global_interrupt_disable();
	user_key_init();
	board_init();
	afe_init();
	bms_init();
	bms_erase_data_error(&selex_bms);
	canopen_service_init();
	app_fault_recover_timeout_ms=APP_FAULT_RECOVER_TIMEOUT_mS;
	bms_update_status(&selex_bms);
	bms_update_cell_voltages(&selex_bms);
	bms_update_current(&selex_bms);
	bms_init_soc_spkf(&selex_bms);
	bms_init_soh_awtls(&selex_bms);
//	bms_init_soc_cc(&selex_bms);
	bms_update_pack_zone_temperature(&selex_bms);
	bms_set_state(&selex_bms,BMS_ST_IDLE);
	sys_timestamp=0;
	afe_reset_error(&app_afe);
	global_interrupt_enable();
}

int main(void){
	app_init();
	while(1){
	}
	return 0;
}

static volatile uint32_t update_counter=0;
static volatile uint32_t balancing_update_counter=0;
static volatile uint32_t boost_up_counter=0;

static volatile uint32_t softstart_off_delay_ms;

static volatile uint32_t timeout_counter=0;
static uint8_t pack_data_buffer[512];
static uint16_t uart_rx_data;
static uint8_t data_error_buffer[1024];

uint8_t* buffer;
uint8_t* buffer_data_log;

//static volatile uint32_t count_second = 0;
//static volatile uint32_t count_minute = 0;

void HAL_STATE_MACHINE_UPDATE_TICK(void){
        sys_timestamp+=APP_STATE_MACHINE_UPDATE_TICK_mS;

        static uint8_t sync_step=0;
	co_read_node_id_signal(&node_id_selex);
	key_update_state(&user_key);
	bms_update_current(&selex_bms);
	bms_update_pack_voltage(&selex_bms);
	CO_process(&selex_bms.co_app, APP_STATE_MACHINE_UPDATE_TICK_mS);
	bms_update_battery_status(&selex_bms);
	bms_update_fet_status(&selex_bms);

	update_counter++;
	if (update_counter == 50) {
		bms_update_status(&selex_bms);
		bms_update_safety_alert_status(&selex_bms);

		bms_check_status_operation(&selex_bms);
		bms_check_temperature_condition(&selex_bms);


		if((selex_bms.status !=0)
				|| (selex_bms.afe->safety.safety_alert_a!=0)
				|| (selex_bms.afe->safety.safety_alert_b !=0)
				|| (selex_bms.afe->safety.safety_status_a !=0)
				|| (selex_bms.afe->safety.safety_status_b !=0))
		{

			selex_bms.error=selex_bms.status;
			selex_bms.last_error[0]=selex_bms.error;
			selex_bms.last_error[1]=selex_bms.afe->safety.safety_alert_a;
			selex_bms.last_error[2]=selex_bms.afe->safety.safety_alert_b;
			selex_bms.last_error[3]=selex_bms.afe->safety.safety_status_a;
			selex_bms.last_error[4]=selex_bms.afe->safety.safety_status_b;

			if(bms_get_state(&selex_bms)!=BMS_ST_FAULT)
			{
				bms_set_state(&selex_bms, BMS_ST_FAULT);
			}
		}
		else{
			bms_update_switch_state(&selex_bms);
		}
	}

	if(update_counter==51){
		bms_update_pack_voltage(&selex_bms);
	}

	if(update_counter==52){
		bms_update_cell_voltages(&selex_bms);
		bms_cell_over_voltage_handle(&selex_bms);
	}

	if(update_counter==53){
		bms_update_pack_zone_temperature(&selex_bms);
	}

	if(update_counter==54){
		bms_update_temperature_ts5(&selex_bms);
	}

	if(update_counter==55){
		bms_update_temperature_ts6(&selex_bms);
	}

	if(update_counter==56){
		bms_update_int_temperature_C(&selex_bms);
	}

	if(update_counter==57){
		bms_update_permanent_fail_state(&selex_bms);
	}

	if(update_counter ==58){
	//	hc05_receive_characters(&hc05_port);
#if 0
		buffer_data_log = data_error_buffer;
		bms_read_data_error(&selex_bms);
		build_log_error_data(&selex_bms, buffer_data_log);
		hc05_hw_sends(&hc05_port, buffer_data_log);
#endif
	}

	if(update_counter==60){
		buffer=pack_data_buffer;
		build_pack_data(&selex_bms,buffer);
		hc05_hw_sends(&hc05_port, buffer);
		update_counter=0;
	}

	PDO_process(&selex_bms.co_app, APP_STATE_MACHINE_UPDATE_TICK_mS);

/*
	current = bms_get_current_mA(&selex_bms);
	if ((current > -1000) & (current < 1000)) {
		count_second+=APP_STATE_MACHINE_UPDATE_TICK_mS;
		if (count_second >= 60000) {
			count_minute++;
			count_second = 0;
		}
		if (count_minute == BMS_STANDBY_TO_SHIPMODE_TIME_MIN) {
			bms_set_state(&selex_bms, BMS_ST_SHIPMODE);
		}
	}
	else{
		count_second=0;
		count_minute=0;
	}
*/
	switch(bms_get_state(&selex_bms)){
	case BMS_ST_FAULT:
	        selex_bms.fault_recover_timeout+=APP_STATE_MACHINE_UPDATE_TICK_mS;
	        if(selex_bms.fault_recover_timeout >=app_fault_recover_timeout_ms){
	                bms_recover_from_fault(&selex_bms);
	        }
	        break;
	case BMS_ST_STANDBY:
			co_update_sdo_port(&bms_sdo);
			if(node_id_selex.current_state == 0){
				bms_set_state(&selex_bms, BMS_ST_IDLE);
			}

			if(keypad_is_hold(&user_key)){
				bms_set_state(&selex_bms,BMS_ST_IDLE);
				keypad_reset_hold_state(&user_key);
			}

			if (selex_bms.balancing_enabled) {

				if (balancing_update_counter++ > 1000) {
					balancing_update_counter = 0;
					bms_active_balancing(&selex_bms);
				}
			}
        break;
	case BMS_ST_DISCHARGING:
	case BMS_ST_CHARGING:
			co_update_sdo_port(&bms_sdo);
			if(node_id_selex.current_state == 0){
				selex_bms.node_id_pin_debounce_counter_ms+=APP_STATE_MACHINE_UPDATE_TICK_mS;
				if(selex_bms.node_id_pin_debounce_counter_ms>= selex_bms.node_id_pin_debounce_counter_timeout){
					node_id_selex.contact_bouncing_detect=1;
						bms_set_state(&selex_bms, BMS_ST_IDLE);
						selex_bms.switch_state=0;
						bms_update_and_write_data_error(&selex_bms, NODE_ID_ERROR);
				}
			}
			else{
					selex_bms.node_id_pin_debounce_counter_ms=0;
			}

	        if(keypad_is_hold(&user_key)){
	                bms_set_state(&selex_bms,BMS_ST_IDLE);
	                keypad_reset_hold_state(&user_key);
	        }
	        break;
	case BMS_ST_IDLE:
//			bms_sdo.bms_node_id = 0;
//			co_update_sdo_port(&bms_sdo);
//			if (keypad_is_hold(&user_key)) {
//				if(co_check_node_id_pin(&node_id_selex) == 0){
//				   bms_set_state(&selex_bms, BMS_ST_SOFTSTART);
//				}
//				else{
//					bms_set_state(&selex_bms, BMS_ST_ID_ASSIGN_START);
//				}
//				keypad_reset_hold_state(&user_key);
//
//			}
//			if (selex_bms.balancing_enabled) {
//
//				if (balancing_update_counter++ > 1000) {
//					balancing_update_counter = 0;
//					bms_active_balancing(&selex_bms);
//				}
//			}
			// check event rising of node id
//			if((node_id_selex.previous_state ==0) &&
//                                        (node_id_selex.current_state == 1)){
//			        bms_set_state(&selex_bms, BMS_ST_ID_ASSIGN_START);
//				}
			//add from bms ra
				bms_sdo.bms_node_id = 0;
				CO_set_node_id(&selex_bms.co_app, 0);
				co_update_sdo_port(&bms_sdo);
				if (keypad_is_hold(&user_key)) {
					if (bms_get_node_select_state(&selex_bms) == NODE_ID_PIN_ST_DESELECT) {
						bms_set_state(&selex_bms, BMS_ST_SOFTSTART);
						keypad_reset_hold_state(&user_key);
						balancing_update_counter = 0;
						shutdown_update_counter = 0;
					} else {
						bms_set_state(&selex_bms, BMS_ST_ID_ASSIGN_START);
						keypad_reset_hold_state(&user_key);
					}
				}
				/*check for active balancing */
				if (selex_bms.balancing_enabled) {

					if (balancing_update_counter++ > CNT_1_MINUTE_mS) {
						bms_active_balancing(&selex_bms);
						balancing_update_counter = 0;
					}
				}
//			if (bms_is_slave_select_request(&selex_bms)) {
//						bms_set_state(&selex_bms, BMS_ST_ID_ASSIGN_START);
//						balancing_update_counter = 0;
//						shutdown_update_counter = 0;
//					}
				if((node_id_selex.previous_state ==0) &&
				                                       (node_id_selex.current_state == 1)){
							        bms_set_state(&selex_bms, BMS_ST_ID_ASSIGN_START);
								}
			break;
	case BMS_ST_ID_ASSIGN_START:
			co_send_node_assgin_request(&can1);
	        bms_set_state(&selex_bms,BMS_ST_ID_ASSIGN_WAIT_CONFIRM);
	        selex_bms.id_assign_timeout=sys_timestamp+3000;
		break;
	case BMS_ST_ID_ASSIGN_WAIT_CONFIRM:
	        if(selex_bms.id_assign_timeout<=sys_timestamp){
	                /* id assign timeoout */
	                bms_set_state(&selex_bms, BMS_ST_IDLE);
	                bms_update_and_write_data_error(&selex_bms, ASSIGN_WAIT_CONFIRM_ERROR);
	        }

	        break;
	case BMS_ST_ID_ASSIGN_CONFIRMED:
//        if(selex_bms.id_assign_timeout<=sys_timestamp){
//                /* id assign timeoout */
//                bms_set_state(&selex_bms, BMS_ST_IDLE);
//                bms_update_and_write_data_error(&selex_bms, ASSIGN_CONFRIM_ERROR);
//        }
//        if(node_id_selex.current_state == 0){
//        	co_send_node_assgin_request(&can1);
//        	bms_set_state(&selex_bms, BMS_ST_ID_ASSIGN_WAIT_SLAVE_SELECT);
//        }
		if (selex_bms.id_assign_timeout <= sys_timestamp) {
					/* id assign timeoout */
					bms_set_state(&selex_bms, BMS_ST_IDLE);
					bms_update_and_write_data_error(&selex_bms,
							ASSIGN_CONFRIM_ERROR);
				}

				if (bms_get_node_select_state(&selex_bms) == NODE_ID_PIN_ST_DESELECT) {
					co_send_node_assgin_request(&can1);

					bms_set_state(&selex_bms, BMS_ST_ID_ASSIGN_WAIT_SLAVE_SELECT);
				}
			break;
	case BMS_ST_ID_ASSIGN_WAIT_SLAVE_SELECT:
        if(selex_bms.id_assign_timeout<=sys_timestamp){
                /* id assign timeoout */
                bms_set_state(&selex_bms, BMS_ST_IDLE);
                bms_update_and_write_data_error(&selex_bms, ASSIGN_WAIT_SLAVE_SELECT_ERROR);
        }
			break;
	case BMS_ST_START_AUTHENTICATE:
	        co_sdo_set_state(SDO_ST_IDLE);
	        bms_set_state(&selex_bms, BMS_ST_AUTHENTICATING);
	        break;
	case BMS_ST_AUTHENTICATING:
//			if(selex_bms.id_assign_timeout<=sys_timestamp){
//					bms_set_state(&selex_bms, BMS_ST_IDLE);
//			}
			timeout_counter +=APP_STATE_MACHINE_UPDATE_TICK_mS;
	        if(timeout_counter >= 3000){
	                bms_set_state(&selex_bms, BMS_ST_IDLE);
	                timeout_counter =0;
	                bms_update_and_write_data_error(&selex_bms, AUTHENICATING_ERROR);
	        }
			if (CO_SDO_get_status(&selex_bms.co_app.sdo_server)
					== CO_SDO_RT_success) {
				CO_SDO_reset_status(&selex_bms.co_app.sdo_server);
				if (bms_sdo.bms_node_id == 4) {
					bms_set_state(&selex_bms, BMS_ST_DISCHARGING);
					selex_bms.switch_state = 3;
					timeout_counter = 0;
				} else {
					bms_set_state(&selex_bms, BMS_ST_STANDBY);
					timeout_counter = 0;
				}
			}

	        break;
	case BMS_ST_SOFTSTART:
//                selex_bms.inrush_lim_time+=APP_STATE_MACHINE_UPDATE_TICK_mS;
//                if(selex_bms.inrush_lim_time>=INRUSH_LIM_TIMEOUT_mS){
//                        bms_set_state(&selex_bms,BMS_ST_FAULT);
//                        bms_update_and_write_data_error(&selex_bms, SOFT_START_ERROR);
//                }
//                if((selex_bms.current >=-INRUSH_CURRENT_EXIT_THRESHOLD_mA) &&
//                                (selex_bms.current <=INRUSH_CURRENT_EXIT_THRESHOLD_mA)){
//
//                        bms_set_state(&selex_bms,BMS_ST_SYSTEM_BOOST_UP);
//                        softstart_off_delay_ms=0;
//                }

//add from BMS RA
                selex_bms.inrush_lim_time += APP_STATE_MACHINE_UPDATE_TICK_mS;
                		if ((selex_bms.current >=-INRUSH_CURRENT_EXIT_THRESHOLD_mA) && (selex_bms.inrush_lim_time <= INRUSH_LIM_TIMEOUT_mS)) {
                			if (bms_check_fet_status(&selex_bms, 5)) {
                				softstart_off_delay_ms = 0;
                				bms_set_state(&selex_bms, BMS_ST_SYSTEM_BOOST_UP);
                				selex_bms.inrush_lim_time = 0;
                				softstart_fail_cnt = 0;
                			}
                		}
                		else {
                			bms_set_state(&selex_bms, BMS_ST_SOFTSTART);
                			softstart_fail_cnt++;
                			selex_bms.inrush_lim_time = 0;
                		}
                		/* reset when unable to softstart */
                		if (softstart_fail_cnt > 5) {
                			bms_set_state(&selex_bms, BMS_ST_IDLE);
                			softstart_fail_cnt = 0;
                		}
                		shutdown_update_counter = 0;
                break;
    case BMS_ST_SYSTEM_BOOST_UP:
//    	softstart_off_delay_ms+= APP_STATE_MACHINE_UPDATE_TICK_mS;
//    	if(softstart_off_delay_ms>=1100){
//    		inrush_limiter_stop(selex_bms.ic_lim);
//    	}
//    		if(node_id_selex.current_state == 1){
//    			inrush_limiter_stop(selex_bms.ic_lim);
//    			bms_set_state(&selex_bms, BMS_ST_START_AUTHENTICATE);
//    			bms_sdo.bms_node_id = 4;
//    			co_update_sdo_port(&bms_sdo);
//    		}
//#if 0
//    		boost_up_counter+=APP_STATE_MACHINE_UPDATE_TICK_mS;
//    		if(boost_up_counter == BMS_SYSTEM_BOOST_UP_mS){
//    			bms_set_state(&selex_bms, BMS_ST_IDLE);
//    			boost_up_counter=0;
//    		}
//#endif
//	        if(keypad_is_hold(&user_key)){
//	                bms_set_state(&selex_bms,BMS_ST_IDLE);
//	                keypad_reset_hold_state(&user_key);
//	        }

 //add from BMS RA
    	softstart_off_delay_ms += APP_STATE_MACHINE_UPDATE_TICK_mS;
    			if (softstart_off_delay_ms >= 1100) {
    				if (bms_sdo.bms_node_id > 4) {
    					bms_set_state(&selex_bms, BMS_ST_DISCHARGING);
    					selex_bms.switch_state = 3;
    				}
    			}

    			if (bms_get_node_select_state(&selex_bms) == NODE_ID_PIN_ST_SELECT) {
    				bms_set_state(&selex_bms, BMS_ST_START_AUTHENTICATE);
    				bms_sdo.bms_node_id = 4;
    				CO_set_node_id(&selex_bms.co_app, 4);
    				co_update_sdo_port(&bms_sdo);
    			}

    			if (abs(selex_bms.current) < CUR_EXIT_BOOT_UP_CentiA){
    				if (shutdown_update_counter++ > CNT_20_MINUTE_10mS) {
    					bms_set_state(&selex_bms, BMS_ST_IDLE);
    					shutdown_update_counter = 0;
    				}
    			} else {
    				shutdown_update_counter = 0;
    			}

    			if (keypad_is_hold(&user_key)) {
    				bms_set_state(&selex_bms, BMS_ST_IDLE);
    				keypad_reset_hold_state(&user_key);
    			}
    		break;
    case BMS_ST_SHIPMODE:
        	return;
	        break;
	default:
	        break;
	}

	selex_bms.update_state_indicator(&selex_bms);
}

void TIMER_SOC_UPDATE_HANDLE(void) {
	if (TIM_GetITStatus(TIMER_SOC, TIM_IT_Update) == SET)
	{
		selex_bms.soc_previous = selex_bms.spkf_data.xhat[2][0];
		bms_update_soc_spkf(&selex_bms);
		bms_update_soh_awtls(&selex_bms);
	}
	TIM_ClearFlag(TIMER_SOC, TIM_FLAG_Update);
}
#if 0
void HC05_COM_HANLDE(void){
	if(USART_GetITStatus(HC05_COM, USART_IT_RXNE) != RESET)
	{
		uart_rx_data=hc05_receive_characters(&hc05_port);
	}

	if(USART_GetITStatus(HC05_COM,USART_IT_ORE)){
	        USART_ClearFlag(HC05_COM,USART_FLAG_ORE);
	}

}
#endif
