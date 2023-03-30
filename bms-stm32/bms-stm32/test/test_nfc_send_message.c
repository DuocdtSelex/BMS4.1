#include "main.h"
#include "stm32f0xx_conf.h"
#include "board.h"
#include "nfc_hardware.h"
#include "nfc.h"
#include "bq_crc.h"
#include "debug_print.h"
#include "CANopen.h"
#include "config_server.h"
#include "task.h"
#include "scheduler.h"
#include "timer.h"
#include "indicator_leds.h"
#include "stdio.h"
#include "stdlib.h"

#include "../board/stm32_bsp/stm32f0xx_it.h"
#include "../service/bms/bms.h"
#include "CANObject_dictionary.h"
#include "debug_print.h"

#define FAULT_RECOVER_TIMEOUT	12

BMS selex_bms;
static Scheduler system_scheduler;
uint8_t pack_state=0;
volatile int16_t columb_couter=0;
volatile uint8_t is_fault=0;
static uint8_t update_tick=0;

/* Private function prototypes -----------------------------------------------*/
static void app_setup();
static void update_battery();
static void vehicle_sync();
static void scheduler_setup();
static void process_ves_command(const uint8_t* data,const uint16_t len);

/* Private functions ---------------------------------------------------------*/
Task vehicle_sync_task;
Task update_battery_task;

int main(void)
{
    const uint8_t* s_nfc_test_msg="SeleX Motors JSC develop smart, electric vehicles and innovative technologies\n";
    uint8_t* p_rx_data=NULL;
    uint16_t len=0;
    uint32_t sync_timestamp=0;
    BMS* p_bms= &selex_bms;
	app_setup();
    led_app_start_indicator();
    delay_ms(1000);
	scheduler_start(&system_scheduler);
	while (1)
	{
#if 0
        if(system_time_stamp >= (sync_timestamp+137)){
            if(nfc_open_link()==true){
                bms_sync_data(p_bms);
                if(nfc_get_error() ==NFC_LINK_OK){
                    sync_timestamp = system_time_stamp;
                }
            }
        }
#endif

	    scheduler_dispatch_task(&system_scheduler);
	}
}

static void app_setup(){

	int success =0;
	board_init();
    indicator_leds_init();
	nfc_link_setup();
    nfc_register_received_handle(process_ves_command);
	bms_construct(&selex_bms);
    debug_init();
	success=-1;
	while(success!=0){
		success=bms_load_param(&selex_bms);
	}

    bms_set_temp_monitor_source(&selex_bms,EXT_THERMISTOR);
	success=-1;
	while(success!=0){
		success=afe_setup();
	}
	scheduler_init(&system_scheduler);
	scheduler_setup();
}

static void scheduler_setup(){
	task_init(&update_battery_task,0,257,50,TASK_PERIODIC,
			TASK_RUNABLE,update_battery);
	task_init(&vehicle_sync_task,1,157,50,TASK_PERIODIC,
			TASK_RUNABLE,vehicle_sync);

	scheduler_add_task(&system_scheduler,&update_battery_task);
	scheduler_add_task(&system_scheduler,&vehicle_sync_task);
}

static void update_battery(){
	bms_scan_status(&selex_bms);	
	bms_update_cell_voltages(&selex_bms);
    bms_update_pack_voltage(&selex_bms);
#if 0
    print_cell_voltate(&selex_bms);
#endif

	if(selex_bms.is_cc_ready){
		bms_read_current(&selex_bms);
	}

	if(selex_bms.error !=0){
        //bms_sync_object(&vehicle_bms,BATTERY_INDEX,BAT_STATUS_SUB);
		bms_set_state(&selex_bms,BMS_FAULT);
	}
	else if(selex_bms.state==BMS_FAULT){
		selex_bms.fault_recover_timeout++;
		if(selex_bms.fault_recover_timeout >FAULT_RECOVER_TIMEOUT){
			bms_recover_from_fault(&selex_bms);
		}
	}
	else{ 
		bms_set_normal_operation(&selex_bms);
	}	

}

static void vehicle_sync(){
    BMS* p_bms= &selex_bms;
    bms_sync_data(p_bms);
    if(nfc_get_error() != NFC_LINK_OK){
        p_bms->nfc_link_retry++;
    }else{
        p_bms->nfc_link_retry=0;
    }
#if 0 
	bms_process_server_request(p_bms);
#endif

}

static void process_ves_command(const uint8_t* data,const uint16_t len){

}

#ifdef sUSE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
