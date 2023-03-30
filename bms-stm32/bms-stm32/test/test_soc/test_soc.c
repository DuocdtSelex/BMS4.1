#include "main.h"
#include "stm32f0xx_conf.h"
#include "board.h"
#include "nfc_hardware.h"
#include "nfc.h"
#include "bq_crc.h"
#include "component/adc/adc.h"
#include "debug_print.h"
#include "CANopen.h"
#include "config_server.h"
#include "task.h"
#include "scheduler.h"
#include "timer.h"
#include "indicator_leds.h"
#include "stdio.h"
#include "stdlib.h"

#include "../../board/stm32_bsp/stm32f0xx_it.h"
#include "../../service/bms/bms.h"
#include "CANObject_dictionary.h"
#include "debug_print.h"
#include "ext_flash.h"
#include "driver/it-flash/flash.h"
#include "component/adc/adc.h"
#include "component/afe/bq_crc.h"
#include "component/can/can.h"


#define FAULT_RECOVER_TIMEOUT	30

BMS selex_bms;

static Scheduler system_scheduler;
uint8_t pack_state=0;
volatile int16_t columb_couter=0;
volatile uint8_t is_fault=0;

//static uint8_t update_tick=0;
uint8_t ext_flash_id[3]={0};

/* Private function prototypes -----------------------------------------------*/
static void app_setup();
static void update_battery();
static void vehicle_sync();
static void scheduler_setup();

/* Private functions ---------------------------------------------------------*/
Task vehicle_sync_task;
Task update_battery_task;

int main(void)
{
	delay_ms(1000);
	app_setup();
    led_app_start_indicator();
    delay_ms(1000);
    scheduler_start(&system_scheduler);
    while (1)
	{
     scheduler_dispatch_task(&system_scheduler);
	}
}

static void app_setup(){

	int success = 0;
	board_init();
	bms_reset_afe1_i2c();
	bms_exit_ship_mode();
	indicator_leds_init();
	nfc_link_setup();
	bms_construct(&selex_bms);
	debug_init();
	ext_flash_init();
	register_extint_channel(AFE_INTERRUPT_CHANNEL,  bms_afe1_alert);
	can_init();
	adc_init();
	success = -1;
	while (success != 0) {
		success = bms_load_param(&selex_bms);
	}
	bms_set_temp_monitor_source(&selex_bms, EXT_THERMISTOR);
	success = -1;
	while (success != 0) {
		success = afe_setup();
	}
	delay_ms(1000);
	success = -1;
	while (success != 0) {
		success = afe_setup();
	}
	scheduler_init(&system_scheduler, &system_time_stamp);
	scheduler_setup();

}

static void scheduler_setup(){
	task_init(&update_battery_task,0,250,50,TASK_PERIODIC,
			TASK_RUNABLE,update_battery);
	task_init(&vehicle_sync_task,1,250,50,TASK_PERIODIC,
			TASK_RUNABLE,vehicle_sync);

	scheduler_add_task(&system_scheduler,&update_battery_task);
	scheduler_add_task(&system_scheduler,&vehicle_sync_task);
}

static void update_battery() {
	bms_scan_status(&selex_bms);
	bms_update_cell_voltages(&selex_bms);
	bms_update_pack_voltage(&selex_bms);
	bms_caculate_coulomb_counter(&selex_bms);
	if (selex_bms.is_cc_ready) {
		bms_read_current(&selex_bms);
		bms_caculate_coulomb_counter(&selex_bms);
	}
	if (selex_bms.error != 0) {
		bms_set_state(&selex_bms, BMS_FAULT);
	}
	if (selex_bms.state == BMS_FAULT) {
		if (selex_bms.fault_recover_timeout++ > FAULT_RECOVER_TIMEOUT) {
			if (bms_check_conditions(&selex_bms)) {
				bms_recover_from_fault(&selex_bms);
				selex_bms.error = 0;
				bms_set_state(&selex_bms,BMS_STANDBY);
			}
			selex_bms.fault_recover_timeout = 0;
		} else {
			bms_reset_sw_btn_object();
		}
	} else {
		bms_set_normal_operation(&selex_bms);
	}
}


static void vehicle_sync(){
    BMS* p_bms= &selex_bms;
    bms_sync_data(p_bms);
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
