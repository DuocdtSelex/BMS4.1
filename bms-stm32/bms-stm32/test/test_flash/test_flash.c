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
#define TEST 		1

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
#if TEST
	uint8_t flash_ids[3] = { 0 };
	uint8_t flash_status = 0;
	uint8_t test_data[3] = { 1, 2, 3 };
	uint8_t new_data[3] = { 15, 24, 48 };
//	uint8_t test_data = 1;
//	uint8_t new_data = 7;
	app_setup();
	uint32_t add = 0x000106;
	ext_flash_erase_all();
	ext_flash_read_bytes(add, test_data, 3);
	delay_ms(30);
	ext_flash_write_bytes(add, new_data, 3);
#endif


#if !TEST
	uint32_t add;
	/* Write 256 bytes data into address with A0-A7 not zeros - NOT OK*/
	ext_flash_erase_all();
	uint8_t page_data[256] = { 0 };
	uint8_t page_read[256] = { 0 };
	for(int index = 0; index < 256; index++) {
		page_data[index] = index;
	}
	add = 0x000112;
	ext_flash_write_bytes(add, page_data ,256);
	ext_flash_read_bytes(add, page_read, 256);
	delay_ms(30);
	ext_flash_read_bytes(add, page_read, 256);
	delay_ms(30);
#endif
#if !TEST
	/* Wrapping read when reach last address - OK */
	add = 0x1FFFFD;
	uint8_t data_test1[3] ={74, 254, 10};
 	ext_flash_write_bytes(add, data_test1, 3);
 	ext_flash_read_bytes(add, test_data, 3);
 	add = 0x000000;
	uint8_t data_test2[3] ={156, 15, 99};
 	ext_flash_write_bytes(add, data_test2, 3);
	ext_flash_read_bytes(add, test_data, 3);
#endif
#if !TEST
    scheduler_start(&system_scheduler);
#endif
    while (1)
	{
#if TEST
	/* Need two read functions to read back data previously-stored after reset */
	ext_flash_read_bytes(add, test_data, 3);
 	delay_ms(30);
	ext_flash_read_bytes(add, test_data, 3);
	delay_ms(30);
#endif

#if !TEST
    	add = 0x1FFFFF;
    	ext_flash_read_bytes(add, test_data, 3);
    	delay_ms(30);
#endif
#if !TEST
    	/* Erase test - OK*/
    	uint8_t data_test2[3] = {137, 243, 39};
    	add = 0x001000;
    	ext_flash_write_bytes(add, data_test2, 3);
    	/* Erase block 0 4k */
    	ext_flash_erase_block(0, BLOCK_4K_SIZE);
    	ext_flash_read_bytes(add, test_data,3);
    	delay_ms(30);
    	ext_flash_read_bytes(add, test_data, 3);
    	/* Erase block 0 32k */
    	ext_flash_erase_block(0, BLOCK_32K_SIZE);
    	ext_flash_read_bytes(add, test_data,3);
    	delay_ms(30);
    	ext_flash_read_bytes(add, test_data, 3);
    	delay_ms(30);

#endif

#if !TEST
    	/* Write new_data with start address is 0x000106 - NOT OK*/
    	add = 0x000105;
        ext_flash_read_bytes(add, test_data, 3);
    	add = 0x000106;
        ext_flash_read_bytes(add, test_data, 3);
    	add = 0x000107;
        ext_flash_read_bytes(add, test_data, 3);
    	add = 0x000108;
        ext_flash_read_bytes(add, test_data, 3);
    	add = 0x000109;
        ext_flash_read_bytes(add, test_data, 3);
    	add = 0x00010A;
        ext_flash_read_bytes(add, test_data, 3);
        delay_ms(30);

#endif


#if !TEST
		ext_flash_erase_block(0, BLOCK_4K_SIZE);
		delay_ms(200);
		ext_flash_read_bytes(0, test_data, 3);
		delay_ms(30);
		ext_flash_write_bytes(0, new_data, 3);
		delay_ms(30);
		ext_flash_read_bytes(0, test_data, 3);
		delay_ms(30);

#endif
#if !TEST
     scheduler_dispatch_task(&system_scheduler);
#endif
	}
}

static void app_setup(){

	int success = 0;
	board_init();
	indicator_leds_init();
#if !TEST
	bms_reset_i2c_bus();
	bms_exit_ship_mode();
	nfc_link_setup();
	bms_construct(&selex_bms);
	debug_init();
#endif
	ext_flash_init();
#if !TEST
	register_extint_channel(AFE_INTERRUPT_CHANNEL,  bms_afe_alert);
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
#endif

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
