/* Includes ------------------------------------------------------------------*/
#include "../main.h"
#include "../service/bms/bms.h"
#include "board.h"
#include "bq_crc.h"
#include "exint_driver.h"
#include "timer.h"
extern unsigned int CellVoltage[CELL_NUM];
uint8_t pack_state=0;
volatile int16_t columb_couter=0;
volatile uint8_t is_fault=0;
void fault_handle(){
	int success=-1;
	while(success!=0){
		success=main_switch_off();
	}
	delay_ms(10000);	
}
void update_bms_status(){
	int success=-1;
	uint8_t state=0;

	read_status_flags(&state);	
	if(state & (CC_READY_BIT)) /* adc for coulumb counter is ready */
	{
		columb_couter= afe_read_current();
	}
	if(is_afe_fault(state)==true){
		delay_ms(200);
		read_status_flags(&state);	
		if(is_afe_fault(state)==true){
			fault_handle();
		}
	}
	state=0xff;
	while(state !=0){
		clear_status_flags(state);
		read_status_flags(&state);
	}
}
int main(void)
{
	uint8_t state;
	uint32_t fault_time_out=0;
	global_interrupt_disable();
	board_init();
	delay_ms(2000);
	//register_extint_channel(AFE_INTERRUPT_CHANNEL,update_bms_status);

	int result=-1;
	while(result != 0){
		result=	afe_setup();
		delay_ms(100);
	}

	result=-1;
	while(result !=0){
		result=enable_adc();
	}

	result =-1;
	while(result != 0){
		result=enalbe_coulump_counter(0);
	}

	afe_stop_balancing();
	set_debug_fault_delay();
	read_status_flags(&state);
	while(state !=0){
		clear_status_flags(0xff);
		read_status_flags(&state);
	}
	delay_ms(100);
	global_interrupt_enable();
	while (1)
	{	
		result=-1;
		while(result !=0){
//			result=main_switch_on();
		}
		if(is_afe_alerting()==true){
			update_bms_status();
		}
	
//		columb_couter=0;
		columb_couter=afe_read_current();
		result = -1;
		while(result != 0){
			result= read_cell_voltage();	
		}
		delay_ms(10);

		/*
		result=-1;
		while(result !=0){
			result=main_switch_off();
		}
		*/
		//read_status_flags(&state);
	}

}

#ifdef  USE_FULL_ASSERT

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
