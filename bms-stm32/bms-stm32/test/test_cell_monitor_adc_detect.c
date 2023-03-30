/* Includes ------------------------------------------------------------------*/
#include "../main.h"
#include "../service/bms/bms.h"
#include "board.h"
#include "bq_crc.h"
#include "exint_driver.h"
extern unsigned int CellVoltage[CELL_NUM];
void update_bms_status(){
	afe_check_status();
}
int main(void)
{
	board_init();
	//register_extint_channel(AFE_INTERRUPT_CHANNEL,update_bms_status);
	int result=-1;
	uint32_t time_out;
	uint16_t current;
	uint8_t balancing_bits[3];
	uint8_t new_balancing_bits[3];
	bool load_present=false;
	while(result != 0){
		result=	afe_setup();
	}
	clear_status_flags(0xFF);	
	result=-1;
	while(result !=0){
		result=enable_adc();
	}
	result =-1;
	while(result != 0){
		result=enalbe_coulump_counter(0);
	}
	afe_stop_balancing();
	result=-1;
	while(result !=0){
		result=turn_off_discharge();
	}

	result=-1;
	while(result !=0){
		result=turn_off_charge();
	}

	uint8_t i=0;
	PACK_OPERATING_STATE pack_state;
	while (1)
	{

		pack_state = get_pack_operating_state();
		if(pack_state == PACK_CHARGING){
			turn_on_discharge();
			turn_on_charge();
			result=-1;
			while(result != 0){
				result= read_cell_voltage();	
			}
			bms_update_balancing();
			afe_active_balancing();
		}
		else if(pack_state==PACK_DISCHARGING){
			turn_on_discharge();
			turn_on_charge();
			afe_stop_balancing();
		}else if(pack_state==PACK_STANDBY){
			turn_off_discharge();
			turn_off_charge();
			result=-1;
			while(result != 0){
				result= read_cell_voltage();	
			}
			bms_update_balancing();
			afe_active_balancing();
		}
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
