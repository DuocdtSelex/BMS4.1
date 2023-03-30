/* Includes ------------------------------------------------------------------*/
#include "../main.h"
#include "board.h"
#include "bq76940.h"

int main(void)
{
	board_init();
	int cell_monitor_error=0;
	cell_monitor_error=bq76940_init();
//	while(cell_monitor_error !=0);
	uint8_t percent=0;
	while (1)
	{
		set_percent_indicators(percent);
		percent++;
		if(percent > 125){
			percent=0;
		}
		//time_out=90000;
		checkSTAT();
		//while(time_out--);
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
