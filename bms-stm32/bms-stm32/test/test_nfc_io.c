/* Includes ------------------------------------------------------------------*/
#include "../main.h"
#include "../board/board.h"
#include "../component/nfc/nfc.h"
#include "mcu.h"
#include "timer.h"

int main(void)
{
	board_init();
	nfc_link_setup();
	while (1)
	{
		TRF_ENABLE
		SLAVE_SELECT_HIGH
		delay_ms(1000);
		TRF_DISABLE
		SLAVE_SELECT_LOW
		delay_ms(1000);
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
