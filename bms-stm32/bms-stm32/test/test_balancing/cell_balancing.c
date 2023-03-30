#include "stm32f0xx.h"
#include "../../component/afe/bq_crc.h"

#define BQ_I2C_ADDR (0x08<<1)

void I2C_init(void){

	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	/* Configure the I2C clock source. The clock is derived from the HSI */
	RCC_AHBPeriphClockCmd(GPIO_Pin_6 | GPIO_Pin_7, ENABLE);
	RCC_AHBPeriphClockCmd(GPIO_Pin_10 | GPIO_Pin_11, ENABLE);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1);
	//Configure pins: SCL and SDA ------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	//I2C_InitStructure.I2C_Timing = 0xA0120227;
	//I2C_InitStructure.I2C_Timing =0x00201D2B; /* for HSI clock source */
	I2C_InitStructure.I2C_Timing = 0x10805E89; /* for SYSCLK 48MHz */
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);
}
int main(void)
{

	unsigned int success = 0;
	unsigned char data= 0b00000000;
	unsigned char data_w = 0b01000011;
	unsigned char data1= 0b00000000;
	unsigned char data2= 0b11111111;
	I2C_init();

	while(1){


		success = I2CReadRegisterByteWithCRC_global(I2C2, BQ_I2C_ADDR, 0x00, &data);
		if (success != 0) return success;
		success = I2CReadRegisterByteWithCRC_global(I2C1, BQ_I2C_ADDR, 0x00, &data);
		if (success != 0) return success;
		for (int j = 1; j < 3; j++) {
			for (int i = 0; i < 5; i++) {
				success = I2CWriteRegisterByteWithCRC_global(I2C1, BQ_I2C_ADDR,
						j, 1 << i);
				if (success != 0)
					return success;
				delay_ms(100);
				success = I2CWriteRegisterByteWithCRC_global( I2C2, BQ_I2C_ADDR,
						j, 1 << i);
				if (success != 0)
					return success;
				delay_ms(100);
			}
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

