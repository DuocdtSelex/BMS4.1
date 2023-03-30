/*
 * node_id_hw.c
 *
 *  Created on: Apr 2, 2021
 *      Author: Admin
 */


#include "node_id_hw.h"

void node_id_hw_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(NODE_ID_GPIO_CLK,ENABLE);
	/*Config GPIO pin: PA8*/
	GPIO_InitStructure.GPIO_Pin = NODE_ID_GPIO_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(NODE_ID_GPIO_PORT, &GPIO_InitStructure);

}
uint8_t node_id_read_input(void){
  return GPIO_ReadInputDataBit(NODE_ID_GPIO_PORT, NODE_ID_GPIO_PIN);
}
