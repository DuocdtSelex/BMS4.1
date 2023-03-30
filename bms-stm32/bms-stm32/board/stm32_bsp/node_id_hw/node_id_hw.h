/*
 * node_id_hw.h
 *
 *  Created on: Apr 2, 2021
 *      Author: Admin
 */

#ifndef BOARD_STM32_BSP_NODE_ID_HW_NODE_ID_HW_H_
#define BOARD_STM32_BSP_NODE_ID_HW_NODE_ID_HW_H_

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"

#define NODE_ID_GPIO_PIN            GPIO_Pin_8
#define NODE_ID_GPIO_PORT           GPIOA
#define NODE_ID_GPIO_CLK            RCC_AHBPeriph_GPIOA

typedef enum NODE_ID_PIN_STATE_t{
        NODE_ID_PIN_ST_DESELECT                   =0,
        NODE_ID_PIN_ST_SELECT                     =1
}NODE_ID_PIN_STATE;

void node_id_hw_init(void);
uint8_t node_id_read_input(void);
#endif /* BOARD_STM32_BSP_NODE_ID_HW_NODE_ID_HW_H_ */
