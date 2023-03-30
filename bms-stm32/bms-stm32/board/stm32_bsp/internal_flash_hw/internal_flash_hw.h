/*
 * internal_flash_hw.h
 *
 *  Created on: Nov 5, 2021
 *      Author: Admin
 */

#ifndef BOARD_STM32_BSP_INTERNAL_FLASH_HW_INTERNAL_FLASH_HW_H_
#define BOARD_STM32_BSP_INTERNAL_FLASH_HW_INTERNAL_FLASH_HW_H_

#include "stm32f0xx.h"
#include "stm32f0xx_flash.h"
#include "stdint.h"

/* Private define ------------------------------------------------------------*/
#ifdef STM32F072
 #define FLASH_PAGE_SIZE         ((uint32_t)0x00000800)   /* FLASH Page Size */
 #define FLASH_USER_START_ADDR   ((uint32_t)0x0801F800)   /* Start @ of user Flash area */
 #define FLASH_USER_END_ADDR     ((uint32_t)0x0801FFFF)   /* End @ of user Flash area */
#elif defined (STM32F091)
 #define FLASH_PAGE_SIZE         ((uint32_t)0x00000800)   /* FLASH Page Size */
 #define FLASH_USER_START_ADDR   ((uint32_t)0x08009000)   /* Start @ of user Flash area */
 #define FLASH_USER_END_ADDR     ((uint32_t)0x08040000)   /* End @ of user Flash area */
#else
 #define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   /* FLASH Page Size */
 #define FLASH_USER_START_ADDR   ((uint32_t)0x08006000)   /* Start @ of user Flash area */
 #define FLASH_USER_END_ADDR     ((uint32_t)0x08007000)   /* End @ of user Flash area */
#endif /* STM32F072 */

#define INTERNAL_FLASH_FAIL			-1

int int_flash_erase_data(uint32_t start_page_address, uint32_t end_page_address);
int int_flash_write_data(uint32_t start_page_address, uint32_t end_page_address, uint32_t data, uint32_t *index);
int int_flash_read_data(uint32_t start_address, uint32_t end_address, uint32_t *data, uint32_t *index);

#endif /* BOARD_STM32_BSP_INTERNAL_FLASH_HW_INTERNAL_FLASH_HW_H_ */
