/*
 * internal_flash_hw.c
 *
 *  Created on: Nov 5, 2021
 *      Author: Admin
 */


#include "internal_flash_hw.h"

uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t number_of_page = 0x00;
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;

int int_flash_erase_data(uint32_t start_page_address, uint32_t end_page_address){
	  /* Unlock the Flash to enable the flash control register access *************/
	  FLASH_Unlock();

	/* Erase the user Flash area
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	  /* Clear pending flags (if any) */
	  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

	  /* Define the number of page to be erased */
	  number_of_page = (end_page_address - start_page_address+1) / FLASH_PAGE_SIZE;

	  /* Erase the FLASH pages */
	  for(EraseCounter = 0; (EraseCounter < number_of_page) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	  {
	    if (FLASH_ErasePage(start_page_address + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
	    {
	     /* Error occurred while sector erase.
	         User can add here some code to deal with this error  */
	    	return INTERNAL_FLASH_FAIL;
	    }
	  }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  FLASH_Lock();

	  return 0;
}
int int_flash_write_data(uint32_t start_page_address, uint32_t end_page_address, uint32_t data, uint32_t *index){


	  if(*index == 512){
		  int_flash_erase_data(start_page_address, end_page_address);
		  *index = 0;
	  }
	/* Unlock the Flash to enable the flash control register access *************/
	  FLASH_Unlock();

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	  Address = start_page_address + *index*4;
	  uint32_t end_address = start_page_address + (*index+1)*4;

	  while (Address < end_address)
	  {
	    if (FLASH_ProgramWord(Address, data) == FLASH_COMPLETE)
	    {
	      Address = Address + 4;
	      *index = *index+1;
	    }
	    else
	    {
	      /* Error occurred while writing data in Flash memory.
	         User can add here some code to deal with this error */
	    	return INTERNAL_FLASH_FAIL;
	    }
	  }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  FLASH_Lock();
	  return 0;
}

int int_flash_read_data(uint32_t start_address, uint32_t end_address, uint32_t *data, uint32_t *index){
	  Address = start_address;
	  uint32_t end_address_read = start_address + *index*4;
	  while (Address < end_address_read)
	  {
	    *data = *(__IO uint32_t *)Address;
	    if(*data == 0xFFFFFFFF)
	    {
	    	*data = '\0';
	    }
	    Address = Address + 4;
	    data++;
	  }
	  return 0;
}
