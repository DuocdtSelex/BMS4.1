/*
 * flash_memory.c
 *
 *  Created on: Nov 4, 2021
 *      Author: Admin
 */

#include "flash_memory.h"
#include "stdlib.h"
#include "stdio.h"
void flash_memory_set_interface(Flash_Memory* const p_flash,const Flash_Memory_Interface* const p_interface){
	p_flash->interface=p_interface;
}

Flash_Memory* flash_memory_create(void){

	Flash_Memory* this=(Flash_Memory*)malloc(sizeof(Flash_Memory));
	while(this==NULL){

	};
	return this;
}
