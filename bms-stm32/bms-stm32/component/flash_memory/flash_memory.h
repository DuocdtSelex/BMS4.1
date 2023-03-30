/*
 * flash_memory.h
 *
 *  Created on: Nov 4, 2021
 *      Author: Admin
 */

#ifndef COMPONENT_FLASH_MEMORY_FLASH_MEMORY_H_
#define COMPONENT_FLASH_MEMORY_FLASH_MEMORY_H_

typedef struct Flash_Memory_t Flash_Memory;
typedef struct Flash_Memory_Interface_t Flash_Memory_Interface;

struct Flash_Memory_t{
	const Flash_Memory_Interface* interface;
};

struct Flash_Memory_Interface_t{
	void (*write_flash_memory)(Flash_Memory* this);
	void (*read_flash_memory)(Flash_Memory* this);
	void (*erase_flash_memory)(Flash_Memory * this);
};

static inline void flash_memory_write_flash(Flash_Memory* p_flash){
	p_flash->interface->write_flash_memory(p_flash);
}

static inline void flash_memory_read_flash(Flash_Memory* p_flash){
	p_flash->interface->read_flash_memory(p_flash);
}
static inline void flash_memory_erase_flash(Flash_Memory* p_flash){
	p_flash->interface->erase_flash_memory(p_flash);
}

#endif /* COMPONENT_FLASH_MEMORY_FLASH_MEMORY_H_ */
