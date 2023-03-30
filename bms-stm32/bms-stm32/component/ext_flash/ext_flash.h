#ifndef EXT_FLASH_H_
#define EXT_FLASH_H_
#include "board.h"

#define EXT_FLASH_PAGE_SIZE             (1<<8) /* in byte */
#define EXT_FLASH_SECTOR_SIZE			(1<<16)
#define EXT_FLASH_CAPACITY              (1<<20)  //1048576/* in byte =1Mbytes */

#define READ_MANUFACTURE_ID             0x9F
#define EXT_FLASH_DUMMY_BYTE		    0x00

#define READ_STATUS_1_CMD		        0x05
#define READ_STATUS_2_CMD		        0x35
#define WRITE_STATUS_CMD		        0x01
#define WRITE_ENABLE_CMD		        0x06
#define WRITE_DISABLE_CMD		        0x04

#define CHIP_ERASE_CMD                  0x60
#define BLOCK_4K_ERASE_CMD              0x20
#define BLOCK_32K_ERASE_CMD             0x52
#define BLOCK_64K_ERASE_CMD             0xD8

#define BYTE_PAGE_WRITE_CMD             0x02
#define READ_ARRAY_CMD                  0x03

#define STATUS_BUSY_BIT                 (1<<0)
#define STATUS_WEL_BIT                  (1<<1)
#define STATUS_BP0_BIT                  (1<<2)
#define STATUS_BP1_BIT                  (1<<3)
#define STATUS_BP2_BIT                  (1<<4)
#define STATUS_TB_BIT                   (1<<5)
#define STATUS_SEC_BIT                  (1<<6)
#define STATUS_SRP0_BIT                 (1<<7)


#define PAGE_ADDRESS(x) ((uint32_t)(x) <<8)
#define BLOCK_4K_ADDRESS(x)  ((uint32_t)x <<12))
#define BLOCK_32K_ADDRESS(x)  ((uint32_t)x <<15))
#define BLOCK_64K_ADDRESS(x)  ((uint32_t)x <<16))

typedef enum BLOCK_SIZE{
    BLOCK_4K_SIZE       =0x00,
    BLOCK_32K_SIZE      =0x01,
    BLOCK_64K_SIZE      =0x02,
}BLOCK_SIZE;


void ext_flash_init();

int ext_flash_read_bytes(const uint32_t address,uint8_t* p_data,const uint16_t len);
int ext_flash_write_bytess(const uint32_t address, const uint8_t* p_data,
		const uint16_t len);
int ext_flash_read_bytess(const uint32_t address,uint8_t* p_data,
        const uint16_t len);
int ext_flash_read_page(const uint32_t page,uint8_t* p_data);
int ext_flash_read_all(uint8_t* p_data);

int ext_flash_write_bytes(const uint32_t address,const uint8_t* p_data,const uint16_t len);
int ext_flash_write_page(const uint32_t page,const uint8_t* p_data);

int ext_flash_erase_block(const uint32_t block,const BLOCK_SIZE block_size);
int ext_flash_erase_all();

int ext_flash_write_enable();
int ext_flash_write_disable();

int ext_flash_read_status1(uint8_t* p_data);
int ext_flash_read_status2(uint8_t* p_data);
int ext_flash_read_status(uint16_t* p_data);
int ext_flash_write_status(const uint8_t data);

int ext_flash_read_manufacture_id(uint8_t* p_data);
int ext_flash_read_serial_number(uint8_t* p_data);
int ext_flash_set_deep_power_down();
int ext_flash_resume_from_deep_power_down();

int ext_flash_read_reg(const uint8_t reg,uint8_t* p_data);
int ext_flash_write_reg(const uint8_t reg,const uint8_t data);
int ext_flash_write_command(const uint8_t command);

#endif
