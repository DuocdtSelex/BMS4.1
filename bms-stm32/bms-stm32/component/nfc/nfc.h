#ifndef _NFC_H_
#define _NFC_H_

#include "nfc.h"
#include "trf79x0.h"
#include "board.h"
#include "mcu.h"
#include "nfc_controller.h"
#include "nfc_config.h"


#define MB_FLAG_BIT         (1<<7)
#define ME_FLAG_BIT         (1<<6)
#define CF_FLAG_BIT         (1<<5)
#define SR_FLAG_BIT         (1<<4)
#define IL_FLAG_BIT         (1<<3)

typedef enum NFC_LINK_ERROR{
	NFC_LINK_OK		        =0x00,
	NFC_RESPONSE_TIMEOUT	=0x01,
	NFC_ACTIVE_TIMEOUT	    =0x02,
	NFC_TRANSMIT_TIMEOUT	=0x03
}NFC_LINK_ERROR;

typedef enum NFC_LINK_STATUS{
	NFC_READY		        =0x00,
	NFC_LINK_NOT_READY	    =0x01
}NFC_LINK_STATUS;

typedef void (*NFC_New_Package_Handle)(const uint8_t* data,const uint16_t len);


void nfc_link_setup();
bool nfc_open_link();
void nfc_register_received_handle(NFC_New_Package_Handle handle);
uint8_t* nfc_get_received_buffer();
uint16_t nfc_get_received_buffer_len();
void nfc_wait_for_response();
void nfc_flush_rx_buffer();
void nfc_sends(const uint8_t* data,const uint16_t len);
void nfc_sync_data(void);
NFC_LINK_ERROR nfc_get_error();
#endif
