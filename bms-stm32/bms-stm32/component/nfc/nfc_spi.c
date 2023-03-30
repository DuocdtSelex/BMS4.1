//*****************************************************************************
//
// nfc_spi.c - SPI Configuration and transmission APIs
//
// Copyright (c) 2015 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************
#include "nfc_spi.h"
#include "board.h"
#include "nfc_hardware.h"
#include "stm32f0xx_spi.h"

#define NFC_SPI_TIMEOUT			7000

static uint8_t nfc_spi_error=0;

//===============================================================

// Direct Mode disabled
uint8_t g_direct_mode = 0;
uint32_t g_nfc_spi_timeout=0;

static bool nfc_wait_for_spi_flag(uint16_t flag,uint32_t status);

void SPI_SendByte(uint8_t data)
{
	uint8_t dummy_read=0;

	if(!nfc_wait_for_spi_flag(SPI_I2S_FLAG_TXE,RESET)){
		nfc_spi_error=NFC_SPI_FAIL;
		return;
	};
        
    /* Send the byte */
    SPI_SendData8(NFC_SPI_DEVICE,data);
    /* Wait to receive a byte*/
    if(!nfc_wait_for_spi_flag(SPI_I2S_FLAG_RXNE,RESET)){
		nfc_spi_error=NFC_SPI_FAIL;
		return;
	};
    /* Return the byte read from the SPI bus */ 
    dummy_read= SPI_ReceiveData8(NFC_SPI_DEVICE);
}

uint8_t SPI_ReceiveByte(){
	/* Wait until the transmit buffer is empty */
    if(!nfc_wait_for_spi_flag(SPI_I2S_FLAG_TXE,RESET)){
		nfc_spi_error=NFC_SPI_FAIL;
		return 0;
	}
    /* Send the byte */
    SPI_SendData8(NFC_SPI_DEVICE, NFC_SPI_DUMMY_BYTE);
    
    /* Wait until a data is received */
    if(!nfc_wait_for_spi_flag(SPI_I2S_FLAG_RXNE,RESET)){
		nfc_spi_error=NFC_SPI_FAIL;
		return 0;
	};
    
    /* Return the shifted data */
    return SPI_ReceiveData8(NFC_SPI_DEVICE);
}

void SPI_directCommand(uint8_t command)
{
//	volatile uint8_t x;

	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// set Address/Command Word Bit Distribution to command
	command = (0x80 | command);					// command
	command = (0x9f & command);					// command code

	SPI_SendByte( command );

	SLAVE_SELECT_HIGH; 						//Stop SPI Mode
}

//===============================================================

void SPI_rawWrite(uint8_t *pbuf, uint8_t length)
{	
//	volatile uint8_t x;

	SLAVE_SELECT_LOW; 						//Start SPI Mode
	while(length-- > 0)	{
	    SPI_SendByte( *pbuf++ );
    }

	if(g_direct_mode == 0x00)
	{
		SLAVE_SELECT_HIGH; 						// Stop SPI Mode
	}
}

void SPI_readCont(uint8_t *pbuf, uint8_t reg, uint8_t length)
{	
//	volatile int8_t x;

	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Word Bit Distribution
	// address, write, single (fist 3 bits = 0)
	reg = (0x1f & reg);					// register address
	reg = (0x60 | reg); 					// address, read, continuous
	SPI_SendByte( reg);

	while(length > 0)
	{
		*pbuf = SPI_ReceiveByte();
		pbuf++;
		length--;
	}

	SLAVE_SELECT_HIGH; 						// Stop SPI Mode

}

void SPI_readSingle(uint8_t *pbuf, uint8_t reg)
{			
//	volatile uint8_t x;

	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Word Bit Distribution
	// address, write, single (fist 3 bits = 0)
	reg = (0x1f & reg);				// register address
	reg = (0x40 | reg); 			// address, read, single
	

	SPI_SendByte(reg);  			// Previous data to TX, RX

	*pbuf = SPI_ReceiveByte();

	if(g_direct_mode == 0x00)
	{
		SLAVE_SELECT_HIGH; 						// Stop SPI Mode
	}
}

void SPI_setup(void)
{	
}

void SPI_writeSingle(uint8_t data, uint8_t reg)
{
//	volatile int8_t x;

	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Word Bit Distribution
	// address, write, single (fist 3 bits = 0)
	reg = (0x1f &reg);				// register address

	SPI_SendByte(reg);
	SPI_SendByte(data);

	if(g_direct_mode == 0x00)
	{
		SLAVE_SELECT_HIGH; 						// Stop SPI Mode
	}
}

void SPI_writeCont(uint8_t *pbuf, uint8_t reg, uint8_t length)
{
//	volatile int8_t x;

	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Word Bit Distribution
	// address, write, single (fist 3 bits = 0)
	reg = (0x20 | reg); 				// address, write, continuous
	reg = (0x3f & reg);					// register address

	SPI_SendByte(reg);


	while(length > 0)
	{
	    SPI_SendByte(*pbuf);

		pbuf++;
		length--;

	}

	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}

void SPI_writePacket(uint8_t *pbuf, uint8_t crc_bit, uint8_t total_length, uint8_t payload_length, bool header_enable, uint8_t broken_bits, bool bNFCAHeader)
{
	uint8_t ui8LenLowerNibble;
	uint8_t ui8LenHigherNibble;
	uint16_t ui16TotalLength;

	ui16TotalLength = total_length;

	// Add one to the total length sent to the TRF7970A
	if(bNFCAHeader == true)
	{
		ui16TotalLength++;
	}

	ui8LenLowerNibble = (ui16TotalLength & 0x0F) << 4;
	ui8LenHigherNibble = (uint8_t) ((ui16TotalLength & 0x0FF0) >> 4);

	SLAVE_SELECT_LOW; 						// Start SPI Mode

	if(header_enable == true)
	{
		// RESET FIFO
		SPI_SendByte(0x8F);

		// CRC COMMAND
		SPI_SendByte( 0x90 | (crc_bit & 0x01));	// Previous data to TX, RX

		// WRITE TO LENGTH REG
		SPI_SendByte(0x3D);

		if(broken_bits == 0x00)
		{
			// LENGTH HIGH Nibble
			SPI_SendByte(ui8LenHigherNibble);	// Previous data to TX, RX

			// LENGTH LOW Nibble
			SPI_SendByte(ui8LenLowerNibble);	// Previous data to TX, RX
		}
		else
		{
			// Broken Bit
			SPI_SendByte(0x00);
			SPI_SendByte((broken_bits << 0x01) | 0x01);
		}

		if(bNFCAHeader == true)
		{
			// Type A Header Byte
			SPI_SendByte(0xF0);
		}
	}
	else
	{
		// Always do a continious write to the FIFO (even if there is only one byte)
		SPI_SendByte(0x3F);
	}

	while(payload_length > 0)
	{
	    SPI_SendByte(*pbuf);					// Previous data to TX, RX
		pbuf++;
		payload_length--;
	}
	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}

static bool nfc_wait_for_spi_flag(uint16_t flag,uint32_t status){

	g_nfc_spi_timeout=0;
    while(SPI_I2S_GetFlagStatus(NFC_SPI_DEVICE,flag)==status){
        if(g_nfc_spi_timeout++ > NFC_SPI_TIMEOUT) return false;
    }
    return true;
}

