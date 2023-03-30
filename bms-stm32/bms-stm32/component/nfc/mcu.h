//*****************************************************************************
//
// mcu.h - MCU Configuration and host interface APIs definitions
//
// Copyright (c) 2015 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************
#ifndef __MCU_H__
#define __MCU_H__

// Include Header Files
//#include "driverlib.h"
#include <stdint.h>
#include <string.h>
#include "types.h"
#include "nfc_config.h"
#include "board.h"
#include "exint_driver.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "timer.h"

// ******** Definitions ******************************************************//

#define MHZ							1000000
#define MCLK_FREQ						(24*MHZ)
#define SMCLK_FREQ						(12*MHZ)
#define MCLK_MS_COUNT						(MCLK_FREQ/1000)
#define UCS_MCLK_DESIRED_FREQUENCY_IN_KHZ   			12000
#define UCS_MCLK_FLLREF_RATIO   				366

 //=====MCU constants=============================================
//#define MOD_DIR_OUT		P2DIR |= BIT1
//#define MOD_OFF			P2OUT &= ~BIT1
//#define MOD_ON			P2OUT |= BIT1

#define	TRF_ENABLE		GPIO_SetBits(NFC_ENABLE_PORT,NFC_ENABLE_PIN);
#define TRF_DISABLE		GPIO_ResetBits(NFC_ENABLE_PORT,NFC_ENABLE_PIN);

#define IRQ_INT_ON		trf_enable_interrupt();

#define IRQ_INT_OFF		trf_disable_interrupt();	
//#define IRQ_EDGE_SET    	GPIO_interruptEdgeSelect(TRF_IRQ_PORT, TRF_IRQ, GPIO_LOW_TO_HIGH_TRANSITION);
//#define IRQ_CLR			GPIO_clearInterruptFlag(TRF_IRQ_PORT, TRF_IRQ);
#define IRQ_CLR			trf_clear_interrupt_flag();	

//#define SLAVE_SELECT_PORT_SET	GPIO_setAsOutputPin(SPI_SS_PORT, SPI_SS);
//#define SLAVE_SELECT_HIGH       SPI_SS_POUT |= SPI_SS;
//#define SLAVE_SELECT_LOW        SPI_SS_POUT &= ~(SPI_SS);
#define SLAVE_SELECT_HIGH	GPIO_SetBits(NFC_SPI_CS_PORT,NFC_SPI_CS_PIN);
#define SLAVE_SELECT_LOW	GPIO_ResetBits(NFC_SPI_CS_PORT,NFC_SPI_CS_PIN);

//#define IRQ_IS_SET()           	(TRF_IRQ_PIN & TRF_IRQ) // GPIO_getInputPinValue(TRF_IRQ_PORT, TRF_IRQ)
#define IRQ_IS_SET()           	trf_interrupt_is_set()

extern bool g_bSerialConnectionEstablished;
extern volatile uint8_t g_ui8BytesReceived;

void MCU_init();
void MCU_delayMillisecond(uint16_t n_ms);
void MCU_delayMicrosecond(uint16_t n_us);
void MCU_clockInit();
void MCU_portInit();
void MCU_timerInit(uint16_t timeout_ms, uint8_t * timeout_flag);
void MCU_timerDisable();

void trf_enable();
void trf_disable();
void trf_enable_interrupt();
void trf_disable_interrupt();
void trf_clear_interrupt_flag();
void trf_cs_set();
void trf_cs_clear();
bool trf_interrupt_is_set();

extern uint8_t convertNibbleToAscii(uint8_t ui8Nibble);

extern void convertByteToAscii(uint8_t ui8byte, uint8_t pui8Buffer[3]);

extern void convertWordToAscii(uint16_t ui16Word, uint8_t pui8Buffer[5]);

#endif
