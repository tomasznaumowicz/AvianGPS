/**
 * @ingroup		SystemAPI
 * @brief		Support for radio communication
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz, Hans-Peter Heitzmann
 * 
 * This is a modified file that was provided by Texas Instruments. Details below:
 */


//----------------------------------------------------------------------------
//  Description:  Header file for cc1100_spi.c
//
//  MSP430/CC1100-2500 Interface Code Library v1.0
//
//  K. Quiring
//  Texas Instruments, Inc.
//  July 2006
//  IAR Embedded Workbench v3.41
//----------------------------------------------------------------------------
//
//	Port to avian
//	Code Composer Essentials for MSP430 version 2.0.0.21
//	
//	H. P. Heitzmann
//	Freie Universität Berlin
//	Institut für Informatik
//	CST (Computer Systems & Telematics) 
//
//----------------------------------------------------------------------------

#ifndef CC1100_SPI_H_
#define CC1100_SPI_H_

#include "../BitOperations.h"
#include "cc1100.h"

#define CC_SELECT				{	CLEAR(CC_CSn_OUT, CC_CSn_PIN);	}
#define CC_DESELECT				{	SET(CC_CSn_OUT, CC_CSn_PIN);	}


void CC_PowerupResetCCxxxx();
void CC_SPIWriteReg(uint8_t addr, uint8_t value);
void CC_SPIWriteBurstReg(uint8_t addr, const uint8_t *buffer, uint8_t count);
uint8_t CC_SPIReadReg(uint8_t addr);
void CC_SPIReadBurstReg(uint8_t addr, uint8_t *bufferOut, uint8_t count);
uint8_t CC_SPIReadStatus(uint8_t addr);
uint8_t CC_SPIStrobe(uint8_t strobe);
void CC_Wait(unsigned int);

#endif /*CC1100_SPI_H_*/
