/**
 * @ingroup		SystemAPI
 * @brief		Support for radio communication
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz, Hans-Peter Heitzmann
 * 
 * This is a modified file that was provided by Texas Instruments. Details below:
 */



//----------------------------------------------------------------------------
//  Description:  This file contains functions that allow the MSP430 device to 
//  access the SPI interface of the CC1100/CC2500.  There are multiple 
//  instances of each function; the one to be compiled is selected by the 
//  system variable CC_RF_SER_INTF, defined in "CC_hardware_board.h".
//
//  MSP430/CC1100-2500 Interface Code Library v1.0
//
//  K. Quiring
//  Texas Instruments, Inc.
//  July 2006
//  IAR Embedded Workbench v3.41
//
//-----------------------------------------------------------------------------
//
//	H. P. Heitzmann
//	Freie Universität Berlin
//	Institut für Informatik
//	CST (Computer Systems & Telematics) 
//
//----------------------------------------------------------------------------

#include "../../Drivers/Device.h"
#include "../USART.h"

#include "cc1100_spi.h"

//----------------------------------------------------------------------------
//  void CC_SPISetup(void)
//
//  DESCRIPTION:
//  Configures the assigned interface to function as a SPI port and
//  initializes it.
//----------------------------------------------------------------------------
//  void CC_SPIWriteReg(char addr, char value)
//
//  DESCRIPTION:
//  Writes "value" to a single configuration register at address "addr".
//----------------------------------------------------------------------------
//  void CC_SPIWriteBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Writes values to multiple configuration registers, the first register being
//  at address "addr".  First data byte is at "buffer", and both addr and
//  buffer are incremented sequentially (within the CCxxxx and MSP430,
//  respectively) until "count" writes have been performed.
//----------------------------------------------------------------------------
//  char CC_SPIReadReg(char addr)
//
//  DESCRIPTION:
//  Reads a single configuration register at address "addr" and returns the
//  value read.
//----------------------------------------------------------------------------
//  void CC_SPIReadBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Reads multiple configuration registers, the first register being at address
//  "addr".  Values read are deposited sequentially starting at address
//  "buffer", until "count" registers have been read.
//----------------------------------------------------------------------------
//  char CC_SPIReadStatus(char addr)
//
//  DESCRIPTION:
//  Special read function for reading status registers.  Reads status register
//  at register "addr" and returns the value read.
//----------------------------------------------------------------------------
//  void CC_SPIStrobe(char strobe)
//
//  DESCRIPTION:
//  Special write function for writing to command strobe registers.  Writes
//  to the strobe at address "addr".
//----------------------------------------------------------------------------
//	void CC_Wait(unsigned int cycles)
//
//	DESCRIPTION:
//	Delay function. # of CPU cycles delayed is similar to "cycles". Specifically,
//	it's ((cycles-15) % 6) + 15.  Not exact, but gives a sense of the real-time
//	delay.  Also, if MCLK ~1MHz, "cycles" is similar to # of useconds delayed.
//----------------------------------------------------------------------------

void CC_Wait(unsigned int cycles)
{
	while(cycles>15)                // 15 cycles consumed by overhead
    cycles -= 6;                    // 6 cycles consumed each iteration
}

void CC_SPI_Activate() {
	SPI_SEL &= ~SPI_MISO_PIN;							// MISO pin in GPIO mode
	SPI_DIR &= ~SPI_MISO_PIN;							// MISO pin as input
	
	CC_SELECT;											// select CC1100
	
	while (SPI_IN & SPI_MISO_PIN);						// wait for CC1100 to activate
	
	SPI_SEL |= SPI_MISO_PIN;							// Set MISO pin to SPI mode
}


void CC_SPIWriteReg(uint8_t addr, uint8_t value) {
	CC_SPI_Activate();

		USART0_SPI_Lock(0x04, 0x00, 0x00);
		
			USART0_SPI_Comm(addr);				// Send address
			USART0_SPI_Comm(value);				// Send value
			
		CC_DESELECT;

		USART0_SPI_Release();
}

void CC_SPIWriteBurstReg(uint8_t addr,const uint8_t *buffer, uint8_t count) {
	CC_SPI_Activate();

		USART0_SPI_Lock(0x04, 0x00, 0x00);
		
			USART0_SPI_Comm(addr | CCxxx0_WRITE_BURST);		// Send address

			uint8_t i;
			for ( i = 0; i < count; i++ )
			{
				USART0_SPI_Comm(buffer[i]);		// Send address
			}
			
		CC_DESELECT;

		USART0_SPI_Release();
}

uint8_t CC_SPIReadReg(uint8_t addr) {
	uint8_t x;

	USART0_SPI_Lock(0x04, 0x00, 0x00);

	CC_SPI_Activate();
		
		USART0_SPI_Comm(addr | CCxxx0_READ_SINGLE);		// Send address

		x = USART0_SPI_Comm(0x00);
			
	CC_DESELECT;

	USART0_SPI_Release();

	return x;
}

void CC_SPIReadBurstReg(uint8_t addr, uint8_t *bufferOut, uint8_t count) {
	unsigned int i;

	CC_SPI_Activate();

		USART0_SPI_Lock(0x04, 0x00, 0x00);
		
			USART0_SPI_Comm(addr | CCxxx0_READ_BURST);		// Send address
	
			for ( i = 0; i < count; i++ ) {
				bufferOut[i] = USART0_SPI_Comm(0x00);
			}
		
	CC_DESELECT;

	USART0_SPI_Release();
}

// For status/strobe addresses, the BURST bit selects between status registers
// and command strobes.

uint8_t CC_SPIReadStatus(uint8_t addr) {
	uint8_t x;

	CC_SPI_Activate();

		USART0_SPI_Lock(0x04, 0x00, 0x00);
		
			USART0_SPI_Comm(addr | CCxxx0_READ_BURST);		// Send address
			x = USART0_SPI_Comm(0x00); 

		CC_DESELECT;

		USART0_SPI_Release();

	return x;
}

uint8_t CC_SPIStrobe(uint8_t strobe) {
	USART0_SPI_Lock(0x04, 0x00, 0x00);

	CC_SPI_Activate();

		uint8_t chipStatusByte = USART0_SPI_Comm(strobe);							// Send strobe
			
	CC_DESELECT;
	
	USART0_SPI_Release();
	
	return chipStatusByte;
}

void CC_PowerupResetCCxxxx() {
	CC_CSn_OUT |= CC_CSn_PIN;
	CC_Wait(60);
	CC_CSn_OUT &= ~CC_CSn_PIN;
	CC_Wait(60);
	CC_CSn_OUT |= CC_CSn_PIN;
	CC_Wait(90);

	USART0_SPI_Lock(0x04, 0x00, 0x00);
		
	CC_SPI_Activate();

		USART0_SPI_Comm(CCxxx0_SRES);						// Send strobe
				
	CC_SPI_Activate();


	CC_DESELECT;

	USART0_SPI_Release();
}
