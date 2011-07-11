/**
 * @brief		Driver: Flash Memory
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#include <msp430x16x.h>
#include <string.h>

#include "Device.h"

#include "../System/BitOperations.h"
#include "../System/Logging.h"

#include "../System/USART.API.h"
#include "../System/Watchdog.API.h"

#include "M25P80.h"
 
#define WREN		0x06	//Write Enable
#define WRDI		0x04	//Write Disable
#define RDID		0x9F	//Read Identification
#define RDSR		0x05	//Read Status Register
#define WRSR		0x01	//Write Status Register 
#define READ		0x03	//Read Data Bytes 
#define FAST_READ	0x0B	//Read Data Bytes at Higher Speed 
#define PP			0x02	//Page Program 
#define SE			0xD8	//Sector Erase 
#define BE			0xC7	//Bulk Erase 
#define DP			0xB9	//Deep Power-down 
#define RES			0xAB	//Release from Deep Power-down,and Read Electronic Signature  / Release from Deep Power-down

#define WIP			BIT0	//Write In Progress Bit
#define WEL			BIT1	//Write Enable Latch Bit
#define BP0			BIT2	//Block Protect Bit 0
#define BP1			BIT3	//Block Protect Bit 1
#define BP2			BIT4	//Block Protect Bit 2
#define SRWD		BIT7	//Status Register Write Protect

#define FLASH_SELECT		CLEAR(FLASH_CS_POUT, FLASH_CS_PIN)
#define FLASH_DESELECT		SET(FLASH_CS_POUT, FLASH_CS_PIN)


/**
 * A struct used to store an address of data in the flash memory.
 * It's useful to work with this struct as single bytes of the address
 * are required for communication with the flash memory.
 */
typedef struct
{
	union
	{
		struct {
			uint8_t Byte0;
			uint8_t Byte1;
			uint8_t Byte2;
			uint8_t Byte3;
		} Bytes;
		uint32_t Value;
	} Address; ///< The address in the flash memory
} _flashAddress_t;


//#define WPR

static void _blockExecution() {
	volatile uint16_t i;
	for( i = 0; i < 60000; i++ ) { _NOP(); _NOP(); }
}

static uint8_t _ReadStatusRegister() {
	uint8_t sr = 0;
	// Read Identification
	if (USART0_SPI_Lock(0x02, 0x00, 0x00)) {

			FLASH_SELECT;
					USART0_SPI_Comm(RDSR);
					sr = USART0_SPI_Comm(0x00);
			FLASH_DESELECT;
			
		USART0_SPI_Release();
	}
	
	return sr;
}

// blocks the program flow when internal operation is in progress
static void _WaitUntilNotBusy() {
	uint8_t sr;
	do {
		sr = _ReadStatusRegister();
	}
	while (sr & WIP);
}

uint16_t Flash_Write(uint32_t address, const void* buffer, uint16_t bytesToWrite) {
	uint16_t bytesWritten;
	_flashAddress_t	currentAddress;
	//pageid are the 3 most significant bytes of the address, the least significant byte of the address should be always ignored (set to 0)
	uint32_t		pageId;

	uint8_t* fieldInBuffer = (uint8_t*) buffer;
	bytesWritten = 0;
	
	//check whether the address is valid (not whether an overflow happens, the application might require it so..)
	if (address > Flash_Max_Address) {
		return 0;
	}
	
	//get starting address
	currentAddress.Address.Value = address;
	
	//set 'current' page to 0xFF FF FF __ (outside the boundary) (__ are 8 LSB from the address)
	//so that the page has to be selected during the first execution of the following while-loop
	pageId = 0xFFFFFF00;

	//execute the following loop for every byte in the buffer
	if (USART0_SPI_Lock(0x02, 0x00, 0x00)) {
		while (bytesToWrite > 0) {
			//compute the target address
			uint32_t newPageId = ( currentAddress.Address.Value & 0xFFFFFF00);
				//if the current page and requested page differ:
				if (newPageId != pageId) {
					
					/**
					 * it looks like a page switch takes only 2-3 ticks of the ACLK
					 * this is minor delay.
					 * 
					 * it doesn't seem to be neccessary to add extra complexity for caching etc. 
					 */
					
					
					//wait until WIP is clear (possible page change or other prev operation)
					FLASH_DESELECT; //_blockExecution();
					_WaitUntilNotBusy();
					//_blockExecution(); 
					FLASH_SELECT;
					USART0_SPI_Comm(WREN); // The Write Enable Latch (WEL) bit must be set prior to every Page Program (PP)
					FLASH_DESELECT;
					_WaitUntilNotBusy();
					FLASH_SELECT;
		
					//start the write op at the inital address, keep the address of the current page in cache
					USART0_SPI_Comm(PP);
					USART0_SPI_Comm(currentAddress.Address.Bytes.Byte2);
					USART0_SPI_Comm(currentAddress.Address.Bytes.Byte1);
					USART0_SPI_Comm(currentAddress.Address.Bytes.Byte0);
					//update the current page address
					pageId = newPageId;
				}
				//write the byte
				USART0_SPI_Comm(*fieldInBuffer);
				
				//update counters
				fieldInBuffer++;					//move to the next byte in the buffer
				bytesToWrite--;						//update the bytes to write counter;
				currentAddress.Address.Value++;		//update the address
				bytesWritten++;
		} //while bytesToWrite > 0
		
		FLASH_DESELECT;
		
		USART0_SPI_Release();
	}
	// return the number of written bytes
	return bytesWritten;
}


 
void Flash_Read(uint32_t address, void* bufferOut, uint16_t bytesToRead) {
	_flashAddress_t flashAddress;
	uint8_t* fieldInBuffer = (uint8_t*) bufferOut;
	
	flashAddress.Address.Value = address;
		
	//LOG("M25P80: address: %lu (%x %x %x)", flashAddress.Address.Value, flashAddress.Address.Bytes.Byte2, flashAddress.Address.Bytes.Byte1, flashAddress.Address.Bytes.Byte0);
	
	if (USART0_SPI_Lock(0x02, 0x00, 0x00)) {
			_WaitUntilNotBusy();
			
			FLASH_SELECT;
					USART0_SPI_Comm(READ);
					USART0_SPI_Comm(flashAddress.Address.Bytes.Byte2);
					USART0_SPI_Comm(flashAddress.Address.Bytes.Byte1);
					USART0_SPI_Comm(flashAddress.Address.Bytes.Byte0);
					
					while (bytesToRead > 0) {
						//read
						*fieldInBuffer = USART0_SPI_Comm(0x00);
			
						//update the pointer and reduce the bytesToRead
						fieldInBuffer++;
						bytesToRead--;
					}
					
			FLASH_DESELECT;

		USART0_SPI_Release();
	}

}
void Flash_Erase() {
	if (USART0_SPI_Lock(0x02, 0x00, 0x00)) {
			_WaitUntilNotBusy();

			FLASH_SELECT;
					USART0_SPI_Comm(WREN);
					FLASH_DESELECT;
					_blockExecution();
					FLASH_SELECT;
					USART0_SPI_Comm(BE);
					
			FLASH_DESELECT;
		
		USART0_SPI_Release();
	}
}

bool Flash_Init() {
	LOGDBG("M25P80/M25P16 Init...");
    
	bool result = false;
	
	// Read Identification
	FLASH_SELECT;
		if (USART0_SPI_Lock(0x02, 0x00, 0x00)) {

			uint8_t manufacturerInformation, memoryType, memoryCapacity;

			USART0_SPI_Comm(RDID);
			manufacturerInformation	= USART0_SPI_Comm(0x00);
			memoryType				= USART0_SPI_Comm(0x00);
			memoryCapacity			= USART0_SPI_Comm(0x00);

			USART0_SPI_Release();

			result = (manufacturerInformation == 0x20) && (memoryType == 0x20) && ((memoryCapacity == 0x14) || (memoryCapacity == 0x15));
			
			//LOGDBG("Device Identifiation: %x (20 expected) %x (20 expected) %x (14 or 15 expected)", manufacturerInformation, memoryType, memoryCapacity);
			
			if (memoryCapacity == 0x14)
				Flash_Max_Address = 0x0FFFFF;
			
			if (memoryCapacity == 0x15)
				Flash_Max_Address = 0x1FFFFF;
			
			LOGDBG("Capacity: %lu bytes", Flash_Max_Address + 1);

			if (result == true)
				LOG("M25P80/M26P16 Init: OK");
		}
	FLASH_DESELECT;
		
	return result;
}

static void _WaitUntilNotBusyLongWithFeedback() {
	uint8_t i = 0;
	
	if (USART0_SPI_Lock(0x02, 0x00, 0x00)) {
		uint8_t sr;
		
		do {
			sr = _ReadStatusRegister();
			if (++i == 0) {
				LED2_TOGGLE;
			}
			
			Watchdog_Reset();
		}
		while (sr & WIP);

		USART0_SPI_Release();
	}
	LED2_OFF;
}

bool Flash_SelfTest() {
	uint32_t currentPage = Flash_Max_Address / 256;

	printf("FLASH MEMORY SELF TEST\r\n%lu pages\r\n", currentPage + 1); // +1 because of index starting from 0
	
	printf("Preparing...\r\n");
	Flash_Erase(); //this doesn't block!
	
	// wait here until erase is done:
	_WaitUntilNotBusyLongWithFeedback();
	
	// prepare test buffer
	uint8_t buffer00[256];
	uint8_t bufferFF[256];
	uint8_t buffer[256];
	
	memset(buffer00, 0x00, 256);
	memset(bufferFF, 0xFF, 256);
	
	do {
		bool pass = true;

		Watchdog_Reset();
		currentPage--;
		LED2_TOGGLE;
		
		// first, the page should be filled with 0xFFs 
		Flash_Read(currentPage * 256, buffer, 256);
		
		if (memcmp(buffer, bufferFF, 256) == 0)
			pass = true;
		else
			pass = false;

		// now, write 0x00s and make sure, its full of 0x00s
		Flash_Write(currentPage * 256, buffer00, 256);
		Flash_Read(currentPage * 256, buffer, 256);
		
		if (memcmp(buffer, buffer00, 256) == 0)
			pass = true;
		else
			pass = false;
		
		if (pass) {
			//     "Page XXXX pass"   
			printf("Page %lu pass \r", currentPage);
		}
		else {
			printf("Page %lu FAIL\r\n", currentPage);
			
			LED2_OFF;
			return false;
		}
	} while (currentPage != 0);
	
	printf("Cleaning up...\r\n");
	Flash_Erase(); //this doesn't block, but the next command will do

	// wait here until erase is done:
	_WaitUntilNotBusyLongWithFeedback();
	
	printf("PASS!\r\n");
	
	return true;
}


#ifndef DOXYGEN_PUBLIC_DOC
/**
 * Copyright (c) 2011 Freie Universitaet Berlin, Tomasz Naumowicz. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 * 
 * - Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 * 
 * - Neither the name of the Freie Universitaet Berlin nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freie Universitaet Berlin BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#endif // DOXYGEN
