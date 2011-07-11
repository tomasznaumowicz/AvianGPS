/**
 * @file		
 * @brief		@b Driver: Flash Memory
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * For the M25P80:
 *  
 *	 The memory is organized as:
 * 		- total 1,048,576 bytes (8 bits each)
 * 		- total 16 sectors (256 pages each. 512 Kbits, 65536 bytes each)
 * 		- total 4096 pages (256 bytes each).
 * 
 * For the M25P16: 
 * 
 * 	The memory is organized:
 * 		- total 2 097 152 bytes (8 bits each)
 * 		- total 32 sectors (seach with 256 pages)
 * 		- total 8192 pages (256 bytes each)
 * 
 * 	Each page can be individually programmed (bits are programmed from 1 to 0).
 * 
 * 	The device is Sector or Bulk Erasable (bits are erased from 0 to 1) but not Page Erasable.
 */

#ifndef M25P80_API_H_
#define M25P80_API_H_

#include <stdint.h>
#include "../System/Types.h"

/**
 * @brief			Write bytes to M25P80/M25P16 flash memory (assumes the memory was erased previously)
 * @param[in]		address			Address in the Flash Memory (000000 - 0FFFFF M25P80) (000000 - 1FFFFF M25P16)
 * @param[in]		buffer			Pointer to buffer with data to write
 * @param[in]		bytesToWrite	Number of bytes to write
 * 
 * It's not possible to overwrite data: bits are programmed from 1 to 0 and erased from 0 to 1
 */
 int16_t Flash_Write(uint32_t address, const void* buffer, uint16_t bytesToWrite);
 
/**
 * @brief			Read bytes from M25P80/M25P16 flash memory
 * @param[in]		address			Address in the Flash Memory (000000 - 0FFFFF M25P80) (000000 - 1FFFFF M25P16)
 * @param[out]		bufferOut		Pointer to buffer for read data
 * @param[in]		size			Number of bytes to read
 */
void Flash_Read(uint32_t address, void* bufferOut, uint16_t size);

/**
 * @brief	Erase entire M25P80/M25P16 flash memory
 */
void Flash_Erase();

/**
 * @brief	Initializes the flash memory
 */
bool Flash_Init();

/**
 * @brief	Performs a self test of the flash memory
 */
bool Flash_SelfTest();

#endif /*M25P80_API_H_*/

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
