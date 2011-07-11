/**
 * @ingroup		DriverAPI
 * @brief		Used for the LogEngine.Sync feature
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#include <msp430x16x.h>

#include "../System/BitOperations.h"

#include "BoolBitArray.h"

const uint8_t BitmaskTable[8] = { BIT0, BIT1, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7 };

void BoolBitArray_Set(boolbitarray_t* array, const uint32_t* index) {
	// compute the index of the byte
	uint32_t byteIndex = *index >> 3;		// divide the index by 8
	
	// prepare a bitmask which can be used to set one bit in the request
	uint8_t bitIndex = (uint8_t) *index & 0x00000007;		// this mask extracts the lower 3 bits that encode numbers [0-7]
	
	// use the bitmask to set the proper bit in the byte selected by the byteIndex
	SET(array[byteIndex], (BitmaskTable[bitIndex]));
}

bool BoolBitArray_Get(const boolbitarray_t* array, const uint32_t* index) {
	// compute the index of the byte
	uint32_t byteIndex = *index >> 3;		// divide the index by 8
	
	// prepare a bitmask which can be used to set one bit in the request
	uint8_t bitIndex = (uint8_t) *index & 0x00000007;		// this mask extracts the lower 3 bits that encode numbers [0-7]
	// test the bit
	if (array[byteIndex] & (BitmaskTable[bitIndex]))
		return true;
	else
		return false;
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
