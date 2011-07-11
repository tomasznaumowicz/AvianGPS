/**
 * @ingroup		DriverAPI
 * @brief		Used for the LogEngine.Sync feature
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef BOOLBITARRAY_H_
#define BOOLBITARRAY_H_

#include <stdint.h>
#include "../System/Types.h"

extern const uint8_t BitmaskTable[8];

typedef uint8_t boolbitarray_t;


/**
 * Usage
 * 
 * boolbitarray_t[40]
 * 
 * the 40 says how many bytes should be allocated, and the actual size of the bit array is 40 * 8
 * this could be covered with few defines, but is not really necessary for now
 */

/**
 * The set and get functions don't check the array boundaries, it's your responsibility.
 */
void BoolBitArray_Set(boolbitarray_t* array, const uint32_t* index);
bool BoolBitArray_Get(const boolbitarray_t* array, const uint32_t* index);

#endif /*BOOLBITARRAY_H_*/


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
