/**
 * @file
 * @ingroup		SystemAPI
 * @brief		@b System: Collection of IO functions.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef SMALLIO_H_
#define SMALLIO_H_

/**
 * @brief		Implementation of the sscanf function.
 * 
 * See http://www.cplusplus.com/reference/clibrary/cstdio/sscanf/ for a description of a sscanf function.
 * 
 * Following format specifiers are supported:
 *  - \%lu			&nbsp;&nbsp;		for uint32_t types
 *  - \%8u			&nbsp;&nbsp;		for uint8_t types
 *  - \%u			&nbsp;&nbsp;		for uint16_t types
 *  - \%c			&nbsp;&nbsp;		for a character (uint8_t)
 *  - \%.[width]s	&nbsp;&nbsp;		for a string, where the [width] specifies the maximum expected length of the string. [width] is not optional.
 *
 * @remarks		Processing of a string will terminate on '\\n', '\\0', ',', or when the [width] restriction is met.
 *  
 * In the follwing example a NMEA RMC sentence is being processed. All fields are being analyzed but only two are used.
 * This is required as it was important to make sure the received RMC sentence has a correct syntax.
 * 
 * @code
 * int fieldsFilled = small_sscanf(input, "$GPRMC,%.12s,A,%.20s,%c,%.20s,%c,%.20s,%.20s,%.12s,",
 *	rmc_time, // read the time
 *	NULL,
 *	NULL,
 *	NULL,
 *	NULL,
 *	NULL,
 *	NULL,
 *	rmc_date // read the date
 * );
 * @endcode
 */
int small_sscanf(const char * input, const char * format, ...);

#endif /*SMALLIO_H_*/


#ifndef DOXYGEN
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
