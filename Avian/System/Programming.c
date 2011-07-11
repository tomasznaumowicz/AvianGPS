/**
 * @file		Programming.API.h
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */


#ifndef DOXYGEN_PUBLIC_DOC


#include <msp430x16x.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>

#include "RTC.API.h"
#include "Commands.h"
#include "Logging.h"

#include "Programming.API.h"
#include "Programming.h"


void Programming_PrintApplicationHeader() {
	printf("\r\n\r\n");
	
	LOG("App: '%s' V: %i.%i for %s with %s",
			ApplicationHeader.ApplicationName,
			ApplicationHeader.ApplicationVersion[0],
			ApplicationHeader.ApplicationVersion[1],
			ApplicationHeader.DeviceName,
			ApplicationHeader.ExtensionBoardName
		);
	
	char rtc_buff[RTC_TIME_STRING_LENGTH];
	
	rawtime_t rawtime;
	time_t time;
	rawtime.miliseconds = 0;
	rawtime.seconds = ApplicationHeader.CompileTime;			
	RTC_ConvertToTime(&rawtime, &time);
	RTC_ToString(&time, rtc_buff);

	LOG("Compiled: %s UTC",
		rtc_buff
		);
}

#endif



void Programming_Infomem_Read(uint16_t offset, void* bufferOut, uint8_t size) {
	uint8_t* address = (uint8_t*)INFOMEM_START + offset;
	memcpy(bufferOut, address, size);
}

bool Programming_Infomem_Write(uint16_t offset,uint8_t count, ...) {
	char backup[INFOMEM_BLOCK_SIZE];
	
	register uint8_t* buffer;
	register uint16_t i;
	uint8_t*	 flash;

	if( offset > ( 2 * INFOMEM_BLOCK_SIZE) )
		return false;

	flash = (uint8_t*)INFOMEM_START;

	if (offset >= INFOMEM_BLOCK_SIZE) {
		flash = (uint8_t*) (INFOMEM_START + INFOMEM_BLOCK_SIZE);
		offset -= INFOMEM_BLOCK_SIZE;
	}

	dint();

	// backup into RAM
	memcpy(backup, flash, INFOMEM_BLOCK_SIZE);

	// merge backup with new data
	va_list argp;
	va_start(argp, count);

	buffer = (uint8_t*)backup + offset;
	for( i = 0; i < count; i++ ) {
		register uint16_t	size;
		register uint8_t* data;

		data = va_arg(argp, uint8_t*);
		size = va_arg(argp, uint16_t);
		memcpy(buffer, data, size);
		buffer += size;
	}
	va_end(argp);

	// setup the clock for flash memory operations
	FCTL2 = FWKEY | FSSEL_SMCLK | FN4;				// flash controller clock: 7372000 / 16 = 460kHz
													// TODO: check whether the power consumption increases: maybe it should be stopped after the update of the infomem

	// init flash access
    FCTL3 = FWKEY;

	// erase flash
    FCTL1 = FWKEY | ERASE;

    *flash = 0;

	// write flash
    FCTL1 = FWKEY | WRT;

    buffer = (uint8_t*)backup;
	
    for( i = 0; i < INFOMEM_BLOCK_SIZE; i++ ) {
		*flash = *buffer;
		buffer++; flash++;
	}
    
    FCTL1 = FWKEY;
    FCTL3 = FWKEY | LOCK;

	eint();
	return true;
}



/**
 * @ingroup Commands
 * @brief	@b app Provides metadata of the application
 */
COMMAND(app, "Configuration: application's metadata", cmd_args) {
	char compileTime_buff[RTC_TIME_STRING_LENGTH];
	
	rawtime_t rawtime;
	time_t time;
	rawtime.miliseconds = 0;
	rawtime.seconds = ApplicationHeader.CompileTime;			
	RTC_ConvertToTime(&rawtime, &time);
	RTC_ToString(&time, compileTime_buff);
	
	snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength,
			"'%.8s' %i.%i compiled %s",
			ApplicationHeader.ApplicationName,
			ApplicationHeader.ApplicationVersion[0],
			ApplicationHeader.ApplicationVersion[1],
			compileTime_buff
	);
}





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
