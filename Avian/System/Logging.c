/**
 * @brief		Support for debug logging to the serial output and over the radio (if enabled)
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#include <string.h>

#include "RTC.API.h"
#include "Radio/Network.h"
#include "System.h"

#include "Logging.h"

#if (LOGGING_ENABLED)

	void Logging_HandlerUart(const char *format, ...) {
		va_list args;
		va_start(args, format);
		
		vprintf(format, args);
		
		va_end(args);
	
		//////////////////////////////////////////////////// gets the current time as a string. this timestamp will be attached to the debug log output (UART)
		time_t currentTime;
		RTC_GetTime(&currentTime);
		char currentTimeString[RTC_TIME_STRING_LENGTH];
		RTC_ToString(&currentTime, currentTimeString);
		
		printf(" \033[1;30m[%s]\033[0m\r\n", currentTimeString);
	}

	void Logging_HandlerRadio(const char *format, ...) {
		// the log entry must be generated out of the format string and its parameters first
		// since its going to be sent over radio, it can't be bigger than a size of a packet
		// 1. prepare a buffer for the text
		char buffer[LAYER1_PAYLOAD_SIZE];

		va_list args;
		va_start(args, format);
		
		// 2. generate the log entry as string 
		vsnprintf(buffer, LAYER1_PAYLOAD_SIZE, format, args);		
		va_end(args);

		// 3. prepare the network packet
		network_payload_t packet;
		Network_InitSendArgs(&packet);
		packet.Buffers[0].Size = LAYER1_PAYLOAD_SIZE;
		packet.Buffers[0].Buffer = buffer;
		
		// 4. broadcast the log entry
		Network_SendWithoutACK(0, NETWORK_PROTOCOL_LOG, &packet);
	}
	
	void Logging_ProcessRemoteLog(uint8_t source, const uint8_t* data) {
		// data contains the string that might be not null delimited
		
		// 1. prepare a copy of the input and make sure it's null-delimited
		char buffer[LAYER1_PAYLOAD_SIZE];					// create the buffer
		memset(buffer, 0, LAYER1_PAYLOAD_SIZE);			// fill it with 0s
		strncpy(buffer, data, LAYER1_PAYLOAD_SIZE - 1);	// and copy data from the input, but take one character less so that the 0 is not overwritten;
		
		// 2, print the debug output
		printf("[%u:L] %s\r\n", source, buffer);
	}

#endif



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
