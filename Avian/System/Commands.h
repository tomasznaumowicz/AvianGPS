/**
 * @brief		Support for local and remote commands. Read the \ref CommandsIntro page for introduction to commands.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "Types.h"
#include "Commands.API.h"

/**
 * States whether the debugging features of the command engine are enabled or not.
 */
#ifndef DEBUG_COMMANDS_ENABLED
	#define		DEBUG_COMMANDS_ENABLED		0
#endif


/**
 * @brief		Holds the maximal number of characters that can be stored in the command response. The command handler must make sure it's not writing more characters to the response buffer.
 * 
 * In this case the lengt of the response is limited by the size of a radio packet since the responses are also transmitted over the radio.
 */
#define COMMAND_RESPONSE_LENGTH 		(LAYER1_PAYLOAD_SIZE - COMMAND_MAX_COMMAND_NAME_LENGTH)


/**
 * @brief		Initialized the command engine
 * 
 * @remarks		Requires radio unit to be initalized already.
 */
void Commands_Init();

/**
 * @brief		Starts to precess the given input as a command
 */
bool Commands_ProcessInput(const char* input);

#endif /*COMMANDS_H_*/


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
