/**
 * @file
 * @brief		@b System: Support for global system configuration.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * The system has support for persistent system settings. Currently only the node id is persisted within
 * the configuration. This could be easily expanded to support e.g. radio channel or transmit power.
 * 
 * During the system init phase, the configuration is loaded and made available in the \ref Configuration variable.
 * 
 * After every change to the content of this variable it should be persisted, call the \ref Configuration_Save function
 * to achieve this.
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <stdint.h>

/**
 * @brief		Describes the configuration of the device.
 */
typedef struct {
	uint8_t			NodeID;					///< Stores the current node id of the device
} configuration_t;

/**
 * @brief	Global variable holding the configuraiton of the device. You can access it to query and update information. Use \ref Configuration_Save to persist changes.
 */
configuration_t Configuration;

/**
 * @brief	Prints the current configuration settings using the \ref LOG macro.
 */
void Configuration_Log();

/**
 * @brief	Persists the current content of the \ref Configuration variable.
 */
void Configuration_Save();



// exclude the rest from the public documentation

#ifndef DOXYGEN_PUBLIC_DOC

/**
 *  @brief		Initializes the configuration feature.
 */
void Configuration_Init();

#endif

#endif /*CONFIGURATION_H_*/


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
