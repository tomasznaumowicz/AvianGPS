/**
 * @file
 * @brief		@b System: Support for interrupt handling for port 1 and port 2
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * Used mainly by hardware and driver developers.
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include "Types.h"

/**
 * @brief	@b Handler: Definition of the handler for processing interrupts.
 * 
 * return true if a wakeup from LPM is required 
 */
typedef bool(*fp_interrupt_handler_t) (void);

/**
 * @ingroup	Handlers
 * @brief @b Handler: Handle interrupt on port 2 (see \ref Handlers for more handlers).
 * 
 * This feature is important mostly for drivers developers.
 * 
 * @remarks		Only one handler can be registered.
 */
fp_interrupt_handler_t Handler_Interrupt_Port2;

/**
 * @ingroup	Handlers
 * @brief @b Handler: Handle interrupt on port 1 (see \ref Handlers for more handlers).
 * 
 * This feature is important mostly for drivers developers.
 * 
 * @remarks		This feature is currently not implemented as it was not required in the current hardware configuration / project.
 */
fp_interrupt_handler_t Handler_Interrupt_Port1;

#endif /*INTERRUPTS_H_*/


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
