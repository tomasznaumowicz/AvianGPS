/**
 * @file
 * @ingroup		SystemAPI
 * @brief		@b System: Header file for the system management.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef SYSTEM_API_H_
#define SYSTEM_API_H_

#include "Types.h"

#ifndef LED_WHEN_IN_ACTIVEMODE_TURN_ON_MACRO
	#define LED_WHEN_IN_ACTIVEMODE_TURN_ON_MACRO
#endif

#ifndef LED_WHEN_IN_ACTIVEMODE_TURN_OFF_MACRO
	#define LED_WHEN_IN_ACTIVEMODE_TURN_OFF_MACRO
#endif

/**
 * @brief @b Handler: Definition of the handler for processing application layer tasks
 * 
 * The application or a driver can request a task to be executed, this can happen e.g. inside of a interrupt handler
 * where long running tasks should be avoided. Only one AppLayer task handler can be registered. Queuing (if required)
 * needs to be implemented in the application layer.
 */
typedef void(*fp_applayer_task_handler_t) (void);

/**
 * @ingroup Handlers
 * 
 * @brief @b Handler: Handler for processing application layer tasks (see \ref Handlers for more handlers).
 * 
 * The application or a driver can request a task to be executed, this can happen e.g. inside of a interrupt handler
 * where long running tasks should be avoided. Only one AppLayer task handler can be registered. Queuing (if required)
 * needs to be implemented in the application layer.
 *
 * @remarks		Only one handler can be registered.
 */
fp_applayer_task_handler_t Handler_AppLayer_Task;

/**
 * @brief		Schedules execution of the application layer task.
 * 
 * This function can be called within your application or within a driver to schedule an execution of the application layer task.
 * It can be also called withing an interrupt handler.
 */
void System_Trigger_AppLayer_Task();

/**
 * @brief Measures the system clock (SMCLK)
 * 
 * @returns	 The frequency of the main clock (SMCLK) in Hz.
 */
uint32_t System_MeasureSMCLK();

#endif /*SYSTEM_API_H_*/


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
