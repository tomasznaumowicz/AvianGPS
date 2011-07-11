/**
 * @file		
 * @brief		@b System: Support for software timers.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * Example of a software timer registration:
 * @code
 * void Blink(uint16_t counter) {
 *	// Toggle the state of the LED1
 *	LED1_TOGGLE;
 *	
 *	// Provide debug output
 *	LOG("Toggled the LED1 %u times", counter);
 *	
 *	// Schedule execution of this function after one second, increase the value of the counter.
 *	Timers_Add(Blink, TICKS_SECOND, counter++);
 * }
 * 
 * // somewhere in your application code, e.g. in the Application_Init() function
 * // ..
 * Timers_Add(Blink, TICKS_SECOND, 1);
 * // ...
 * @endcode 
 *  */

#ifndef TIMERS_API_H_
#define TIMERS_API_H_

#include <msp430x16x.h>
#include <stdint.h>
#include "Types.h"

/**
 * @name		Constants used for Timers_Add or Timers_Block function calls.
 * 
 * This constants simplify working with sofware timers. Software timers accept "ticks" values as parameters,
 * where 8192 ticks equal 1 second. This helps to implement timer operations where the delay needs to 
 * be smaller than one second. Usually you'll be working with delays much larger than 1 second. Use
 * those constants to reduce the amount of multiplications in your software. Please note that
 * e.g. using Timers_Add(60 * 8192) might result in erronous results as the compiler might use wrong
 * datatypes and not handle overflows properly.
 * 
 * @{
 */

#define TICKS_PER_SECOND				8192				///< Number of ticks per second
#define TICKS_PER_MILISECOND			(8.192)				///< Number of ticks per milisecond

#define TICKS_SECOND					8192UL				///< 1 second
#define TICKS_1SECOND					8192UL				///< 1 second
#define TICKS_2SECONDS					16384UL				///< 2 seconds
#define TICKS_5SECONDS					40960UL				///< 5 seconds
#define TICKS_10SECONDS					81920UL				///< 10 seconds
#define TICKS_20SECONDS					163840UL			///< 20 seconds
#define TICKS_30SECONDS					245760UL			///< 30 seconds
#define TICKS_60SECONDS					491520UL			///< 60 seconds / 1 minute
#define TICKS_MINUTE					491520UL			///< 1 minute
#define TICKS_1MINUTE					491520UL			///< 1 minute
#define	TICKS_2MINUTES					983040UL			///< 2 minutes
#define TICKS_5MINUTES					2457600UL			///< 5 minutes
#define TICKS_10MINUTES					4915200UL			///< 10 minutes
#define TICKS_30MINUTES					14745600UL			///< 30 minutes

/** @} */

/**
 * @brief Blocks program execution for the specified number of miliseconds
 */
#define TIMERS_BLOCK_MS(ms)        Timers_Block((uint16_t)(8 * (uint16_t) ms))			

/**
 * @brief Definition of a handler for processing of an expired software timer.
 */
typedef void (*fp_timer_t) (uint16_t);

/**
 * @brief Delivers the current TAR value using majority voting with 2 samples.
 * Useful for benchmarking.
 */
#define COPY_CURRENT_TAR		({ register uint16_t _TAR_Copy; do { _TAR_Copy = TAR;} while (_TAR_Copy != TAR);  _TAR_Copy; })

/**
 * @brief Provides the current number of ACLK ticks since power on event
 */
uint32_t *TICKS_SINCE_POR();

/**
 * @name		Managing software timers
 * @{
 */

/**
 * @brief	Registers a software timer handler.
 * 
 * Multiple software timer handlers can be registered. After the specified number of ticks
 * elapses, the registered function handler will be scheduled for execution.
 *
 * Example:
 * @code
 * void Blink(uint16_t counter) {
 *	// Toggle the state of the LED1
 *	LED1_TOGGLE;
 *	
 *	// Provide debug output
 *	LOG("Toggled the LED1 %u times", counter);
 *	
 *	// Schedule execution of this function after one second, increase the value of the counter.
 *	Timers_Add(Blink, TICKS_SECOND, counter++);
 * }
 * 
 * // somewhere in your application code, e.g. in the Application_Init() function
 * // ..
 * Timers_Add(Blink, TICKS_SECOND, 1);
 * // ...
 * @endcode 
 * 
 * @param[in]	timerHandler	Function that shall be executed after the specified delay
 * @param[in]	ticks			Delay in ticks (you might use the defines provided by this API file such as \ref TICKS_MINUTE
 * @param[in]	data			A number that will be passed to the software timer handler.
 * 
 * @return				timer id (can be used to deactivate a timer) if successful;
 * 						-1 otherwise.
 *
 * @remarks
 *  - Maximum delay is about 145 hours. The exact value is (2^32)-1 ticks: 4294967295 ticks and with 8192 ticks per second it equals 524287 seconds / 8738 minutes / 145.6.. hours. 
 *  - Benchmark: 26us with empty timers collection, 28us with 6 entries in the timers collection.
 */
int16_t Timers_Add(const fp_timer_t timerHandler, uint32_t ticks, uint16_t data);

/**
 * @brief		Removes a scheduled software timer.
 * 
 * @param[in]	timerId			The timer id provided by the \ref Timers_Add function
 */
bool Timers_Remove(int16_t timerId);

/** @} */

/**
 * @brief		Blocks the execution for the specified number of ticks.
 *
 * @remarks
 * 	- 1 tick == 122us
 * 	- It doesn't block interrupts.
 * 	- It's not using any energy saving modes.
 */
void Timers_Block(uint16_t ticks);

#endif /*TIMERS_API_H_*/


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
