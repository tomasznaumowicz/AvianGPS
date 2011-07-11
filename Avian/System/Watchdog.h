/**
 * @brief		Header file for the watchdog support (internal).
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <sys/inttypes.h>
#include <msp430x16x.h>

#include "Watchdog.API.h"

/**
 * Used to determine whether reset reason information was
 * set by the software. Simple check but good enough.
 */
#define WD_RESETINFO_PWD		0xAB

/**
 * used to count the watchdog interrupt hits
 * after a certain number of hits is detected or the memory Guard1 and Guard2 get corrupted
 * a reset of the system will be enforced
 */
typedef struct {
	uint16_t 	Guard1;			///< Guard1 will be set to 0x00 and checked periodically if it's still 0x00		
	uint16_t	HitCounter;		///< Counts how often the watchdog ISR was invoked.
	uint16_t	Guard2;			///< Guard2 will be set to 0x00 and checked periodically if it's still 0x00
} wd_state_t;

/**
 * Holds the watchdog software counter state
 */
volatile wd_state_t WD_State;

/**
 * Holds information about the reset reason.
 * 
 * @remarks
 * The variable is defined as extern as it's provided within the linker script.
 * It's placed at the beginning of RAM in the uninitialized area
 * so that the state is preserved accross resets of the system.
 */
extern wd_resetinfo_t __wd_resetinfo;

/**
 * @brief		Resets the watchdog reset timer
 * 
 * You are required to execute a watchdog reset when processing a long running operation.
 * Otherwise the device will reset automatically.
 * 
 * Example:
 * @code
 * // long running operation... e.g. a loop
 * WD_RESET;
 * // long running operation continues 
 * @endcode
 */
#define WD_RESET					{WDTCTL = WDT_ADLY_1000; WD_State.HitCounter = 60;}

/**
 * Initializes the Watchdog state and the support for hardware watchdog.
 * 
 * The watchdog state is kept in memory. It's used to determine whether a reset timeout elapsed.
 * 
 * The reason for this implementation:
 * 		The maximum hardware timeout in current hardware configuration is 1000ms although
 * 		some functions require more time to complete. Such functions are required to reset
 * 		the watchdog hardware timer often.
 * 
 * 		In order to simplify this process, the watchdog hardware is not resetting the device
 * 		when the interrupt occurs, but it reduces a value of a software counter.
 * 		Only when the software counter reaches the predefined value (in current implementation the value 10),
 * 		the device will be reset.
 */
void Watchdog_Init();

/**
 * This function shall be called after powering up the device. It clears the possible WDTIFG flag
 * that was set when the device was reset by the watchdog hardware.
 * This function also prints out the reset reasons as debug output.
 * 
 * After each reset caused by the watchdog or properly requested by the software, a reset information is 
 * stored in protected area of memory. This function is able to access this area and print
 * the recent reset reason info. This information is currently printed
 * using the LOGDBG macro and is not secured in any log file.
 * 
 * This function is executed once during the system initialization.
 */
void Watchdog_ProcessResetInfo();

#endif /*WATCHDOG_H_*/



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
