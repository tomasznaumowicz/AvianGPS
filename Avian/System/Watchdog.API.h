/**
 * @file
 * @ingroup		SystemAPI
 * @brief		@b System: Header file for the watchdog support.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef WATCHDOG_API_H_
#define WATCHDOG_API_H_

#include <sys/inttypes.h>
#include <msp430x16x.h>

/**
 * @brief Describes the reset reason of the device.
 */
enum wd_reset_reason {
	WD_RESET_REASON_UNKNOWN					= 0,	///< Unknown (e.g. new battery installed).
	WD_RESET_REASON_WATCHDOGGUARDVIOLATION	= 1,	///< The watchdog state in memory was corrupted. Critical.
	WD_RESET_REASON_REQUESTED				= 2,	///< The reset was requested, e.g. by using the "reset" command.
	WD_RESET_REASON_RADIO 					= 3,	///< The radio driver requested a reset (driver critial state, used for debugging).
	WD_RESET_REASON_WATCHDOG				= 4,	///< The regular watchdog reset: an operation was blocking for too long. 
	WD_RESET_REASON_USER1					= 5,	///< User defined reset reason (used by an application).
	WD_RESET_REASON_USER2					= 6,	///< User defined reset reason (used by an application).
	WD_RESET_REASON_USER3					= 7,	///< User defined reset reason (used by an application).
	WD_RESET_REASON_USER4					= 8		///< User defined reset reason (used by an application).
};

/**
 * @brief Describes information about recent reset of the device. Use \ref Watchdog_ReadResetReason to access this information.
 * 
 * @remarks
 * To firmware developers: the size of the wd_resetinfo_t == 6, but because the whole struct is then aligned in memory, it's 8 bytes long.
 * If the size of this struct is changed, the linker script needs to be updated as well.
 * Don't change this structure unles you are really understand what you are doing!
 */
typedef struct {
	uint16_t				FunctionPointer;		///< pointer to the function where the firmware was aborted by the watchdog
	uint16_t				StackPointer;			///< latest stack pointer
	uint8_t					Password;				///< Value 0xAB allows to assume that the data was written by the firmware
	enum wd_reset_reason	ResetReason;			///< informs about the reason of the reset (set by the code)
} wd_resetinfo_t; 								


/**
 * @brief		Resets the watchdog reset timer
 * 
 * You are required to execute a watchdog reset when processing a long running operation.
 * Otherwise the device will reset automatically. In the current configuration the device
 * will reset 10 seconds after the last call to the \ref Watchdog_Reset function. The main loop
 * of the system calls \ref Watchdog_Reset periodicaly.
 * 
 * Example:
 * @code
 * // long running operation... e.g. a loop
 * Watchdog_Reset;
 * // long running operation continues 
 * @endcode
 */
void Watchdog_Reset();

/**
 * @brief		Forces a device reset. Sets the reset reason to WD_RESET_REASON_REQUESTED
 */
void Watchdog_DeviceReset();

/**
 * @brief		Provides information about the previous reset reason.
 * 
 * 
 * @param[out]	resetReasonOut		Information about the reset reason. 
 */
void Watchdog_ReadResetReason(wd_resetinfo_t* resetReasonOut);

#endif /*WATCHDOG_APIH_*/


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
