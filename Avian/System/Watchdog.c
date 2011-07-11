/**
 * @file		Watchdog.API.h
 */

#ifndef DOXYGEN_PUBLIC_DOC

#include <msp430x16x.h>
#include <signal.h>

#include <string.h>

#include "BitOperations.h"
#include "Logging.h"
#include "Configuration.h"
#include "Commands.h"
#include "Watchdog.h"
#include "Watchdog.API.h"

void Watchdog_Init() {
	memset((void*) &WD_State, 0, sizeof(wd_state_t));
	WD_RESET;											// reset the watchdog
	IE1 |= WDTIE; 										// make sure IE1 isn't overwritten anywhere!
	
}

void Watchdog_ProcessResetInfo() {
	if(IFG1 & WDTIFG) {
		CLEAR(IFG1, WDTIFG);

		LOGDBG("R:(%u) reason: %u (FP: %p SP: %p)",
				__wd_resetinfo.Password == WD_RESETINFO_PWD,		// tells whether the information is valid (whether the password was set prior to the reset.
				__wd_resetinfo.ResetReason,
				__wd_resetinfo.FunctionPointer,
				__wd_resetinfo.StackPointer);
	}
	else {
		LOGDBG("R:?");
		
		/**
		 * The reset reason is unknown since the WDTIFG flag is not set.
		 * But the watchdog resetinfo is still filled with some recent data,
		 * this needs to be removed.
		 * 
		 * setting all fields to 0 using memset also resets the Password and in this
		 * way it invalidates the entire entry.
		 */
		
		memset(&__wd_resetinfo, 0, sizeof(wd_resetinfo_t));
	}
}

/**
 *  Watchdog Timer interrupt service routine is being executed once per second
 */
interrupt (WDT_VECTOR) watchdog_ISR(void)
{
	// a countdown reduces the value of the HitCounter
	if (WD_State.Guard1 == 0x00 && WD_State.Guard2 == 0x00 && WD_State.HitCounter > 0) {
		WD_State.HitCounter--;
		return;
	}

	// the HitCounter hit 0 and the condition for a resed forced by the watchdog was reached:
	
	// collect debug crash information
	__wd_resetinfo.FunctionPointer = * (((uint16_t*)GET_FRAME_ADDR(watchdog_ISR)) + 1);
	__wd_resetinfo.StackPointer = READ_SP;

	// set the resetinfo password so that the data can be identified as valid
	__wd_resetinfo.Password = WD_RESETINFO_PWD;
	
	// try detect and store the reason of the reset
	if (WD_State.Guard1 == 0x00 && WD_State.Guard2 == 0x00)
		// the guards were not violated, this indicated that it's a regular watchdog reset
		__wd_resetinfo.ResetReason = WD_RESET_REASON_WATCHDOG;
	else
		// the guards were violated, this means that the device is in an unpredictable state and should be reset
		__wd_resetinfo.ResetReason = WD_RESET_REASON_WATCHDOGGUARDVIOLATION;
	
	// force the device reset
	WDTCTL = 0xDEAD;
}

#endif /* DOXYGEN_PUBLIC_DOC */

void Watchdog_ReadResetReason(wd_resetinfo_t* resetReasonOut) {
	memcpy(resetReasonOut, &__wd_resetinfo, sizeof(wd_resetinfo_t));
}

void Watchdog_Reset() {
	WD_RESET;
}

void Watchdog_DeviceReset() {
	__wd_resetinfo.Password = WD_RESETINFO_PWD;
	__wd_resetinfo.ResetReason = WD_RESET_REASON_REQUESTED;
	WDTCTL = 0xDEAD;
}

/**
 * @ingroup Commands
 * @brief	@b rr Provides information about the reset reason of the device.
 *
 * @returns
 *  - @b 0 for @b WD_RESET_REASON_UNKNOWN
 *  - @b 1 for @b WD_RESET_REASON_WATCHDOGGUARDVIOLATION
 *  - @b 2 for @b WD_RESET_REASON_REQUESTED (e.g. the "reset" command)
 *  - @b 3 for @b WD_RESET_REASON_RADIO (e.g. the radio driver crashed)
 *  - @b 4 for @b WD_RESET_REASON_WATCHDOG (e.g. a long running operation was too long or a function hung)
 *  - @b 5 for @b WD_RESET_REASON_USER1 (can be used by the user in the application)
 *  - @b 6 for @b WD_RESET_REASON_USER2 (can be used by the user in the application)
 *  - @b 7 for @b WD_RESET_REASON_USER3 (can be used by the user in the application)
 *  - @b 8 for @b WD_RESET_REASON_USER4 (can be used by the user in the application)
 * 
 * More details on the reset reason enumeration: \ref wd_reset_reason
 */
COMMAND(rr, "Debug command: Provides the recent reset reason", cmd_args) {
	if (__wd_resetinfo.Password == WD_RESETINFO_PWD)
	{
		snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength,
				"reset reason: %u (FP: %p SP: %p)",
				(__wd_resetinfo.ResetReason),
				(void*)(__wd_resetinfo.FunctionPointer),
				(void*)(__wd_resetinfo.StackPointer));
	}
	else {
		snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength,
				"reset reason: unknown");
	}
}

/**
 * @ingroup Commands
 * @brief	@b reset Forces the device to reset.
 * 
 * Sets the reset reason to @b 2 (WD_RESET_REASON_REQUESTED)
 */
COMMAND(reset, "Resets the device", cmd_args) {
	Watchdog_DeviceReset();
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
