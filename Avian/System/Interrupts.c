#include <msp430x16x.h>
#include <stdio.h>
#include <signal.h>


#include "BitOperations.h"
#include "Commands.h"
#include "RTC.h"
#include "System.h"

#include "Interrupts.h"


/**
 * The Port2 is interrupt capable.
 * 
 * Since multiple devices rely on this interrut and an ISR can be easily defined only once,
 * it was located in a separate file. Device specific subroutines will be called from here
 */

interrupt(PORT2_VECTOR) P2_ISR(void) {
	// process interal stuff first
	if( P2IFG & CC_GDO0_PIN )	//Check P2IFG Bit P2.7 - CC1100 (GDO0) Rx Packet
	{
		CLEAR( P2IFG, CC_GDO0_PIN );
		
		SystemFlags.Flag.Interrupt_Radio = true;
		END_LPM;
	}

	// and check whether something else should be handled
	if (P2IFG) {
		// only if there are any unhandled interrupt flags
		if (Handler_Interrupt_Port2 != NULL) {
			// and only if there is a handler registered
			bool wakeup_request = Handler_Interrupt_Port2();
			
			// wakeup the device if this was requested
			if (wakeup_request)
				END_LPM;
		}
	}
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
