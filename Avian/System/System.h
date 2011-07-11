/**
 * @ingroup		SystemAPI
 * @brief		Header file for the system management.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_


#include <stdint.h>
#include "Types.h"

#include <msp430x16x.h>	// for BIT0, BIT1 etc.

#include "../Drivers/Device.h"

/**
 * @brief Target frequency of the MCLK
 */
#define FREQUENCY_TARGET				4000000L

/**
 * @brief Lower boundary for frequency adjustments.
 * 
 * Adjusting stops here TARGET FREQUENCY - 1% because the measurement accuracy with 4MHz is 0.8%!
 */
#define FREQUENCY_LOWER_BOUNDARY 			(FREQUENCY_TARGET - ((FREQUENCY_TARGET * 1 ) / 100))		

/**
 * @brief Upper boundary for frequency adjustments.
 * 
 * Adjusting stops here TARGET FREQUENCY + 1% because the measurement accuracy with 4MHz is 0.8%!
 */
#define FREQUENCY_UPPER_BOUNDARY 			(FREQUENCY_TARGET + ((FREQUENCY_TARGET * 1 ) / 100))

/**
 * Holds information about whether the system is currently in a low power mode or not.
 */
volatile bool SystemStateIsPowerDown;

#define st(x)      do { x } while (__LINE__ == -1)

/**
 * @brief	Enables "atomic" operations with interrupts disabled.
 */
#define atomic(x)	do { _DINT(); { x; } _EINT(); } while (__LINE__ == -1)

/**
 * @brief	Puts the microcontroller in a Low Power Mode.
 */
#define START_LPM 	{if (SystemFlags.AllFlags == 0) {SystemStateIsPowerDown = true; _BIS_SR(LPM3_bits + GIE);}}

/**
 * @brief	Exits the Low Power Mode
 */
#define END_LPM		{LPM3_EXIT; SystemStateIsPowerDown = false;}

/**
 * @brief	Exits the Low Power Mode but only when the \ref SystemStateIsPowerDown is set
 */
#define END_LPM_WITH_CONDITION_CHECK 	{ if (SystemStateIsPowerDown) { SystemStateIsPowerDown = false; LPM3_EXIT; } } 


/**
 * Used to monitor system state and communicate with the main loop.
 * ISR modify the System_runtime_flag to report events
 * The main loop checks the state of the System_runtime_flag and executed aproperiate actions.
 */
typedef union {
	struct {
		uint16_t	Pending_Uart0_NewLine 	: 1;
		uint16_t	Pending_Uart1_NewLine	: 1;
		uint16_t	Pending_Software_Timer	: 1;
		uint16_t	Pending_DCO_Tune		: 1;
		uint16_t	Pending_AppLayer_Task	: 1;
		uint16_t	Pending_WakeUp_Request	: 1;
		uint16_t	Edge_Processing_Uart0	: 1;
		uint16_t	Edge_Processing_Uart1	: 1;
		uint16_t	Interrupt_Radio			: 1;
	} Flag; ///< Access to flags.
	uint16_t AllFlags; ///< Access to all flags at once
} system_flags_t;

volatile system_flags_t SystemFlags;

/**
 * @brief	Configures the DCO and tries to adjust the MCLK to the specified target frequency
 */
uint16_t System_SetDCO( uint32_t target_frq, uint32_t min_frq, uint32_t max_frq );

/**
 * @brief	Random number generator
 * 
 * @returns random number between 0 and k-1
 */
uint16_t irand(uint16_t k);

#endif /*SYSTEM_H_*/


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
