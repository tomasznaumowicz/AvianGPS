/**
 * @file		Timers.API.h
 * 
 * @section idotsti Internal description of the software timers implementation:
 * 
 * TimerA runs with ACLK as clock source, ACKL runs with 32768 Hz, divided by 4 resulting in 8192 ticks / second
 * 
 * Overflow interrupt is used to update the system time by 8 seconds (overflow every 0xFFFF ticks == 65535 == 8 * 8192)
 * and to process software timers.
 * 
 * TACCR0 is used for the blocking delay function, no interrupt support is required, a loop waits for the CCIFG flag to be set.
 * 
 * TACCR1 is used for the timers software support.
 *
 * @b Timers: registered timers are stored as a tuple (ticks, timerHandler, timerId) and executed as soon as specified number of ticks gets counted
 * The timers are checked on every overflow of the TAR every 8 seconds. if a timer is scheduled to be processed before the next TAR overflow,
 * then the TACCR1 is used. It's configured to process the ISR after the remaining ticks.
 * Timers relay on a _virtualTAR variable, which is used to keep track of TAR.
 * This allow to use timers with delay bigger then uint16_t (65535 ticks==8 seconds), the _virualTAR is uint32_t *
 * 
 * @section strah Software timers registration and handling:
 * 
 * Timers are added via \ref Timers_Add function, number of ticks is specified, after the number of ticks is counted,
 * the timerHandler should be invoked. The timerHandler will be invoked outside of an interrupt --> a flag will be 
 * set, the main loop has to invoke the the function "Timers_ProcessPendingTimerHandler()". It will invoke the timerHandler.
 * 
 * Timers are stored in a _timers array (called collection)
 * 
 * Timers_Add adds new entry to the collection, the entry in the first row of the table has to be invoked as the first one.
 * Timers_Add has to check whether it shoud be placed first or not.
 * The entry in the table contains the number of ticks until the handler invocation. _virtualTAR is used as time base.
 *
 * @section e Errata 
 *  
 * 		This code uses workaround documented in MSP430F1611 errata   
 * 		http://focus.ti.com/lit/er/slaz018b/slaz018b.pdf
 * 
 * 		TA12 - Bug description:
 * 		Module: Timer_A, Function: Interrupt is lost (slow ACLK)
 * 		Timer_A counter is running with slow clock (external TACLK or ACLK) compared to MCLK. The
 * 		compare mode is selected for the capture/compare channel and the CCRx register is incremented
 * 		by 1 with the occurring compare interrupt (if TAR = CCRx).
 * 		Due to the fast MCLK the CCRx register increment (CCRx = CCRx + 1) happens before the
 * 		Timer_A counter has incremented again. Therefore, the next compare interrupt should happen at
 * 		once with the next Timer_A counter increment (if TAR = CCRx + 1). This interrupt gets lost.
 * 		
 * 		Workaround:
 * 		Switch capture/compare mode to capture mode before the CCRx register
 * 		increment. Switch back to compare mode afterwards. 
 */ 

#include <stdint.h>
#include <signal.h>
#include <stdio.h>

#include "Logging.h"
#include "RTC.h"
#include "Commands.h"
#include "System.h"

#include "Timers.h"

#ifndef DOXYGEN_PUBLIC_DOC

/**
 * Used to support software timers.
 * 
 * used to hold the sum of the ticks of the TimerA, reduces the number of operations required in the ISR
 * this a virtual TAR with 32 bit length, it gets reseted after every timerHandler invocation
 * in that case all entries are normalized to the new _virtualTAR after the reset
 * _virtualTAR is updated on every TAR overflow, in the case when the exact number of ticks is required
 * since the last normalisation, the TAR and the _virtualTAR have to be added. 
 */
static volatile uint32_t	_virtualTAR;

/**
 * Entry in the collection of active software timers.
 */
typedef struct {
	uint32_t 	remainingTicks;		///< ticks to be counted before the timer handler gets executed
	fp_timer_t 	timerHandler; 		///< pointer to the timer handler
	int16_t		timerId;			///< the ID of the timer entry (might be used to remove a timer)
	int16_t		data;				///< a 16bit value can be passed to the timer handler
} timers_collection_entry_t;

/**
 * Collection of active software timers.
 * Invariant: the first entry has alwas the lowest remainingTicks value in the collection.
 * Modify the TIMERS_MAX_TIMERS_COUNT if more software timers are required.
 */
static timers_collection_entry_t _timers[TIMERS_MAX_TIMERS_COUNT];

/**
 * Number of entries stored currently in the _timers collection
 */
volatile uint8_t _timers_count;

/**
 * Holds the last assigned timerId, this value is incremented every time an timerId is assigned
 */
static int16_t	_lastAssignedTimerId;


/**
 * holds the total number of ticks since power on (POR == power on reset), incremented on every overflow of the TAR by the value of 0xFFFF ticks
 * used to provide the uptime of the device.
 */
static volatile uint32_t _ticks_since_POR_backend;

/**
 * this is a helper variable: after the current number of ticks gets requested, the _ticks_since_POR_backend and current value of the TAR are added
 * and the address of _ticks_since_POR_current is returned;
 */ 
static uint32_t _ticks_since_POR_current;

/**
 * Reads and decrements _virtualTAR, decrements all timers[i].remainingTicks accordingly.
 */
static void _Timers_ReduceCounters() {
  uint16_t i;
  
  // Reset the _virtualTAR. But: it is possible that this gets interrupted by the
  // ISR for the TimerA overflow. This has to be synchronized with the interrupts.
  // Good and safe way to synchronize with the interrupt without disabling it:
  // First read ticks and then subtract it. Both operations are atomic, if
  // they get interrupted in between no tick will get lost, it will stay
  // in ticks due to the subtract.
  
  uint32_t virtualTARcopy = _virtualTAR;
  _virtualTAR -= virtualTARcopy;
  
  if (virtualTARcopy == 0)						// nothing happened, the _virtualTAR was not modified
	  return;
  
  // subtract the _virtualTAR from all entries in the _timers collection
  for (i = 0; i < _timers_count; i++) {
    if (_timers[i].remainingTicks > virtualTARcopy)
    	_timers[i].remainingTicks -= virtualTARcopy;
    else
    	_timers[i].remainingTicks = 0; // should be executed
  }
}

/**
 * This function should be called to invoke pending timerHandlers.
 * It should be invoked outside of an interrupt when the "pending timers" flag is set.
 */
void Timers_ProcessPendingTimerHandler() {
	uint16_t i = 0;
	
	_Timers_ReduceCounters();								// use the time outside of an interrupt to clean normalize the counters
	
	uint16_t _TAR_Copy = COPY_CURRENT_TAR;

	uint32_t now = _virtualTAR + _TAR_Copy;

	while (i < _timers_count) {
		if (_timers[i].remainingTicks <= now) {					// this entry should be processed
			fp_timer_t timerHandler = _timers[i].timerHandler;	// copy the pointer to the timerHandler
			uint16_t timerData = _timers[i].data;
			_timers[i] = _timers[_timers_count - 1];			// remove the expired timer from the collection
			_timers_count--;
			
			// this is critical section: user code is executed and it is possible, that timers were added or removed:
			// the _timers collection could have changed, the 'now' changed, the order of entries in the table changed
			timerHandler(timerData);										// execute the timerHandler !, this blocks me, update _virtualTimer when done
			
		}
		else {													// if not, test the next one
			i++;
		}
	}
	
	// clean up the _timers collection
	if (_timers_count > 1) {									// select the new minimum to execute next
		uint8_t smallest = 0;
		for (i = 1; i < _timers_count; i++) { 
			if (_timers[i].remainingTicks < _timers[smallest].remainingTicks)
				smallest = i;
		}
		// 'smallest' contains the index of the smallest value
		// and if it's not the one that already is stored at the index 0, then replace:
		if (smallest != 0) {
			timers_collection_entry_t temp = _timers[0];
			_timers[0] = _timers[smallest];
			_timers[smallest] = temp;
		}
	}
	
	if (_timers_count > 0) {
		// the timers are checked on every TAR overflow (every 8 seconds)
		// if the first timer has to execute eariler, the TACCR1 should be used
		if ((_virtualTAR + 0x0000FFFF) > _timers[0].remainingTicks) {		// the first timer should be executed _before_ the overflow of the TAR
			uint32_t tmp = _timers[0].remainingTicks - _virtualTAR; 
			TACCTL1 |= CAP;
			TACCR1 = (uint16_t) tmp;								// setup a TACCR1 so that an ISR gets activated
			TACCTL1 &= ~CAP;
			TACCTL1 |= CCIE;
			
			// it is possible, that the timer expired already!
			// just set the CCIFG flag now.
			_TAR_Copy = COPY_CURRENT_TAR;

			if (TACCR1 <= _TAR_Copy) {
				TACCTL1 |= CCIFG;
				TACCTL1 |= CCIE;
			}
		}
	}
}

/**
 * Timer A1 interrupt service routine
 * This ISR serves CC1, CC2 and TA
 * TA is the interrupt on the overflow
 * (0xFFEA Timer A CC1-2, TA)
 */
interrupt (TIMERA1_VECTOR) Timer_A(void)
{
	if ((TACCTL1 & CCIFG) && (TACCTL1 & CCIE)) {	// a timer gets executed before the TAR overflow, no real special handling needed:
		TACCR1 = 0x0000;
		TACCTL1 &= ~CCIE;							// disable the interrupt for TACCR1
		TACCTL1 &= ~CCIFG;							// clear the interrupt pending flag
		
		// I have to check whether the 1st entry in the _timers collection should be activated or not
		// BUT: this is critical section: TimerA interrupts are disabled only if there is no other solution in order to achieve accurate timing
		// The TACCR is set up based on a computation of remainingTicks and current virtualTAR and TAR: TACCR1 ISR will not be activated if there
		// is no software timer scheduled for that time.
		// previous version checked, if the first entry in the _timers collection indeed expired. it didn't take into account
		// that in the meantime the collection may be edited by either deleting or adding a timer.
		// In that case it is possible, that the _timers collection will be reordered.
		// The other case is the actual timer execution: a timer is selected to be processed: in that moment the last timer from the table
		// is copied into its slot in the _timers table: this is the one with the index 0.
		// the table is reordered afterwards. any operation in the meantime may produce false results: like here
		//
		//		if (_timers_count > 0 && (_virtualTAR + _TAR_Copy) >= _timers[0].remainingTicks) {
		//			System_runtime_flag = 1;
		//		}
		//
		// the _timers[0].remainingTicks is not the one that should be analysed.
		// Two options to resolve the problem:
		//		never break the invariant of the _timers collection (better, but slower operations)
		//		don't check the invariant here, the microcontroller will wake up and process the timers, since it happens outside of the interrupt, the array will be ordered already
		SystemFlags.Flag.Pending_Software_Timer = true;

		END_LPM_WITH_CONDITION_CHECK;

		return;
	}
	
	
	if (TACTL & (TAIFG | TAIE)) {					// overflow of the TimerA: do periodic tasks:
		//every 8 seconds, the node shall blink:
		HW_LED1_ON;
		
		SystemFlags.Flag.Pending_DCO_Tune = true;
		// ask for tune of the DCO now as min 8 seconds elapsed since last tune operation
		// but don't end LPM, just leave the DCO tune scheduled. it won't wake up the device
		// for a tune, but will execute it as soon as the device wakes up for some reason.

		if (SystemFlags.AllFlags) {				// a task is queued
			// but ignore Pending_DCO_Tune flag
			system_flags_t Only_DCO_Tune;
			Only_DCO_Tune.AllFlags = 0;
			Only_DCO_Tune.Flag.Pending_DCO_Tune = true;
			
			if (SystemFlags.AllFlags != Only_DCO_Tune.AllFlags)
				END_LPM;								// there was a bug where the system_state.POWERDOWN was low but the node was in LPM3. this is a brute force test every 8 seconds with a wakeup without the condition check
		}
		
		TACTL &= ~TAIFG;							// clear the interrupt pending flag

		_virtualTAR += 0x0000FFFF;					// overflow behavior was analysed.
		_ticks_since_POR_backend += 0x0000FFFF;

		// process the RTC tasks
		RTC_ProcessTAI();
		
		// process the software timers tasks 
		
		// I have to check whether the 1st entry in the _timers collection should be activated or not
		if (_timers_count > 0) {
			if(_virtualTAR >= _timers[0].remainingTicks) {
				SystemFlags.Flag.Pending_Software_Timer = true;
					END_LPM_WITH_CONDITION_CHECK;				
			}
			else {
				if (_timers[0].remainingTicks < (_virtualTAR + 0x0000FFFF)) {	// activity earlier than the overflow
					uint32_t tmp = _timers[0].remainingTicks - _virtualTAR;
					
					/*
					 * sometimes the tmp is very small ( == 1) in this case we might miss the TAR==1 and then, the next execution of the ISR
					 * happenes after next overflow, 8 seconds too late. additionaly the TACCR1 will activate at the TAR == 1, this has no use
					 * and 8 seconds are lost..
					 * 
					 * this is a documented bug of the MSP430F1612:
					 * this bug is relevant when TAR == 0 and TACCR1 should be set to 1
					 * 
					 * source:
					 * http://focus.ti.com/lit/er/slaz018b/slaz018b.pdf
					 * 
					 * TA12 - Bug description:
					 * Module: Timer_A, Function: Interrupt is lost (slow ACLK)
					 * Timer_A counter is running with slow clock (external TACLK or ACLK) compared to MCLK. The
					 * compare mode is selected for the capture/compare channel and the CCRx register is incremented
					 * by 1 with the occurring compare interrupt (if TAR = CCRx).
					 * Due to the fast MCLK the CCRx register increment (CCRx = CCRx + 1) happens before the
					 * Timer_A counter has incremented again. Therefore, the next compare interrupt should happen at
					 * once with the next Timer_A counter increment (if TAR = CCRx + 1). This interrupt gets lost.
					 * 
					 * Workaround:
					 * Switch capture/compare mode to capture mode before the CCRx register
					 * increment. Switch back to compare mode afterwards.
					 */
					
					TACCTL1 &= ~CCIFG;							// clear the interrupt pending flag if there is any
					TACCTL1 |= CAP;
					TACCR1 = (uint16_t) tmp;					// setup a TACCR1 so that an ISR gets activated
					TACCTL1 &= ~CAP;
					TACCTL1 |= CCIE;
				}
			}
		} // if [ timers count > 0 ]
		
		HW_LED1_OFF;
	
	} // if [ overflow of the TimerA: the periodic task ]
}

#endif // doxygen public doc





















uint32_t* TICKS_SINCE_POR() {
	_ticks_since_POR_current = _ticks_since_POR_backend + COPY_CURRENT_TAR;
	
	return &_ticks_since_POR_current;
}

void Timers_Block(uint16_t ticks) {
	//realised via capture/compare without interrupt
	if (ticks == 1) {
		uint16_t loop;
		for(loop = 0; loop < 147; loop++) {
			_NOP();
		}
		return;
	}
	
	TACCTL0 &= ~CCIFG;									// clear the CCIFG flag
	
	uint16_t _TAR_Copy = COPY_CURRENT_TAR;

	TACCR0 = _TAR_Copy + ticks;							// use CCR0 to be informed when 'ticks' will be reached
	
	while (!(TACCTL0 & CCIFG)) {						// break when the CCIFG flag is set (TACCR0 value reached)
		_NOP();											// empty loop
		//NOTE: we could optimize this a bit and go into LPM1 at least, be careful with LPM3 due to possible UART conficts!
	}

	TACCTL0 &= ~CCIFG;									// clear the CCIFG flag
}

int16_t Timers_Add(const fp_timer_t timerHandler, uint32_t ticks, uint16_t data) {
	// safety check
	if (_timers_count >= TIMERS_MAX_TIMERS_COUNT) {
		LOGERR("Timers_Add: FULL");
		/*
		uint8_t index;
		for (index = 0; index < _timers_count; index++) {
			uint32_t tar_copy = COPY_CURRENT_TAR;
			uint32_t inTicks = _timers[index].remainingTicks - _virtualTAR - tar_copy; 
			LOG(" %u: %p in %lu ticks\r\n", index + 1, _timers[index].timerHandler, inTicks);
		}
		*/
		return -1;
	}
	
	uint16_t _TAR_Copy = COPY_CURRENT_TAR;

	// increase the _lastAssignedTimerId since a new one has will be assigned and there is empty space in the table
	_lastAssignedTimerId++;
	//remember to handle handle overfows!
	if (_lastAssignedTimerId < 0)
		_lastAssignedTimerId = 1;
	
	// when shoud the registered timer be executed? use _virtualTAR as time base
	uint32_t executeAtVirtualTAR = _virtualTAR + _TAR_Copy + ticks;
	
	if (executeAtVirtualTAR < _virtualTAR) {
		// overflow, the _virtualTAR and the entries in the _timers collestion have to be normalized
		_Timers_ReduceCounters();
		
		// update the new entry for the _timers collection
		executeAtVirtualTAR = _virtualTAR + _TAR_Copy + ticks;
	}
	
	// there are no registered timers or the the new entry should be executed earlier than all other in the table
	if (_timers_count == 0 || executeAtVirtualTAR < _timers[0].remainingTicks) {
		//the new timers has to be executed first
		// --> reorder
		_timers[_timers_count] = _timers[0];			// if _timers_count == 0 then this operation does nothing.

		_timers[0].remainingTicks = executeAtVirtualTAR;
		_timers[0].timerHandler = timerHandler;
		_timers[0].timerId = _lastAssignedTimerId;
		_timers[0].data = data;
		
		// the timers are checked on every TAR overflow (every 8 seconds)
		// if the first timer has to execute eariler, the TACCR1 should be used
		// first: check whethter the computation is required
		if (ticks < 0x0000FFFF) {								// the timer should be processed earlier than the latest possible TAR overflow (0xFFFF == 8 seconds)

			// setup TACCR1
			if (_TAR_Copy + ticks > _TAR_Copy) {				// the first timer should be executed _before_ the overflow of the TAR
				TACCTL1 |= CAP;				
				TACCR1 = _TAR_Copy + ticks;						// setup a TACCR1 so that an ISR gets activated
				TACCTL1 &= ~CAP;
				TACCTL1 |= CCIE;
	
				if (TACCR1 <= _TAR_Copy) {
					TACCTL1 |= CCIFG;
					TACCTL1 |= CCIE;
				}	
			}
			
		}
		
	}
	else {
		//the new timer has to be executed later, after the first entry in the _timers collection
		// --> just store in the _timers collection
		_timers[_timers_count].remainingTicks = executeAtVirtualTAR;
		_timers[_timers_count].timerHandler = timerHandler;
		_timers[_timers_count].timerId = _lastAssignedTimerId;
		_timers[_timers_count].data = data;
	}

	_timers_count++;
	
	// if the new entry should be executed in less earlier then the next overflow, an ISR should be scheduled here.
	return _lastAssignedTimerId;
}

bool Timers_Remove(int16_t timerId) {
	uint8_t index;
	
	for (index = 0; index < _timers_count; index++) {
		if (_timers[index].timerId == timerId) {
			//move overwrite it with the last entry from the collection and reduce the counter
			_timers[index] = _timers[_timers_count-1];
			_timers_count--;
			// the timer entry was removed, now the list has to be consistend again:
			// the first entry has to have the smallest .ticks value
			// this has to be checked only if the _timers_count > 1 and the entry with the index 0 was removed:
			
			if (index == 0) {
				// the timer with the index 0 has to be removed. disable TACCR1 in this case
				TACCTL1 &= CCIE;
				TACCR1 = 0x0000;
			}
			
			if (index == 0 && _timers_count > 1) {
				//special case: the timer that was removed was the first one and there are others in the list
				//the next timer to be executed is somewhere in the table and has to be found now and placed at the index 0
				//we can re-use the index variable since we will return after the timers get replaced
				uint8_t smallest = 0;
				for (index = 1; index < _timers_count; index++) { 
					if (_timers[index].remainingTicks < _timers[smallest].remainingTicks)
						smallest = index;
				}
				// 'smallest' contains the index of the smallest value
				// and if it's not the one that already is stored at the index 0, then replace:
				if (smallest == 0) {
					timers_collection_entry_t temp = _timers[0];
					_timers[0] = _timers[smallest];
					_timers[smallest] = temp;
				}
			}
			
			if (index == 0 && _timers_count > 0) {
				
				// the timer was removed and some other one was moved to the index 0
				// it is possible that it has to be activated before the TAR overflow
				// the timers are checked on every TAR overflow (every 8 seconds)
				// if the first timer has to execute eariler, the TACCR1 should be used
				// first: check whethter the computation is required

					if ((_virtualTAR + 0xFFFF) > _timers[0].remainingTicks) {		// the first timer should be executed _before_ the overflow of the TAR
						uint32_t tmp = _timers[0].remainingTicks - _virtualTAR; 
						TACCTL1 |= CAP;
						TACCR1 = (uint16_t) tmp;								// setup a TACCR1 so that an ISR gets activated
						TACCTL1 &= ~CAP;
						TACCTL1 |= CCIE;
					}			
			}
			
			// the entry was removed, the table didn't have to be corrected
			return true;
		}
	}
	
	// the timer with the timerId was not found
	return false;
}

void Timers_Init() {
	/**
	 * 1. preset the RTC with 0 as time
	 * 2. clean up table with registered timers
	 * 
	 * 
	 * TimerA Setup:
	 * TimerA is used to drive the RTC and support software timers
	 * 
	 * Timer_A is configured in the continous mode.
	 * It will be clocked via the ACLK (supplied by the XT: External 32kHz quartz is attached (32768Hz))
	 * The ACLK is divided by 4, this gives 8192 ticks every second, a tick every 122us.
	 * 
	 * The Interrupt will be raised after the timer counts from 0xFFFF to 0x0000, this is after:
	 * 		65535 + 1 = 65536 ticks of the clock
	 * 		65536 ticks of the clock == 8 seconds with 8192 ticks every second
	 * 
	 * RTC Support: 	the _CurrentTime value from the RTC.c is updated on every overflow of the TimerA.
	 * 					The overflow occurs every 8 seconds.
	 * 					When the applications tries to read the current time, the TAR has to be read
	 * 						(be aware of race conditions explained in the user guide - this code takes care of this)
	 * 
	 * Software timers:	the table with timers will be checked on every overflow
	 * 					should a timer be scheduled for a time before the overflow, an match register TACCR1 will be used
	 */
	
	////////////////////////////////////////////////////////// Reset the TimerA
	TACCTL0 = 0x00;											// clear all capture/compare entries
	TACCTL1 = 0x00;
	TACCTL2 = 0x00;
	
	TACTL = TACLR;											// reset the TimerA
	
	////////////////////////////////////////////////////////// Setup the TimerA
	TACTL = TASSEL_ACLK | ID_DIV4 | MC_CONT;				// Source ACLK, Divide by 4, Continous Mode
	TACTL |= TAIE;											// Enable the TAIE interrupt (overflow)
	
	_ticks_since_POR_backend = 0;
	_lastAssignedTimerId = 1;
}



/**
 * @ingroup Commands
 * @brief	@b timers Debug command: Provides information about registered software timers.
 */
COMMAND(timers, "Debug command: Provides information about registered software timers.", cmd_args) {
	uint8_t index;
	for (index = 0; index < _timers_count; index++) {
		uint32_t tar_copy = COPY_CURRENT_TAR;
		uint32_t inTicks = _timers[index].remainingTicks - _virtualTAR - tar_copy;
		uint32_t inSeconds = inTicks / TICKS_PER_SECOND;
		printf(" %u: %p in %lu ticks (%lu seconds)\r\n", index + 1, _timers[index].timerHandler, inTicks, inSeconds);
	}
	snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength, "%u timer(s)", index);
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
