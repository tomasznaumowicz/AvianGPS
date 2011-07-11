#include "Application.API.h"

#include "FSM.h"
#include "FSM_Blink.h"

// 1. Define all actions here.
//    This could happen in the header file, but you might like to keep those functions local.
//    This definition is required for the definition of the transition table

void LedTurnOn();
void LedTurnOff();

// 2. Define states you are going to use in your state machine
enum FSM_Blink_States {
	FSM_LedIsOn						= 0,
	FSM_LedIsOff 					= 1
};

// 3. Count your states
#define FSM_Blink_States_Count		2

// 4. Define events used in your state machine
//    Leave 'Timeout' at the beginning
enum FSM_Blink_Events {
	Timeout							= 0,
	ForceLedTurnOff					= 1
};

// 5. Count your events
#define FSM_Blink_Events_Count 		2

// 6. Define timeouts for your states
//    When no event happens within the timeout value, the state change will be executed (signalled as timeout)
//    0 stands for no timeout.
uint32_t fsm_blink_timeouts[FSM_Blink_States_Count] = {
		TICKS_5SECONDS,				// FSM_LedIsOn
		TICKS_5SECONDS				// FSM_LedIsOff 		
};

// 7. Define your state transitions.
//    Horiziontal:	current state
//    Vertical:		the event
//    Entry:		Action to call and the next state.
 FSM_Entry fsm_blink_transitions[FSM_Blink_Events_Count][FSM_Blink_States_Count] = {
/*							FSM_LedIsOn						FSM_LedIsOff					*/
/* Timeout			*/	{	{ LedTurnOff, FSM_LedIsOff }, 	{ LedTurnOn, FSM_LedIsOn } 	},				
/* ForceLedTurnOff	*/ 	{	{ LedTurnOff, FSM_LedIsOff }, 	{ NULL, 0 } 				}		
};

// 8. Initialize the Finite State Machine (FSM)
FSM_NEW(fsm_blink, FSM_Blink_States_Count, FSM_Blink_Events_Count, fsm_blink_transitions, fsm_blink_timeouts);



void FSM_Blink_Init() {
	// prepare state machine init args
	fsm_startparameters_t init;
	
	init.FSM 			= &fsm_blink;
	init.Delay 			= TICKS_SECOND;
	init.InitialState 	= FSM_LedIsOff;

	// start the fsm
	FSM_Start(&init);
}

void LedTurnOn() {
	LED2_ON;
}

void LedTurnOff(){
	LED2_OFF;
}

COMMAND(force, "Forces the LED to disable", args) {
	// perform FSM transition
	FSM_ProcessEvent(&fsm_blink, ForceLedTurnOff);
}
	



#ifndef DOXYGEN_PUBLIC_DOC
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
