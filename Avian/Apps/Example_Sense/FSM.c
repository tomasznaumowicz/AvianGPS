/**
 * @brief		@b Module: Finite State Machine (FSM) Engine: Implementation of a state machine that can be used to simplify application development. 
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * This file contains the implementation of the FSM. See FSM.h for more documentation.
 */
#include "Application.API.h"
#include "FSM.h"


/**
 * The TimeoutHandler is a wrapper around the FSM_ProcessEvent function.
 * The Timers_Add registration allows only one uint16_t variable to pass the state,
 * this is not enough to pass the reference to the fsm_t structure.
 * Either dynamic storage of reference ids had to be created or a wrapper function
 * This wrapper function is created by the FSM_NEW macro. The function has information about the newly created FSM.
 */
void FSM_ScheduleTimeoutEvent(fsm_t* fsm, uint32_t ticks) {
	// remove the scheduled event (if there is any)
	if (fsm->CurrentScheduledTimeout_TimerId  > 0) {
		Timers_Remove(fsm->CurrentScheduledTimeout_TimerId);
		fsm->CurrentScheduledTimeout_TimerId = -1;
	}
	
	if (ticks == 0)
		return;
	
	fsm->CurrentScheduledTimeout_TimerId = Timers_Add(fsm->TimeoutHandler, ticks, 0);
}

bool FSM_ProcessEvent(fsm_t* fsm, uint16_t event) {
	/**
	 * The Transitions table has two dimensions.
	 * but locally, we'll have treat it as an array with only one dimension.
	 * 
	 * Only the pointer to the first element is stored in the fsm_t structure.
	 * The address of the FSM_Entry with coordintates [event_index][state_index] needs to be computed
	 * The address = event_index * state_count + state_index
	 */
	uint16_t index = event * fsm->StatesCount + fsm->CurrentState;
	
	if (fsm->Transitions[index].Action == NULL) {
		// there is no action associated. ignore the action, don't modify the state
		LOGERR_SWITCHED(DEBUG_FSM_ENABLED, "FSM %p: Ignored unexpected event %u while in state %u.", fsm, event, fsm->CurrentState);
		return false;
	}
	
	// remove the scheduled event (if there is any)
	if (fsm->CurrentScheduledTimeout_TimerId > 0) {
		Timers_Remove(fsm->CurrentScheduledTimeout_TimerId);
		fsm->CurrentScheduledTimeout_TimerId = -1;
	}
	
	LOGDBG_SWITCHED(DEBUG_FSM_ENABLED, "FSM %p: Performing 'Action' associated with transition: %u -> %u caused by event %u", fsm, fsm->CurrentState, fsm->Transitions[index].NextState, event);

	// perpare information for the Action
	fsm->PreviousState = fsm->CurrentState;
	// update the current state
	fsm->CurrentState = fsm->Transitions[index].NextState;
	
	// perform the action associated to the event and state
	fsm->Transitions[index].Action();

	if (fsm->Timeouts[fsm->CurrentState] > 0) {
		FSM_ScheduleTimeoutEvent(fsm, fsm->Timeouts[fsm->CurrentState]);
		LOGDBG_SWITCHED(DEBUG_FSM_ENABLED, "FSM %p: Scheduled 'Timeout' of %lus defined by the state %u", fsm, fsm->Timeouts[fsm->CurrentState] / TICKS_PER_SECOND, fsm->CurrentState);
	}

	return true;
}

void FSM_Start(fsm_startparameters_t* initargs) {
	initargs->FSM->CurrentState = initargs->InitialState;
	FSM_ScheduleTimeoutEvent(initargs->FSM, initargs->Delay);
}

void FSM_Stop(fsm_t* fsm) {
	Timers_Remove(fsm->CurrentScheduledTimeout_TimerId);
	fsm->CurrentScheduledTimeout_TimerId = 0;
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
