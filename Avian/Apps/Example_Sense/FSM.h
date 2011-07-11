/**
 * @file		
 * @brief		@b Module: Finite State Machine (FSM) Engine: Implementation of a state machine that can be used to simplify application development. 
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * @note
 * The FSM can simplify implementation of applications for this platform. We used it to built applications for the advanced
 * scenario for Skomer Island 2011 deployment. We implemented the Tracker logic and the Base Station logic using this FSM.
 * We advise to review the documentation first, later our simple examples and optionally, if you're interested, the Tracker and Base Station
 * implementations before building your own applications.
 * 
 * @section FSM_Tutorial Tutorial
 * 
 * You need to copy FSM.h and FSM.c files into your Application folder as those two files implement the FSM functionality.
 * You might want to enable FSM debugging for a while while working on your FSM implementation, simply modify the value of the \ref DEBUG_FSM_ENABLED definition.
 * 
 * The FSM built in this tutorial will implement a simple LED blink engine. On of the LEDs on the AvianGPS board will be enabled and disabled either by
 * a Timeout event or by an event raised by the user code. This will showcase two ways of influencing the behavior of an FSM.
 * 
 * The FSM will switch between two states:
 *  - LED is Off, and
 *  - LED is On.
 * 
 * The FSM will start in the "FSM is Off" state and per default, the transitions between states will be caused by Timeout events. Besides that,
 * the user has the option to force a transition from the "FSM is On" state to the "FSM is Off" state by raising a ForceLedTurnOff event from
 * the user code.
 * 
 * The follwing graph shows the visualisation of states, events and valid state transitions:
 * 
 * @dot
 * digraph fsm {
 * 		graph [fontname=Helvetica, fontsize=10];
 * 		edge [fontname=Helvetica, fontsize=10];
 * 
 *		node [shape=doublecircle, fontname=Helvetica, fontsize=10];
 * 		led_is_off					[label = "LED is Off"];
 * 
 *		node [shape=circle, fontname=Helvetica, fontsize=10];
 * 		led_is_on					[label = "LED is On"];
 * 
 * 		led_is_on -> led_is_off		[label="   Timeout   \n   Turn Off the LED   ", style="dashed"];
 * 		led_is_off -> led_is_on		[label="   Timeout   \n   Turn On the LED   ", style="dashed"];
 * 
 * 		led_is_on -> led_is_off		[label="   'force' command   \n   Turn Off the LED   "];
 * }
 * @enddot
 * 
 * @subsection fsmt1 1. Preparing your FSM implementation files
 * 
 * Next, you should create header (h) and implementation (c) files for your FSM in the Application folder, in this tutorial following file names will
 * be used:
 *  - FSM_Blink.h (the header file)
 *  - FSM_Blink.c (the implementation file)
 * 
 * @subsection fsmt2 2. Preparing the header file
 * 
 * Put the following content in your FSM_Blink.h:
 * @code
 * #ifndef FSM_BASE_H_
 * #define FSM_BASE_H_
 * 
 * #include "Application.API.h"
 * 
 * void FSM_Blink_Init();
 * #endif
 * @endcode
 * 
 * This will make the definition of the @b FSM_Blink_Init function available. Such initialization function
 * for your FSM is required: it configures the FSM and starts the state machine. You might add other functions
 * later, e.g. function like @b FSM_Blink_Abort might be useful for more advanced appliations.
 * 
 * @subsection fsmt3 3. Preparing the implementation file
 *  
 * The implementation file will contain the definitions of FSM states, events and transitions. It will also contain the implementation of action functions.
 * Open the FSM_Blink.c file and initialize it with the following content:
 * 
 * @code
 * #include "Application.API.h"
 * #include "FSM.h"
 * #include "FSM_Blink.h"
 * // content created during steps 4.1 - 4.9 comes here after the last #include statement.
 * void FSM_Blink_Init() {
 * }
 * @endcode
 * 
 * @subsection fsmt4 4. Defining the FSM
 * 
 * In order to configure your FSM, following steps need to be completed:
 * 
 * @subsubsection fsmt41 4.1. Define headers of all actions
 * 
 * This could be dcone in the FSM_Blink.h header file, but you might like to keep those functions local.
 * Definitions need to be prepared as they are required when working with the transitions table later.
 * 
 * Follwing action functions will be defined here:
 * @code
 * void LedTurnOn();
 * void LedTurnOff();
 * @endcode
 * 
 * @subsubsection fsmt42 4.2. Define states you are going to use in your state machine
 * 
 * This simply defines the available FSM states. The FSM implemented here can have only two states.
 * 
 * @note It's important to define the numerical values behind the enum and pay attention to the order of states later. 
 * 
 * @code
 * enum FSM_Blink_States {
 * 	FSM_LedIsOn			= 0,
 * 	FSM_LedIsOff 			= 1
 * };
 * @endcode
 * 
 * @subsubsection fsmt43 4.3. Count your states
 * 
 * Simply count the number of states just defined:
 * 
 * @code
 * #define FSM_Blink_States_Count		2
 * @endcode
 * 
 * @subsubsection fsmt44 4.4. Define events used in your state machine
 * 
 * Define the list of possible events that your FSM should be able to process.
 * 
 * @note
 *  -# The first event that's defined must be the @b Timeout event and it must have the numerical value 0.
 *  -# It's important to define the numerical values behind the enum and pay attention to the order of events later. 
 * 
 * @code
 * enum FSM_Blink_Events {
 * 	Timeout				= 0,
 * 	ForceLedTurnOff			= 1
 * };
 * @endcode
 * 
 * @subsubsection fsmt45 4.5. Count your events
 * 
 * Simply count the number of events just defined:
 * 
 * @code
 * #define FSM_Blink_Events_Count 	2
 * @endcode
 * 
 * @subsubsection fsmt46 4.6. Define timeouts for your states
 * 
 * Many applications require support for automatic state transitions after a specified timeout. The FSM engine contains support for this feature.
 * In this step you can define the timeout values.
 * 
 * @note 
 *  -# The order of timeout values must correspond to the order of states within the definition above from the step 4.2
 *  -# 0 stands for indefinite disabling an automatic transition.
 *  -# Timeout values can be modified at runtime.
 * 
 * In this example, automatic state transitions will happen every 5 seconds.
 * 
 * @code
 * uint32_t fsm_blink_timeouts[FSM_Blink_States_Count] = {
 * 	TICKS_5SECONDS,			// FSM_LedIsOn
 *	TICKS_5SECONDS			// FSM_LedIsOff
 * };
 * @endcode
 * 
 * @subsubsection fsmt47 4.7. Define your state transitions
 * 
 * This step defines the behavior of your FSM. It allows you to define the set of valid transition between states and to wire action functions
 * to those valid transitions.
 * 
 * The definition is a simple array: columns stand for the current state and rows stand for an event to process. Each entry defines
 * the next state for the combination of the state + event. An entry also defines what action should be performed @b after @b the @b state @b transition.
 * Setting the action value to @b NULL marks a transition as invalid.
 * 
 * Please note that the size of the array is defined using the @b FSM_Blink_Events_Count and @b FSM_Blink_States_Count definitions.
 * 
 * @code
 * FSM_Entry fsm_blink_transitions[FSM_Blink_Events_Count][FSM_Blink_States_Count] = {
 * 	//  FSM_LedIsOn			  FSM_LedIsOff			
 * 	{ { LedTurnOff, FSM_LedIsOff }, { LedTurnOn, FSM_LedIsOn } 	},	// Timeout
 * 	{ { LedTurnOff, FSM_LedIsOff }, { NULL, 0 } 			}	// ForceLedTurnOff
 * };
 * @endcode
 * 
 * Take a look at the first entry in the table above: { LedTurnOff, FSM_LedIsOff }
 * 
 * Because of it's position in the first column and first row, it describes the behavior of the FSM when it's in the
 * FSM_LedIsOn state and receives the @b Timeout event. The entry says that the @b LedTurnOff action should be executed
 * and that a transition to the @b FSM_LedIsOff state should be performed.
 * 
 * On the other hand, the entry in the second column and the second row: { NULL, 0 }
 * 
 * This entry will be evaluated when the FSM is in the state @b FSM_LedIsOff and receives the @b ForceLedTurnOff event. As the
 * entry contains a @b NULL as an associated action and means that the transition is invalid, the event will be ignored by the FSM engine. 
 * 
 * @subsubsection fsmt48 4.8. Initialize the Finite State Machine (FSM)
 * 
 * Now, the FSM needs to be created. Simply select a name for the FSM and use the \ref FSM_NEW macro ("constructor"). In this example, @b fsm_blink was selected
 * as the name of the FSM.
 * 
 * @code
 * FSM_NEW(fsm_blink, FSM_Blink_States_Count, FSM_Blink_Events_Count, fsm_blink_transitions, fsm_blink_timeouts);
 * @endcode
 * 
 * @subsubsection fsmt49 4.9. Implement the initialization function
 * 
 * Finally, you need to implement the @b FSM_Blink_Init function that will initialize and start the FSM.
 * 
 * @code
 * void FSM_Blink_Init() {
 * 	// prepare state machine init args
 * 	fsm_startparameters_t init;
 * 	init.FSM 		= &fsm_blink;
 * 	init.Delay 		= TICKS_SECOND;
 * 	init.InitialState 	= FSM_LedIsOff;
 * 	// start the fsm
 * 	FSM_Start(&init);
 * }
 * @endcode
 * 
 * @subsection fsmt410 4.10. Implement the user code that sends an event to the FSM
 * 
 * @code
 * COMMAND(force, "Forces the LED to disable", args) {
 * 	// perform FSM transition
 * 	FSM_ProcessEvent(&fsm_blink, ForceLedTurnOff);
 * }
 * @endcode
 * 
 * @subsection fsmt411 4.11. Implement the action functions
 * 
 * @code
 * void LedTurnOn() {
 * 	LED2_ON;
 * }
 * 
 * void LedTurnOff(){
 * 	LED2_OFF;
 * }
 * @endcode
 * 
 * 
 * @subsection fsmt5 5. Using your FSM
 * 
 * Simply call the FSM_Blink_Init function somewhere in your application code, e.g. from the \ref Application_Init function.
 */


#ifndef FSM_H_
#define FSM_H_

#include "Application.API.h"

/**
 * @name		Default configuration.
 * 
 * You can overwrite this configuration by putting the same defines with adjusted values in your \ref AppConfig.h
 * 
 * @{
 */

/**
 * @brief		Controls the Finite State Machine Engine. Select the name of the define for more details.
 * 
 * States whether additional debug output is provided by the FSM Engine while performing transitions. Select 1 to enable and 0 to disable the debug output.
 */
#ifndef DEBUG_FSM_ENABLED
#define		DEBUG_FSM_ENABLED		1
#endif

/** @} */

/**
 * @brief		Definition of the type of an action function.
 */
typedef void (*fsm_action)(void);

/**
 * @brief		Defines description of an entry in the FSM definition table. Used internally by the FSM engine.
 */
typedef struct {
	fsm_action Action; ///< The action associated with a transition within the FSM
	uint16_t NextState; ///< The next state of the transition within the FSM
} FSM_Entry;

/**
 * @brief		Defines the internal structure of the state of the Finite State Machine. Used internally by the FSM engine.
 * 
 * You shouldn't access fields of the state directly but rather pass a pointer of the instance to the FSM management functions.
 * 
 * @note
 * There is one case where @b read @b access to fields defined here might useful: if a similar action should be performed
 * for more transitions and when the action depends on the ( \ref PreviousState , \ref CurrentState ) tuple. You can
 * access those fields from your action function implementation.
 */
typedef struct {
	uint16_t CurrentState; 						///< Current state of the FSM
	uint16_t PreviousState;						///< The state the FSM switched from: useful when action differs a bit depending on the (PreviousState,CurrentState) tuple
	int16_t CurrentScheduledTimeout_TimerId; 	///< Holds a timer id of the scheduled Timeout Event
	uint32_t* Timeouts; 						///< Pointer to the timeout definition table which was provided by the user in the FSM_NEW macro
	FSM_Entry* Transitions; 					///< Pointer to the FSM transitions definition table which was provided by the user in the FSM_NEW macro
	uint16_t StatesCount; 						///< Number of all states defined by the user
	uint16_t EventsCount; 						///< Number of all events defined by the user
	fp_timer_t TimeoutHandler; 					///< TimeoutHandler which was auto-generated by the FSM_NEW macro. This TimeoutHandler holds a reference to its FSM
} fsm_t;

/**
 * @brief This macro creates an instance of your FSM.
 * 
 * A number of predefined parameters are required, please refer to examples for more details.
 * 
 * @param[in]	name				The name of the FSM. You'll use this name to refer to your FSM in code later. 
 * @param[in]	states_count		Number of states of the FSM you've defined.
 * @param[in]	events_count		Number of events supported by the FSM you've defined.
 * @param[in]	transitions_table	The core of your FSM definition. The table with transitions and associated actions.
 * @param[in]	timeouts_table		The table with timeout values for the states you've defined.
 * 
 * @note
 * This macro allocates a number of global variables within the file where it's used. Currently those variables aren't static.
 */
#define FSM_NEW(name, states_count, events_count, transitions_table, timeouts_table)\
	void _ ## name ## _timeout_handler(uint16_t event); \
    fsm_t name = {0, 0, 0, timeouts_table, &(transitions_table[0][0]), states_count, events_count, _ ## name ## _timeout_handler}; \
	void _ ## name ## _timeout_handler(uint16_t event) { FSM_ProcessEvent(&name, event); };

/**
 * @brief		Defines start parameters required for the \ref FSM_Start function.
 * 
 * This solution is needed as there is a limit on the number of parameters that can be passed to a single function when working with the mspgcc compiler.
 */
typedef struct {
	fsm_t* FSM;					///< The pointer to the FSM you're about to start.
	uint16_t InitialState;		///< The state the FSM is supposed to be started in.
	uint32_t Delay;				///< The delay in ticks. When the delay timeouts, a @b Timeout @b Event will be sent to the FSM. Use 0 to disable this feature.
} fsm_startparameters_t;

/**
 * @brief Starts the FSM specified in the startParameters
 * The FSM is triggered with a Timeout event.
 * 
 * The InitialState you provide within the startParameters should handle the Timeout event correctly.
 */
void FSM_Start(fsm_startparameters_t* startParameters);

/**
 * @brief	Aborts the processing of the specified FSM.
 * 
 * This function cancels any pending @b Timeout @b Event and as such, stops the processing of the FSM.
 * Any event submitted to the FSM might cause it to operate again. If you require a finite stop state,
 * you must add it to your FSM definition.
 * 
 * @param[in]	fsm		The pointer to the FSM initialized using the \ref FSM_NEW macro.
 */
void FSM_Stop(fsm_t* fsm);


/**
 * @brief Sends an event to the selected FSM.
 * 
 * The FSM engine calls associated action and performs a state transition
 * as defined in the state transitions table.
 * 
 * @param[in]	fsm		The reference to your FSM definition.
 * @param[in]	event	The event that should be sent to the FSM.
 * 
 * @returns		true when transition was possible, false otherwise. 
 */
bool FSM_ProcessEvent(fsm_t* fsm, uint16_t event);

#endif /*FSM_H_*/

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
