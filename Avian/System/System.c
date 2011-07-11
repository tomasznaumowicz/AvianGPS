#include <signal.h>
#include <stdlib.h>

#include "BitOperations.h"
#include "Logging.h"
#include "USART.h"
#include "Programming.API.h"
#include "Programming.h"
#include "Watchdog.h"
#include "Watchdog.API.h"
#include "Timers.h"
#include "RTC.h"
#include "Commands.h"
#include "Configuration.h"
#include "Radio/Network.h"

#include "../Drivers/Device.h"

#include "System.h"
#include "System.API.h"

// used to determine whether DCO code can be debugged with printfs etc.
// DCO code shouldn't produce any output during the device initialization because the UART isn't setup yet
bool _dco_debugging;


int RESERVE_RAM(16) main(void){
	////////////////////////////////////// boot sequence
	WDTCTL = WDTPW | WDTHOLD;			// stop watchdog timer for the initialization
	
	////////////////////////////////////// setup hardware ports
	Avian_InitPorts();					// init the AvianGPS board (configure ports)

	LED2_ON;
	LED1_ON;
	
	////////////////////////////////////// Important: clock configuration
	_dco_debugging = false;
		uint32_t frq_on_init = System_MeasureSMCLK();
		
		uint16_t steps = System_SetDCO(FREQUENCY_TARGET, FREQUENCY_LOWER_BOUNDARY, FREQUENCY_UPPER_BOUNDARY);
		
	_dco_debugging = true;

	////////////////////////////////////// done.
	  
	////////////////////////////////////// setup the ACLK to be supplied by the XT: External 32kHz quartz is attached (32768Hz)
	// --> it's preset already after POR/PUC
	////////////////////////////////////// done.

	////////////////////////////////////// basic initialization completed
	_EINT();							// make sure that interrupts are enabled after the initialization

	////////////////////////////////////// initialize pheripherials
	USART_Init(USART0, 115200L);

	////////////////////////////////////// Logging features (LOG, LOGDBG, etc.)
	#if (LOGGING_ENABLED)
		Handler_Logging_Local = Logging_HandlerUart;
	#endif

	////////////////////////////////////// initialize extended firmware componentes and start printing output
	Timers_Init();						// Init the software timers (sets up the TimerA and serves the TimerA interrupts)
	RTC_Init();							// Init the software real time clock  (requires Timers_Init)
	
	Configuration_Init();				// read the configuration settings from the infomem area
	
	////////////////////////////////////// Display application details
	Programming_PrintApplicationHeader();

	////////////////////////////////////// Display configuration details.
	Configuration_Log();
	
	Watchdog_ProcessResetInfo();
	
	LOG("SMCLK: %lu Hz (from %lu Hz in %u steps) [Aimed: %lu (%u), Min: %lu, Max: %lu]",
			System_MeasureSMCLK(),
			frq_on_init,
			steps,
			FREQUENCY_TARGET,
			(uint16_t) (FREQUENCY_TARGET / 32768L),
			FREQUENCY_LOWER_BOUNDARY,
			FREQUENCY_UPPER_BOUNDARY
			);
	
	Network_Init();						// start the radio support
	
	#if (LOGGING_ENABLE_REMOTE_LOG_OUTPUT)
		Handler_Logging_Remote = Logging_HandlerRadio;
		
		extern void Logging_ProcessRemoteLog(uint8_t source, const uint8_t* data);
		Network_RegisterHandler(NETWORK_PROTOCOL_LOG, Logging_ProcessRemoteLog);
	#endif
	
	
	Commands_Init();					// init commands support

	////////////////////////////////////// boot sequence complete.
	
	////////////////////////////////////// Enable the watchdog timer
	Watchdog_Init();
	
	
	HW_LED2_OFF;
	HW_LED1_OFF;
	
	LOG("System init completed.");

	extern void Application_Init(); 
	Application_Init();
	
	LOG("App init completed.");

	// main loop
	while (1) {
		WD_RESET;						// reset the watchdog every time
		
		if (SystemFlags.Flag.Pending_DCO_Tune) {
			SystemFlags.Flag.Pending_DCO_Tune = false;

#ifdef DEBUG_DCO_TUNE
			uint32_t prev = System_MeasureSMCLK();
			
			uint16_t steps = System_SetDCO(FREQUENCY_TARGET, FREQUENCY_LOWER_BOUNDARY, FREQUENCY_UPPER_BOUNDARY);
#else
			System_SetDCO(FREQUENCY_TARGET, FREQUENCY_LOWER_BOUNDARY, FREQUENCY_UPPER_BOUNDARY);
#endif
			
#ifdef DEBUG_DCO_TUNE
			if (steps > 0) {
				uint32_t now = System_MeasureSMCLK();

				LOGDBG("%lu (%u) in %u to %lu (%u)", prev, (uint16_t) (prev / 32768L), steps, now, (uint16_t) (now / 32768L));
			} else {
				LOGDBG("%lu Hz", prev);
			}
#endif
		}
		
		if (SystemFlags.Flag.Pending_Software_Timer) {
			SystemFlags.Flag.Pending_Software_Timer = false;
			
			Timers_ProcessPendingTimerHandler();
		}
		
		if (SystemFlags.Flag.Pending_Uart0_NewLine) {
			SystemFlags.Flag.Pending_Uart0_NewLine = false;
			
			uint8_t input[USART_MAX_RECEIVED_LINE_LENGTH];
			USART0_UART_ReadLine(input);
			
			Commands_ProcessInput(input);
		}
		
		if (SystemFlags.Flag.Pending_Uart1_NewLine) {
			SystemFlags.Flag.Pending_Uart1_NewLine = false;
			
			uint8_t input[USART_MAX_RECEIVED_LINE_LENGTH];
			USART1_UART_ReadLine(input);

			if (UART1_LineReceivedHandler != NULL)
				UART1_LineReceivedHandler(input);
		}

		
		if (SystemFlags.Flag.Edge_Processing_Uart0) {
			// this should prevent the UART from waiting for a character that is not going to be received
			// this problem occurs with our RFID reader: during power on/power off it can produce peaks on the TX/RX line
			// that are detected by the 'edge detect' of the UART. the UART wakes up the controller, the EDGE_DETECT flag is set
			// and the controller stays in the AM. the EDGE_DETECT flag is cleared in USART_RXISR, when a character (or error) is received
			// since nothing shows up on the RX line, the controller stays in the AM
			// I didn't find any proper solution than allowing a timespan for character reception and then
			// clearing the flag manually:
			if (UART0_EdgeDetectCounter > 0)
				UART0_EdgeDetectCounter--;
			else
				SystemFlags.Flag.Edge_Processing_Uart0 = false;
		}

		if (SystemFlags.Flag.Edge_Processing_Uart1) {
			// this should prevent the UART from waiting for a character that is not going to be received
			// this problem occurs with our RFID reader: during power on/power off it can produce peeks on the TX/RX line
			// that are detected by the 'edge detect' of the UART. the UART wakes up the controller, the EDGE_DETECT flag is set
			// and the controller stays in the AM. the EDGE_DETECT flag is cleared in USART_RXISR, when a character (or error) is received
			// since nothing shows up on the RX line, the controller stays in the AM
			// I didn't find any proper solution than allowing a timespan for character reception and then
			// clearing the flag manually:
			if (UART1_EdgeDetectCounter > 0)
				UART1_EdgeDetectCounter--;
			else
				SystemFlags.Flag.Edge_Processing_Uart1 = false;
		}
		
		if (SystemFlags.Flag.Interrupt_Radio) {
			SystemFlags.Flag.Interrupt_Radio = false;
				
			/**
			 * it's possible, that a packet was actually just sent out, and not received.
			 * Setting the GDO0 to 0x06 adds this information.
			 * 
			 * It's possible to disable the interrupt for the duration of data transmittion,
			 * but then, the send function would have to block. maybe it's easier to just send,
			 * and when the notification comes, to drop it, with some kind of state variables?
			 * 
			 * well, it's possible that the CC is in RX and receiving a packet, a TX is submitted:
			 * 		in such case, the RX is not aborted and the TX is queued
			 * so basically, after a Send command is submitted, it's possible, that the next interrupt 
			 * is indeed a notficiation about data that is awaiting. I can't ignore this and need
			 * to check, whether there is something in the queue. even if it's not very likely, it's possible.
			 * 
			 * NO, WRONG: RX will be qeued, TX is not queued, TX will be ignored when state transition is not possible
			 */
			Layer0_OnRX();
		}
		
		if (SystemFlags.Flag.Pending_AppLayer_Task) {
			SystemFlags.Flag.Pending_AppLayer_Task = false;
			
			if (Handler_AppLayer_Task != NULL)
				Handler_AppLayer_Task();
		}

		if (SystemFlags.AllFlags == 0) {
			//this was the blocking way of waiting: while (!(U1TCTL & TXEPT)); // Confirm no TXing before --> LPM3
			
			if (U0TCTL & TXEPT) { //Transmitter shift register and UxTXBUF are empty or SWRST=1
				WDTCTL = WDTPW | WDTHOLD;	// suspend the watchdog during sleep mode (it's also suspended in interrupts!)
				LED_WHEN_IN_ACTIVEMODE_TURN_OFF_MACRO;
				atomic(START_LPM);
				LED_WHEN_IN_ACTIVEMODE_TURN_ON_MACRO;
			}
		}
	}

	return 0;
}

/**
 * @brief Uses the TimerB to measure the frequency of the SMCLK.
 * 
 * 		Drive TimerB from the SMCLK (unknown frequency) and measure the period of the ACLK (known).
 * 		This works by using the Capture feature of the TimerB. There is an internal connection from
 * 		ACLK to the TimerB comparator register CCTL6.
 * 		The content of the TBR is stored in TBCCR6 after the capture event occures.
 * 		The capture event occures on raising edge of the ACLK.
 * 		By performing two captures, the period can be precisely measured and expressed in ticks of the ACLK.
 * 		Multiplying the result with the ACLK frequency results in the SMCLK frequency.
 * 
 * The implementation wasn't reliable since it wasn't disabling interrutps.
 * A ISR could kick in between the measurements. As a result, not the amount
 * of ticks between subsequent ACLK edges was measured. The TimerB has support
 * for detecting such overflows:
 * 
 * The COV bit is set in the TBCCTLx when a second capture was taken before the
 * latest 
 * 
 */
static uint16_t _System_MeasureSMCLKinTicksPerACLK() {
	/**
	 * Use the TimerB to measure the frequency of the SMCLK:
	 */

	uint16_t first_capture, second_capture, difference;

	uint8_t cov_caused_retries = 0;
	
	////////////////////////////////////////////////////////// Setup the TimerB:
	TBCTL = TBCLR;											// Reset the TimerB
	TBCCTL6 = CM_POS | CCIS_1 | CAP | SCS;					// CCIS_1 is internally connected to the ACLK that runs with 32768Hz (quartz)
	TBCTL = TBSSEL_SMCLK | MC_CONT;							// Start the TimerB, source from SMCLK, don't divide, Continous Mode
	do {
		cov_caused_retries++;
		CLEAR(TBCCTL6, COV);
		
		////////////////////////////////////////////////////////// Measurement:
		CLEAR(TBCCTL6, CCIFG);	while (!(TBCCTL6 & CCIFG));		// wait for the first edge on the ACLK
		first_capture = TBCCR6;									// read the time stamp from the capture
	
		CLEAR(TBCCTL6, CCIFG); while (!(TBCCTL6 & CCIFG));		// wait for the next edge on the ACLK
		
		second_capture = TBCCR6;								// read the time stamp of the second edge
	}
	// if there was a overflow, the measurement needs to be repeated!
	while (TBCCTL6 & COV);
	
	////////////////////////////////////////////////////////// Cleaning up:
	TBCTL = MC_STOP;										// Stop the TimerB
	TBCCTL6 = 0;											// reset the CCTL6 configuration

#ifdef DEBUG_DCO_TUNE
	if (_dco_debugging) {
		if (cov_caused_retries > 1)
			LOGDBG("COV: %u", cov_caused_retries);
	}
#endif
	
	// a possible overflow of the TBR register needs to be taken into account during the computation
	if (second_capture > first_capture)	
		difference = second_capture - first_capture; 			// the simple case (without overflow)
	else
		difference = 0xFFFF - first_capture + second_capture;	// the case with the overflow
	
	return difference;
}

uint32_t System_MeasureSMCLK() {
	uint16_t difference = _System_MeasureSMCLKinTicksPerACLK();
	
	uint32_t smclk_frequency = (uint32_t) difference * 32768L;
	
	return smclk_frequency;
}

uint16_t System_SetDCO( uint32_t target_frq, uint32_t min_frq, uint32_t max_frq )
{
	uint32_t current_frq = System_MeasureSMCLK();
	
	uint16_t steps = 0;
	
	if( current_frq > min_frq && current_frq < max_frq ) {
		// current_frq is in the safe range, return
		return steps;
	}
	
	uint8_t dco_mod =	DCOCTL;
	uint8_t rsel = 		BCSCTL1 & 0x07;
	
	while( true )
	{
		current_frq = System_MeasureSMCLK();

		if( current_frq > min_frq && current_frq < max_frq ) break;

		if( current_frq > max_frq ) // too fast, slow down
		{
			steps++;

			dco_mod--; // slowing down
			if (dco_mod == 0xFF) { // it rolled over! it means, it was 0x00 in the previous run
				if (rsel > 0) // only if it's sill possible to recue the RSEL
					rsel--; // select the next, over RSEL
				else {
					// can't get any slower
					dco_mod = 0x00;
					// should stop somehow or disable the ROSC etc.
				}
			}
		}

		if( current_frq < min_frq ) // too slow, speed up
		{
			steps++;

			dco_mod++; // speeding up
			if (dco_mod == 0x00) { // it rolled over! it means, it was 0xFF in the previous run
				if (rsel < 7) // only if it's sill possible to increase the RSEL
					rsel++; // select the next RSEL
				else {
					// can't get any faster
					// should stop somehow or enable the ROSC etc.
				}
			}

		}
		
		DCOCTL = dco_mod;
		BCSCTL1 &= 0xF8;
		BCSCTL1 |= rsel;
		
	}

	return steps;
}

uint16_t irand(uint16_t k) {
	uint16_t result;
	result = rand();
	result = result % k;
	return result;
}

void System_Trigger_AppLayer_Task() {
	SystemFlags.Flag.Pending_AppLayer_Task = true;
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
