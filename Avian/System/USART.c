/**
 * @brief		Management of the Universal Serial Interface (used mainly by drivers)
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#include <msp430x16x.h>
#include <sys/cdefs.h>
#include <stdint.h>
#include <signal.h>
#include <mspgcc/ringbuffer.h>

#include "BitOperations.h"
#include "Watchdog.h"
#include "Timers.h"
#include "System.h"
#include "System.API.h"
#include "Logging.h"

#include "USART.h"

#define UART0_RESET_RX()		{ IFG1 &= ~URXIFG0; }
#define	UART0_RESET_RXTX()		{ IFG1 &= ~(URXIFG0 | UTXIFG0); }
#define	UART0_WAIT_RX()			while( (IFG1 & URXIFG0) == 0 ) { _NOP(); }

#define UART0_WAIT_TXDONE()		while( (U0TCTL & TXEPT) == 0 ) { _NOP(); }
#define	UART0_WAIT_TX()			UART0_WAIT_TXDONE()

#define UART1_WAIT_TXDONE()		while( (U1TCTL & TXEPT) == 0 ) { _NOP(); }

/**
 * @brief	Describes USART configurations, used for UART registers caching.
 */
typedef struct
{
	uint8_t br0;		///< Cached settings for the BR0 register
	uint8_t br1;		///< Cached settings for the BR1 register
	uint8_t mctl;		///< Cached settings for the MCTL register
} UART_Configuration_t;

/**
 * @brief	Stores USART configurations, used for UART registers caching.
 */
static UART_Configuration_t _UART_Configurations[2];

//used to detect whether UART mode can be used
//it's a number so that the locks can be counted (imagine nested calls to spi interface with the same device/the same configuration)
static volatile uint8_t			_usart0_spi_locked;

//all characters received via UART are stored in this ringbuffer.
RINGBUFFER_NEW(_uart0_rx_ringbuffer, UART0_RX_BUFFER_SIZE);
RINGBUFFER_NEW(_uart1_rx_ringbuffer, UART1_RX_BUFFER_SIZE);

/**
 * it's important to keep track of the number of lines stored in the incoming ring buffer.
 * it's not enough to assume, there is a line already when there are bytes in the buffer
 * it's not safe to clear the new line flag after getting a line from the buffer when there could
 * be another one stored in the meantime.
 * 
 * soulution: count lines in the buffer, increase on add, decrease on remove. inconsistency could happen.
 * a better soulution might be implemented in future.
 */
static volatile uint8_t _uart1_rx_lines_in_buffer;

// this variable will be used to monitor the length of the ringbuffer for the uart1
uint16_t _uart1_rx_max_buffer_length;

/**
 * @brief	Computes configuration registers for selected usart and specified baudrate.
 * 
 * @author	Hans-Peter Heitzmann
 */ 
static void _PrepareUartConfiguration(enum usart_id usart, uint32_t baudrate) {
	unsigned long tmp_long = 0;
	unsigned int i;
	unsigned char mod, modmask;
	float frac, frac_sum;
	
	unsigned long system_clock = System_MeasureSMCLK();
	
	tmp_long = system_clock/baudrate;
	frac = (float)system_clock/(float)baudrate;
	frac -= tmp_long;						
	frac = 1.0 / frac;						
	frac_sum = frac;
	modmask = 0x80;
	mod = 0;
	
	if( frac > 0.1 )
	{
		for( i = 1; i < 9; i++ )
		{
			if( frac_sum < (float)(i + 1) && frac_sum > (float)(i - 1) )
			{
				mod |= modmask;
				modmask >>= 1;
				frac_sum += frac;
				continue;
			}
			modmask >>= 1;
		}
	}
	
	_UART_Configurations[usart].br0 = (unsigned char)( tmp_long & 0x00FF ); 
	_UART_Configurations[usart].br1 = (unsigned char)( tmp_long >> 8 );
	_UART_Configurations[usart].mctl = (unsigned char)mod;
}

/**
 * Interrupt handler for USART0 in UART mode
 */
interrupt(USART0RX_VECTOR) Usart0_RX_ISR(void) {
	// If start edge detected, toggle & return
	
	/*
	 * When URXSE, URXIEx and GIE are set and a start edge occurs on URXDx,
	 * the internal signal URXS will be set. When URXS is set, a receive interrupt
	 * request is generated but URXIFGx is not set. User software in the receive
	 * interrupt service routine can test URXIFGx to determine the source of the
	 * interrupt. When URXIFGx = 0 a start edge was detected and when URXIFGx
	 * = 1 a valid character (or break) was received.
	 */
	
	if (!(IFG1 & URXIFG0)) {
		CLEAR(U0TCTL, URXSE);
		SET(U0TCTL, URXSE);
		
		//this flag is set in order to prevent the main loop from entering a LPM mode
		//when the CLK for the UART is stopped, no character can be received then.
		SystemFlags.Flag.Edge_Processing_Uart0 = true;
		UART0_EdgeDetectCounter = 132;			//this was built to handle a case where an edge was detected, but no data arrives: in such case the microcontroller stayed in the AM.
												//the variable is decremented in the main loop and when it reaches 0, the FLAG_UART_EDGE_DETECT is cleared. 

		// Wake up from LPM3 (the controller has to stay in AM (LPM0,1) until the character is received! the FLAG_UART_EDGE_DETECT is used for this)
		END_LPM_WITH_CONDITION_CHECK;

		return;
	}
	
	//the flag was detected, the controller was activated.
	//the controller wasn't sent to LPM3 during character processing
	//because the FLAG_UART_EDGE_DETECT was set.
	//now, the character was received and it's ok to clear this flag
	SystemFlags.Flag.Edge_Processing_Uart0 = false;

	uint8_t input;								// used to hold the received character

	// there was no error during the communication:
	if (U0RCTL & RXERR) { 						//the receive error flag was set
		input = U0RXBUF;						//clear the error flag by reading the character and leaving the ISR
		return;									//and return!
	}
	
	input = U0RXBUF;							//read the rx buffer of the USART
	
#if RESET_ON_ESC_ON_UART0

		if (input == 27) {						// 27 is the ASCII code for the ESC character
			__wd_resetinfo.Password = WD_RESETINFO_PWD;
			__wd_resetinfo.ResetReason = WD_RESET_REASON_REQUESTED;
			WDTCTL = 0xDEAD;
		}
		
#endif
	
	// allow to modify the behaviour of the ISR: the application can preview characters and interrupt its processing
	if (UART0_CharacterPreview != NULL) {		//if the preview handler was definded
		if (UART0_CharacterPreview(input))  	//use the preview handler: the handler returns TRUE if the processing of the characters should be stopped
			return;
	}
	
	// regular processing of entered character: storage in the ring buffer and notifications via system flags after the NEW LINE character
	
	if (input == '\n')	
		return;
	
	if (input == '\b')							//ingore backspace	
		return;
	
	if (input == '\r') {
		ringbuffer_put(&_uart0_rx_ringbuffer, '\0');
		SystemFlags.Flag.Pending_Uart0_NewLine = true;
		END_LPM_WITH_CONDITION_CHECK;		
		return;
	}

	ringbuffer_put(&_uart0_rx_ringbuffer, input);		//store all other characters
}

/**
 * Interrupt handler for USART0 in UART mode
 */
interrupt(USART1RX_VECTOR) Usart1_RX_ISR(void) {
	
	/**
	 * we should care only, if there is something that monitors UART in software anyway.
	 * so if there is no handler active, we can just ignore everything
	 * 
	 * this is done below:
	 */
	
	// If start edge detected, toggle & return
	
	/*
	 * When URXSE, URXIEx and GIE are set and a start edge occurs on URXDx,
	 * the internal signal URXS will be set. When URXS is set, a receive interrupt
	 * request is generated but URXIFGx is not set. User software in the receive
	 * interrupt service routine can test URXIFGx to determine the source of the
	 * interrupt. When URXIFGx = 0 a start edge was detected and when URXIFGx
	 * = 1 a valid character (or break) was received.
	 */
	
	if (!(IFG2 & URXIFG1)) {
		CLEAR(U1TCTL, URXSE);
		SET(U1TCTL, URXSE);
		
		// don't care, if nobody is registered
		if (UART1_CharacterPreview == NULL && UART1_LineReceivedHandler == NULL)
			return;

		//this flag is set in order to prevent the main loop from entering a LPM mode
		//when the CLK for the UART is stopped, no character can be received then.
		SystemFlags.Flag.Edge_Processing_Uart1 = true;
		UART1_EdgeDetectCounter = 256;				//this was built to handle a case where an edge was detected, but no data arrives: in such case the microcontroller stayed in the AM.
														//the variable is decremented in the main loop and when it reaches 0, the FLAG_UART_EDGE_DETECT is cleared. 

		// Wake up from LPM3 (the controller has to stay in AM (LPM0,1) until the character is received! the FLAG_UART_EDGE_DETECT is used for this)
		END_LPM_WITH_CONDITION_CHECK;

		return;
	}
	
	//the flag was detected, the controller was activated.
	//the controller wasn't sent to LPM3 during character processing
	//because the FLAG_UART_EDGE_DETECT was set.
	//now, the character was received and it's ok to clear this flag
	SystemFlags.Flag.Edge_Processing_Uart1 = false;

	uint8_t input;								// used to hold the received character

	// there was no error during the communication:
	if (U1RCTL & RXERR) { 						//the receive error flag was set
		input = U1RXBUF;						//clear the error flag by reading the character and leaving the ISR
		return;									//and return!
	}
	
	input = U1RXBUF;							//read the rx buffer of the USART
	
	// allow to modify the behaviour of the ISR: the application can preview characters and interrupt its processing
	if (UART1_CharacterPreview != NULL) {		//if the preview handler was definded
		
		//use the preview handler: the handler returns TRUE if the processing of the characters should be stopped
		bool abort = UART1_CharacterPreview(input);

		// test the wake-up condition and wake up the uc if it was requested
		if (SystemFlags.Flag.Pending_WakeUp_Request) {
			SystemFlags.Flag.Pending_WakeUp_Request = false;
			END_LPM_WITH_CONDITION_CHECK;
		}
	
		// stop processing if the character was accquired by the character preview function
		if (abort)
			return;
	}
	
	// don't care, if nobody is registered
	if (UART1_LineReceivedHandler == NULL)
		return;
	
	// regular processing of entered character
	
	if (input == '\n')	
		return;
	
	if (input == '\r') {
		ringbuffer_put(&_uart1_rx_ringbuffer, '\0');
		_uart1_rx_lines_in_buffer++;
		SystemFlags.Flag.Pending_Uart1_NewLine = true;
		END_LPM_WITH_CONDITION_CHECK;		
		return;
	}

	int result = ringbuffer_put(&_uart1_rx_ringbuffer, input);		//store all other characters
	
	if (result == -1) {
		_uart1_rx_lines_in_buffer = 0;
		ringbuffer_clear(&_uart1_rx_ringbuffer);
	}
	
}


void USART0_UART_ReadLine(uint8_t* output) {
	int8_t index = -1;
	int8_t character;
	
	do {
		character = ringbuffer_get(&_uart0_rx_ringbuffer);
		if (character == -1)								// if the ringbuffer was empty: break the loop
			break;
		
		index++;
		output[index] = character;
	} while (output[index] != '\0' && index < USART_MAX_RECEIVED_LINE_LENGTH);
	
	/**
	 * the loop terminated: check if the data is correct
	 * the valid state is when:
	 * 		_lastReceivedLine[index] == '\0'
	 * it might be, that the received line was longer than the maximum allowed length
	 * in that case the loop breaks the execution and the '\0' would be missing.
	 * in that case the ring buffer could contain the rest of the message: the state of the ring buffer
	 * would be corrupted: it would still hold a message, that wouldn't be processed until the next NEW LINE
	 * is received.
	 * In order to make sure, the output is in the valid state, '\0' will be stored 
	 * at the end of the line.
	 */
	
	output[index] = '\0';
	
	/**
	 * in order to make sure that the ring buffer is in the correct state:
	 * if the character == -1: the buffer was empty, the buffer is in the correct state.
	 * the character != -1: there is still something in the buffer: set the fimrware flag to NEW LINE,
	 * so that the application is forced to pick up the data again.
	 * it might be, that the last character in the buffer was the '\0'
	 * this would mean, that the state of the ringbuffer is correct.
	 * in order to see, if data in the buffer is available, the length of the ringbuffer will be used:
	 */
	
	if (ringbuffer_len(&_uart0_rx_ringbuffer) > 0)
		SystemFlags.Flag.Pending_Uart0_NewLine = true;
}

/**
 * Reads a received string/line from the internal ring buffer
 */
void USART1_UART_ReadLine(uint8_t* output) {
	int8_t index = -1;
	int8_t character;
	
	uint16_t rbuff_len = ringbuffer_len(&_uart1_rx_ringbuffer);
	if (rbuff_len > _uart1_rx_max_buffer_length) {
		_uart1_rx_max_buffer_length = rbuff_len;
		LOG("UART1 RX BUFF LEN: %u", _uart1_rx_max_buffer_length);
	}
	
	do {
		character = ringbuffer_get(&_uart1_rx_ringbuffer);
		if (character == -1) {								// if the ringbuffer was empty: break the loop
			break;
		}
		
		index++;
		output[index] = character;
	} while (output[index] != '\0' && index < USART_MAX_RECEIVED_LINE_LENGTH);
	
	/**
	 * the loop terminated: check if the data is correct
	 * the valid state is when:
	 * 		_lastReceivedLine[index] == '\0'
	 * it might be, that the received line was longer than the maximum allowed length
	 * in that case the loop breaks the execution and the '\0' would be missing.
	 * in that case the ring buffer could contain the rest of the message: the state of the ring buffer
	 * would be corrupted: it would still hold a message, that wouldn't be processed until the next NEW LINE
	 * is received.
	 * In order to make sure, the output is in the valid state, '\0' will be stored 
	 * at the end of the line.
	 */
	
	output[index] = '\0';
	
	/**
	 * in order to make sure that the ring buffer is in the correct state:
	 * if the character == -1: the buffer was empty, the buffer is in the correct state.
	 * the character != -1: there is still something in the buffer: set the fimrware flag to NEW LINE,
	 * so that the application is forced to pick up the data again.
	 * it might be, that the last character in the buffer was the '\0'
	 * this would mean, that the state of the ringbuffer is correct.
	 * in order to see, if data in the buffer is available, the length of the ringbuffer will be used:
	 */
	
	/**
	 * there is an issue that can become visible in the following case:
	 * a line is put into the ringbuffer, but one character gets corrupted and is set to 0
	 * 	as a result, the ringbuffer contains "datadatadata0datadtadta0" but the lines_in_buffer counter is only set to 1
	 * then, data is read, and only the first part untio 0 is extracted. the rest is still in the buffer but the lines_in_buffer counter says 0...
	 * 
	 * when this happens, the most recent line will be lost and so this error can propagate...
	 * detection is difficult but -> once the buffer is full, the interrupt flushes all the data and executes internal uart1 logic reset
	 */ 

	if (_uart1_rx_lines_in_buffer > 0)
		_uart1_rx_lines_in_buffer--;

	if (_uart1_rx_lines_in_buffer > 0)
		SystemFlags.Flag.Pending_Uart1_NewLine = true;
}

/**
 * Implementation of the putchar function from the stdio.h
 * This function is required for printf etc to work
 */
int putchar(int c) {
	if (_usart0_spi_locked)
		return -1;
	
	UART0_WAIT_TX();
	U0TXBUF = c;
	UART0_WAIT_TX();
	
	return c;
}

/**
 * @brief	This function has a delay built in as it was developed parallel to work with the GPS chip attached to the USART1 on the AvianGPS V4. It should be updated in the future
 */
int USART1_UART_TX(int c) {
	Timers_Block(256);
	
	UART1_WAIT_TXDONE();
	U1TXBUF = c;
	
	return c;
}

void USART1_UART_TX_Buffer(int length, const uint8_t* buffer) {
	uint8_t index;
	for (index = 0; index < length; index++) {
		USART1_UART_TX(buffer[index]);
	}
}

void _USART1_UART_Reconfigure() {
	
	UART1_WAIT_TXDONE();
	
	//this is a critical section, we have to disable all interrupts.
	_DINT();
	
	// The required USART initialization/re-configuration process is:
	// 1) Set SWRST (BIS.B #SWRST,&UxCTL)
	U1CTL = SWRST;
	// 2) Initialize all USART registers with SWRST = 1 (including UxCTL)
	U1CTL |= CHAR;     					// 8-bit character
	U1TCTL |= SSEL0 + SSEL1 + URXSE;	// UCLK = SCLK  + enable UART receive start-edge
	U1RCTL = 0;

	//set the speed according to the stored configuration
	U1BR0 = _UART_Configurations[USART1].br0;
	U1BR1 = _UART_Configurations[USART1].br1;
	U1MCTL = _UART_Configurations[USART1].mctl;
			
	// 3) Enable USART module via the MEx SFRs (URXEx and/or UTXEx)
	ME2 |= UTXE1 | URXE1;				// Enable USART1 TXD/RXD	

	// 4) Clear SWRST via software (BIC.B #SWRST,&UxCTL)
	U1CTL &= ~SWRST;

	// 5) Enable interrupts (optional) via the IEx SFRs (URXIEx and/or UTXIEx)
	//IFG1 &= ~UTXIFG0;
	//IFG1 &= ~URXIFG0;
	SET(IE2, URXIE1);
	
	//re-enable interrupts
	_EINT();
}

void USART1_PowerOn() {
	// flush the buffer first
	_uart1_rx_lines_in_buffer = 0;
	_uart1_rx_max_buffer_length = 0;
	
	ringbuffer_clear(&_uart1_rx_ringbuffer);
	
	SET(IE2, URXIE1);			//interrupt enable for RX
	SET(P3SEL, BIT6 | BIT7);	//enable 2nd fct mode of the UTX1 and URX1
}

void USART1_PowerOff() {
	CLEAR(IE2, URXIE1); 		//disable interrupts on RX1
	CLEAR(P3SEL, BIT6 | BIT7);	//disable 2nd fct mode of the UTX1 and URX1, this fallbacks to GPIO mode where both pins are configured as output with value 0 (important so that the GPS doesn't steal power)	
}

void USART_Init(const enum usart_id usart, const uint32_t baudrate) {
	switch (usart) {
		case USART0: {
			_usart0_spi_locked = 0;
			UART0_CharacterPreview = NULL;
			
			USART_UART_Configure(USART0, baudrate);
			
			break;
		}
		case USART1: {
			_uart1_rx_lines_in_buffer = 0;

			USART_UART_Configure(USART1, baudrate);
			
			break;
		}
	}
}

void USART_UART_Configure(enum usart_id usart, uint32_t baudrate) {
	//prepare parameters for the current frequency (the output is cached by the called function)
	_PrepareUartConfiguration(usart, baudrate);
	
	//reconfigure the system if not locked. currently USART1 is used only for GPS, so we don't care about SPI implementation for USART1 yet
	switch (usart) {
		case USART0: {
			if (_usart0_spi_locked == 0) {
				//this function reconfigures the USART0 to UART mode. The prepared speed parameters are used.
				USART0_SPI_Release();
			}
		}
		case USART1: {
			_USART1_UART_Reconfigure();			
		}
	}
}

bool USART0_SPI_Lock(uint8_t br0, uint8_t br1, uint8_t mctl) {
	if (_usart0_spi_locked) {
		//the SPI is already locked in a specific configuration
		//if the new configuration is different, no lock can be accquired
		if (!(U0BR0 == br0 && U0BR1 == br1 && U0MCTL == mctl))
			return false;			// configurations differn, no lock can be accquired
	}
	
	UART0_WAIT_TXDONE();

	CLEAR(P3SEL, BIT4 | BIT5);		//make sure UTX0 and URX0 are not in the 2nd fct mode
	
	//reconfigure to SPI mode and use the speed parameters
	//disable interrupts of the USART0
	
	// The required USART initialization/re-configuration process is:
	// 1) Set SWRST (BIS.B #SWRST,&UxCTL)
	U0CTL = SWRST;
	// 2) Initialize all USART registers with SWRST = 1 (including UxCTL)
	U0CTL |= CHAR | SYNC | MM;		// 8-bit SPI Master
	U0TCTL = CKPH | SSEL1 | SSEL0 | STC;	// SMCLK, 3-pin mode, clock idle low, data valid on rising edge, UCLK delayed

	//set the speed according to the predefined configuration for the sd card
	U0BR0 = br0;
	U0BR1 = br1;
	U0MCTL = mctl;
			
	// 3) Enable USART module via the MEx SFRs (URXEx and/or UTXEx)
	ME1 |= USPIE0;							// Enable USART0 SPI

	// 4) Clear SWRST via software (BIC.B #SWRST,&UxCTL)
	U0CTL &= ~SWRST;

	// 5) Enable/Disable interrupts (optional) via the IEx SFRs (URXIEx and/or UTXIEx)
	IE1 &= ~URXIE0; //disable interrupt for RX
	IFG1 &= ~URXIE0;
	
	SET(P3SEL, BIT1 | BIT2 | BIT3);		//make sure SPI0 pins are the 2nd fct mode

	_usart0_spi_locked++;
	
	return true;
}


void USART0_SPI_Release() {
	if (_usart0_spi_locked)
		_usart0_spi_locked--;
	
	if (_usart0_spi_locked)
		return;
	//reconfigure the USART0 to UART mode, use the cached speed parameters

	//this is a critical section, we have to disable all interrupts.
	_DINT();
	
	//wait for all buffered data to be transmitted
	//UART0_WAIT_TXDONE();
	CLEAR(P3SEL, BIT1 | BIT2 | BIT3);		//make sure SPI0 pins are not in the 2nd fct mode
	
	// The required USART initialization/re-configuration process is:
	// 1) Set SWRST (BIS.B #SWRST,&UxCTL)
	U0CTL = SWRST;
	// 2) Initialize all USART registers with SWRST = 1 (including UxCTL)
	U0CTL |= CHAR;     			// 8-bit character
	U0TCTL |= SSEL0 + SSEL1 + URXSE;					// UCLK = SCLK  + enable UART receive start-edge
	U0RCTL = 0;

	//set the speed according to the stored configuration
	U0BR0 = _UART_Configurations[USART0].br0;
	U0BR1 = _UART_Configurations[USART0].br1;
	U0MCTL = _UART_Configurations[USART0].mctl;
			
	// 3) Enable USART module via the MEx SFRs (URXEx and/or UTXEx)
	ME1 |= UTXE0 | URXE0;				// Enable USART1 TXD/RXD	

	// 4) Clear SWRST via software (BIC.B #SWRST,&UxCTL)
	U0CTL &= ~SWRST;

	// 5) Enable interrupts (optional) via the IEx SFRs (URXIEx and/or UTXIEx)
	//IFG1 &= ~UTXIFG0;
	//IFG1 &= ~URXIFG0;
	IE1 |= URXIE0; //interrupt enable for RX
	
	SET(P3SEL, BIT4 | BIT5);		//make sure UTX0 and URX0 are in the 2nd fct mode
	
	//re-enable interrupts
	_EINT();
}

uint8_t USART0_SPI_Comm(uint8_t outgoing_byte) {
	//return 0 if not in SPI mode
	if (!_usart0_spi_locked)
		return 0;

	//while (P3IN & BIT2);				// no nonoWait for high auf SOMI --> device ready 
	IFG1 &= ~URXIFG0;					// Clear flag set during previous read
	//the line above could be added -> it's not clear whether other 'drivers' clear the flag properly
	// (the flag is cleaerd after reading the U0RXBUF, but maybe not every device is interested in the resulting data (?)
	
	U0TXBUF = outgoing_byte;			// Send data
	while ( !( IFG1 & URXIFG0 ) );		// Wait for RX (TX) to finish (could sleep now)

	return U0RXBUF;			// Read data    
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
