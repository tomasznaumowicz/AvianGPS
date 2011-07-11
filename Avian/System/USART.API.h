/**
 * @file		
 * @brief		@b System: Management of the Universal Serial Interface (used mainly by drivers)
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef USART_API_H_
#define USART_API_H_

#include "Types.h"

/**
 * @name		Default configuration.
 * 
 * You can overwrite this configuration by putting the same defines with adjusted values in your \ref AppConfig.h
 * 
 * @{
 */

#ifndef UART0_RX_BUFFER_SIZE
	#define UART0_RX_BUFFER_SIZE					40 ///< Size of the receive buffer used of the UART0 (the serial debug output on the AvianGPS) 
#endif

#ifndef UART1_RX_BUFFER_SIZE
	#define UART1_RX_BUFFER_SIZE					80  ///< Size of the receive buffer used of the UART1 (the GPS device output on the AvianGPS) 
#endif

#ifndef USART_MAX_RECEIVED_LINE_LENGTH
	#define USART_MAX_RECEIVED_LINE_LENGTH			120 ///< Maximum length of a single string that can be received by the UART
#endif

#ifndef RESET_ON_ESC_ON_UART0
	#define RESET_ON_ESC_ON_UART0					1 ///< Controls the reset feature: when enabled a single ESC key can reset the device. Useful when debugging as no handler for a command needs to be executed in order to reset a device.
#endif

/** @} */

/**
 * @brief Selection of the USART interface
 */
enum usart_id {
	USART0		= 0,	///< USART 0 (the serial debug output on the AvianGPS borard, shared with SPI devices: radio, memory, SCP sensor)
	USART1		= 1		///< USART 1 (the GPS device on the AvianGPS board)
};

/**
 * @brief @b Handler: Definition of the handler for processing of strings received via USART
 * 
 * One parameter is supported:
 * 	- the received string
 */
typedef void (*fp_lineInputHandler_t) (const char* input);

/**
 * @ingroup	Handlers
 * @brief @b Handler: Access to the string (a single line) received via USART 1 (see \ref Handlers for more handlers).
 * 
 * You can access the process received strings in your application code by assigning 
 * your handler to this variable.
 * 
 * 
 * One parameter is supported:
 * 	- the received string
 * 
 * @remarks		Only one handler can be registered.
 * 				This handler is used by the \ref Venus.API.h GPS device. Use its handlers to access valid NMEA strings.
 */
fp_lineInputHandler_t UART1_LineReceivedHandler;

/**
 * @brief @b Handler: Definition of the handler for previewing characters received via UART
 * 
 * One parameter is supported:
 * 	- the received character
 * 
 * Returning true will cancel processing of this character by the System API.
 */
typedef bool (*fp_previewCharacter_t) (uint8_t);

/**
 * @ingroup	Handlers
 * @brief @b Handler: Previewing a character received via UART 0 (see \ref Handlers for more handlers).
 * 
 * You can preview and process characters received in your application code by assigning 
 * your handler to this variable.
 * 
 * One parameter is supported:
 * 	- the received character
 * 
 * Returning true stops further processing of this character by the System API.
 * 
 * @remarks		Only one handler can be registered.
 */
fp_previewCharacter_t UART0_CharacterPreview;

/**
 * @ingroup	Handlers
 * @brief @b Handler: Previewing a character received via UART 1 (see \ref Handlers for more handlers).
 * 
 * You can preview and process characters received in your application code by assigning 
 * your handler to this variable.
 * 
 * One parameter is supported:
 * 	- the received character
 * 
 * Returning true stops further processing of this character by the System API.
 * 
 * @remarks		Only one handler can be registered.
 * 				This handler is used by the \ref Venus.API.h GPS device.
 */
fp_previewCharacter_t UART1_CharacterPreview;


/**
 * @brief		Initializes the selected USART
 * 
 * This function must be called first before a USART can be used.
 * USART0 is initialized by the System API.
 * 
 * The USART defaults to the UART mode.
 * 
 * Example:
 * @code
 * USART_Init(USART0, 115200L);
 * @endcode
 * 
 * @remarks		In the AvianGPS implementation, \ref Venus.API.h is initializing USART1.
 * 				In current implementation only UART mode is supported for the USART1. 
 *
 * @param[in]	usart		The USART to initialize
 * @param[in]	baudrate	The buadrate to be preconfigured.
 * 
 */
void USART_Init(const enum usart_id usart, const uint32_t baudrate);

/**
 * @brief		Reconfigures the selected UART
 * 
 * Updates the baudrate of the selected UART interface. If the interface is currently
 * in SPI mode, the new baudrate will be cached and used later, when the interface
 * switches back to the UART mode.
 * 
 * @param[in]	usart		The USART to initialize
 * @param[in]	baudrate	The buadrate to be preconfigured.
 * 
 */
void USART_UART_Configure(enum usart_id usart, uint32_t baudrate);

/**
 * @name		SPI support for USART0
 * @{
 */

/**
 * @brief		Locks USART0 in the SPI mode
 * 
 * Calling this function locks the USART0 in the SPI mode and disables debug output. Important
 * when communicating with devices sharing the same USART.
 * 
 * @remarks		Multlple locks are possible (they are counted).
 * 
 * @param[in]	br0			BR0 divider
 * @param[in]	br1		`	BR1 divider
 * @param[in]	mctl		MCTL divider
 * 
 * @returns		true on success
 */
bool 	USART0_SPI_Lock(uint8_t br0, uint8_t br1, uint8_t mctl);

/**
 * @brief		Releases the lock on USART0
 *
 * @remarks		Multiple locks are possible (they are counted). When the last lock is released, the device defaults to the UART mode. 
 */
void 	USART0_SPI_Release();

/**
 * @brief		Transmits and receives a byte over the SPI on USART0
 * 
 * Transmit a 0x00 when only interested in receiving data.
 * 
 * @param[in]	outgoing_byte		Byte to transmit (transmit 0x00 when only interested in receiving)
 * 
 * @returns		Byte received.
 */
uint8_t USART0_SPI_Comm(uint8_t outgoing_byte);

/** @} */

/**
 * @name		USART1 management
 * @{
 */

/**
 * @brief		Transmits a character over the USART1 in UART mode.
 * 
 * @param[in]	c Character to transmit
 * 
 * @remarks		The USART1 must be in the UART mode. This function blocks for 256 ACLK ticks before transmitting a character.
 * 
 * @returns		The transmitted character.
 */
int USART1_UART_TX(int c);

/**
 * @brief		Transmits a buffer over the USART1 in UART mode.
 * 
 * @param[in]	length		The lenght of the buffer to transmit
 * @param[in]	buffer		The buffer to transmit
 * 
 * @remarks		The USART1 must be in the UART mode.
 */
void USART1_UART_TX_Buffer(int length, const uint8_t* buffer);

/**
 * @brief		Enables the USART1. Defaults to the UART mode.
 */
void USART1_PowerOn();

/**
 * @brief		Disables the USART1.
 */
void USART1_PowerOff();

/** @} */

#endif /*USART_API_H_*/


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
