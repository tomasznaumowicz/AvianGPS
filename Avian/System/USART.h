/**
 * @brief		Management of the Universal Serial Interface (used mainly by drivers)
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef USART_H_
#define USART_H_

#include "Types.h"
#include "USART.API.h"


/**
 * @brief		Configures the UART mode of the selected USART device.
 * 
 * If the USART device is in an SPI mode, the configuration will be used during next
 * UART activation. If the USART device is in the UART mode, the USART will be reconfigured. 
 */
void USART_UART_Configure(enum usart_id usart, uint32_t baudrate);

/**
 * @brief		Reads recent line received by the USART0 in the UART mode.
 */
void USART0_UART_ReadLine(uint8_t* output);

/**
 * @brief		Reads recent line received by the USART0 in the UART mode.
 */
void USART1_UART_ReadLine(uint8_t* output);

uint16_t volatile UART0_EdgeDetectCounter; ///< Used to suppress system's transition to a LPM. Too early transition aborts the process of receiving a character over UART.
uint16_t volatile UART1_EdgeDetectCounter; ///< Used to suppress system's transition to a LPM. Too early transition aborts the process of receiving a character over UART.

#endif /*USART_H_*/


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
