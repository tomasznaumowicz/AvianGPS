/**
 * @ingroup		SystemAPI
 * @brief		Support for radio communication
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */


#ifndef CC_LAYER0_H_
#define CC_LAYER0_H_

/*
 * Packet Format: Layer 0
 * Remark: Length and Address are used by the CC1101 and need
 * to be put at the beginning of the packet
 * 
 * Length | Address | Source |  Flags  |    Payload    |
 * 1 byte   1 byte    1 byte   1 byte    <= 58 bytes
 * 
 * Flags:
 *  not used yet: the goal is to use it for seqence numbers, etc., later
 * 
 * also: the RX FIFO of the CC1101 can store up to 64 bytes:
 * 2 bytes are attached at the end of the received data and contain RSSI and LQI bytes.
 * 
 * Available payload:
 * 		64 - 1 (length) - 1 (address) - 1 (source) - 1 (flags) - 2 (RSSI+LQI) = 58
 */

#define LAYER0_PAYLOAD_SIZE		(58)

/**
 * The definition of a struct that describes a packet on the layer 0
 */
typedef struct
{
	uint8_t Length;							///< Length of the packet but without the length byte
	struct {
		uint8_t Address;					///< "to" address
		uint8_t Source;						///< "from" address
		uint8_t Flags;						///< packet flags
	} Header;								///< Header of the packet
	uint8_t Payload[LAYER0_PAYLOAD_SIZE];	///< Pahload of the packet (data)
} layer0_packet_t  __attribute__ ((packed));							//size 62 bytes


/**
 * should be called outside of an interrupt when a packet is received
 * (notified by the P2_ISR)
 */
void Layer0_OnRX();

typedef void (*layer0_packet_handler_t) (uint8_t source, const uint8_t* payload, uint8_t payload_length); //source, payload, payload length

void Layer0_Init(const layer0_packet_handler_t rxPacketHandler);

void Layer0_TX(uint8_t destination, const uint8_t* payload, uint8_t payload_length);

#endif /*CC_LAYER0_H_*/

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
