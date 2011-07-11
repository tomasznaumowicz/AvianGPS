/**
 * @file
 * @ingroup		SystemAPI
 * @brief		@b System/Radio: Support for radio communication
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * This layer hides the complexity of the radio communication. If you are interested in the way the radio
 * features were implemented you must check the contents of the System/Radio directory as many
 * implementation files were not included in the generated documentation.
 * 
 * @note
 * This is a simple implementation with no support for WOR capabilities of the CC1101. Feel free to add this
 * functionality and get in touch with us in order to make this functionality to the next release.
 * 
 * Example:

 * @code
 * // the data to send
 * char text = "hello";
 * uint32_t number = 123456;
 * // prepare the payload structure
 * network_payload_t packet;
 * Network_InitSendArgs(&packet);
 * packet.Buffers[0].Buffer = text;
 * packet.Buffers[0].Size = sizeof(text);
 * packet.Buffers[1].Buffer = &number;
 * packet.Buffers[1].Size = sizeof(uint32_t);
 * Network_SendWithoutACK(1, 101, &packet);
 * @endcode
 * 
 */

#ifndef NETWORK_H_
#define NETWORK_H_

#include <stdint.h>
#include "../Types.h"
#include "cc_layer0.h"


#ifndef DOXYGEN_PUBLIC_DOC

/**
 * Packet Format: Layer 1
 * 
 * Payload is max 58 bytes (determined by the layer 0 via LAYER0_PAYLOAD_SIZE)
 * 
 * |    Protocol    |    L1Payload    |
 *       1 byte         <= 57 bytes
 */

/**
 * @brief	Defines the maximum lenght of payload on the layer 1. Firmware takes care of enforcing this restriction.
 */
#define LAYER1_PAYLOAD_SIZE					(LAYER0_PAYLOAD_SIZE - 1)

/**
 * @brief	Describes a radio packet
 */
typedef struct
{
	uint8_t Protocol;						///< The Network Protocol ID
	uint8_t Payload[LAYER1_PAYLOAD_SIZE];	///< The Payload
} layer1_packet_t __attribute__ ((packed));

/**
 * @brief	Number of supported buffers within one Network Radio Packet
 */
#define NETWORK_PACKET_PAYLOAD_COUNT			2

/**
 * @brief	Defines the maximum lenght of a buffer within the Network Radio Packet (think as of layer 2)
 * 
 * Content: [sizebyte][payload][sizebyte][payload] where sizebyte might be a 0 and then payload would have length 0
 */
#define NETWORK_BUFFER_MAXSIZE					(LAYER1_PAYLOAD_SIZE - 1)


/**
 * @brief		Initializes the Network feature.
 */
void Network_Init();

#endif //DOXYGEN_PUBLIC_DOC





/**
 * @brief	Describes a structure of a radio packet send buffer
 */
struct netpacket_send_buffer {
	uint8_t	Size;		///< Number of bytes stored in the Buffer
	void*	Buffer;		///< Pointer to first byte of data
};

/**
 * @brief	Describes a structure of an outgoing radio packet
 */
typedef struct  {
	struct netpacket_send_buffer		Buffers[NETWORK_PACKET_PAYLOAD_COUNT];	///< Content of the radio packet
} network_payload_t;

/**
 * @b Handler: Definition of the handler for processing received radio packets
 * 
 * Two parameters are supported:
 * 	- the sender
 * 	- the raw data
 */
typedef void (*network_packet_handler_t) (uint8_t,const uint8_t*);



/**
 * @name		Communication
 * @{
 */

/**
 * @brief		Registers a handler for a specified Network Protocol ID
 * 
 * The Network engine is processing all incoming radio packets and inspecting the "Network Protcol ID" field.
 * Next, the list of known Handlers is investigated. If there is a handler registered, it will be invoked.
 * 
 * This function can be used to register new handlers within the Network engine.
 * 
 * @param[in]	packetType		The Network Protocol ID 
 * @param[in]	packetHandler	Pointer to a handler of type \ref network_packet_handler_t
 * 
 * @remarks If you try to register a handler and use an Network Protocol ID that's already in use, the handler will be overwritten without warning.
 */ 
void Network_RegisterHandler(uint8_t packetType, const network_packet_handler_t packetHandler);

/**
 * @brief		Initializes the payload structure. You must call this method always before filling the payload with data.
 */
void Network_InitSendArgs(network_payload_t* payloadOut);

/**
 * @brief		Sends data over the radio
 * 
 * The following example sends "hello" string to a device with a node id 1. The Network Protocol ID is set to 101.
 * 
 * @note The \ref Network_SendWithoutACK function takes care of building up a radio packet, it's enough to setup the \ref network_payload_t structure properly.
 * 
 * @code
 * // the data to send
 * char text = "hello";
 * uint32_t number = 123456;
 * // prepare the payload structure
 * network_payload_t packet;
 * Network_InitSendArgs(&packet);
 * packet.Buffers[0].Buffer = text;
 * packet.Buffers[0].Size = sizeof(text);
 * packet.Buffers[1].Buffer = &number;
 * packet.Buffers[1].Size = sizeof(uint32_t);
 * Network_SendWithoutACK(1, 101, &packet);
 * @endcode
 * 
 * @param[in]	destination		The node id of the target device. Select 0 for broadcast. 
 * @param[in]	protocolId		The Network Protocol ID
 * @param[in]	payload			Pointer to the data.
 * 
 * @remarks		The radio packet is sent only once and there is no acknowledgement support.
 */
bool Network_SendWithoutACK(uint8_t destination, uint8_t protocolId, const network_payload_t* payload);

/** @} */

/**
 * @name		Configuration
 * @{
 */

/**
 * @brief		Changes the address of the device
 * 
 * Typically the address of the device equals to the node id.
 * 
 * @param[in]	id	The new address of the device
 */
void Network_SetAddress(const uint8_t id);

/**
 * @brief		Changes the radio channel
 * 
 * 127 private channels are currently supported.
 * 
 * @remarks The channel selection is not permanent. The channel will reset to 0 after a reset of the device.
 * 
 * @param[in]	channel	The requested channel number.
 */ 
void Network_SetChannel(const uint8_t channel);

/**
 * @brief		Provides the number of the current radio channel.
 * 
 * @returns	Number of the current radio channel.
 */
uint8_t Network_GetChannel();

/**
 * @brief		Enables the receive (RX) mode of the radio.
 */
void Network_PowerOn_RX();

/**
 * @brief		Disables the receive (RX) mode of the radio.
 */
void Network_PowerOff();

/** @} */

#endif /*NETWORK_H_*/

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
