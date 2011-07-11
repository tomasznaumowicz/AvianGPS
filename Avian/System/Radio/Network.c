/**
 * @file		Network.h
 * @ingroup		SystemAPI
 */


#ifndef DOXYGEN_PUBLIC_DOC

#include <stdlib.h>
#include <string.h>

#include "../Logging.h"
#include "../Commands.API.h"

#include "cc1100_spi.h"

#include "Network.h"

#define 	MAX_HANDLERS		12 ///< Specifies the maximal number of registered radio handlers

/**
 * Structure describing one entry in the radio handlers table
 */
typedef struct {
	uint8_t 					protocolId;
	network_packet_handler_t	handler;
} handler_registration_t;

/**
 * Table holding all registered radio handlers
 */
static handler_registration_t handlers[MAX_HANDLERS];

static int8_t FindHandlerIndex(const uint8_t protocolId)
{
	uint8_t index;
	for (index = 0; index < MAX_HANDLERS; index++) {
		if (handlers[index].protocolId == protocolId) break;
	}
	
	if (index == MAX_HANDLERS)
		return -1;
	else
		return index;
}

/**
 * handles data received by layer0. this function should be registered
 * within layer0 as the data receiver.
 * 
 * it extracts the layer1 protocol id from the received data
 * and passes the payload to a protocol specific handler
 * which should be previously registered using the 
 * Network_RegisterHandler function.
 */
static void Layer1_Handler(uint8_t source, const uint8_t* payload, uint8_t payload_length) {
	layer1_packet_t* received_packet = (layer1_packet_t*) payload;
	
	int8_t handler_index = FindHandlerIndex(received_packet->Protocol);
	
	if (handler_index == -1 || handlers[handler_index].handler == NULL) {
		LOGDBG("Network: no handler for %u", received_packet->Protocol);
		return;
	}
	
	handlers[handler_index].handler(source, received_packet->Payload);
}

void Network_Init() {
	// Initialize arrays, variables
	memset(handlers, 0, sizeof(handlers));
	
	// Initializing the radio
	CC_PowerupResetCCxxxx();
	CC1100_PrintInformation();
	CC1100_Init_PowerOn_RX(); // or 
	
	Layer0_Init(Layer1_Handler);
}

#endif // DOXYGEN_PUBLIC_DOC









void Network_InitSendArgs(network_payload_t* payloadOut) {
	memset(payloadOut, 0, sizeof(network_payload_t));
}

bool Network_SendWithoutACK(uint8_t destination, uint8_t protocolId, const network_payload_t* payload) {
	layer1_packet_t data_to_send;
	
	// create payload, fill the buffer
	uint8_t current_payload_index;
	uint8_t bytes_written = 0;
	
	for( current_payload_index = 0; current_payload_index < NETWORK_PACKET_PAYLOAD_COUNT; current_payload_index++ ) {
		if (bytes_written + payload->Buffers[current_payload_index].Size > sizeof(data_to_send.Payload)) {
			return false; // data too big
		}
		
		memcpy(&(data_to_send.Payload[bytes_written]), payload->Buffers[current_payload_index].Buffer, payload->Buffers[current_payload_index].Size);
		bytes_written += payload->Buffers[current_payload_index].Size;
	}
	
	// store the protocolId
	data_to_send.Protocol = protocolId;
	
	// estimate the length of the packet
	uint8_t layer1_packet_length = bytes_written + sizeof(data_to_send.Protocol);
	
	// send the data
	Layer0_TX(destination, (uint8_t*) &data_to_send, layer1_packet_length);
	
	return true;
}

void Network_RegisterHandler(uint8_t protocolId, const network_packet_handler_t packetHandler) {
	// first, check whether the protocol is already registred. this registration should be updated in such case
	int8_t index = FindHandlerIndex(protocolId);
	
	if (index > -1) {
		LOGERR("Network: Overwriting handler for protocol %u", protocolId);
	}
	
	if (index == -1) {
		// the protocol wasn't registered yet, find a free index
		index = FindHandlerIndex(0);
	}
	
	if (index == -1) {
		LOGERR("Network: rejected %u - table is full", protocolId);
		return; // no slots available
	}
	
	handlers[index].protocolId = protocolId;
	handlers[index].handler = packetHandler;
	
	LOGDBG("Network: Registered handler for protocol %u", protocolId);
}


void Network_SetAddress(const uint8_t nodeid) {
	CC110x_SetAddress(nodeid);
}


void Network_SetChannel(const uint8_t channel) {
	CC110x_SetChannel(channel);
}

uint8_t Network_GetChannel() {
	return CC110x_GetChannel();
}

void Network_PowerOn_RX() {
	CC110x_PowerOn_RX();
}

void Network_PowerOff() {
	CC1100_PowerOff();
}



/**
 * @ingroup Commands
 * @brief	@b channel Reconfigures the current radio channel
 * 
 * 127 private channels are currently supported
 */
COMMAND(channel, "Radio: communication channel", cmd_args) {
	// check whether the NodeID change was requested
	if (cmd_args->ArgumentCount > 0) {
		uint16_t channel = atoi(cmd_args->ArgumentArray[0]);
		
		LOGREMOTE("switching to channel %u...", channel);
		
		Network_SetChannel(channel);
	}
	
	snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength, "%u", Network_GetChannel());
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
