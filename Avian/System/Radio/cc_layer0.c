/**
 * @ingroup		SystemAPI
 * @brief		Support for radio communication
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */


#include <msp430x16x.h>
#include <stdint.h>
#include <string.h>

#include "../../Drivers/Device.h"

#include "../Types.h"
#include "../Timers.h"
#include "../Logging.h"
#include "../Commands.h"
#include "../USART.h"

#include "Network.h"

#include "cc1100.h"
#include "cc_layer0.h"


static layer0_packet_handler_t Layer0_RegisteredPacketHandler; 

/**
 * used to detect whether a tx was requested by the rx handler during the OnRX execution
 * RX shouldn't be forced when TX is stil being prepared by the CC chip.
 * this soultion won't help in the case when e.g. a timer sends out a packet.
 * although -> sending sets the flag to true always. and when the OnRX is called,
 * either the TX is done or RX was succesul. so it shoud be good enough.
 */
static bool tx_requested_by_rx_handler;

void Layer0_OnRX() {
	// it's safe to clear this flag: either the TX completed an this function is called or data was received (== TX not executing)
	tx_requested_by_rx_handler = false;
	
	// the transceiver notified the firmware about a waiting packet
	// the entire packet needs to be read from the RX FIFO
	uint8_t rx_buffer[sizeof(layer0_packet_t)];
	
	if ( RFReceivePacket( rx_buffer, sizeof(rx_buffer)))
	{
		// the RX FIFO was read, process the data
		layer0_packet_t* packet = (layer0_packet_t*) rx_buffer;
		
		// compute the length of the payload. it is stored in the length byte but the stored value contains also the length of the header
		uint8_t payload_length = packet->Length - sizeof(packet->Header);

		if (Layer0_RegisteredPacketHandler != NULL) {
			Layer0_RegisteredPacketHandler(packet->Header.Source, packet->Payload, payload_length);
			// it's possible, that the RX handler is sending data (this should be taken into account in the low level CC code) 
		}
	} else {
		//LOGDBG("receive.false");
	}
	
	// don't switch to RX mode, when the RX hanlder started the TX
	if (tx_requested_by_rx_handler == true)
		return;
	
	// enter into the RX mode again (forces callibration)
	CC_SPIStrobe(CCxxx0_SRX);
}

void Layer0_Init(const layer0_packet_handler_t rxPacketHandler) {
	Layer0_RegisteredPacketHandler = rxPacketHandler;
}

void Layer0_TX(const uint8_t destination, const uint8_t* payload, uint8_t payload_length) {
	layer0_packet_t packet;
	
	if (payload_length > LAYER0_PAYLOAD_SIZE)
		return;

	packet.Header.Address	= destination;	packet.Header.Source 	= Configuration.NodeID;	packet.Header.Flags 	= 0; // not used yet	
	packet.Length = sizeof(packet.Header) + payload_length;
	uint8_t layer0length = packet.Length + 1; // add 1 for the packet.Length byte
	
	memcpy(&(packet.Payload), payload, payload_length);
	
	RFSendPacket((char*) &packet, layer0length); // 1 extra byte for the length of the entire packet
	
	tx_requested_by_rx_handler = true;
}

COMMAND(ccrx, "Advanced: for firmware developers", args) {
	CC_SPIStrobe(CCxxx0_SRX);
}
COMMAND(ccfrx, "Advanced: for firmware developers", args) {
	Layer0_OnRX();
}

COMMAND(ccget, "Advanced: for firmware developers", args) {
	// get status of the CC RX FIFO
    // This status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
	uint8_t cc_rxbytes_register = CC_SPIReadStatus(CCxxx0_RXBYTES);	// access the CC register
	
	// extract information from the cc_rxbytes_register
		uint8_t	num_rxbytes 			= cc_rxbytes_register & CCxxx0_NUM_RXBYTES; // extract the number of bytes in the fifo
	
		// extract information about a overflow of the fifo
		bool	rxfifo_overflow 		= false;
		if (cc_rxbytes_register & CCxxx0_RXFIFO_OVERFLOW)
			rxfifo_overflow = true;

	LOG("CC.ov: %u, CC.rx: %u", rxfifo_overflow, num_rxbytes);
	
	uint8_t buffer[64];
	CC_SPIReadBurstReg(CCxxx0_RXFIFO, buffer, num_rxbytes); // Pull all data
	if (num_rxbytes == 65)
		num_rxbytes = 64;
	
	uint8_t i;
	for (i = 0; i < num_rxbytes; i++)
		printf("%.2X ", buffer[i]);
	printf("\r\n");
}

COMMAND(ccoff, "Advanced: for firmware developers", args) {
	CC1100_PowerOff();
}

COMMAND(ccidle, "Advanced: for firmware developers", args) {
	TP2_TOGGLE;
		USART0_SPI_Lock(0x04, 0x00, 0x00);
			CC_SPIStrobe( CCxxx0_SIDLE );							// Change state to Idle
		USART0_SPI_Release();
	
	Timers_Block(TICKS_2SECONDS);

	TP2_TOGGLE;
	CC1100_PowerOff();

	Timers_Block(TICKS_2SECONDS);
	TP2_TOGGLE;

}

COMMAND(cc, "Advanced: for firmware developers", args) {
	printf("%u", CC_SPIReadStatus(CCxxx0_MARCSTATE));
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
