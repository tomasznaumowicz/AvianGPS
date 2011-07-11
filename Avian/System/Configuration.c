#include <stdlib.h>
#include <string.h>

#include "Programming.API.h"
#include "Programming.h"
#include "Commands.API.h"
#include "Logging.h"

#include "Radio/Network.h"

#include "Configuration.h"

#define CONFIGURATION_NODEID_MINIMUM		1
#define CONFIGURATION_NODEID_MAXIMUM		254

extern configuration_t Configuration;
extern appconfig_t AppConfiguration;


void Configuration_Save() {
	// compute checksum
	uint8_t checksum = 0;
	
	register uint8_t index;
	for (index = 0; index < sizeof(configuration_t); index ++) {
		checksum += ((uint8_t*)&Configuration)[index];
	}
	
	// store the configuration into the infomem
	Programming_Infomem_Write(INFOMEM_NODE_CONFIGURATION_OFFSET, 2, &Configuration, sizeof(configuration_t), &checksum, 1);
	
	// update configuration of depentent components
	Network_SetAddress(Configuration.NodeID);
	
	// handle app config
	checksum = 0;
	for (index = 0; index < sizeof(appconfig_t); index ++) {
		checksum += ((uint8_t*)&AppConfiguration)[index];
	}
	// store the configuration into the infomem
	Programming_Infomem_Write(INFOMEM_APP_CONFIGURATION_OFFSET, 2, &AppConfiguration, sizeof(appconfig_t), &checksum, 1);
}

bool _Configuration_Read() {
	//compute checksum of the Configuration from the infomem
	uint8_t buffer[sizeof(configuration_t)];
	uint8_t bufferAppConfig[sizeof(appconfig_t)];
	Programming_Infomem_Read(INFOMEM_NODE_CONFIGURATION_OFFSET, buffer, sizeof(configuration_t));
	Programming_Infomem_Read(INFOMEM_APP_CONFIGURATION_OFFSET, bufferAppConfig, sizeof(appconfig_t));
	
	memset(&AppConfiguration, 0, sizeof(appconfig_t));
	
	uint8_t checksum_infomem = 0;
	uint8_t checksumAppConfig_infomem = 0;
	
	register uint16_t index;
	for (index = 0; index < sizeof(configuration_t); index ++) {
		checksum_infomem += buffer[index];
	}
	for (index = 0; index < sizeof(appconfig_t); index ++) {
		checksumAppConfig_infomem += bufferAppConfig[index];
	}
	
	//compare with the stored checksum
	uint8_t checksum_stored;
	uint8_t checksumAppConfig_stored;
	
	Programming_Infomem_Read(INFOMEM_NODE_CONFIGURATION_OFFSET + sizeof(configuration_t), &checksum_stored, 1);
	Programming_Infomem_Read(INFOMEM_APP_CONFIGURATION_OFFSET + sizeof(appconfig_t), &checksumAppConfig_stored, 1);
	
	if (checksum_infomem != checksum_stored)
		return false;
	
	memcpy(&Configuration, buffer, sizeof(configuration_t));
	
	if (checksumAppConfig_infomem == checksumAppConfig_stored) {
		memcpy(&AppConfiguration, bufferAppConfig, sizeof(appconfig_t));
	}
	
	return true;
}

void Configuration_Init() {
	
	if (_Configuration_Read())
		return;					// configuration settings read fro the infomem: ok
	
	// failed to read the configuration. set the default one and save it
	Configuration.NodeID = 1;
	
	Configuration_Save();
}

void Configuration_Log() {
	LOG("NodeID: %u", Configuration.NodeID);
}

COMMAND(id, "Configuration: node id", cmd_args) {
	// check whether the NodeID change was requested
	if (cmd_args->ArgumentCount == 1) {
		uint16_t newNodeID = atoi(cmd_args->ArgumentArray[0]);
		if (newNodeID >= CONFIGURATION_NODEID_MINIMUM && newNodeID <= CONFIGURATION_NODEID_MAXIMUM) {
			Configuration.NodeID = newNodeID; 
			Configuration_Save();
		}
	}
	
	snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength, "%u", Configuration.NodeID);
}
/*
COMMAND(txpower, cmd_args) {
	// check whether the NodeID change was requested
	if (cmd_args->arguments_count == 1) {
		uint16_t newTxPower = atoi(cmd_args->arguments);
		if (newTxPower >= 0 && newTxPower <= 12) {
			Configuration.RadioTransmitPower = newTxPower; 
			Configuration_Save();
		}
	}
	
	snprintf(cmd_args->response, cmd_args->response_length, "%u", Configuration.RadioTransmitPower);
}
*/


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
