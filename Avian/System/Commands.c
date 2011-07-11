#include <stdlib.h>
#include <string.h>

#include "Radio/Network.h"
#include "Logging.h"
#include "Timers.h"

#include "Commands.API.h"
#include "Commands.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

static fp_command_t _Commands_GetCommand(const char* name) {
	//find to string function
	extern command_t __commands_start;
	extern command_t __commands_end;
	
	command_t* current;
	
	for (current = &__commands_start; current <= &__commands_end; current++) {
		if (strncmp(current->commandName, name, COMMAND_MAX_COMMAND_NAME_LENGTH) == 0) {
			return current->handler;
		}
	}
	
	return NULL;
}

static void _Commands_PrintResponse(uint8_t executing_node, const char* command_name, const char* response) {
	if (response[0] != '\0')
		printf("[%u:%." TOSTRING(COMMAND_MAX_COMMAND_NAME_LENGTH) "s] %s\r\n", executing_node, command_name, response);
	else
		printf("[%u:%." TOSTRING(COMMAND_MAX_COMMAND_NAME_LENGTH) "s] OK\r\n", executing_node, command_name);
}

static bool _Commands_SendResponse(uint8_t destination, const char* command_name, const char* response) {
	LOGDBG_SWITCHED(DEBUG_COMMANDS_ENABLED, "[ct] d:%u c:'%-" TOSTRING(COMMAND_MAX_COMMAND_NAME_LENGTH) "." TOSTRING(COMMAND_MAX_COMMAND_NAME_LENGTH) "s' '%s'", destination, command_name, response);

	network_payload_t payload;
	Network_InitSendArgs(&payload);
	
	payload.Buffers[0].Size = COMMAND_MAX_COMMAND_NAME_LENGTH;
	payload.Buffers[0].Buffer = (void*) command_name;
	
	payload.Buffers[1].Size = strlen(response) + 1; // 1 for the terminating 0x00 that's contained in the response string
	payload.Buffers[1].Buffer = (void*) response;

	if (payload.Buffers[1].Size > LAYER1_PAYLOAD_SIZE - payload.Buffers[0].Size) {
		// make sure, the response is trunctated
		payload.Buffers[1].Size = LAYER1_PAYLOAD_SIZE - payload.Buffers[0].Size;
		// it's not possible to trunctate the payload.Buffers[1].Buffer, since we shouldn't modify the data 
	}
	return Network_SendWithoutACK(destination, NETWORK_PROTOCOL_COMMAND_RESPONSE, &payload);
}


static bool _Commands_ExecuteCommand(uint8_t requestedByNodeID, const char* input) {
	// preview the input by external command previewer (if registered)
	// it's possible that the command requested is actually not available
	
	if (Handler_Command_Preview != NULL) {
		bool aborted = false;
		
		if (input[0] != 0x00) // test only if the input string is not empty
			aborted = Handler_Command_Preview(requestedByNodeID, input);
		
		if (aborted)
			return false;
	}
	
	//tokenize input
	//   <command><space><arg><space><arg><space><arg>\0
	uint8_t inputLength = strlen(input);
	char inputCopy[inputLength];
	
	strncpy(inputCopy, input, inputLength);
	inputCopy[inputLength] = 0x00;  // terminate the string;

	// identify the command name and the arguments of the command
	const char* command = NULL;
	const char* arguments = NULL;
	
	// extract the command string and have it separate
	command = strtok(inputCopy, " "); //input gets modified
	if (command == NULL)
		return false;
	
	// measure the length of the command
	// the input string might contain parameters after the command string.

	uint8_t commandLength = strlen(command);
	arguments = &(input[commandLength]) + 1; // skip the ' ' after the command
	
	// but there is a ' ' after the command that wasn't counted with the strlen command. so this ' ' needs to be skipped manually (this happens because length isn't == to the index)
	// we could search for the next non' ' character but it's not really necessary here and would cost extra bytes in the program memory.
	
	// prepare the command_args_t struct for the command handler
	command_args_t commandArgs;
	
	commandArgs.ExecutingNode = Configuration.NodeID;
	commandArgs.CommandName = command;
	commandArgs.ArgumentString = arguments;
	commandArgs.ArgumentCount = 0;
	
	// tokenize parameters
	char* currentArgument = strtok(NULL, " ");
	while
		(
			(commandArgs.ArgumentCount < COMMAND_MAX_ARGUMENT_COUNT)
			&&
			(currentArgument != NULL)
		) {
			// arguments is != null so there is an argument
			commandArgs.ArgumentArray[commandArgs.ArgumentCount] = currentArgument;			// store this token
			commandArgs.ArgumentCount++;													// increase the argument counter
			
			currentArgument = strtok(NULL, " ");											// extract next token
		}
	
	//prepare the response buffer
	char response_buffer[COMMAND_RESPONSE_LENGTH]; response_buffer[0] = '\0';
	commandArgs.ResponseString = response_buffer;
	commandArgs.ResponseStringMaxLength = COMMAND_RESPONSE_LENGTH;
	
	// commandArgs are prepared for execution
	LOGDBG_SWITCHED(DEBUG_COMMANDS_ENABLED, "[cs] f:%u t:%u c:'%-" TOSTRING(COMMAND_MAX_COMMAND_NAME_LENGTH) "." TOSTRING(COMMAND_MAX_COMMAND_NAME_LENGTH) "s' a:%u '%s'", requestedByNodeID, commandArgs.ExecutingNode, commandArgs.CommandName, commandArgs.ArgumentCount, commandArgs.ArgumentString);
	uint8_t testindex;
	for (testindex = 0; testindex < commandArgs.ArgumentCount; testindex++) {
		LOGDBG_SWITCHED(DEBUG_COMMANDS_ENABLED, "[cs] a:%u '%s'", testindex, commandArgs.ArgumentArray[testindex]);
	}
	
	// find the command handler
	fp_command_t cmd_handler = NULL;
	
	cmd_handler = _Commands_GetCommand(commandArgs.CommandName);
	
	// be aware of command's side effects, eg. change of the NodeID. The NodeID is used to deciede what to do with the command's response
	uint8_t _nodeID = Configuration.NodeID;

#if DEBUG_COMMANDS_ENABLED
	uint32_t benchmark_ticks_start = COPY_CURRENT_TAR;
#endif
	
	if (cmd_handler == NULL) {
		snprintf(commandArgs.ResponseString, commandArgs.ResponseStringMaxLength, "'%." TOSTRING(COMMAND_MAX_COMMAND_NAME_LENGTH) "s' unknown", commandArgs.CommandName);
	} 
	else {
		// the command will be executed now
		cmd_handler(&commandArgs);
	}
	
#if DEBUG_COMMANDS_ENABLED
	uint32_t benchmark_ticks_stop = COPY_CURRENT_TAR;
#endif

	if (requestedByNodeID == _nodeID) {
		_Commands_PrintResponse(commandArgs.ExecutingNode, commandArgs.CommandName, commandArgs.ResponseString);
		LOGDBG_SWITCHED(DEBUG_COMMANDS_ENABLED, "[c] %lu ticks", benchmark_ticks_stop - benchmark_ticks_start);
	}
	else {
		return _Commands_SendResponse(requestedByNodeID, commandArgs.CommandName, commandArgs.ResponseString);
	}

	return true;
}

bool Commands_ProcessInput(const char* input) {
	char* pchar = (char*) input;

	// check whether a NodeID was specified and identify the NodeID of the node selected for execution of the command:
	// if the input starts with @ then the command should be executed on the selected node
	// (allow to select the current node)
	uint8_t executing_node = Configuration.NodeID;
	if (*pchar == '@') {
		pchar++;						// skip the '@'
		executing_node = atoi(pchar);
		while (*pchar != ' ')			// skip the nodeid
			pchar++;
		while (*pchar == ' ')			// skip the white spaces after the id
			pchar++;
	}
	
	// the NodeID of the executing node was found, decide whether to process the input locally or to forward it to tne requested noe:
	if (executing_node == Configuration.NodeID)
		return _Commands_ExecuteCommand(Configuration.NodeID, pchar);
	else {
		network_payload_t payload;
		Network_InitSendArgs(&payload);
		
		uint16_t request_length = strlen(pchar) + 1; // + 1 because the terminating '\0' should be transmitted as well.
		if (request_length > LAYER1_PAYLOAD_SIZE)
			return false; // the termiating 0x00 can't be transmitted and this can lead to errors on the receiver side.
		
		payload.Buffers[0].Size = (uint8_t) request_length;
		payload.Buffers[0].Buffer = pchar;
		
		return Network_SendWithoutACK(executing_node, NETWORK_PROTOCOL_COMMAND_REQUEST, &payload);
	}
}

static void _Commands_ProcessRadioRequest(uint8_t source, const uint8_t* data) {
	// example: data == "id 3"
	// tries to execute the command
	
	char* data_copy = (uint8_t*) data; //holds the reference to the first character (the command name)
	bool executed = _Commands_ExecuteCommand(source, data);
	if (executed)
		return; //everything went fine, the response was sent.

	//TODO: this will be dead code now, deal with this when also working on remote commands etc.
	// execution has failed, send an error message
	_Commands_SendResponse(source, data_copy, "ERROR");
}

static void _Commands_ProcessRadioResponse(uint8_t source, const uint8_t* data) {
	_Commands_PrintResponse(source, &data[0], &data[COMMAND_MAX_COMMAND_NAME_LENGTH]);
}

void Commands_Init() {
	Network_RegisterHandler(NETWORK_PROTOCOL_COMMAND_REQUEST, _Commands_ProcessRadioRequest);
	Network_RegisterHandler(NETWORK_PROTOCOL_COMMAND_RESPONSE, _Commands_ProcessRadioResponse);
	
	Handler_Command_Preview = NULL;
}

COMMAND(help, "", cmd_arg)
{
	extern command_t __commands_start;
	extern command_t __commands_end;
	
	command_t* current;
	
	printf("Commands:\r\n");

	for (current = &__commands_start; current < &__commands_end; current++) {
		printf(" %-" TOSTRING(COMMAND_MAX_COMMAND_NAME_LENGTH) "." TOSTRING(COMMAND_MAX_COMMAND_NAME_LENGTH) "s [%p] %s\r\n", current->commandName, current->handler, current->commandHelp);
	}
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
