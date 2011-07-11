/**
 * @file		
 * @brief		@b System: Support for local and remote commands. Read the \ref CommandsIntro page for introduction to commands.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef COMMANDS_API_H_
#define COMMANDS_API_H_

#include "Types.h"

/**
 * @name		Default configuration.
 * 
 * You can overwrite this configuration by putting the same defines with adjusted values in your \ref AppConfig.h file.
 * 
 * @{
 */

#ifndef COMMAND_HELP_ENABLED
	#define COMMAND_HELP_ENABLED			1	///< States whether the "help" command should provide additional information about commands. Enabling this option might increase program size. 
#endif

/** @} */

/**
 * @ingroup NetworkProtocols
 * @brief	This protocol id is reserved for the Command Engine
 */
#define		NETWORK_PROTOCOL_COMMAND_REQUEST		2		

/**
 * @ingroup NetworkProtocols
 * @brief	This protocol id is reserved for the Command Engine
 */
#define		NETWORK_PROTOCOL_COMMAND_RESPONSE		3

/**
 * @brief @b Handler: Definition of the handler for previewing commands submitted to the command engine. Visit the \ref Handlers module for more details.
 * 
 * Two parameters are supported:
 * 	- the id of a node that requested the command execution
 * 	- the entire command string (command name and command arguments) 
 * 
 * Returns:
 *  - bool value specifying whether the execution of the command shall be aborted (true stands for abort)
 */
typedef bool (*fp_commandpreview_handler_t)(uint8_t , const char*);

/**
 * @ingroup	Handlers

 * @brief @b Handler: Preview commands submitted to the command engine (see \ref Handlers for more handlers).
 * 
 * You can access a command that was submitted for execution and deciede whether
 * it should be executed or not. Main use of this feature is logging.
 * 
 * Two parameters are supported:
 * 	- the id of a node that requested the command execution
 * 	- the entire command string (command name and command arguments) 
 * 
 * Returns:
 *  - bool value telling whether the execution of the command shall be aborted (true stands for abort)
 * 
 * @remarks		Only one handler can be registered.
 */
fp_commandpreview_handler_t Handler_Command_Preview;

/**
 * @brief		Specifies the maximal number of arguments that the command engine will tokenize and forward to the command handler. You can still access the entire argument string by working with the \ref command_args_t 
 */
#define COMMAND_MAX_ARGUMENT_COUNT				8

/**
 * @brief		Specifies the maximal length of a command.
 */
#define COMMAND_MAX_COMMAND_NAME_LENGTH			8

/**
 * @brief		The structure describing command arguments passed to the command hanlder.
 * 
 * This structure provides you with details about the command parameters. You can also access a pointer to the response string that can be filled inside of the command handler.
 * The content of the response will be either printed out locally, in a case of a local command exectutin, or transmitted over the radio, in the case of
 * a remote command execution.
 */
typedef struct {
	uint8_t			ExecutingNode;									///< The id of a device where the command is supposed to be executed.
	const char*		CommandName;									///< The name of the command, e.g. "app".
	const char*		ArgumentString;									///< Provides the argument string for further processing within the command handler.
	char*			ArgumentArray[COMMAND_MAX_ARGUMENT_COUNT];		///< Provides tokenized arguments (separated by space or a series of spaces) for further processing within the command handler.
	uint8_t			ArgumentCount;									///< Provides a number of detected and tokenized command arguments that are provided in the \ref ArgumentArray
	char*			ResponseString;									///< Points to an array that can be filled with a string by a command handler. The string will be processed as a command response.
	uint8_t			ResponseStringMaxLength;						///< Provides the maximual number of characters that can be stored in the command response. The command handler must make sure it's not writing more characters to the response buffer.
} command_args_t;








// exclude the rest from the public documentation

#ifndef DOXYGEN_PUBLIC_DOC

/**
 * @brief		Definition of a command handler
 */
typedef void (*fp_command_t) (const command_args_t*);

/**
 * @brief		Describes a command handler. This struct is used to store information about a command.
 */
typedef struct {
	const fp_command_t		handler;										///< Pointer to the command handler function.
	const char				commandName[COMMAND_MAX_COMMAND_NAME_LENGTH];	///< Contains the name of the command.
	const char*				commandHelp;									///< Contains the command help used by the "help" command.
} command_t;

#if COMMAND_HELP_ENABLED

	#define COMMAND(cmd_name, cmd_help_text, cmd_args)			const char _cmd_ ## cmd_name ## _help[] = cmd_help_text ; \
																extern void _cmd_ ## cmd_name ## _handler (const command_args_t* cmd_args ); \
																__attribute__((section(".commands"))) const command_t _cmd_ ## cmd_name ## _registration = { _cmd_ ## cmd_name ## _handler, #cmd_name, _cmd_ ## cmd_name ## _help }; \
																__attribute__((noinline)) \
																void _cmd_ ## cmd_name ## _handler(const command_args_t* cmd_args)

#else

	#define COMMAND(cmd_name, cmd_help_text, cmd_args)			\
																extern void _cmd_ ## cmd_name ## _handler (const command_args_t* cmd_args ); \
																__attribute__((section(".commands"))) const command_t _cmd_ ## cmd_name ## _registration = { _cmd_ ## cmd_name ## _handler, #cmd_name, NULL }; \
																__attribute__((noinline)) \
																void _cmd_ ## cmd_name ## _handler(const command_args_t* cmd_args)

#endif

#endif

#endif /*COMMANDS_API_H_*/


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
