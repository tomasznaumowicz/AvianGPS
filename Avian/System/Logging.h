/**
 * @file
 * @brief		@b System: Support for debug logging to the serial output and over the radio (if enabled)
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * Simple support for debug log output. Several different macros were made available, review the definitions below.
 * 
 * Example:
 * @code
 * // current state of the algorithm
 * uint8_t currentState;
 * // ... algorithm executes, the state of the algorithm changes
 * //...
 * LOG("Current State = %u", currentState); // prints the current state to the debug output
 * @endcode
 * 
 * @section sitac Short introduction to ANSI colors usage
 * 
 * You can use ANSI colors with certain terminals. The default implementation of the debug logging engine
 * uses this feature to display regular log entries, error log entries and debug log entries in different colors.
 * You might want to modify this behaviour.
 * 
 * The code below is an example of how ANSI colors can be used:
 * 
 * @code 
 *
 * #define LOGSTYLE_RESET			0
 * #define LOGSTYLE_BRIGHT		1
 * #define LOGSTYLE_DIM			2
 * #define LOGSTYLE_UNDERLINE		3
 * #define LOGSTYLE_BLINK			4
 * #define LOGSTYLE_REVERSE		7
 * #define LOGSTYLE_HIDDEN		8
 * 
 * #define LOGSTYLE_BLACK			0
 * #define LOGSTYLE_RED			1
 * #define LOGSTYLE_GREEN			2
 * #define LOGSTYLE_YELLOW		3
 * #define LOGSTYLE_BLUE			4
 * #define LOGSTYLE_MAGENTA		5
 * #define LOGSTYLE_CYAN			6
 * #define LOGSTYLE_WHITE			7
 *
 * void textcolor(int attr, int fg, int bg);
 * 
 * void textcolor(int attr, int fg, int bg) {
 * 	char command[13];
 * 	sprintf(command, "%c[%d;%d;%dm", 0x1B, attr, fg + 30, bg + 40);
 * 	printf("%s", command);
 * }
 * @endcode
 * 
 * Another example, direct in the code:
 * 
 * @code
 * printf("\033[1;34mtesttest\033[0m \a \r\n");
 * @endcode
*/

#ifndef LOGGING_H_
#define LOGGING_H_

#include <msp430x16x.h>
#include <stdio.h>

#include "Configuration.h"


/**
 * @name		Default configuration.
 * 
 * You can overwrite this configuration by putting the same defines with adjusted values in your \ref AppConfig.h file.
 * 
 * @{
 */

#ifndef LOGGING_ENABLE_DEBUG_OUTPUT
	/**
	 * @brief Logging Engine: Controlls the verbose debug output
	 *
	 * Many components of the System components and Drivers generate extensive debug output over the
	 * serial port. The debug output feature can be disabled using this switch.
	 * 
	 * Printing text is expensive in terms of code size, execution time (printing text is slow) and as 
	 * a consequence, in terms of power consumption and life time of the device.
	 * 
	 * Please consider this during the application development and thing about disabling the debug output
	 * when deploying the device.
	 * 
	 * @remarks 	Remember to test the entire application after disabling this feature.
	 * 				The execution time of some functions will be reduced. This can cause a change in behavior of your application.
	 * 
	 * The debug output feature can be used in your application.
	 * 
	 * Example:
	 * @code
	 * // current state of the algorithm
	 * uint8_t currentState;
	 * // ... algorithm executes, the state of the algorithm changes
	 * //...
	 * LOGDBG("Current State = %u", currentState); // prints the current state to the debug output
	 * @endcode
	 */
	#define LOGGING_ENABLE_DEBUG_OUTPUT			1
#endif

#ifndef LOGGING_ENABLE_LOG_OUTPUT
	/**
	 * @brief Logging Engine: Controlls the log output
	 *
	 * Components of the System components and Drivers generate some log output using this feature. The log output 
	 * feature can be disabled using this switch.
	 * 
	 * Printing text is expensive in terms of code size, execution time (printing text is slow) and as 
	 * a consequence, in terms of power consumption and life time of the device.
	 * 
	 * Please consider this during the application development and limit the amout of printed text.
	 * 
	 * @remarks 	Remember to test the entire application after disabling this feature.
	 * 				The execution time of some functions will be reduced. This can cause a change in behavior of your application.
	 * 
	 * The log output feature can be used in your application.
	 *  
	 * Example:
	 * @code
	 * LOG("Experiment: done"); // prints information using the log output
	 * @endcode
	 */
	#define LOGGING_ENABLE_LOG_OUTPUT			1
#endif

#ifndef LOGGING_ENABLE_REMOTE_LOG_OUTPUT
	/**
	 * @brief Logging Engine: Controlls the radio log output
	 *
	 * You can use this feature to print text on remote devices. This can be useful to distribute
	 * status information during the development phase. The message will be transmitted over the radio
	 * as a broadcast. All devices that hear the message and have this feature enabled as well, will 
	 * print the received message over the serial port.
	 * 
	 * Printing and transmitting text over radio is expensive in terms of code size, execution time (printing text is slow) and as 
	 * a consequence, in terms of power consumption and life time of the device.
	 * 
	 * Please consider this during the application development and limit the usage of this feature.
	 * 
	 * @remarks 	Remember to test the entire application after disabling this feature.
	 * 				The execution time of some functions will be reduced. This can cause a change in behavior of your application.
	 * 
	 * The log output feature can be used in your application.
	 *  
	 * Example:
	 * @code
	 * LOGREMOTE("Received format request.");
	 * @endcode
	 */
	#define LOGGING_ENABLE_REMOTE_LOG_OUTPUT	1
#endif

/** @} */

/**
 * @brief Definition of a log handler.
 */
typedef void(*fp_logHandler_t) (const char*, ...);

#if (LOGGING_ENABLE_DEBUG_OUTPUT | LOGGING_ENABLE_LOG_OUTPUT | LOGGING_ENABLE_REMOTE_LOG_OUTPUT)
	#define LOGGING_ENABLED		1
#endif

#if (LOGGING_ENABLED)

/**
 * @ingroup NetworkProtocols
 * @brief	This protocol id is reserved for the debug logging engine
 */
#define		NETWORK_PROTOCOL_LOG					1	

/**
 * @ingroup	Handlers
 * @brief @b Handler: Handling local debug output (see \ref Handlers for more handlers).
 * 
 * The system provides its own debug output support but you can replace it with your own
 * implementation.
 * 
 * Following parameters are provided:
 * 	- the format string
 * 	- variable list of parameters 
 * 
 * @remarks		Only one handler can be registered.
*/
fp_logHandler_t Handler_Logging_Local;

/**
 * @ingroup	Handlers
 * @brief @b Handler: Handling remote debug output (see \ref Handlers for more handlers).
 * 
 * The system provides its own remote output support but you can replace it with your own
 * implementation.
 * 
 * Following parameters are provided:
 * 	- the format string
 * 	- variable list of parameters 
 * 
 * @remarks		Only one handler can be registered.
*/

fp_logHandler_t Handler_Logging_Remote;

/**
 * @brief	Default system local logging handler.
 */
void Logging_HandlerUart(const char *fmt, ...);
	
/**
 * @brief	Default system remote logging handler.
 */
void Logging_HandlerRadio(const char *fmt, ...);

#endif

#if LOGGING_ENABLE_LOG_OUTPUT

/**
 * @name		Local debug output
 * @{
 */

/**
 * @brief		This macro allows you to provide simple output to the user. The actual output is realized via a registered \ref Handler_Logging_Local
 * 
 * As long as the debug logging is enabled, you can provide debout output to users of the application.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * // current state of the algorithm
 * uint8_t currentState;
 * // ...
 * // ... algorithm executes, the state of the algorithm changes to 5
 * // ...
 * LOG("Current State = %u", currentState); // prints the current state to the debug output
 * @endcode
 * 
 * Prints "[1:L] Current State = 5" in the default output.
 */
	#define LOG(_format, args...)						Handler_Logging_Local("[%u:L] " _format, Configuration.NodeID, ##args)

/**
 * @brief		This macro allows you to provide simple output to the user. This output will be formatted in red and generate a beep signal in the terminal window. The actual output is realized via a registered \ref Handler_Logging_Local
 * 
 * As long as the debug logging is enabled, you can provide debout output to users of the application.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * LOGERR("Init Fail!");
 * @endcode
 * 
 * Prints "[1:L] Init Fail!" in red.
 */
	#define LOGERR(_format, args...)					Handler_Logging_Local("\a\033[1;31m[%u:L] " _format "\033[0m", Configuration.NodeID, ##args)

/**
 * @brief		This macro allows you to provide simple output to the user. This output will be formatted in yellow and generate a beep signal in the terminal window. The actual output is realized via a registered \ref Handler_Logging_Local
 * 
 * As long as the debug logging is enabled, you can provide debout output to users of the application.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * LOGATT("Init Success!");
 * @endcode
 * 
 * Prints "[1:L] Init Success!" in yellow.
 */
	#define LOGATT(_format, args...)					Handler_Logging_Local("\a\033[1;33m[%u:L] " _format "\033[0m", Configuration.NodeID, ##args)

/** @} */

#else

/**
 * @name		Local debug output
 * @{
 */

/**
 * @brief		This macro allows you to provide simple output to the user. The actual output is realized via a registered \ref Handler_Logging_Local
 * 
 * As long as the debug logging is enabled, you can provide debout output to users of the application.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * // current state of the algorithm
 * uint8_t currentState;
 * // ...
 * // ... algorithm executes, the state of the algorithm changes to 5
 * // ...
 * LOG("Current State = %u", currentState); // prints the current state to the debug output
 * @endcode
 * 
 * Prints "[1:L] Current State = 5" in the default output.
 */
	#define LOG(...)
/**
 * @brief		This macro allows you to provide simple output to the user. This output will be formatted in red and generate a beep signal in the terminal window. The actual output is realized via a registered \ref Handler_Logging_Local
 * 
 * As long as the debug logging is enabled, you can provide debout output to users of the application.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * LOGERR("Init Fail!");
 * @endcode
 * 
 * Prints "[1:L] Init Fail!" in red.
 */
	#define LOGERR(...)
/**
 * @brief		This macro allows you to provide simple output to the user. This output will be formatted in yellow and generate a beep signal in the terminal window. The actual output is realized via a registered \ref Handler_Logging_Local
 * 
 * As long as the debug logging is enabled, you can provide debout output to users of the application.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * LOGATT("Init Success!");
 * @endcode
 * 
 * Prints "[1:L] Init Success!" in yellow.
 */
	#define LOGATT(...)

/** @} */

#endif

#if (LOGGING_ENABLE_DEBUG_OUTPUT)

/**
 * @name		Local debug output
 * @{
 */

/**
 * @brief		This macro allows you to provide simple output to the user. This output will be formatted in red and generate a beep signal in the terminal window. The actual output is realized via a registered \ref Handler_Logging_Local
 * 
 * As long as the debug logging is enabled, you can provide debout output to users of the application.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * LOGDBG("State Machine: State 34");
 * @endcode
 * 
 * Prints "[1:D] State Machine: State 34" in dark grey.
 */
	#define LOGDBG(_format, args...)					Handler_Logging_Local("\033[1;30m[%u:D]  " _format "\033[0m", Configuration.NodeID, ##args)

/** @} */

#else

/**
 * @name		Local debug output
 * @{
 */

/**
 * @brief		This macro allows you to provide simple output to the user. This output will be formatted in red and generate a beep signal in the terminal window. The actual output is realized via a registered \ref Handler_Logging_Local
 * 
 * As long as the debug logging is enabled, you can provide debout output to users of the application.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * LOGDBG("State Machine: State 34");
 * @endcode
 * 
 * Prints "[1:D] State Machine: State 34" in dark grey.
 */
	#define LOGDBG(...)

/** @} */

#endif

#if (LOGGING_ENABLE_REMOTE_LOG_OUTPUT)

/**
 * @name		Remote debug output
 * @{
 */

/**
 * @brief		This macro allows you to propagate simple output over the radio.
 * 
 * You can use this feature to print debug messages on remote devices. Useful for transmission of notifications.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * LOGREMOTE("device is online");
 * @endcode
 * 
 * Prints "[1:L] device is online" on every device that's in 1-hop neighbourhood of the device with the nodeid 1.
 */
	#define LOGREMOTE(_format, args...)					Handler_Logging_Remote(_format, ##args)

/* @} */

#else

/**
 * @name		Remote debug output
 * @{
 */

/**
 * @brief		This macro allows you to propagate simple output over the radio.
 * 
 * You can use this feature to print debug messages on remote devices. Useful for transmission of notifications.
 * 
 * Example run on devide with nodeid 1:
 * Example:
 * @code
 * LOGREMOTE("device is online");
 * @endcode
 * 
 * Prints "[1:L] device is online" on every device that's in 1-hop neighbourhood of the device with the nodeid 1.
 */
	#define LOGREMOTE(...)

/** @} */

#endif

#if (LOGGING_ENABLE_REMOTE_LOG_OUTPUT)
	#if (LOGGING_ENABLE_LOG_OUTPUT)

/**
 * @name		Remote debug output
 * @{
 */

/**
 * @brief		This macro combines the \ref LOG and the \ref LOGREMOTE macro and their functionality.
 */

		#define LOGREMOTEANDLOCAL(_format, args...)		{ Handler_Logging_Local("[%u:L] " _format, Configuration.NodeID, ##args); Handler_Logging_Remote(_format, ##args); }

/** @} */

	#endif
#else


/**
 * @name		Remote debug output
 * @{
 */

/**
 * @brief		This macro combines the \ref LOG and the \ref LOGREMOTE macro and their functionality.
 */
		#define LOGREMOTEANDLOCAL(...)

/** @} */

#endif



	

#ifndef DOXYGEN_PUBLIC_DOC

#define CAT(prefix, suffix) prefix##suffix
#define _LOGDBG_INTERNAL_1(_format, args...)	LOGDBG(_format,##args)
#define _LOGDBG_INTERNAL_0(...)

#endif


/**
 * @name		Local debug output (switchable)
 * @{
 */

/**
 * @brief The macro can be used to create debug log entries that are easily enabled and disabled in code
 * 
 * The parameter must be a constant/literal in code, e.g. defined somewhere in your application
 * 
 * You can pass one parameter that is either 0 or 1. this parameter will deciede wether the debug entry
 * should be processed by the logging engine or not. Using 0 causes the entire line to be removed by the
 * preprocessor.
 * 
 * Example:
 * @code
 * #define DEBUG_RADIO	1
 * 
 * ...
 * LOGDBG_SWITCHED(DEBUG_RADIO, "init result %u", a_number);
 * ...
 * @endcode
 * 
 * Replacing the DEBUG_RADIO define with 0 will disable the debug output.
 */
#define LOGDBG_SWITCHED(enabled,_format, args...)		CAT(_LOGDBG_INTERNAL_,enabled(_format,##args))

/** @} */

#ifndef DOXYGEN_PUBLIC_DOC

#define _LOGERR_INTERNAL_1(_format, args...)	LOGERR(_format,##args)
#define _LOGERR_INTERNAL_0(...)

#endif

/**
 * @name		Local debug output (switchable)
 * @{
 */

/**
 * @brief The macro can be used to create debug log entries that are easily enabled and disabled in code
 * 
 * The parameter must be a constant/literal in code, e.g. defined somewhere in your application
 * 
 * You can pass one parameter that is either 0 or 1. this parameter will deciede wether the debug entry
 * should be processed by the logging engine or not. Using 0 causes the entire line to be removed by the
 * preprocessor.
 * 
 * Example:
 * @code
 * #define DEBUG_RADIO	1
 * 
 * ...
 * LOGERR_SWITCHED(DEBUG_RADIO, "Error when writing 0xFF");
 * ...
 * @endcode
 * 
 * Replacing the DEBUG_RADIO define with 0 will disable the debug output.
 */
#define LOGERR_SWITCHED(enabled,_format, args...)		CAT(_LOGERR_INTERNAL_,enabled(_format,##args))

/** @} */
	
#endif /*LOGGING_H_*/



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
