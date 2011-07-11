/**
 * @file		AppConfig.h
 * @brief		Main configuration file for the System and the Application components.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * The AppConfig.h file is evaluated first, as it is loaded using the --preinclude switch.
 * All definitions required to configure the application, drivers, and system should be placed in this file.
 * 
 * Review the \ref APPConfig module as well.
 */

#ifndef APPCONFIG_H_
#define APPCONFIG_H_


#include <stdint.h>

/**
 * @defgroup 	APPConfig Application Configuration
 * 
 * @brief		This module presents ways of configuring an application.
 * 
 * The System API and Drivers provide a set of definitions that can be tuned in order to modify the configuration
 * of the drivers and the system. The user can also specify his own Defines that
 * are required for his application to work properly. All relevant details can
 * be found in the file \ref AppConfig.h
 * 
 * The Application Configuration module also allows to define the persisten application settings.
 */

 /**
 * @name Persistent application settings
 * 
 * The System API is taking care of persistent storage of application settings. The user defines his set of application
 * settings by modifying the definition of the \ref appconfig_t struct in the \ref AppConfig.h file.
 * @n@n
 * After the power on event of the device, application settings are loaded and made available in the global \ref AppConfiguration variable.
 * Any changes of the application settings should be performed on the \ref AppConfiguration variable and persisted by calling the
 * Configuration_Save() function.
 * 
 * Example 1: Accessing the Configuration:
 * @code
 * LOG("Threshold: %u", AppConfiguration.Threshold);
 * @endcode
 * 
 * Example 2: Updating the Configuration:
 * @code
 * AppConfiguration.Threshold = 120;
 * Configuration_Save();
 * @endcode
 * 
 * @{
 */

/**
 * @brief User defined set of application settings. Settings are stored in \ref AppConfiguration variable and persisted by the System API.
 * 
 * @remarks The appconfig_t struct must not be empty.
 */
typedef struct {
	uint32_t		Tracking_PowerUpDelay;				///< Initial sleep time before enabling GPS for the first time 			
	uint32_t		Tracking_IdleTime;					///< Sleep time between GPS activity
	uint32_t		Tracking_MaxTTFTime;				///< Maximal time spent for waiting on a fix
	uint32_t		Tracking_LoggingTime;				///< Defines the time that should be spent logging the position after a fix was acqquired
	uint32_t		Radio_PowerUpDelay;					///< The time to stay in RX after power on event of the device
	uint32_t		Radio_WaitForBaseTime;						///< The time to wait for response from the base station
	uint32_t		Radio_IdleTime;				///< Time to sleep between RX events
	uint8_t			Suppress_Activation;
	uint8_t			Suppress_Deactivation;
	uint8_t			Suppress_Configured;
	uint8_t			Radio_Configured;	///< useful to detect whether the configuration was already set at least once (otherwise use defaults in the app code)
	uint8_t			Tracking_Configured;		///< useful to detect whether the configuration was already set at least once (otherwise use defaults in the app code)
} appconfig_t;

/**
 * @ingroup APPConfig
 * @brief	Global variable holding user defined application settings
 */
appconfig_t AppConfiguration;


/** @} */

/**
 * @name	System configuration
 * 
 * The following definitions can be modified in order to tune the behavior and the configuration of the System and Drivers.
 * If you aren't sure about the proper value of the Define you can just remove it from the \ref AppConfig.h file,
 * in such case, a default value will be used.

 * 
 * @{
 */

/**
 * @ingroup	APPConfig
 * @{
 */

#define UART1_RX_BUFFER_SIZE				512	///< Size of the receive buffer used to store NMEA data received from the GPS device. Note: one NMEA sentence contains of around 80 characters. 

/**
 * @brief NMEA RMC sentence update rate. Read the full description for more details.
 * 
 * The RMC sentence contains date and time information received from the GPS system.
 * The frequency with which the RMC sentece is proviced by the GPS receiver depends on the system position rate.
 * The number provided here specifies the requency relative to the system position rate.
 * 
 * Example:
 * 	Setting the value to 200 means, the RMC sentence will be delivered every 200th time. Depending on the system position rate, e.g. when
 * 	the system position rate is set to 10Hz, the RMC sentence will be delivered every 20 seconds
 *  and if the system position rate is set to 1Hz, the RMC sentence will be delivered every 200 seconds (3 minutes 20 seconds).
 */
#define	VENUS_RMC_UPDATE_RATE		2

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

/**
 * @brief Commands Engine: Controls the functionality of the "help" command
 *
 * Setting it to 1 causes the engine to print short help about every command
 * when the help command is executed
 * 
 * @remarks 	The description of a command requireds extra space when your application is compiled.
 * 				Set this switch to 0 if you are running low of program memory.
 */
#define COMMAND_HELP_ENABLED				1

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

/**
 * @brief		Controls the LED1 feature on the device.
 * 
 * You can enable LED1 usage in the software by setting this switch to 1. Setting
 * this switch to 0 will disable the usage of the LED1 by the software.
 * 
 * @remarks		The periodic "alive" blink signal that happens every 8 seconds will execute independent of this setting. 
 */
#define LED1_ENABLED						1
	
/**
 * @brief		Controls the LED2 feature on the device.
 * 
 * You can enable LED2 usage in the software by setting this switch to 1. Setting
 * this switch to 0 will disable the usage of the LED2 by the software.
 */
#define LED2_ENABLED						1

/** @} */
/** @} */


#endif /*APPCONFIG_H_*/

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
