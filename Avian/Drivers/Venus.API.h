/**
 * @file
 * @brief		@b Driver: Venus GPS. The GPS receiver.
 * @ingroup		DriverAPI
 */

#ifndef VENUS_API_H_
#define VENUS_API_H_

#include <stdint.h>
#include "../System/Types.h"
#include "../System/RTC.h"

#include "LogEngine.API.h"

/**
 * @brief @b Handler: Definition of the handler for processing valid NMEA RMC sentences
 * 
 * Three parameters are supported:
 * 	- the received NMEA string
 * 	- the decoded time as time_t
 * 	- the decoded time as rawtime_t 
 */
typedef void(*fp_gps_rmc_handler_t) (const char*, const time_t*, const rawtime_t*);

/**
 * @ingroup	Handlers

 * @brief @b Handler: Access to valid NMEA RMC sentences (see \ref Handlers for more handlers).
 * 
 * You can access the decoded NMEA sentence in your application code by assigning 
 * your handler to this variable.
 * 
 * Three parameters are supported:
 * 	- the received NMEA string
 * 	- the decoded time as time_t
 * 	- the decoded time as rawtime_t
 * 
 * Example:
 * @code
 * // sample RMC handler
 * void MyHandler(const char* nmea, const time_t* time, const rawtime_t* rawtime) {
 *	printf("received valid RMC!\r\n");
 * }
 * // somewhere in your application code, e.g. in the Application_Init() function
 * // ..
 * Handler_GPS_NMEA_RMC = MyHandler;
 * // ...
 * @endcode
 * 
 * @remarks		Only one handler can be registered.
 */
fp_gps_rmc_handler_t Handler_GPS_NMEA_RMC;


/**
 * @brief @b Handler: Definition of the handler for processing valid NMEA GGA sentences
 * 
 * Following parameters are provided:
 * 	- the received NMEA string
 * 	- the decoded position as a log entry structure 
 */
typedef void(*fp_gps_gga_handler_t) (const char*, const logentry_t*);

/**
 * @ingroup	Handlers
 * @brief @b Handler: Access to valid NMEA GGA sentences (see \ref Handlers for more handlers).
 * 
 * You can access the decoded NMEA sentence in your application code by assigning 
 * your handler to this variable.
 * 
 * Following parameters are provided:
 * 	- the received NMEA string
 * 	- the decoded position as a log entry structure 
 * 
 * Example:
 * @code
 * 		// sample GGA handler
 * 		void MyHandler(const char* nmea, const logentry_t* entry) {
 * 			printf("received valid GGA!\r\n");
 * 		}
 * 
 * 		// somewhere in your application code, e.g. in the Application_Init() function
 * 		// ..
 * 		Handler_GPS_NMEA_GGA = MyHandler;
 * 		// ...
 * @endcode
 * 
 * @remarks		Only one handler can be registered.
*/
fp_gps_gga_handler_t Handler_GPS_NMEA_GGA;

/**
 * @name		Managing the Venus GPS device
 * 
 * @{
 */

/**
 * @brief		Configures the GPS device.
 * 
 * This function must be called during the application initialization phase.
 * 
 * @remarks		This function turns off the GPS device after the initialization.
 * 
 * @return		true on success, false on failure.
 */
bool Venus_Init();

/**
 * @brief		Turns on the GPS receiver.
 */
void Venus_PowerOn();

/**
 * @brief		Initializes NMEA processing. Required for location data.
 * 
 * You need to register handlers for \ref Handler_GPS_NMEA_RMC or \ref Handler_GPS_NMEA_GGA in order to 
 * process the incoming data in your application.
 * 
 * The system is processing RMC sentences and will update the system time. 
 * 
 * @remark		This function expects that the GPS receiver is powered on.
 */
void Venus_StartProcessingNMEA();

/**
 * @brief		Turns off the GPS receiver.
 */
void Venus_PowerOff();

/** @} */


/**
 * @name		Configuring the Venus GPS device
 * 
 * @{
 */

/**
 * @brief		Changes the position update rate of the GPS device.
 * 
 * Allows to change the position update rate. Possible values for the position update rate:
 * - 1
 * - 2
 * - 4
 * - 5
 * - 8
 * - 10
 *  
 * @remarks		This function expects that the GPS receiver is powered on.
 * 				This function aborts the NMEA processing.
 * 
 * @param[in]	requestedPositionUpdateRate		The requested udpate rate.
 */
void Venus_ChangePositionUpdateRate(uint8_t requestedPositionUpdateRate);

/** @} */

#endif /*VENUS_API_H_*/

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
