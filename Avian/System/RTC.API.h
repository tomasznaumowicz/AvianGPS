/**
 * @file		
 * @brief		@b System: Support for software Real Time Clock.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * Real Time Clock (RTC) supports time representation in the human readable format: HH:MM:SS DD-MM-YYYY.
 * Resolution can be configured using deifines \ref RTC_RESOLUTION_125_MILISECONDS \ref RTC_RESOLUTION_8_MILISECONDS and \ref RTC_RESOLUTION_IN_TICKS
 * where the \ref RTC_RESOLUTION_IN_TICKS is the quickest in terms of computation costs.
 * You need to set the preferred option to 1 in your \ref AppConfig.h file. The default setting is \ref RTC_RESOLUTION_IN_TICKS.
 * 
 * @remarks
 * In general, working with \ref rawtime_t and functions that use \ref rawtime_t is preferred as less conversions are executed.
 * 
 * Printing current system time:
 * @code
 * time_t		currentTime;
 * char		timeString[RTC_TIME_STRING_LENGTH];
 * 
 * RTC_GetTime(&currentTime);
 * RTC_ToString(&currentTime, timeString);
 * 
 * printf("Current Time: %s", timeString);
 * @endcode
 */ 


#ifndef RTC_API_H_
#define RTC_API_H_

#include <stdint.h>
#include "Types.h"

/**
 * @name		RTC precision configuration
 * @{
 */

/**
 * @brief RTC resolution in 125 miliseconds. Miliseconds value will be rounded to 0, 125, 250, 375, 500, 625, 750, 875, 1000.
 */
#ifndef RTC_RESOLUTION_125_MILISECONDS
	#define RTC_RESOLUTION_125_MILISECONDS	0
#endif

/**
 * @brief RTC resolution in 8 miliseconds. Miliseconds value will be rounded to 0, 8, 16, 32, ... This option is very expensive in terms of computation time. Each converstion to time format requires 0.5s to complete.
 */
#ifndef RTC_RESOLUTION_8_MILISECONDS
	#define RTC_RESOLUTION_8_MILISECONDS	0
#endif

/**
 * @brief RTC resolution in ticks. No conversion from internal format to miliseconds is performed. This is he fastest option. 1000 ms will be represented as a value of 8192. Conversions need to be performed offline in the evaluation software. The number of ticks needs to be diviced by 8.192 to produce the milisecond value.
 */
#ifndef RTC_RESOLUTION_IN_TICKS
	#define RTC_RESOLUTION_IN_TICKS			0
#endif

#if ((RTC_RESOLUTION_IN_TICKS == 0) && (RTC_RESOLUTION_8_MILISECONDS == 0) && (RTC_RESOLUTION_125_MILISECONDS == 0))
	#undef RTC_RESOLUTION_IN_TICKS
	#define RTC_RESOLUTION_IN_TICKS			1
#endif

#if RTC_RESOLUTION_8_MILISECONDS
	#warning "Uses slow floating point operations. Every sampling of the time takes 500us (41 times slower than the 125ms resolution)"
#endif

#if (RTC_RESOLUTION_125_MILISECONDS && RTC_RESOLUTION_8_MILISECONDS)   
	#error "Selected configuration is not valid!"
#endif

#if (RTC_RESOLUTION_125_MILISECONDS && RTC_RESOLUTION_IN_TICKS)   
	#error "Selected configuration is not valid!"
#endif

#if (RTC_RESOLUTION_8_MILISECONDS && RTC_RESOLUTION_IN_TICKS)   
	#error "Selected configuration is not valid!"
#endif

/** @} */

/**
 * @brief This define informs about the length of string representation of RTC time. You need to provide an array of a size specified here when performing conversions from time_t format to string representation.
 */
#if RTC_RESOLUTION_125_MILISECONDS || RTC_RESOLUTION_8_MILISECONDS
	#define RTC_TIME_STRING_LENGTH				24	//hh-mm-ss.mss dd-mm-yyyy\n
#else
#if RTC_RESOLUTION_IN_TICKS
	#define RTC_TIME_STRING_LENGTH				25	//hh-mm-ss.tick dd-mm-yyyy\n
#else
	#define RTC_TIME_STRING_LENGTH				20	//hh-mm-ss dd-mm-yyyy\n
#endif
#endif

/**
 * @brief Definition of a structure for storing time values. This format is useful when working with time internally and no human readable form is required.
 */
typedef struct {
  uint32_t seconds;				///< Seconds
  uint16_t miliseconds;			///< Miliseconds (with precision selected via the defines provided)
} rawtime_t;

/**
 * @brief Definition of a structure for storing time values. This format is useful when comunicating with end user where human readable representation is required.
 */
typedef struct {
  uint8_t year;			///< Years since 1970
  uint8_t month;		///< 1-12
  uint8_t day;			///< 1-31
  uint8_t hour;			///< 0-23
  uint8_t minute;		///< 0-59
  uint8_t second;		///< 0-59
  uint16_t ms;			///< depending on the configuration (0-999 or 0-8191)
} time_t;


/**
 * @name		Setting the RTC
 * @{
 */

/**
 * @brief Sets the current time
 * 
 * Sets the current time of the software RTC. Software timers are not affected.
 * 
 * @param[in]	time		The new time.
 * 
 * @remarks		This function calls \ref RTC_SetRawTime after performing a conversion from \ref time_t to \ref rawtime_t
 */
void RTC_SetTime(const time_t* time);

/**
 * @brief Sets the current time
 * 
 * Sets the current time of the software RTC. Software timers are not affected.
 * 
 * @param[in]	time		The new time.
 * 
 * @remarks			Benchmark: 8us
 */
void RTC_SetRawTime(const rawtime_t* time);

/** @} */

/**
 * @name		Reading the RTC
 * @{
 */

/**
 * @brief Reads the current time
 * 
 * Reads the current time from the software RTC.
 * 
 * @param[out]	time		The current time.
 * 
 * @remarks
 *  - Benchmark with \ref RTC_RESOLUTION_IN_TICKS 10us
 *  - Benchmark with \ref RTC_RESOLUTION_125_MILISECONDS 12us
 *  - Benchmark with \ref RTC_RESOLUTION_8_MILISECONDS 500us
 */
void RTC_GetRawTime(rawtime_t* time);

/**
 * @brief Reads the current time
 * 
 * Reads the current time from the software RTC.
 * 
 * Example
 * @code
 * time_t		currentTime;
 * char		timeString[RTC_TIME_STRING_LENGTH];
 * 
 * RTC_GetTime(&currentTime);
 * RTC_ToString(&currentTime, timeString);
 * 
 * printf("Current Time: %s", timeString);
 * @endcode
 *
 * @param[out]	time		The current time.
 */
void RTC_GetTime(time_t* time);

/** @} */

/**
 * @brief Quick plausibility check for the time.
 * 
 * This functions checks whether the provided time is between year 2010 and 2020.
 * 
 * @param[in]	time		The time.
 * 
 * @returns		true when the time provided is valid,
 * 				false otherwise.
 */
bool RTC_ValidTime(const rawtime_t* time);

/**
 * @name		Converting time representations
 * @{
 */

/**
 * @brief Converts \ref rawtime_t to \ref time_t
 * 
 * @param[in]	from	Time in the \ref rawtime_t representation
 * @param[out]	to		Time in the \ref time_t representation
 * 
 * @remarks		Benchmark: 4 ticks ~ 500-625 us
 */ 
void RTC_ConvertToTime(const rawtime_t* from, time_t* to);

/**
 * @brief Converts \ref time_t to \ref rawtime_t
 * 
 * @param[in]	from	Time in the \ref time_t representation
 * @param[out]	to		Time in the \ref rawtime_t representation
 */ 
void RTC_ConverToRawTime(const time_t* from, rawtime_t* to);

/**
 * @brief Converts \ref time_t to a string.
 * 
 * You need to provice a buffer for the resulting string, the size is defined in the \ref RTC_TIME_STRING_LENGTH define.
 * 
 * Example:
 * @code
 * time_t		currentTime;
 * char		timeString[RTC_TIME_STRING_LENGTH];
 * 
 * RTC_GetTime(&currentTime);
 * RTC_ToString(&currentTime, timeString);
 * 
 * printf("Current Time: %s", timeString);
 * @endcode
 *
 * @param[in]		time	The time.
 * @param[out]		buffer	The time in string representation.
 *  
 * @remarks Benchmark: 40-50 ticks 
 */
void RTC_ToString(const time_t* time, char* buffer);

/**
 * @brief Converts time in string representation to \ref time_t
 * 
 * Input can be provided in following formats:
 *  - "hh-mm-ss dd-mm-yyyy"
 *  - "hh-mm-ss.mss dd-mm-yyyy"
 */
void RTC_FromString(const char* input, time_t* time);

/** @} */

#endif /*RTC_API_H_*/


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
