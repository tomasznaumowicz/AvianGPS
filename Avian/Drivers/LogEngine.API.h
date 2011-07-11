/**
 * @file		
 * @brief		@b Driver: Logging Engine. Support for storing and reading log entries.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 *
 * Simple Logging Engine (LogEngine) is used to store all information collected by the device. Also debug data like the "Power On" event or
 * commands being executed are logged. The LogEngine can be easily extended. 
 * 
 * The LogEngine was developed in early stages of the project as a quick solution for
 * data collection during the platform evaluation. It is very simple: its core is built around a definition of a 
 * log entry structure which has a fixed size. New log entry types can be easily added by modifying the definition
 * of the log entry structure (\ref logentry_t) in the LogEngine.h file. The LogEngine copies the content of a log entry without any processing
 * to the underlaying flash memory. Following limitations of the current LogEngine must be kept in mind:
 *  - only the entire log memory can be erased (limitation caused by the type of memory chip used)
 *  - no compression, every log entry consumes the same amout of memory, even if it's only a simple "Power On" log entry,
 *  - only one log entry can be transmitted over the radio at once.
 * 
 * An improved version of the LogEngine was developed by Markus Rudolph but wasn't integrated yet. We encourage you to integrate the new
 * LogEngine and get in touch with us in order to get it published on this site. The LogEngine 2.0 contains support for data compression
 * and dynamic log entries. It was built as a ring buffer and allows erasing of old entries.
 */

#ifndef LOGENGINE_API_H_
#define LOGENGINE_API_H_

#include <stdint.h>
#include "../System/Types.h"

#include "../System/RTC.API.h"

/**
 * @brief Types of defined log entries
 */ 
enum LogEntryTypeId {
	SCP					= 0,		///< Data from the pressure and temperature sensor
	Light				= 1,		///< Data from the light sensor
	POR					= 2,		///< power on event
	GPGGA				= 3,		///< Data from the GPS sensor (selected information from the NMEA GGA sentence)
	Battery				= 4,		///< Votlage of the battery
	Debug				= 5,		///< General debug entry used to log more events
	CommandExecution	= 6,		///< Data about the command being executed
	Heartbeat			= 7,		///< Periodic heartbeat that can be transmitted by the application
	Text				= 8			///< Free text
};

/**
 * @brief	The structure describing a log entry
 * 
 * The logentry_t is used to store data about an event in memory.
 * The payload is defined as a union of different possible structures specific to the data being stored.
 * The TypeId should be used to inform the developer and the LogEngine about the specific structure used within the union.
 * 
 * Example:
 * @code
 * logentry_t entry;
 * LogEngine_PrepareLogEntry(&entry);
 * entry.TypeId = Battery;
 * entry.Payload.Battery.Value = 1234;
 * @endcode
 */
typedef struct
{
	uint32_t				HostSequenceNo;		///< Internal (used to store the sequence number of this log entry that was used on the previous host of this log entry, relevant for radio downloads) 
	rawtime_t				Timestamp;			///< Timestap of this log entry
	uint8_t					Preamble;			///< Internal (used to determine whether it's a valid entry - as no checksums are used in this version of the LogEngine)
	uint8_t					SourceNodeId;		///< NodeId of the device that actually collected the dat
	uint8_t					HostNodeId;			///< Internal (NodeId of the device that was storing this log entry)
	enum LogEntryTypeId		TypeId;				///< TypeId of this log entry

	/**
	 * @brief The data to be logged
	 */
	union										
	{
		/**
		 * @bfief Data from the temperature and pressure sensor
		 */ 
		struct {
			uint32_t Pressure;					///< The pressure
			uint16_t Temperature;				///< The temperature
		} SCP;
		struct {
			uint16_t Value;						///< The light intensity 
		} Light;
		struct {
			uint16_t Value;						///< The battery voltage
		} Battery;
		struct {
			uint32_t Uptime;					///< The uptime of the device
			uint32_t LogSize;					///< The size of the local log
			uint16_t NewFixCount;				///< Number of fixes since reset
			uint16_t NewLogEntriesCount;		///< Number of new log entries since reset
			uint16_t BatteryLevel;				///< Battery Level
			uint16_t RFC;						///< A Standard for the Transmission of IP Datagrams on Avian Carriers is proposed in the RFC 1149. Set value of this variable to 1149 prior transmitting a Heartbeat over an avian carrier.
			uint16_t AppVersion;				///< Encoded application version. Only if the API versions are equal, automatic data download. 
		} Heartbeat;
		struct {
			uint32_t Value6_32bit;				///< Debug
			uint16_t Value4_16bit;				///< Debug
			uint16_t Value5_16bit;				///< Debug
			uint8_t Value1_8bit;				///< Debug
			uint8_t Value2_8bit;				///< Debug
			uint8_t Value3_8bit;				///< Debug
			uint8_t DebugCode;					///< Debug
		} Debug;
		struct {
			uint32_t 	EncodedTimestamp;		///< Timestamp from the NMEA GGA sentence
			int16_t 	LatitudeIntegral;		///< The integral of the latitude: -9060 .. +9060 (- is set when S, + when N)
			uint16_t 	LatitudeFractional;		///< The fractional of the latitude
			int16_t 	LongitudeIntegral;		///< The integral longitude: -18060 .. +18060 (- is set when W, + when E)
			uint16_t 	LongitudeFractional;	///< The fractional of the longitude
			int16_t 	AltitudeIntegral;		///< The integral of the altitude: (-xxx) - +xxx alititude: 5000 meters e.g.
			uint16_t 	EncodedHDop;			///< Encoded DHop value
			uint8_t  	FixQuality;				///< Fix Quality [0-9]
			uint8_t  	TrackedSatellites;		///< Number of tracked satellites [0-99]
			uint8_t 	AltitudeFractional;		///< The fractional fo the altitude [0-9]
		} GPGGA;
		struct {
			uint8_t		CommandText[12];		///< Substring from the submitted command
			uint8_t		RequestingNode;			///< The NodeId of the node that submitted the command
		} CommandExecution;
		struct {
			uint8_t		Content[13];			///< free text
		} Text;									
	} Payload;									///< Payload of the log entry
	
	uint8_t					Dummy;				///< This was used during development and should be removed soon (check for side effects first).
} logentry_t __attribute__((packed));							

/**
 * @brief Initializes the LogEngine.
 * 
 * Initializes the underlaying flash memory and displays status information.
 * 
 * @remarks		This functions needs to be called each time when the underlaying flash memory is erased. 
 * 
 * @returns		true on success, false on failure
 */
bool LogEngine_Init();

/**
 * @brief Saves a log entry to the storage.
 * 
 * Saves a log entry to the flash memory.
 * 
 * Example:
 * @code
 * logentry_t entry;
 * LogEngine_PrepareLogEntry(&entry);
 * entry.TypeId = Battery;
 * entry.Payload.Battery.Value = 1234;
 * LogEngine_Save(&entry);
 * @endcode
 *  
 * @remarks		No feedback is provided when this call fails (e.g. when the flash is full).
 * 
 * @param[in]	newEntry		The log entry.
 */
void LogEngine_Save(const logentry_t* newEntry);

/**
 * @brief Reads a log entry from storage.
 * 
 * Example:
 * @code
 * logentry_t entry;
 * uint32_t index = 1;
 * LogEngine_Read(&index, &entry);
 * @endcode
 *  
 * @param[in]	index			The absolute index of the requested log entry.
 * @param[out]	entryOut		The log entry.
 */
void LogEngine_Read(uint32_t* index, logentry_t* entryOut);

/**
 * @brief Initializes a log entry.
 * 
 * Call this function in order to initialize the log entry. The function fills
 * the basic data like the timestamp or the NodeId of the data source.
 * 
 * Example:
 * @code
 * logentry_t entry;
 * LogEngine_PrepareLogEntry(&entry);
 * entry.TypeId = Battery;
 * entry.Payload.Battery.Value = 1234;
 * @endcode
 * 
 * @param[out]	entryOut		The log entry.
 */
void LogEngine_PrepareLogEntry(logentry_t* entryOut);

/**
 * @brief Provides the current size of the log.
 * 
 * @returns		The size of the log.
 */
uint32_t LogEngine_GetLogSize();

/**
 * @brief States whether the underlaying storage is full or not.
 */
bool LogEngine_IsFull();

/**
 * @brief Saves a log entry of type Text to the storage.
 * 
 * Saves a log entry in the underlyaing structure within the flash memory.
 * 
 * Example:
 * @code
 * LogEngine_SaveText("Status: %u", 5);
 * @endcode
 * 
 * @remarks		No feedback is provided when this call fails (e.g. when the flash is full).
 * 
 * @param[in]	format the format string followed by variable list of parameters 
 */
void LogEngine_SaveText(const char *format, ...);

#endif /*LOGENGINE_API_H_*/

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
