/**
 * @file		LogEngine.API.h		
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../System/Logging.h"

#include "../System/Watchdog.API.h"
#include "../System/Commands.API.h"
#include "../System/Timers.API.h"

#include "../System/Radio/Network.h"

#include "BoolBitArray.h"
#include "SCP1000-D01.API.h"
#include "ADC_Devices.API.h"
#include "M25P80.h"
#include "M25P80.API.h"

#include "LogEngine.API.h"
#include "LogEngine.h"


static bool Handler_LogCommand(uint8_t requestedByNodeID, const char* input) {
	/**
	 * This handler is registered within the Commands Engine to preview Commands prior to execution.
	 * It will log every command.
	 */

	// only logs the command and does not abort it
	logentry_t entry;
	LogEngine_PrepareLogEntry(&entry);
	
	entry.TypeId = CommandExecution;
	entry.Payload.CommandExecution.RequestingNode = requestedByNodeID;
	strncpy(entry.Payload.CommandExecution.CommandText, input, sizeof(entry.Payload.CommandExecution.CommandText) - 1); // leave the last one open so that it keeps the 0x00 as string delimiter
	
	LogEngine_Save(&entry);
	
	return false;
}

void _LogEngine_snprint(uint32_t index, const logentry_t* entry, char* bufferOut, uint8_t bufferSize, bool mimimalOutput) {
	char timeAsString[RTC_TIME_STRING_LENGTH];

	if (mimimalOutput) {
		snprintf(timeAsString, RTC_TIME_STRING_LENGTH, "%lu.%u", entry->Timestamp.seconds, entry->Timestamp.miliseconds);
	}
	else {
		time_t time;
		rawtime_t rawtime;

		rawtime.seconds = entry->Timestamp.seconds;
		rawtime.miliseconds = entry->Timestamp.miliseconds;

		RTC_ConvertToTime(&rawtime, &time);
		RTC_ToString(&time, timeAsString);
	}
	
	//format: index:nodeid:typeid:timestamp:Pressure:Temperature:Light
	switch (entry->TypeId) {
			case SCP: {
				float temperature;
				uint32_t pressure;
				SCP_Convert(&temperature, &pressure, entry->Payload.SCP.Temperature, entry->Payload.SCP.Pressure);
				int high, low;

				high = temperature;
				low = abs((temperature - high) * 100);
				
				snprintf(bufferOut, bufferSize,
							"%lu\t%lu\t%u\t%u\t%u\t%s\t%lu\t%luPa\t%u\t%d.%02dC",
							index,
							entry->HostSequenceNo,
							entry->HostNodeId,
							entry->SourceNodeId,
							entry->TypeId,
							timeAsString,
							entry->Payload.SCP.Pressure,
							pressure,
							entry->Payload.SCP.Temperature,
							high, low
						);
				break;
			}
			case Light: {
				snprintf(bufferOut, bufferSize,
							"%lu\t%lu\t%u\t%u\t%u\t%s\t%u",
							index,
							entry->HostSequenceNo,
							entry->HostNodeId,
							entry->SourceNodeId,
							entry->TypeId,
							timeAsString,
							entry->Payload.Light.Value
						);
				break;
			}
			case Battery: {
				char batteryAsString[BATTERY_READ_STRING_LENGTH];
				
				ADC_Devices_Battery_Convert(entry->Payload.Battery.Value, NULL, batteryAsString);
				
				snprintf(bufferOut, bufferSize,
							"%lu\t%lu\t%u\t%u\t%u\t%s\t%u\t%s",
							index,
							entry->HostSequenceNo,
							entry->HostNodeId,
							entry->SourceNodeId,
							entry->TypeId,
							timeAsString,
							entry->Payload.Battery.Value,
							batteryAsString
						);
				break;
			}
			case Heartbeat: {
				char batteryAsString[BATTERY_READ_STRING_LENGTH];
				
				ADC_Devices_Battery_Convert(entry->Payload.Heartbeat.BatteryLevel, NULL, batteryAsString);
				
				snprintf(bufferOut, bufferSize,
							"%lu\t%lu\t%u\t%u\t%u\t%s\t%lu\t%lu\t%u\t%u\t%u\t%s",
							index,
							entry->HostSequenceNo,
							entry->HostNodeId,
							entry->SourceNodeId,
							entry->TypeId,
							timeAsString,
							entry->Payload.Heartbeat.Uptime, /* %lu */
							entry->Payload.Heartbeat.LogSize, /* %u */
							entry->Payload.Heartbeat.NewLogEntriesCount, /* %u */
							entry->Payload.Heartbeat.NewFixCount, /* %u */
							entry->Payload.Heartbeat.BatteryLevel,
							batteryAsString
						);
				break;
			}
			case POR: {
				snprintf(bufferOut, bufferSize,
							"%lu\t%lu\t%u\t%u\t%u\t%s",
							index,
							entry->HostSequenceNo,
							entry->HostNodeId,
							entry->SourceNodeId,
							entry->TypeId,
							timeAsString
						);
				break;
			}
			case GPGGA: {
				uint32_t encoded_timestamp = entry->Payload.GPGGA.EncodedTimestamp;
				uint32_t timestampHigh = encoded_timestamp / 1000;
				uint16_t timestampLow = (uint16_t) (encoded_timestamp - (timestampHigh * 1000));
				
				uint16_t encoded_hdop =  entry->Payload.GPGGA.EncodedHDop;
				uint16_t hdop_high = encoded_hdop / 10;
				uint16_t hdop_low = encoded_hdop - (hdop_high * 10);
				
				snprintf(bufferOut, bufferSize,
						"%lu\t%lu\t%u\t%u\t%u\t%s\tGPGGA\t%lu.%.3u\t%i.%.4u\t%i.%.4u\t%u\t%u\t%u.%u\t%i.%u\tOK",
						index,
						entry->HostSequenceNo,
						entry->HostNodeId,
						entry->SourceNodeId,
						entry->TypeId,
						timeAsString,
						timestampHigh,
						timestampLow,
						entry->Payload.GPGGA.LatitudeIntegral,
						entry->Payload.GPGGA.LatitudeFractional,
						entry->Payload.GPGGA.LongitudeIntegral,
						entry->Payload.GPGGA.LongitudeFractional,
						entry->Payload.GPGGA.FixQuality,
						entry->Payload.GPGGA.TrackedSatellites,
						hdop_high,
						hdop_low,
						entry->Payload.GPGGA.AltitudeIntegral,
						entry->Payload.GPGGA.AltitudeFractional
					);				
				break;
			}
			case Debug: {
				if (entry->Payload.Debug.DebugCode == 3) {
					rawtime_t rt;
					rt.seconds = entry->Payload.Debug.Value6_32bit;
					rt.miliseconds = entry->Payload.Debug.Value5_16bit;
					time_t time;
					RTC_ConvertToTime(&rt, &time);
					char timeAsStr[RTC_TIME_STRING_LENGTH];
					RTC_ToString(&time, timeAsStr);
					
					snprintf(bufferOut, bufferSize,
								"%lu\t%lu\t%u\t%u\t%u\t%s\t%u\tUpdated RTC from %s",
								index,
								entry->HostSequenceNo,
								entry->HostNodeId,
								entry->SourceNodeId,
								entry->TypeId,
								timeAsString,
								entry->Payload.Debug.DebugCode,
								timeAsStr
							);
					break;
				}
				
				if (entry->Payload.Debug.DebugCode == 11) {
					snprintf(bufferOut, bufferSize,
								"%lu\t%lu\t%u\t%u\t%u\t%s\t%u\tpw:%x\trr:%u\tfp:%p\tsp:%p",
								index,
								entry->HostSequenceNo,
								entry->HostNodeId,
								entry->SourceNodeId,
								entry->TypeId,
								timeAsString,
								entry->Payload.Debug.DebugCode,
								entry->Payload.Debug.Value1_8bit,
								entry->Payload.Debug.Value2_8bit,
								(void*)(entry->Payload.Debug.Value4_16bit),
								(void*)(entry->Payload.Debug.Value5_16bit)
							);
					break;
				}
				
				char* text = NULL;

				if (text == NULL) {
					snprintf(bufferOut, bufferSize,
								"%lu\t%lu\t%u\t%u\t%u\t%s\t%u\t?\t%u\t%u\t%u\t%u\t%u\t%lu",
								index,
								entry->HostSequenceNo,
								entry->HostNodeId,
								entry->SourceNodeId,
								entry->TypeId,
								timeAsString,
								entry->Payload.Debug.DebugCode,
								entry->Payload.Debug.Value1_8bit,
								entry->Payload.Debug.Value2_8bit,
								entry->Payload.Debug.Value3_8bit,
								entry->Payload.Debug.Value4_16bit,
								entry->Payload.Debug.Value5_16bit,
								entry->Payload.Debug.Value6_32bit
							);
				} else {
					snprintf(bufferOut, bufferSize,
								"%lu\t%lu\t%u\t%u\t%u\t%s\t%u\t%s",
								index,
								entry->HostSequenceNo,
								entry->HostNodeId,
								entry->SourceNodeId,
								entry->TypeId,
								timeAsString,
								entry->Payload.Debug.DebugCode,
								text
							);
				}
				break;
			}			
			
			case CommandExecution: {
					snprintf(bufferOut, bufferSize,
								"%lu\t%lu\t%u\t%u\t%u\t%s\t%u\t%s",
								index,
								entry->HostSequenceNo,
								entry->HostNodeId,
								entry->SourceNodeId,
								entry->TypeId,
								timeAsString,
								entry->Payload.CommandExecution.RequestingNode,
								entry->Payload.CommandExecution.CommandText
							);
					
					break;
				}
			
			case Text: {
					snprintf(bufferOut, bufferSize,
								"%lu\t%lu\t%u\t%u\t%u\t%s\t%s",
								index,
								entry->HostSequenceNo,
								entry->HostNodeId,
								entry->SourceNodeId,
								entry->TypeId,
								timeAsString,
								entry->Payload.Text.Content
							);
					
					break;
				}
				
			default: {
				snprintf(bufferOut, bufferSize,
							"%lu\t%u\t%u\t%u\t%s\t%lu\t%u\t%u\tUNKNOWN",
							index,
							entry->SourceNodeId,
							entry->HostNodeId,
							entry->TypeId,
							timeAsString,
							entry->Payload.SCP.Pressure,
							entry->Payload.SCP.Temperature,
							entry->Payload.Light.Value
						);
			}
		}//switch
}

uint32_t _nextAddress;

bool LogEngine_InitializeStorageAddress() {
	/**
	 * this is not the best solution.. it needs to scan through all log entries
	 */
	uint8_t preamble;
	
	_nextAddress = 0;
	
	do {
		Watchdog_Reset();
		
		_nextAddress = _nextAddress + sizeof(logentry_t);	//skips the first block, can be used for debug data etc.
		
		//detect end of memory here
		Flash_Read(_nextAddress + 10, &preamble, 1); // + 10 because preamble is not at the beginning of the log entry...
	} while (preamble == LOG_ENTRY_PREAMBLE && _nextAddress < Flash_Max_Address);

	//printf("finshed at: %lu\r\n", _nextAddress);
	
	// the loop terminated: a free block was found or the end of memory was reached
	uint32_t totalSlots = Flash_Max_Address / sizeof(logentry_t) - 1; //first slot is not used
	uint32_t selectedSlot = _nextAddress / sizeof(logentry_t);
	
	if (selectedSlot > totalSlots)
		return false;
	
	if (preamble != LOG_ENTRY_PREAMBLE)
		return true;

	return false;
}

bool _logEngineEnabled;

/**
 * @ingroup Commands
 * @brief	@b format Formats the flash memory
 */
COMMAND(format, "Formats the flash memory", cmd_args) {
	printf("please wait...\r\n");
	Flash_Erase();
	LogEngine_InitializeStorageAddress();
}

bool LogEngine_IsFull() {
	return (_nextAddress + (sizeof(logentry_t) * 10) > Flash_Max_Address);
}

void LogEngine_Save(const logentry_t* newEntry) {
	if (!_logEngineEnabled)
		return;
	
	if (LogEngine_IsFull()) {
		LOGERR("FLASH FULL!");
		return;
	}
	
	if (newEntry->Preamble != LOG_ENTRY_PREAMBLE) {
		LOGERR("Was requested to log an entry with missing preamble! ignored.");
		return;
	}
	
	uint16_t bytesWritten;
	bytesWritten = Flash_Write(_nextAddress, (void*) newEntry, sizeof(logentry_t));
	
	if (bytesWritten == sizeof(logentry_t))
		_nextAddress = _nextAddress + sizeof(logentry_t);
	
	return;
}

void LogEngine_Read(uint32_t* index, logentry_t* entryOut) {
	if (*index == 0)
		return;
	
	uint32_t address = *index * sizeof(logentry_t);
	
	Flash_Read(address, (void*) entryOut, sizeof(logentry_t));
}

void LogEngine_PrepareLogEntry(logentry_t* entryOut) {
	
	memset(entryOut, 0, sizeof(logentry_t));
	
	rawtime_t rawtime;
	RTC_GetRawTime(&rawtime);
	
	entryOut->SourceNodeId			= Configuration.NodeID;
	entryOut->Preamble				= LOG_ENTRY_PREAMBLE;
	entryOut->Timestamp.seconds		= rawtime.seconds;
	entryOut->Timestamp.miliseconds	= rawtime.miliseconds;
}

#define BENCHMARK_READ		0

/**
 * @ingroup Commands
 * @brief	@b read Displays log entries in specified range
 */
COMMAND(read, "Displays log entries in specified range",  cmd_args) {

	// default configuration
	uint32_t firstIndex = 1;
	uint32_t lastIndex = _nextAddress / sizeof(logentry_t) - 1;
	
	if (cmd_args->ArgumentCount > 0) {
		// first index was specified
		firstIndex = atol(cmd_args->ArgumentArray[0]);
	}
	
	if (cmd_args->ArgumentCount > 1) {
		// last index was specified
		lastIndex = atol(cmd_args->ArgumentArray[1]);
	}
	
	printf("Selected range [%lu - %lu]\r\n", firstIndex, lastIndex);
	
	uint32_t currentIndex;
	for (currentIndex = firstIndex; currentIndex <= lastIndex; currentIndex++) {
#if BENCHMARK_READ
		uint32_t total1 = COPY_CURRENT_TAR;
#endif
		Watchdog_Reset();
		
		logentry_t entry;
		
#if BENCHMARK_READ
		uint32_t t1 = COPY_CURRENT_TAR;
#endif
		LogEngine_Read(&currentIndex, &entry);

#if BENCHMARK_READ
		uint32_t t2 = COPY_CURRENT_TAR;
#endif	
		uint32_t t3, t4;

		/* disalbedq quick gpgga printing
		if (entry.TypeId == GPGGA) {
				// most often we deal with GPS position data. optimize this here:
				uint32_t encoded_timestamp = entry.Payload.GPGGA.EncodedTimestamp;
				uint32_t timestampHigh = encoded_timestamp / 1000;
				uint16_t timestampLow = (uint16_t) (encoded_timestamp - (timestampHigh * 1000));
				
				uint16_t encoded_hdop =  entry.Payload.GPGGA.EncodedHDop;
				uint16_t hdop_high = encoded_hdop / 10;
				uint16_t hdop_low = encoded_hdop - (hdop_high * 10);
				
				printf("[%u:L] %lu,%u,%u,%lu.%u,GPGGA,%lu.%.3u,%i.%.4u,%i.%.4u,%u,%u,%u.%u,%i.%u\r\n",
						Configuration.NodeID,
						currentIndex,
						entry.NodeId,
						entry.TypeId,
						entry.Timestamp.seconds,
						entry.Timestamp.miliseconds,
						timestampHigh,
						timestampLow,
						entry.Payload.GPGGA.LatitudeIntegral,
						entry.Payload.GPGGA.LatitudeFractional,
						entry.Payload.GPGGA.LongitudeIntegral,
						entry.Payload.GPGGA.LongitudeFractional,
						entry.Payload.GPGGA.FixQuality,
						entry.Payload.GPGGA.TrackedSatellites,
						hdop_high,
						hdop_low,
						entry.Payload.GPGGA.AltitudeIntegral,
						entry.Payload.GPGGA.AltitudeFractional
					);
				t3 = COPY_CURRENT_TAR;
				t4 = t3;

		} else*/ {
			// everything else can be processed by the original code.
			if (entry.Preamble == LOG_ENTRY_PREAMBLE) {
				char buffer[120];
				_LogEngine_snprint(currentIndex, &entry, buffer, 120, false);
				
				t3 = COPY_CURRENT_TAR;
	
				printf("[%u:L] %s\r\n", Configuration.NodeID, buffer);
				//LOG("%s", buffer);
			} else 
				printf("[%u:L] invalid\r\n", Configuration.NodeID);
			
			t4 = COPY_CURRENT_TAR;
		}

#if BENCHMARK_READ
		printf("(%lu)%lu/%lu/%lu\r\n", t4-total1, t2-t1, t3-t2, t4-t3);
#endif
	}
}

uint32_t LogEngine_GetLogSize() {
	return _nextAddress / sizeof(logentry_t) - 1;
}

/**
 * @ingroup Commands
 * @brief	@b logsize Provides the current number of stored log entries
 */
COMMAND(logsize, "Number of local log entries", cmd_args) {
	snprintf(
			cmd_args->ResponseString,
			cmd_args->ResponseStringMaxLength,
			"{%lu} slots used",
			LogEngine_GetLogSize()
		);
}


void LogEngine_SaveText(const char *format, ...) {
	logentry_t entry;
	LogEngine_PrepareLogEntry(&entry);
	entry.TypeId = Text;

	va_list args;
	va_start(args, format);
	vsnprintf(entry.Payload.Text.Content, sizeof(entry.Payload.Text.Content) - 1, format, args);		
	va_end(args);

	LogEngine_Save(&entry);
}

bool LogEngine_Init() {
	_logEngineEnabled = false;


	 if (Flash_Init() == false) {
		 // flash init failed
		 LOG("LogEngine Init failed.");
		 return false;
	 }
	 
	 _nextAddress = 0;
	 
	 uint32_t totalSlots = Flash_Max_Address / sizeof(logentry_t) - 1; //first slot is not used
	 LOGDBG("LogEngine: total slots %lu (log entry size %u)", totalSlots, sizeof(logentry_t));
	 if (LogEngine_InitializeStorageAddress()) {
		 LOGDBG("LogEngine: slots used %lu (log entry size %u)", _nextAddress / sizeof(logentry_t) - 1, sizeof(logentry_t));
		 LOG("LogEngine Init: OK");
		 _logEngineEnabled = true;
	 }
	 else {
		 LOGERR("LogEngine Init failed.");
		 return false;
	 }
	 
	 Handler_Command_Preview = Handler_LogCommand;
	
	 return true;
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
