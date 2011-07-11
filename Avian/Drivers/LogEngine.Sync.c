#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../System/Logging.h"

#include "../System/Watchdog.API.h"
#include "../System/Commands.API.h"
#include "../System/Timers.API.h"

#include "../System/Radio/Network.h"

#include "BoolBitArray.h"

#include "LogEngine.API.h"
#include "LogEngine.h"
#include "LogEngine.Sync.h"

#define		DEBUG_SYNC		0

// network protocol identifiers that shall be unique across the network. used to route radio packets inside of the application


// specific defines required to properly compute the size of request buffers. they need to fit a single radio packet, that's why they depend on the NETWORK_BUFFER_MAXSIZE
#define		LOG_REQUEST_BITMASK_SIZE				(NETWORK_BUFFER_MAXSIZE - 4) 		// 4 == sizeof FirstSequenceNumber
#define		LOG_REQUEST_SEQUENCE_SIZE				(LOG_REQUEST_BITMASK_SIZE * 8)		// multiplied by 8 because it byte can hold information about 8 entries.



/**
 * @brief Definition of a sequence request with support for gaps.
 * 
 * This structure defines radio request for a sequence of log entries. The request can contain gaps,
 * e.g. one can specify to request an entry with ID 1, 120 and 154 without being forced
 * to contstruct the list of requested IDs.
 *
 * The sequence request contains the sequence number (ID) of the first requested entry and
 * a bool array which contains information about requested entries (true stands for 'requested').
 * 
 * The request has a limited size as it needs to be transmitted over the radio.
 * 
 * @remarks		Since a bitmask is used, it reduces the number of queries when a number of different, not subsequent, entries
 * 				is requested, but it also reduces the maximal data range that can be handled by one request.
 */
typedef struct {
	uint32_t			FirstSequenceNumber;						///< The sequence number of the first requested log entry
	boolbitarray_t		SequenceMask[LOG_REQUEST_BITMASK_SIZE];		///< Bool array with the sequence mask.
} sequenceRequest_t __attribute__ ((packed));

#if DEBUG_SYNC
	void _LogEngine_PrintRequest(const sequenceRequest_t* request) {
		int8_t index;
		
		printf("Start:\t\t%lu\r\nContent:\t", request->FirstSequenceNumber);
		for (index = 0; index < LOG_REQUEST_BITMASK_SIZE; index++)
			printf("%.2X", request->SequenceMask[index]);
		printf("\r\n");
	}
#endif

/**
void _LogEngine_AddToRequest(sequenceRequest_t* request, const uint32_t* index) {
	// find the index of the byte in the request bitmask, if it's too large, return.
	uint32_t byteIndex = *index >> 3;		// divide the index by 8
	
	if (byteIndex > LOG_REQUEST_BITMASK_SIZE)
		return;

	BoolBitArray_Set(request->SequenceMask, index);
}
*/

	
////// SEQUENCE REQUESTS //////

/**
 * This functions handles sequence requests. It doesn't know whether it's a part
 * of an autogaps or just a regular fillgaps query
 */
	
void Handler_SequenceRequest(uint8_t requestSource, const uint8_t* payload) {
	// it is crucial to copy the data in memory (this helps to align data types in memory) 
	sequenceRequest_t request;
	memcpy(&request, payload, sizeof(sequenceRequest_t));

	// display information for the user
	LOGDBG_SWITCHED(DEBUG_SYNC, "Handling request from %u for sequence starting at %lu", requestSource, request.FirstSequenceNumber);

	// statistics collected during the operation and displayed to the user when the handler completes
	uint16_t counter_served = 0;
	
	// used to address local log entries
	uint32_t local_logsize = LogEngine_GetLogSize();
	
	Timers_Block(128); // empirical value. wait for the other side to prepare

	uint32_t index; // used to cycle through all values in the request
	for (index = 0; index < LOG_REQUEST_SEQUENCE_SIZE; index++) { // for every entry in the boolbitarray
		// update the sequence number that is requested by the request
		uint32_t currentSequenceNo = request.FirstSequenceNumber + index;
		
		// break the loop if it looks like an entry from outside of the local data range was requested
		if (currentSequenceNo > local_logsize)
			break;
		
		// process when the boolbitarray was set to 'true' at the tested index
		if (BoolBitArray_Get(request.SequenceMask, &index)) {
			
			// load the requested entry
			logentry_t entry;
			LogEngine_Read(&currentSequenceNo, &entry);
			
			if (entry.Preamble != LOG_ENTRY_PREAMBLE) // this logentry is invalid, skip it
				continue;
			
			// update the sequence number
			entry.HostSequenceNo = currentSequenceNo;
			entry.HostNodeId = Configuration.NodeID;
			
			// serve the requested entry over radio
			network_payload_t packet;
			Network_InitSendArgs(&packet);
		
			packet.Buffers[0].Buffer = &entry;
			packet.Buffers[0].Size = sizeof(logentry_t);

			Timers_Block(5); // empirical value. wait for the other side to process the previous packet
			Network_SendWithoutACK(requestSource, NETWORK_PROTOCOL_SEQUENCE_RESPONSE, &packet);
			
			counter_served++; // count served
		}
	}
	
	LOGDBG_SWITCHED(DEBUG_SYNC, "Served %u packets", counter_served);
}



/**
 * private variables required for the LogEngine_FindGap function to operate (used for state retention)
 */
static uint32_t 		_LogEngine_FindGap_CurrentQueriedRemoteSequenceNo;
static uint32_t			_LogEngine_FindGap_LastQueriedRemoteSequenceNo;

static uint32_t 		_LogEngine_FindGap_FirstCachedRemoteSequenceNo;
static uint32_t 		_LogEngine_FindGap_LastCachedRemoteSequenceNo;
static uint16_t			_LogEngine_FindGap_CachedTargetNodeId;
static boolbitarray_t* 	_LogEngine_FindGap_Buffer;

/**
 * provide all parameters to init the function and find the first gap.
 * set all parameters to NULL (for reference parameters) or 0 (for value parameters) to find the subsequent gaps
 * 
 * @returns 	pointer to a uint32_t containing the sequence number of a missing log entry
 * 				NULL when no gap is found
 */
const uint32_t* LogEngine_FindGap(gap_search_query_t* arguments, boolbitarray_t* buffer, const uint16_t bufferSize) {
	if (arguments != NULL) {
		// if the buffer is too small for the request debug waring should be generated and NULL should be returned (as for no gaps found)
		uint32_t maximalLength = bufferSize << 3; // each byte in the buffer stores 8 bool values
		uint32_t requestedLength = arguments->Request.LastSeqNo - arguments->Request.FirstSeqNo + 1;
		
		if (requestedLength > maximalLength) {
			LOGERR("[LogEngine_FindGap] The selected range (%lu) of sequence numbers is too large (max. %lu)!", requestedLength, maximalLength);
			return NULL;
		}
		
		_LogEngine_FindGap_LastQueriedRemoteSequenceNo = arguments->Request.LastSeqNo;
		_LogEngine_FindGap_CurrentQueriedRemoteSequenceNo = arguments->Request.FirstSeqNo;
		
		if (_LogEngine_FindGap_Buffer != NULL) {
			// there is something in the buffer, try to reuse it now
			
			// 1. check boundaries -> if violated, force a refresh
			// 2. check targetnodeid
			if (
					arguments->Request.FirstSeqNo < _LogEngine_FindGap_FirstCachedRemoteSequenceNo
					||
					arguments->Request.LastSeqNo > _LogEngine_FindGap_LastCachedRemoteSequenceNo
					||
					arguments->Request.TargetNodeId != _LogEngine_FindGap_CachedTargetNodeId
				) {
				_LogEngine_FindGap_Buffer = NULL;
			}
		}

		// there is no buffer that could be reused, in such case, the function needs to be initialized
		if (_LogEngine_FindGap_Buffer == NULL) {
			_LogEngine_FindGap_LastCachedRemoteSequenceNo = arguments->Request.FirstSeqNo + maximalLength - 1;
			_LogEngine_FindGap_FirstCachedRemoteSequenceNo = arguments->Request.FirstSeqNo;
			_LogEngine_FindGap_CachedTargetNodeId = arguments->Request.TargetNodeId;
			_LogEngine_FindGap_Buffer = buffer;
			
			memset(buffer, 0, bufferSize);
			
			uint32_t logSize = LogEngine_GetLogSize();
			
			// scan the entire selected area from the local memory and put information about data availability in the buffer
			// the selected are starts at the localSeqNo provided and ends at the end of the buffer
			uint32_t currentLocalSeqNo;
			for(currentLocalSeqNo = arguments->Request.LocalSeqNo; currentLocalSeqNo <= logSize; currentLocalSeqNo++) {
				// reset the watchdog - this is a long running operation
				Watchdog_Reset();
				
				// load the entry
				logentry_t entry;
				LogEngine_Read(&currentLocalSeqNo, &entry);
				
				 // skip invalid entries and process only entries that are relevant for the query
				if (
						entry.Preamble != LOG_ENTRY_PREAMBLE
						||
						entry.HostNodeId != _LogEngine_FindGap_CachedTargetNodeId
						||
						entry.HostSequenceNo < _LogEngine_FindGap_FirstCachedRemoteSequenceNo
						||
						entry.HostSequenceNo > _LogEngine_FindGap_LastCachedRemoteSequenceNo
					)
					continue;
				
				// a valid entry was found (such that matches the query and is in the selected data range
				// now, information about it's presence should be stored in the buffer
				
				// prepare the index of this entry
				uint32_t index = entry.HostSequenceNo - arguments->Request.FirstSeqNo;
				BoolBitArray_Set(buffer, &index);
			}
		} // init code end
	}
	else {
		// this is a subsequent call to find the next gap. the gap found during the previous call was reported so the counter needs to be incremented
		_LogEngine_FindGap_CurrentQueriedRemoteSequenceNo++;
	}
	
	

	
	
	
	// now, the memory was scanned, and the buffer should be used to investigate whether certain log entries are available
	
	// _LogEngine_FindGap_CurrentRemoteSequenceNo is not initialized here on purpose.
	// it is stored and reused accross sequential calls to this function.
	for (; _LogEngine_FindGap_CurrentQueriedRemoteSequenceNo <= _LogEngine_FindGap_LastQueriedRemoteSequenceNo; _LogEngine_FindGap_CurrentQueriedRemoteSequenceNo++) {
		uint32_t index = _LogEngine_FindGap_CurrentQueriedRemoteSequenceNo - _LogEngine_FindGap_FirstCachedRemoteSequenceNo;
		if (BoolBitArray_Get(_LogEngine_FindGap_Buffer, &index))
			continue; // the entry with the requested sequence number was marked ad available in the buffer 
		
		return &_LogEngine_FindGap_CurrentQueriedRemoteSequenceNo;
	}
	
	// if this line was reached, no (more) gaps were found
	return NULL;
}


gap_search_query_t _fillgaps_query;

void LogEngine_ProcessGaps_ResetState() {
	_LogEngine_FindGap_Buffer = NULL;
	
	_fillgaps_query.Response.ExpectedCount	= 0;
	_fillgaps_query.Response.MissingCount	= 0;
}

enum ProcessGapsActions {
	DisplayOnly,
	DisplayAndRequest,
	RequestOnly
};


static boolbitarray_t _LogEngine_ProcessGaps_Buffer[1024];
/**
 * This function locates gaps in the local log based on the specified query.
 * It can either only display the gaps or also request the missing data from the remote node.
 * 
 * @returns		number of processed gaps
 */
uint32_t LogEngine_ProcessGaps(gap_search_query_t* arguments, enum ProcessGapsActions action) {
	// display information for the user
	printf(	"Local Sequence No:\t%lu\r\n"
			"Target NodeID:\t\t%u\r\n"
			"First Sequence No:\t%lu\r\n"
			"Last Sequence No:\t%lu\r\n"
			"Missing sequence numbers (searching...):",
			arguments->Request.LocalSeqNo, arguments->Request.TargetNodeId, arguments->Request.FirstSeqNo, arguments->Request.LastSeqNo);

	uint32_t expectedLength = arguments->Request.LastSeqNo - arguments->Request.FirstSeqNo + 1; // +1 because the first and the last entry needs to be processed as well
	uint32_t missingEntries = 0;

	// initialize the request struct (this request will be used only when the action is set to DisplayAndRequest
	sequenceRequest_t request;
	if (action == DisplayAndRequest || action == RequestOnly) {
		memset(&request, 0, sizeof(sequenceRequest_t));
		request.FirstSequenceNumber = arguments->Request.FirstSeqNo;
	}

	// find the first missingEntry and initialize the FindGap function
	// this function is able to cache data between initializations. it needs to be reseted
	// if really everything should be removed from memory
	// this function reuses a buffer if it's larger than needed
	const uint32_t* missingEntry = LogEngine_FindGap(arguments, _LogEngine_ProcessGaps_Buffer, sizeof(_LogEngine_ProcessGaps_Buffer));
		
	while (missingEntry != NULL) {
		uint32_t index = *missingEntry - arguments->Request.FirstSeqNo; 	// computes the index of the missing entry

		if (action == DisplayOnly || action == DisplayAndRequest)
			printf(" %lu", *missingEntry);							// display for the user
		
		if (action == DisplayAndRequest || action == RequestOnly) {
			BoolBitArray_Set(request.SequenceMask, &index);
		}
		
		missingEntries++; // used for statistics

		// try to find the next entry
		missingEntry = LogEngine_FindGap(NULL, NULL, 0); // finding the next one
	}
	// statistics:
	uint32_t percentMissing = (100 * missingEntries) / expectedLength;
	
	printf("\r\nDetected missing:\t%lu%% (%lu/%lu)\r\n",
			percentMissing, missingEntries, expectedLength);
	
	if (action == DisplayAndRequest || action == RequestOnly) {
		if (missingEntries > 0) {
			// send a request only when missing entries were detected
			network_payload_t requestPacket;
			Network_InitSendArgs(&requestPacket);
			requestPacket.Buffers[0].Buffer = &request;
			requestPacket.Buffers[0].Size = sizeof(sequenceRequest_t);
			
			Network_SendWithoutACK(arguments->Request.TargetNodeId, NETWORK_PROTOCOL_SEQUENCE_REQUEST, &requestPacket);
		}
	}
	
	return missingEntries;
}

COMMAND(showgaps, "Shows gaps in downloaded logs", cmd_args) {
	if (cmd_args->ArgumentCount != 4) {
		printf("Help: gaps [local sequence no] [nodeid] [first remote sequence no] [last remote sequence no]\r\n");
		return;
	}
	
	// build the query
	gap_search_query_t searchQuery;
	searchQuery.Request.LocalSeqNo 		= atol(cmd_args->ArgumentArray[0]);
	searchQuery.Request.TargetNodeId 	= atoi(cmd_args->ArgumentArray[1]);
	searchQuery.Request.FirstSeqNo	 	= atol(cmd_args->ArgumentArray[2]);
	searchQuery.Request.LastSeqNo 		= atol(cmd_args->ArgumentArray[3]);
	
	LogEngine_ProcessGaps_ResetState();
	uint32_t missingEntries = LogEngine_ProcessGaps(&searchQuery, DisplayOnly);

	// statistics:
	uint32_t expectedLength = searchQuery.Request.LastSeqNo - searchQuery.Request.FirstSeqNo + 1; // +1 because the rist and the last entry needs to be processed as well
	uint32_t percentMissing = (100 * missingEntries) / expectedLength;
	
	snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength,
			"Detected missing %lu%% (%lu/%lu)",
			percentMissing, missingEntries, expectedLength);
}


/**
 * This function uses _autogaps_query variable to holt the status of the qutogap query
 * it registers itself via timer_add per periodic execution until the last seqno is reached.
 */
void FillGapsScheduleProcessor(uint16_t notUsed) {
	if (_fillgaps_query.Request.FirstSeqNo > _fillgaps_query.Request.LastSeqNo) {
		printf("\r\n");
		LOGATT("Schedule processing completed.");
		
		// update statistics if there is a listener that's interested in those details:
		if (_fillgaps_query.Request.Handler_Sync_Completed != NULL) {
			*(_fillgaps_query.Internal.ExportExpectedCount)	= _fillgaps_query.Response.ExpectedCount;
			*(_fillgaps_query.Internal.ExportMissingCount)	= _fillgaps_query.Response.MissingCount;
			
			_fillgaps_query.Request.Handler_Sync_Completed();
		}
		return;
	}
	else {
		uint32_t remainingItems = _fillgaps_query.Request.LastSeqNo - _fillgaps_query.Request.FirstSeqNo + 1;
		printf("\r\n");
		LOGATT("Processing remaining %lu items...", remainingItems);

		sequenceRequest_t dummyRequest;
		uint32_t maxQueryRange = sizeof(dummyRequest.SequenceMask) * 8;

		// build a query. reuse the current configuration but update the LastSeqNo so the maxQueryRange isn't violated
		gap_search_query_t query;
		memcpy(&query, &_fillgaps_query, sizeof(gap_search_query_t));
		// update the LastSeqNoaut
		if (query.Request.LastSeqNo > query.Request.FirstSeqNo + maxQueryRange - 1)
			query.Request.LastSeqNo = query.Request.FirstSeqNo + maxQueryRange - 1;
		
		// update the state of _autogaps_query
		_fillgaps_query.Request.FirstSeqNo = query.Request.LastSeqNo + 1;
		
		// collect statistics information
		uint32_t expectedLength = query.Request.LastSeqNo - query.Request.FirstSeqNo + 1; // +1 because the first and the last entry needs to be processed as well

		// execute the query
		uint32_t missingGaps = LogEngine_ProcessGaps(&query, RequestOnly);
		
		// update the statistics
		_fillgaps_query.Response.ExpectedCount += expectedLength;
		_fillgaps_query.Response.MissingCount += missingGaps;
		
		// schedule query execution
		// remark: each log entry received as a result of LogEngine_ProcessGaps will extend this schedule by two seconds.
		_fillgaps_query.Internal.CurrentScheduleTimerId = Timers_Add(FillGapsScheduleProcessor, (uint32_t) TICKS_2SECONDS, 0);
	}
}

void Handler_SequenceResponse(uint8_t source, const uint8_t* data) {
	// it's very important to copy the data: it's required for proper alignment of data in memory
	logentry_t entry;
	memcpy(&entry, data, sizeof(logentry_t));
	
	LogEngine_Save(&entry);
	
	putchar('.'); // process indication for the user
	LogEngine_Sync_EntriesSynced++;
	
	// if there is another call fo the FillGapsScheduleProcessor schedule, it should be postponed
	// by few seconds since since we just received some data transmitted as a resonse
	// it might be, there is still more to come
	
	// just try to remove the timer and if there was any, register it again
	bool removed = Timers_Remove(_fillgaps_query.Internal.CurrentScheduleTimerId); 
	if (removed) {
		// the CurrentScheduleTimerId was defined, this means, the fillgaps feature was still running
		_fillgaps_query.Internal.CurrentScheduleTimerId = Timers_Add(FillGapsScheduleProcessor, (uint32_t) TICKS_2SECONDS, 0);
	}
}

void LogEngine_Sync_FillGaps(gap_search_query_t* query) {
	// read the parameters
	memcpy(&_fillgaps_query, query, sizeof(gap_search_query_t));
	_fillgaps_query.Internal.ExportExpectedCount = &(query->Response.ExpectedCount);
	_fillgaps_query.Internal.ExportMissingCount = &(query->Response.MissingCount);

	LogEngine_ProcessGaps_ResetState(); // the cache needs to be reset now.
	
	_fillgaps_query.Internal.CurrentScheduleTimerId = Timers_Add(FillGapsScheduleProcessor, TICKS_SECOND, 0);
	
	LOGATT("Scheduled the operation. Wait for the schedule to complete!");
}

COMMAND(fillgaps, "Finds gaps in downloaded logs and requests missing entries from remote devices", cmd_args) {
	if (cmd_args->ArgumentCount != 4) {
		snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength,
				"Help: fillgaps [local] [nodeid] [first] [last]");
		return;
	}

	// read the parameters
	gap_search_query_t query;
	memset(&query, 0, sizeof(gap_search_query_t));
	query.Request.LocalSeqNo 	= atol(cmd_args->ArgumentArray[0]);
	query.Request.TargetNodeId	= atoi(cmd_args->ArgumentArray[1]);
	query.Request.FirstSeqNo 	= atol(cmd_args->ArgumentArray[2]);
	query.Request.LastSeqNo		= atol(cmd_args->ArgumentArray[3]);

	LogEngine_Sync_FillGaps(&query);
}

////// RANGE REQUESTS //////

void Handler_RangeRequest(uint8_t requestSource, const uint8_t* payload) {
	// it is crucial to copy the data in memory (this helps to align data types in memory)
	// (as you see, it's important to know the layout of data in your request packet..)
	uint32_t firstSeqNo;
	uint32_t lastSeqNo;
	
	memcpy(&firstSeqNo, &(payload[0]), sizeof(uint32_t));
	memcpy(&lastSeqNo, &(payload[4]), sizeof(uint32_t));

	// display information for the user
	LOGDBG_SWITCHED(DEBUG_SYNC, "Handling request from %u for range [%lu-%lu]", requestSource, firstSeqNo, lastSeqNo);

	// statistics collected during the operation and displayed to the user when the handler completes
	uint16_t counter_served = 0;
	
	// used to address local log entries
	uint32_t local_logsize = LogEngine_GetLogSize();
	
	Timers_Block(128); // empirical value. wait for the other side to prepare

	//serve all entries between the first and the last SeqNo requested
	//use firstSeqNo varialbe to point to the current log entry being processed
	for (; firstSeqNo <= lastSeqNo; firstSeqNo++) {
		// break the loop if it looks like an entry from outside of the local data range was requested
		if (firstSeqNo > local_logsize)
			break;
		
		// reset the watchdog to avoid resets during long queries
		Watchdog_Reset();
		
		// load the requested entry
		logentry_t entry;
		LogEngine_Read(&firstSeqNo, &entry);
		
		if (entry.Preamble != LOG_ENTRY_PREAMBLE) // this logentry is invalid, skip it
			continue;
		
		// update the sequence number
		entry.HostSequenceNo = firstSeqNo;
		entry.HostNodeId = Configuration.NodeID;
		
		// serve the requested entry over radio
		network_payload_t packet;
		Network_InitSendArgs(&packet);
	
		packet.Buffers[0].Buffer = &entry;
		packet.Buffers[0].Size = sizeof(logentry_t);

		Timers_Block(6); // empirical value. wait for the other side to process the previous packet
		Network_SendWithoutACK(requestSource, NETWORK_PROTOCOL_RANGE_RESPONSE, &packet);
		
		counter_served++; // count served
	}
	
	LOGDBG_SWITCHED(DEBUG_SYNC, "Served %u packets", counter_served);
}

void Handler_RangeResponse(uint8_t source, const uint8_t* data) {
	// it's very important to copy the data: it's required for proper alignment of data in memory
	logentry_t entry;
	memcpy(&entry, data, sizeof(logentry_t));
	
	LogEngine_Save(&entry);
	
	putchar('.'); // process indication for the user
	LogEngine_Sync_EntriesSynced++;
}

uint32_t LogEngine_Sync_Request(uint16_t target, uint32_t* firstSeqNo, uint32_t* lastSeqNo) {
	// build the query
	gap_search_query_t searchQuery;
	searchQuery.Request.TargetNodeId 	= target;
	searchQuery.Request.FirstSeqNo	 	= *firstSeqNo;
	searchQuery.Request.LastSeqNo 		= *lastSeqNo;
	searchQuery.Request.LocalSeqNo		= LogEngine_GetLogSize();
	
	if (searchQuery.Request.TargetNodeId == Configuration.NodeID) {
		LOGERR("Target node id needs can't be equal to the local node id.");
		return 0;
	}

	printf(	"Local Sequence No:\t%lu\r\n"
			"Target NodeID:\t\t%u\r\n"
			"First Sequence No:\t%lu\r\n"
			"Last Sequence No:\t%lu\r\n",
			searchQuery.Request.LocalSeqNo, searchQuery.Request.TargetNodeId, searchQuery.Request.FirstSeqNo, searchQuery.Request.LastSeqNo);

	network_payload_t packet;
	Network_InitSendArgs(&packet);
	
	packet.Buffers[0].Buffer 	= (uint8_t*) &searchQuery.Request.FirstSeqNo;
	packet.Buffers[0].Size 		= sizeof(searchQuery.Request.FirstSeqNo);
	packet.Buffers[1].Buffer 	= (uint8_t*) &searchQuery.Request.LastSeqNo;
	packet.Buffers[1].Size 		= sizeof(searchQuery.Request.LastSeqNo);
	
	Network_SendWithoutACK(searchQuery.Request.TargetNodeId, NETWORK_PROTOCOL_RANGE_REQUEST, &packet);
	
	return searchQuery.Request.LocalSeqNo;
}

COMMAND(request, "Requests a sequence of remote log entries", cmd_args) {
	if (cmd_args->ArgumentCount != 3) {
		snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength,
				"Help: request [nodeid] [first] [last]");
		return;
	}
	
	// build the query
	uint16_t targetNodeId 	= atoi(cmd_args->ArgumentArray[0]);
	uint32_t firstSeqNo	 	= atol(cmd_args->ArgumentArray[1]);
	uint32_t lastSeqNo 		= atol(cmd_args->ArgumentArray[2]);
	
	LogEngine_Sync_Request(targetNodeId, &firstSeqNo, &lastSeqNo);
}

/**
 * This handler is registered in the \ref LogEngine_Sync_Init function
 * as a receiver for log entries received over the radio.
 * 
 * The behavior is to store the log entry in the local storage.
 */
void Handler_LogEntry(uint8_t source, const uint8_t* data) {
	// it is important to copy the data in memory so that data types alignment doesn't get corrupted.
	// not relevant for a series of uint8_t bytes, but very relevant for e.g. uint32_t
	logentry_t entry;
	memcpy(&entry, data, sizeof(logentry_t));

	LogEngine_Save(&entry);
	/**
	 * This example shows how to access some detailed radio parameters about the received packet:
	 * 
	 * @code
	 * printf("[source:%u rssi:%i lqi:%u]\r\n", source, CC_LastRSSI, CC_LastLQI);
	 * @endcode
	*/
}

void LogEngine_Sync_Transmit(const uint16_t nodeid, const logentry_t* newEntry) {
	network_payload_t packet;
	Network_InitSendArgs(&packet);
	
	packet.Buffers[0].Buffer = (void*) newEntry;
	packet.Buffers[0].Size = sizeof(logentry_t);
	
	Network_SendWithoutACK(nodeid, 208, &packet);
}

void LogEngine_Sync_Init() {
	LogEngine_Sync_EntriesSynced = 0; 
	Network_RegisterHandler(208, Handler_LogEntry);
	 
	Network_RegisterHandler(NETWORK_PROTOCOL_SEQUENCE_REQUEST, Handler_SequenceRequest);
	Network_RegisterHandler(NETWORK_PROTOCOL_SEQUENCE_RESPONSE, Handler_SequenceResponse);

	Network_RegisterHandler(NETWORK_PROTOCOL_RANGE_REQUEST, Handler_RangeRequest);
	Network_RegisterHandler(NETWORK_PROTOCOL_RANGE_RESPONSE, Handler_RangeResponse);
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
