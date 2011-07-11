/**
 * @file
 * @brief		@b Driver: Logging Engine. Support for radio transmission of log entries.
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef LOGENGINE_SYNC_H_
#define LOGENGINE_SYNC_H_

#include "LogEngine.API.h"

/**
 * @ingroup NetworkProtocols
 * @brief	This protocol id is reserved for the LogEngine.Sync component
 */
#define		NETWORK_PROTOCOL_SEQUENCE_REQUEST		10

/**
 * @ingroup NetworkProtocols
 * @brief	This protocol id is reserved for the LogEngine.Sync component
 */
#define		NETWORK_PROTOCOL_SEQUENCE_RESPONSE		11

/**
 * @ingroup NetworkProtocols
 * @brief	This protocol id is reserved for the LogEngine.Sync component
 */
#define		NETWORK_PROTOCOL_RANGE_REQUEST			12

/**
 * @ingroup NetworkProtocols
 * @brief	This protocol id is reserved for the LogEngine.Sync component
 */
#define		NETWORK_PROTOCOL_RANGE_RESPONSE			13


/**
 * Parameter structure for the LogEngine_FindGap function (as the number of parameters that can be passed to
 * a function is limited.
 */
typedef struct {
	struct {
		uint32_t	FirstSeqNo;						///< The sequence number of the first requested entry
		uint32_t	LastSeqNo;						///< The sequence number of the last requested entry
		uint32_t	LocalSeqNo;						///< The local log engine sequence number from which the search should start
		uint16_t	TargetNodeId;					///< The node id of the selected device that should transmit the data
		void		(*Handler_Sync_Completed)();	///< Pointer to the handler that will be called after the Sync task completes.
	} Request;										///< The request
	struct {
		uint32_t	ExpectedCount;					///< Statistics: the number of exepected log entries
		uint32_t	MissingCount;					///< Statistics: the number of missing log entries
	} Response;										///< The response
	struct {
		int16_t		CurrentScheduleTimerId;			///< Timer used to schedule next request during autogaps - procedure (internal)
		uint32_t*	ExportExpectedCount;			///< Internal
		uint32_t*	ExportMissingCount;				///< Internal
	} Internal;										///< Internal
} gap_search_query_t;

/**
 * Stores the number of all entries that were received during the synchronisation process since the power on event. Can be used for benchmarking.
 */
uint32_t			LogEngine_Sync_EntriesSynced;

/**
 * @brief Transmit a log entry over the radio.
 *
 * @remarks
 * The LogEngine.Sync automatically stores all log entries received over radio. No notification is provided. 
 *
 * @param[in]	nodeid			The NodeId of the target. 
 * @param[in]	newEntry		The log entry.
 */
void LogEngine_Sync_Transmit(const uint16_t nodeid, const logentry_t* newEntry);

/**
 * @brief Sends a synchronisation request
 *
 * This is the equivalent of the @b range command (see \ref Commands module). Log entries stored at the selected device with sequence numbers
 * between the defined boundaries will be sent in a batch by the selected devices. It's likely that some data
 * will be lost during the batch transmission. Use the \ref LogEngine_Sync_FillGaps function to detect and request
 * the missing entries.
 * See implementation of the automatic base station for examples.
 * 
 * @param[in]	target			The NodeId of the target. 
 * @param[in]	firstSeqNo		The sequence number of the first log entry requested
 * @param[in]	lastSeqNo		The sequence number of the last log entry requested
 * 
 * @returns						The current local sequence number at the beginning of the synchronisation.
 */
uint32_t LogEngine_Sync_Request(uint16_t target, uint32_t* firstSeqNo, uint32_t* lastSeqNo);

/**
 * @brief Performs the gaps filling procedure
 *
 * This is the equivalent of the @b fillgaps command (see \ref Commands module). Take a look at the  implementation of the automatic base station for usage examples.
 * 
 * @param[in]	query			The detailed query 
 */
void LogEngine_Sync_FillGaps(gap_search_query_t* query);

/**
 * @brief Initializes the Sync engine for LogEngine
 *
 * Radio handlers are registered here. 
 *
 */
void LogEngine_Sync_Init();

#endif /*LOGENGINE_SYNC_H_*/

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
