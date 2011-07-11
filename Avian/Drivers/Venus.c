#include <string.h>
#include <mspgcc/ringbuffer.h>

#include "Device.h"

#include "../System/System.h"
#include "../System/BitOperations.h"
#include "../System/Logging.h"

#include "../System/RTC.API.h"
#include "../System/Timers.API.h"
#include "../System/USART.API.h"
#include "../System/Commands.API.h"

#include "NMEA.h"

#include "Venus.h"
#include "Venus.API.h"

/**
 * Maximu size of of the buffer used for binary communication with the Venus GPS chip.
 */ 
#define		VENUS_RX_BUFFER_MAX_LENGTH			160

#ifndef		VENUS_RMC_UPDATE_RATE
/*
 * The RMC sentence contains date and time information received from the GPS system.
 * The frequency with which the RMC sentece is proviced by the GPS receiver depends on the system position rate.
 * The number provided here specifies the requency relative to the system position rate.
 * 
 * Example:
 * 	Setting the value to 200 means, the RMC sentence will be delivered every 200th time. Depending on the system position rate, e.g. when
 * 	the system position rate is set to 10Hz, the RMC sentence will be delivered every 20 seconds
 *  and if the system position rate is set to 1Hz, the RMC sentence will be delivered every 200 seconds (3 minutes 20 seconds).
 */
	#define		VENUS_RMC_UPDATE_RATE		200
#endif

/**
 * used for debugging only
 */
#define		VENUS_INCOMING_READ_MESSAGE_DEBUG		0

/**
 * Ringbuffer used to cache data received from the Venus GPS chip when it's operating in the binary communication mode.
 */
RINGBUFFER_NEW(_venus_rx_ringbuffer, VENUS_RX_BUFFER_MAX_LENGTH);

/**
 * Internal counter used to count number of valid binary messages
 * received from the Venus GPS chip and cached in the ring buffer.
 * this value is increased by the ISR and reduced by the read message function.
 */
volatile uint8_t _binary_messages_in_buffer;

/// states used by the state machines while reading and receiving binarry messages
enum venus_message_processing_states {
	WaitForSOS_1st,
	WaitForSOS_2nd,
	WaitForLength_1st,
	WaitForLength_2nd,
	WaitForMsgId_Payload_CS,
	WaitForEOS_1st,
	WaitForEOS_2nd
};

/// this variable is used to maintain the state of the state machine while it's receiving messages via ISR byte by byte
volatile enum venus_message_processing_states _venus_message_receive_state;

/// message parts (skytraq binary protocol)
#define		SOS_1st		0xA0
#define		SOS_2nd		0xA1
#define		EOS_1st		0x0D
#define		EOS_2nd		0x0A

/**
 * the following block contains definitions of binary commands used later in the code
 */

// definition of a struct that will store a command
typedef struct {
	const uint8_t* 				command;			///< the command id
	uint8_t						commandLength;		///< the length of the command
} venus_command_t;

#define UPDATE_TO_SRAM		0x00
#define UPDATE_TO_FLASH		0x01

// command contents:
const uint8_t venus_command_ConfigureMessageTypeNmea[] = 			{ 0x09, 0x01, UPDATE_TO_SRAM };
const uint8_t venus_command_ConfigureMessageTypeBinary[] = 			{ 0x09, 0x02, UPDATE_TO_SRAM }; // not in flash
const uint8_t venus_command_QuerySoftwareVersion[] = 				{ 0x02, UPDATE_TO_FLASH };
const uint8_t venus_command_ConfigureSerialPort38400[] = 			{ 0x05, 0x00, 0x03, UPDATE_TO_FLASH };
const uint8_t venus_command_ConfigureSerialPort115200[] = 			{ 0x05, 0x00, 0x05, UPDATE_TO_SRAM }; // not used - should be removed in final version
const uint8_t venus_command_QueryNavigationMode[] = 				{ 0x3D };
const uint8_t venus_command_QueryPositionUpdateRate[] = 			{ 0x10 };
const uint8_t venus_command_ConfigureSystemPositionRate_10Hz[] =	{ 0x0E, 10, UPDATE_TO_FLASH };
const uint8_t venus_command_ConfigureSystemPositionRate_8Hz[] = 	{ 0x0E, 8, UPDATE_TO_FLASH };
const uint8_t venus_command_ConfigureSystemPositionRate_5Hz[] = 	{ 0x0E, 5, UPDATE_TO_FLASH };
const uint8_t venus_command_ConfigureSystemPositionRate_4Hz[] = 	{ 0x0E, 4, UPDATE_TO_FLASH };
const uint8_t venus_command_ConfigureSystemPositionRate_2Hz[] = 	{ 0x0E, 2, UPDATE_TO_FLASH };
const uint8_t venus_command_ConfigureSystemPositionRate_1Hz[] = 	{ 0x0E, 1, UPDATE_TO_FLASH };
const uint8_t venus_command_ConfigureNavigatnionMessageInterval[] = { 0x11, 1, UPDATE_TO_SRAM }; // doesn't work, should be removed
const uint8_t venus_command_ConfigureNavigatnionModeCar[] = 		{ 0x3C, 0x00, UPDATE_TO_FLASH }; 
const uint8_t venus_command_ConfigureNavigatnionModePedestrian[] = 	{ 0x3C, 0x01, UPDATE_TO_FLASH }; 
const uint8_t venus_command_QueryWaasStatus[] = 					{ 0x38 };
																	//m_id  GGA   GSA   GSV   GLL   RMC   VTG   ZDA   ATTR
const uint8_t venus_command_ConfigureNMEAMessage[] = 				{ 0x08, 0x01, 0x00, 0x00, 0x00, VENUS_RMC_UPDATE_RATE, 0x00, 0x00, UPDATE_TO_FLASH }; 
																	//m_id  GGA   GSA   GSV   GLL   RMC   VTG   ZDA   ATTR
const uint8_t venus_command_ConfigureNMEAMessage_FullSet[] = 		{ 0x08, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, UPDATE_TO_FLASH }; 
/**
 * Explanation to the Configure NMEA Message Command:
 * 
 * RMC 0xC8 == 200 decimal means:
 * 	the RMC sentence will be delivered every 200th time. Depending on the system position rate, e.g. when
 * 	the system position rate is set to 10Hz, the RMC sentence will be delivered every 20 seconds
 *  and if the system position rate is set to 1Hz, the RMC sentence will be delivered every 200 seconds (3 minutes 20 seconds)
 */

// storing commands in an array for furhter reuse
// it's important that the order in the array and the enum fields don't change
const venus_command_t venus_commands[] = 
{
	{ venus_command_ConfigureMessageTypeBinary, 			sizeof(venus_command_ConfigureMessageTypeBinary) },
	{ venus_command_ConfigureMessageTypeNmea, 				sizeof(venus_command_ConfigureMessageTypeNmea) },
	{ venus_command_QuerySoftwareVersion, 					sizeof(venus_command_QuerySoftwareVersion) },
	{ venus_command_ConfigureSerialPort38400,				sizeof(venus_command_ConfigureSerialPort38400) },
	{ venus_command_ConfigureSerialPort115200,				sizeof(venus_command_ConfigureSerialPort115200) },
	{ venus_command_QueryNavigationMode, 					sizeof(venus_command_QueryNavigationMode) },
	{ venus_command_QueryPositionUpdateRate, 				sizeof(venus_command_QueryPositionUpdateRate) },
	{ venus_command_ConfigureSystemPositionRate_10Hz, 		sizeof(venus_command_ConfigureSystemPositionRate_10Hz) },
	{ venus_command_ConfigureSystemPositionRate_8Hz, 		sizeof(venus_command_ConfigureSystemPositionRate_8Hz) },
	{ venus_command_ConfigureSystemPositionRate_5Hz, 		sizeof(venus_command_ConfigureSystemPositionRate_5Hz) },
	{ venus_command_ConfigureSystemPositionRate_4Hz, 		sizeof(venus_command_ConfigureSystemPositionRate_4Hz) },
	{ venus_command_ConfigureSystemPositionRate_2Hz, 		sizeof(venus_command_ConfigureSystemPositionRate_2Hz) },
	{ venus_command_ConfigureSystemPositionRate_1Hz, 		sizeof(venus_command_ConfigureSystemPositionRate_1Hz) },
	{ venus_command_ConfigureNavigatnionMessageInterval, 	sizeof(venus_command_ConfigureNavigatnionMessageInterval) },
	{ venus_command_ConfigureNavigatnionModeCar, 			sizeof(venus_command_ConfigureNavigatnionModeCar) },
	{ venus_command_ConfigureNavigatnionModePedestrian, 	sizeof(venus_command_ConfigureNavigatnionModePedestrian) },
	{ venus_command_QueryWaasStatus, 						sizeof(venus_command_QueryWaasStatus) },
	{ venus_command_ConfigureNMEAMessage, 					sizeof(venus_command_ConfigureNMEAMessage) },
	{ venus_command_ConfigureNMEAMessage_FullSet, 			sizeof(venus_command_ConfigureNMEAMessage_FullSet) }
};

// enum definition for easier command usage in the code
enum venus_command_type {
	ConfigureMessageTypeBinary 			= 0,
	ConfigureMessageTypeNmea 			= 1,
	QuerySoftwareVersion 				= 2,
	ConfigureSerialPort38400 			= 3,
	ConfigureSerialPort115200 			= 4,
	QueryNavigationMode 				= 5,
	QueryPositionUpdateRate 			= 6,
	ConfigureSystemPositionRate_10Hz 	= 7,
	ConfigureSystemPositionRate_8Hz 	= 8,
	ConfigureSystemPositionRate_5Hz 	= 9,
	ConfigureSystemPositionRate_4Hz 	= 10,
	ConfigureSystemPositionRate_2Hz 	= 11,
	ConfigureSystemPositionRate_1Hz 	= 12,
	ConfigureNavigatnionMessageInterval = 13,
	ConfigureNavigationModeCar 			= 14,
	ConfigureNavigationModePedestrian 	= 15,
	QueryWaasStatus 					= 16,
	ConfigureNMEAMessage 				= 17,
	ConfigureNMEAMessage_FullSet 		= 18
};

/**
 * end of the block containing command definitions.
 */


/**
 * @brief	Reads requested binary message from internal buffer.
 * 
 * All other messages stored in the internal buffer prior to the requested message that don't match the MessageId will be discarded.
 * As concequence, if you request a unknown MessageId, all MessageOds from the buffer will be discarded.
 *  
 * @param[in]	requestedMessageId		MessageId of the requested message.
 * @param[out]	bufferOut				Buffer which should be used to store the message.
 * @param[out]	buffer_length			The length of the bufferOut buffer. 
 * 
 * @returns		Number of bytes read. 0 on failure.
 */
uint8_t Venus_ReadMessage(uint8_t requestedMessageId, uint8_t* bufferOut, uint8_t buffer_length) {
	/**
	 * now, messages from the buffer will be processed. all messages that weren't requested will be discarded
	 * so if e.g. the message 0x80 is ordered, all ACKs will be skipped/discarded
	 */
	
	// used to control the position in the output buffer, will be increased after every protocol conform character and set to 0 at failure
	uint8_t positionInBuffer = 0;
	
	// read from the message, the message specification contains payload length. this value is used to detect the end of message (since EOS could be part of the payload)
	uint16_t messageLength;
	
	// used co control the state of the payload format detection state machine
	enum venus_message_processing_states _venus_message_read_state = WaitForSOS_1st;
	
	// used to force the state machine to reinitialize itself and start analyzying
	// the data from the selected position, including the currently active byte from the RB.
	bool retry = false;
	
	// temporary variable used to store the recent byte read from the ring buffer
	int16_t byteFromBuffer = 0x00;
	
	// checksum of the message, only valid messages should be processed
	uint8_t checksum = 0x00;

	// this is the main loop, it stops when there are no messages in the buffer
	// or the buffer is too small.
	// at the end of the loop there will be an additional break condition:
	// it will stop the loop when the requested message is received.
	while (_binary_messages_in_buffer > 0 && positionInBuffer < buffer_length) {

		if (retry == false) {
			// get next entry from the ring buffer
			byteFromBuffer = ringbuffer_get(&_venus_rx_ringbuffer);
			if (byteFromBuffer == -1) {
				// the buffer was empty, error
				_binary_messages_in_buffer = 0; 
				return 0;
			}
		}
		
		retry = false;
		
		uint8_t data = (uint8_t) byteFromBuffer;
		
#if VENUS_INCOMING_READ_MESSAGE_DEBUG
printf("{0x%.2X}", data);
#endif
		
		// now, copy the character into the output buffer
		bufferOut[positionInBuffer] = data;
		// prepare the index for the next character (it might be, it will be reset in the state machine code!)
		positionInBuffer++;
		
		// now, use the state machine to deciede whether the loop should break (locate the SOS, EOS, check the CS)
		
		switch (_venus_message_read_state) {
			case WaitForSOS_1st : {

#if VENUS_INCOMING_READ_MESSAGE_DEBUG
putchar('1');
#endif

				if (data == SOS_1st)
					// success path:
					_venus_message_read_state = WaitForSOS_2nd;
				else
					// failure path:
					positionInBuffer = 0; // error handling, ignore the character just read, it wasn't SOS
				
				break;
			}
			case WaitForSOS_2nd : {

#if VENUS_INCOMING_READ_MESSAGE_DEBUG
putchar('2');
#endif
				if (data == SOS_2nd) {
					// success path: next, the length of the message should be available
					messageLength = 0; // initialize the message length variable
					_venus_message_read_state = WaitForLength_1st;
				} else {
					// failure path
					_venus_message_read_state = WaitForSOS_1st; // reset the state machine
					positionInBuffer = 0; // and go back to the start of the buffer
					
					retry = true; // the same character should be evaluated, it might be SOS
				}
	
				break;
			}
			case WaitForLength_1st : {
				
#if VENUS_INCOMING_READ_MESSAGE_DEBUG
putchar('3');
#endif
				// in this case, no verification can be made, we just need to trust the data as it comes
				uint8_t* length_pointer = (uint8_t*) &messageLength;
				length_pointer[1] = data;
				
				_venus_message_read_state = WaitForLength_2nd;
				
				break;
			}
			case WaitForLength_2nd : {
				
#if VENUS_INCOMING_READ_MESSAGE_DEBUG
putchar('4');
#endif

				// in this case, no verification can be made, we just need to trust the data as it comes
				uint8_t* length_pointer = (uint8_t*) &messageLength;
				length_pointer[0] = data;
				
				_venus_message_read_state = WaitForMsgId_Payload_CS;
				
				break;
			}
			case WaitForMsgId_Payload_CS : {
				
#if VENUS_INCOMING_READ_MESSAGE_DEBUG
putchar('5');
#endif

				if (messageLength > 0) {
					// countdown the counter, receive the payload, no easy data verification possible
					checksum ^= data; // compute the checksum
					messageLength--;
				} else {
					// now, the payload was received, and the CS should be read, also, the state changes
					
					if (checksum == data) {
						_venus_message_read_state = WaitForEOS_1st;
					} else {
						_venus_message_read_state = WaitForSOS_1st;
						positionInBuffer = 0;
						checksum = 0x00;
						retry = true; // the same character should be evaluated, it might be SOS
					}
				}
	
				break;
			}
			case WaitForEOS_1st : {
				
#if VENUS_INCOMING_READ_MESSAGE_DEBUG
putchar('6');
#endif

				if (data == EOS_1st) {
					// success path:
					_venus_message_read_state = WaitForEOS_2nd;
				} else {
					// failure path:
					_venus_message_read_state = WaitForSOS_1st;
					positionInBuffer = 0;
					
					retry = true; // the same character should be evaluated, it might be SOS
				}
		
				break;
			}
			case WaitForEOS_2nd : {
				
#if VENUS_INCOMING_READ_MESSAGE_DEBUG
putchar('7');
#endif

				if (data == EOS_2nd) {
					// success path:
					_binary_messages_in_buffer--;
				} else {
					// failure path
					positionInBuffer = 0; // this has to stay here since it's used in a if-statement below to detect whether the message was read
					retry = true; // the same character should be evaluated, it might be SOS
				}
				
				_venus_message_read_state = WaitForSOS_1st;				
	
				break;
			}
		} // switch
		
		// now we could check whether the message was completely read
		// this is simple: this is the only possible condition for a properly read message
		if (_venus_message_read_state == WaitForSOS_1st && positionInBuffer > 0) {
			if (bufferOut[4] == requestedMessageId) {
				// the messageId in the message's payload matches the requested message id
				// break the while loop:
				
#if VENUS_INCOMING_READ_MESSAGE_DEBUG
printf("STOPPED[0x%.2X<]", bufferOut[4]);
#endif
				break;
			} else {
				
#if VENUS_INCOMING_READ_MESSAGE_DEBUG
printf("SKIPPED[0x%.2X<]", bufferOut[4]);
#endif

				positionInBuffer = 0; // reset the index to the buffer
			}
		}
		
	} // while messagesinbuffer > 0
	
	// now, it's very likely, the buffer contains the reply
	// but, it mitht also be that the loop exited becase the buffer was too small, etc
	
	if (_venus_message_read_state != WaitForSOS_1st || positionInBuffer == 0 || bufferOut[4] != requestedMessageId) {
		positionInBuffer = 0;
	}
			
#if VENUS_INCOMING_READ_MESSAGE_DEBUG
	uint8_t ii;
	printf("\r\n< ");
	for (ii = 0; ii < positionInBuffer; ii++) {
		printf("0x%.2X ", bufferOut[ii]);
	}
	printf(">\r\n");
#endif
	
	return positionInBuffer;
}

/**
 * Blocks until ACK for a specified MessageId arrives.
 * A read is executed few times with a blocking delay in between.
 * 
 * @remarks		This function blocks for max. 20*128*122us = 0.3 s
 * 
 * @returns		True on ACK and False on NACK or no reply
 */
bool Venus_WaitForACK(uint8_t messageIdToACK) {
	uint8_t retryCounter = 20;
	
	uint8_t buffer[10]; // ACK message has only 9 bytes
	
	while (retryCounter > 0) {
		Timers_Block(128);

		uint8_t readLength = Venus_ReadMessage(0x83, buffer, 10);

		// if there was an ACK received, check whether the requested command was ACKed
		if (readLength > 0 && buffer[5] == messageIdToACK)
			return true;

		// no ACK received, retry
		retryCounter--;
	}
	
	return 0;
}

/**
 * Submits a command to the Venus chip.
 * 
 * The functions expects the VenusGPS to be enabled and UART1 configured.
 */
bool Venus_SubmitCommand(enum venus_command_type command) {
	/**
	 * Command Format:
	 * 
	 * [fixed]					[computed]					[ cmd					]	[computed]			[fixed]
	 * Start Of Sequence (SOS)	Payload Length (2 bytes)	Message ID	Message Body	Checksum (1 byte)	End Of Sequence (EOS)
	 * 0xA0 0xA1				0x?? 0x??					0x??		0x??...			0x??				0x0D 0x0A
	 */
	
	uint8_t StartOfSequence[]	= { 0xA0, 0xA1 };  
	uint8_t EndOfSequence[]		= { 0x0D, 0x0A };

	// the cmd pointer and the cmd_length need to be configured by the switch/case code below
	const uint8_t*	cmd = venus_commands[command].command;
	uint16_t cmd_length = venus_commands[command].commandLength;
	
	//// now, the message will be transmitted
	
	// SOS
	USART1_UART_TX_Buffer(2, StartOfSequence);

	// Length (as two bytes)
	uint8_t* cmd_length_pointer = (uint8_t*) &cmd_length;
	USART1_UART_TX(cmd_length_pointer[1]);
	USART1_UART_TX(cmd_length_pointer[0]);
	
	// The command
	USART1_UART_TX_Buffer(cmd_length, cmd);
	
	// check sum (use cmd_length variable as a counter since it's not going to be used in this function anymore
	uint8_t cs = 0;
	for (; cmd_length > 0; cmd_length--) {
		cs ^= cmd[cmd_length - 1];
	}
	USART1_UART_TX(cs);

	// EOS
	USART1_UART_TX_Buffer(2, EndOfSequence);
	
	/**
	 * the command was submitted.
	 * now, wait for the ACK/NACK and return the answer as a bool to the user.
	 */
	
	return Venus_WaitForACK(cmd[0]); // the message id is stored at the position 0 in the array
}

/**
 * this variable is used to cache the lenght of the message being currently read.
 * this is required since the Handler needs to know when the message is going to end.
 */
volatile uint16_t _incomingBytesHandler_currentMessageLength;

static bool _Venus_IncomingBytesHandler (uint8_t data) {
	if (ringbuffer_len(&_venus_rx_ringbuffer) == VENUS_RX_BUFFER_MAX_LENGTH) {
		// critical state, the buffer is full, looks like the data isn't processed..
		ringbuffer_clear(&_venus_rx_ringbuffer);
		_binary_messages_in_buffer = 0;
		_venus_message_receive_state = WaitForSOS_1st;
		LOGDBG("VENUS RINGBUFFER FULL!"); // should be logged to flash when Markus is done
		
		SystemFlags.Flag.Pending_WakeUp_Request = true; // wake up the systen so that the error can be processed.
	}
	
	switch (_venus_message_receive_state) {
		case WaitForSOS_1st : {
			if (data == SOS_1st) {
				ringbuffer_put(&_venus_rx_ringbuffer, data);
				
				_venus_message_receive_state = WaitForSOS_2nd;
			}

			break;
		}
		case WaitForSOS_2nd : {
			if (data == SOS_2nd) {
				ringbuffer_put(&_venus_rx_ringbuffer, data);
				
				_incomingBytesHandler_currentMessageLength = 0; // initialize the variable
				
				_venus_message_receive_state = WaitForLength_1st;
			} else {
				_venus_message_receive_state = WaitForSOS_1st; // reset the state machine
			}

			break;
		}
		case WaitForLength_1st : {
			ringbuffer_put(&_venus_rx_ringbuffer, data);

			// in this case, no verification can be made, we just need to trust the data as it comes
			uint8_t* length_pointer = (uint8_t*) &_incomingBytesHandler_currentMessageLength;
			length_pointer[1] = data;
			
			_venus_message_receive_state = WaitForLength_2nd;
			
			break;
		}
		case WaitForLength_2nd : {
			ringbuffer_put(&_venus_rx_ringbuffer, data);
			
			// in this case, no verification can be made, we just need to trust the data as it comes
			uint8_t* length_pointer = (uint8_t*) &_incomingBytesHandler_currentMessageLength;
			length_pointer[0] = data;
			
			_venus_message_receive_state = WaitForMsgId_Payload_CS;
			
			break;
		}
		case WaitForMsgId_Payload_CS : {
			ringbuffer_put(&_venus_rx_ringbuffer, data);
			
			if (_incomingBytesHandler_currentMessageLength > 0) {
				// countdown the counter, receive the payload
				_incomingBytesHandler_currentMessageLength--;
			} else {
				// now, the payload was received, and the CS should be read, also, the state changes
				_venus_message_receive_state = WaitForEOS_1st;
			}

			break;
		}
		case WaitForEOS_1st : {
			if (data == EOS_1st) {
				ringbuffer_put(&_venus_rx_ringbuffer, data);
				
				_venus_message_receive_state = WaitForEOS_2nd;
			}
	
			break;
		}
		case WaitForEOS_2nd : {
			if (data == EOS_2nd) {
				ringbuffer_put(&_venus_rx_ringbuffer, data);
				
				_binary_messages_in_buffer++;
				
				_venus_message_receive_state = WaitForSOS_1st;	
				
				SystemFlags.Flag.Pending_WakeUp_Request = true; // wake up the system so that the response can be processed
			}

			break;
		}
	}
	
	return true;
}

static void _Venus_NMEA_Processing_Handler(const char* input) {
	if (NMEA_ValidSentence(input) == false) {
		// try to find the end of string for debugging:
		uint8_t index;
		for (index = 0; index < 120; index++) {
			if (input[index] == 0x00)
				break;
		}

		LOGERR("CHKSUM FAIL (0x00 at %u): >%s<", index, input);

		LogEngine_SaveText("NMEAchecksum");

		return;
	}
	
	if (!strncmp( input, "$GPGGA", 6 )) {
		// continue processing the GGA entry only when a handler was provided
		if (Handler_GPS_NMEA_GGA != NULL) {
			// try to decode the entry
			logentry_t entry;
			if (NMEA_Read_GPGGA(input, &entry)) {
				// foward to the application for processing, e.g. logging (the check whether the handler was provided was already executed
				Handler_GPS_NMEA_GGA(input, &entry);				
			}	
		}
		return;
	}
	
	if (!strncmp( input, "$GPRMC", 6 )) {
		time_t time_gps;
		rawtime_t rawtime_gps;
		
		if (NMEA_Read_GPRMC(input, &time_gps, &rawtime_gps)) {
			
			if (RTC_ValidTime(&rawtime_gps)) {
				// the gps time is valid. compute the difference now.

				rawtime_t rawtime_system;
				RTC_GetRawTime(&rawtime_system);
				
				uint32_t timeDifference = 0;
				if (rawtime_system.seconds > rawtime_gps.seconds)
					timeDifference = rawtime_system.seconds - rawtime_gps.seconds;
				else
					timeDifference = rawtime_gps.seconds - rawtime_system.seconds;
				
				// if the difference is big enough, update the system time
				 
				if (timeDifference > 2) {
					// update
					RTC_SetRawTime(&rawtime_gps);

					// and log this event
					logentry_t entry;
					LogEngine_PrepareLogEntry(&entry);
					entry.TypeId = Debug;
					entry.Payload.Debug.DebugCode = 3; // updated the time
					entry.Payload.Debug.Value6_32bit = rawtime_system.seconds;
					entry.Payload.Debug.Value5_16bit = rawtime_system.miliseconds;
					LogEngine_Save(&entry);
				} // if the timediff was big enough
			} // if gps time is valid
			
			// received a valid RMC sentence from the device.
			// foward to the application for processing
			if (Handler_GPS_NMEA_RMC != NULL)
				Handler_GPS_NMEA_RMC(input, &time_gps, &rawtime_gps);
			
		} // if the RMC sentence was read
		
		return;
	}
}


void _Venus_NMEA_Debug_Handler(const char* input) {
	if (NMEA_ValidSentence(input) == false) {
		
		// try to find the end of string for debugging:
		uint8_t index;
		for (index = 0; index < 120; index++) {
			if (input[index] == 0x00)
				break;
		}

		LOGERR("CHKSUM FAIL (0x00 at %u): >%s<", index, input);
		
		return;
	}
}

void Venus_PowerOn() {
	// turn on the power supply of the GPS chip
	SET(GPS_PWR_OUT, GPS_PWR_PIN);
	Timers_Block(4096);
	
	USART1_PowerOn();
	
	LogEngine_SaveText("VenusOn");
}

void Venus_PowerOff() {
	// turn off the power supply of the GPS chip
	CLEAR(GPS_PWR_OUT, GPS_PWR_PIN);
	
	// dislable the interrupt on UART1 RX and unregister handlers
	USART1_PowerOff();
	UART1_CharacterPreview = NULL;
	UART1_LineReceivedHandler = NULL;
	
	LogEngine_SaveText("VenusOff");
}

void Venus_StartProcessingNMEA() {
	// test whether the GPS chip is enabled.
	// dont activate NMEA processing if GPS isn't enabled
	if (GPS_PWR_OUT & GPS_PWR_PIN) {
	 	Venus_SubmitCommand(ConfigureMessageTypeNmea);
		
		UART1_CharacterPreview = NULL;
		UART1_LineReceivedHandler = _Venus_NMEA_Processing_Handler;
	}
}

bool Venus_Init(){
	// initialise state machines
	_venus_message_receive_state = WaitForSOS_1st;
	_binary_messages_in_buffer = 0;
	
	LOGDBG("VenusGPS Init...");
	
	// Initializing UART: no line handler (this one is important only while debugging)
	//USART1_Init(NULL, _Venus_IncomingBytesHandler);
	USART_Init(USART1, 9600L);
	UART1_CharacterPreview = _Venus_IncomingBytesHandler;

	// power on the GPS module
	SET(GPS_PWR_OUT, GPS_PWR_PIN);
	Timers_Block(4096);

	// probe the venus gps at 9600 baud and reconfigure if necessary
	if (Venus_SubmitCommand(ConfigureMessageTypeBinary) == true) {
		LOGDBG("VenusGPS: Detected at 9600 baud. Reconfiguring to 38400...");
		if (Venus_SubmitCommand(ConfigureSerialPort38400)) {
			LOGDBG("VenusGPS: Reconfiguring to 38400: OK");
		} else {
			LOGDBG("VenusGPS: Reconfiguring to 38400: FAIL!");
			Venus_PowerOff();
			return false;
		}
	}
	
	// now, the VenusGPS should run at 38400 baud
	// the UART of the MSP should be reconfigured as well
	USART_UART_Configure(USART1, 38400L);
	
	Timers_Block(2048);

	// try to talk to the venusgps again
	if (Venus_SubmitCommand(ConfigureMessageTypeBinary))
		LOGDBG("VenusGPS: Detected at 38400 baud");
	else {
		Venus_PowerOff();
		return false;
	}

	// debug information: query software version
	if (Venus_SubmitCommand(QuerySoftwareVersion)) {
		uint8_t buffer[21];
		if (Venus_ReadMessage(0x80, buffer, sizeof(buffer)) > 0)
			LOGDBG(
				"VenusGPS: Kernel Version: %u.%u.%u, ODM Version: %u.%u.%u, Revision: 20%.2u.%.2u.%.2u",
				buffer[7], buffer[8], buffer[9],
				buffer[11], buffer[12], buffer[13],
				buffer[15], buffer[16], buffer[17]
			);
	} else {
		Venus_PowerOff();
		return false;
	}

	// WAAS status (WAAS works only in the US!)
	if (Venus_SubmitCommand(QueryWaasStatus)) {
			uint8_t buffer[10];
			if (Venus_ReadMessage(0xB3, buffer, 10) > 0)
				LOGDBG("VenusGPS: WAAS Status: %u", buffer[5]);
	}	

	// query navigation mode
	if (Venus_SubmitCommand(QueryNavigationMode)) {
		uint8_t buffer[10];
		if (Venus_ReadMessage(0xB5, buffer, sizeof(buffer)) > 0)
			LOGDBG("VenusGPS: Navigation Mode: %u (0: car, 1: pedestrian)", buffer[5]);
	}
	
	/* Don't configure the PositionUpdateRate: leave it for the user.
	if (Venus_SubmitCommand(QueryPositionUpdateRate)) {
			uint8_t buffer[10];
			if (Venus_ReadMessage(0x86, buffer, 10) > 0) {
				if (buffer[5] != 10) {
					Venus_SubmitCommand(ConfigureSystemPositionRate_10Hz);
				}
			}
	}
	*/
	
	if (Venus_SubmitCommand(QueryPositionUpdateRate)) {
			uint8_t buffer[10];
			if (Venus_ReadMessage(0x86, buffer, 10) > 0) {
				LOGDBG("VenusGPS: Position Update Rate: %u Hz", buffer[5]);
			}
	}
	
	if (Venus_SubmitCommand(ConfigureNMEAMessage)) {
		LOGDBG("VenusGPS: NMEA configured: GGA: %u GSA: %u GSV: %u GLL: %u RMC: %u VTG: %u ZDA: %u",
				venus_command_ConfigureNMEAMessage[1],
				venus_command_ConfigureNMEAMessage[2],
				venus_command_ConfigureNMEAMessage[3],
				venus_command_ConfigureNMEAMessage[4],
				venus_command_ConfigureNMEAMessage[5],
				venus_command_ConfigureNMEAMessage[6],
				venus_command_ConfigureNMEAMessage[7]
			);
	}

	Venus_PowerOff();
	
	LOG("VenusGPS Init: OK");
	
	return true;
}

void Venus_ChangePositionUpdateRate(uint8_t requestedPositionUpdateRate) {
	if ((GPS_PWR_OUT & GPS_PWR_PIN) != GPS_PWR_PIN) {
		return; // return if the GPS is not enabled.
	}

	UART1_CharacterPreview = _Venus_IncomingBytesHandler;
	UART1_LineReceivedHandler = NULL;
	
	if (Venus_SubmitCommand(ConfigureMessageTypeBinary) == false) {
		LOGERR("Venus: Communication failure.");
	}
	if (Venus_SubmitCommand(ConfigureMessageTypeBinary) == false) {
		LOGERR("Venus: Communication failure.");
	}
	
	if (Venus_SubmitCommand(QueryPositionUpdateRate)) {
			uint8_t buffer[10];
			if (Venus_ReadMessage(0x86, buffer, 10) > 0) {
				if (buffer[5] != requestedPositionUpdateRate) {
					switch (requestedPositionUpdateRate) {
						case 1: Venus_SubmitCommand(ConfigureSystemPositionRate_1Hz); break;
						case 2: Venus_SubmitCommand(ConfigureSystemPositionRate_2Hz); break;
						case 4: Venus_SubmitCommand(ConfigureSystemPositionRate_4Hz); break;
						case 5: Venus_SubmitCommand(ConfigureSystemPositionRate_5Hz); break;
						case 8: Venus_SubmitCommand(ConfigureSystemPositionRate_8Hz); break;
						case 10: Venus_SubmitCommand(ConfigureSystemPositionRate_10Hz); break;
					}
				}
			}
	}
	
	if (Venus_SubmitCommand(QueryPositionUpdateRate)) {
		uint8_t buffer[10];
		if (Venus_ReadMessage(0x86, buffer, 10) > 0) {
			LOGDBG("VenusGPS: Position Update Rate: %u Hz", buffer[5]);
		}
	}
	
	UART1_CharacterPreview = NULL;
}

bool _Venus_IncomingBytesDebugHandler (uint8_t data) {
	putchar(data);
	LED1_TOGGLE;
	//UART0_WAIT_TXDONE();
	SystemFlags.Flag.Pending_WakeUp_Request = true;
	return false;
}

COMMAND(venus, "Venus GPS debug interface", cmd_args) {
	if (cmd_args->ArgumentCount != 1) {
		printf("Usage: venus [operation]\r\n"
				" 1: Full NMEA debug mode (one way, reset to disable)\r\n"
				" 2: Toggle NMEA preview mode\r\n"
				" 3: Toggle Navigation Mode (permanently)\r\n"
				" 4: Toggle Power of the Venus Chip\r\n"
				" 5: Toggle _Venus_NMEA_Debug_Handler\r\n"
				" 6: Enable Venus, Start Logging (one way)\r\n"
				);
		return;
	}
	
	 switch (cmd_args->ArgumentArray[0][0]) {
		 case '1' : {
			 	Venus_PowerOn();

			 	Timers_Block(8192); // a delay is required. the time required is unknown but since it's the debug mode it's not important righ tnow
			 	
				UART1_CharacterPreview = _Venus_IncomingBytesHandler;
			 	
				Venus_SubmitCommand(ConfigureNMEAMessage_FullSet);

				Venus_SubmitCommand(ConfigureSystemPositionRate_10Hz);
			 	
				UART1_CharacterPreview = NULL; //_Venus_IncomingBytesDebugHandler;
				UART1_LineReceivedHandler = _Venus_NMEA_Debug_Handler;
				
				break;
		 }
		 case '2' : {
			 	if (UART1_CharacterPreview == _Venus_IncomingBytesDebugHandler)
			 		UART1_CharacterPreview = NULL;
			 	else
			 		UART1_CharacterPreview = _Venus_IncomingBytesDebugHandler;
			 	
				break;
		 }
		 case '3' : {
			 
				SET(GPS_PWR_OUT, GPS_PWR_PIN);
				
				// enable the interrupt on UART1 RX and register neccessary handlers
				USART1_PowerOn();
				UART1_CharacterPreview = _Venus_IncomingBytesHandler;
				
				// query navigation mode
				if (Venus_SubmitCommand(QueryNavigationMode)) {
					uint8_t buffer[10];
					if (Venus_ReadMessage(0xB5, buffer, sizeof(buffer)) > 0) {
						if (buffer[5] == 0) {
							Venus_SubmitCommand(ConfigureNavigationModePedestrian);
						} else {
							Venus_SubmitCommand(ConfigureNavigationModeCar);
						}
					}
				}
				
				if (Venus_SubmitCommand(QueryNavigationMode)) {
					uint8_t buffer2[10];
					if (Venus_ReadMessage(0xB5, buffer2, sizeof(buffer2)) > 0) {
						printf("Current Navigation Mode: %u (0: car, 1: pedestrian)\r\n", buffer2[5]);
					}
				}
				
				Venus_PowerOff();
		 }
		 case '4' : {
			 if (GPS_PWR_OUT & GPS_PWR_PIN) {
				 	Venus_PowerOff();
			 }
			 else {
			 	Venus_PowerOn();
			 }
			 	
			UART1_CharacterPreview = NULL;
			UART1_LineReceivedHandler = NULL;

			break;
		 }
		 case '5' : {
			 	if (UART1_LineReceivedHandler == _Venus_NMEA_Debug_Handler)
			 		UART1_LineReceivedHandler = NULL;
			 	else
			 		UART1_LineReceivedHandler = _Venus_NMEA_Debug_Handler;

				break;
		 }
		 
		 case '6' : {
			 	Venus_PowerOn();
			 	Venus_StartProcessingNMEA();
			 	
				break;
		 }
		 
		 default : {
			 // error
		 }
	 }
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
