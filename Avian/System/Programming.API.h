/**
 * @file
 * @brief		@b System: Support for handling persistent storage in the information memory
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * Programming API exposes access to the information memory of the MSP430 microcontroller.
 * You can use it for persistent storage. This is used already in the firmware to support
 * persistent storage of System and Application configuration. In most cases you should
 * be fine with using the features exposed by the \ref AppConfig.h file (review the \ref APPConfig module and
 * the Persistent application settings section). 
 */

#ifndef PROGRAMMING_API_H_
#define PROGRAMMING_API_H_

#include <stdint.h>

#include "Types.h"

/**
 * @brief	Offset of the infomem area available to the user application.
 * 
 * The information memory on the MSP430F1611 has the size of 256 bytes.
 * The area below 128 byte is reserved for the system (although not entirely used).
 * Use this offset when addressing data in the informemory.
 */
#define INFOMEM_USER_SPACE		128

/**
 * @brief	Read bytes from infomemory
 * 
 * @param[out]		buffer		Pointer to buffer for read data
 * @param[in]		offset		Offset in infomemory (0-254)
 * @param[in]		size		Number of bytes to read
 */
void Programming_Infomem_Read(uint16_t offset, void* buffer, uint8_t size);

/**
 * @brief			Write bytes to infomemory
 * 
 * @param[in]		offset		Offset in infomemory (0-254)
 * @param[in]		count		Number of items following
 *								each item is a pair pointer, length
 *
 * Example:
 * @code
 * // store the configuration
 * Programming_Infomem_Write(INFOMEM_USER_SPACE, 2, &Configuration, sizeof(configuration_t), &checksum, 1);
 * @endcode
 *
 * This code stored content of two variables starting from the offset \ref INFOMEM_USER_SPACE in the information memory.
 * The first variable was the configuration of the system with the size specified by the sizeof(configuration_t),
 * the second variable was a uint8_t checksum with the size fo 1 byte. 
 * 
 * @remarks
 * The MSP430 has two consecutive blocks of infomemory. Each is 128 Bytes large.
 * The offset is the relative address starting at the beginning of the first block.
 * You can write an arbitrary number of bytes at any offse, but this 
 * function cannot write across the two blocks of infomemory.
 */
bool Programming_Infomem_Write(uint16_t offset, uint8_t count, ...);


/**
 * @brief		Describes metadata about the installed application
 */
typedef struct {					
	uint8_t		DeviceName[9];				///< name of supported hardware platform (8 characters + \0)			
	uint8_t		ExtensionBoardName[9];		///< name of supported extension board (8 characters + \0)			
	uint8_t		ApplicationName[9];			///< name of application (8 characters + \0)							
	uint8_t		ApplicationVersion[2];		///< major and minor version of application						
	uint16_t	ApplicationResetAddress;	///< address of application reset function (required for bootloader support)						
	uint32_t	CompileTime;				///< compile time in seconds since 1970-01-01 00:00:00 UTC		
	uint8_t		Checksum;					///< checksum of the header 										
} appHeader_t; 																						

/**
 * @brief		Contains metadata about the installed application (read only)
 * 
 * Contents are setup at compile time. You can access the variable to find out about the name and version of
 * a running application, the compile time is provided as well. 
 */

extern const appHeader_t ApplicationHeader;


#ifndef DOXYGEN_PUBLIC_DOC

#ifndef __COMPILE_TIME
	#define __COMPILE_TIME	0
	#warning "Define __COMPILE_TIME through compiler to current UTC time!"
#endif

#ifndef __APP_NAME
	#define __APP_NAME	"UNKNOWN "
	#error "Define __APP_NAME in the makefile"
#endif

#ifndef __APPVER_MINOR
	#define __APPVER_MINOR	0
	#error "Define __APPVER_MINOR in the makefile"
#endif

#ifndef __APPVER_MAJOR
	#define __APPVER_MINOR	0
	#error "Define __APPVER_MINOR in the makefile"
#endif

#ifndef __PLATFORM_BOARD
	#define __PLATFORM_BOARD		"MSB430H "
#endif

#ifndef __PLATFORM_EXTBOARD
	#define __PLATFORM_EXTBOARD		"NOBOARD "
#endif

#define APP(appName, appVerH, appVerL)									\
	extern void __attribute__((__noreturn__))   _reset_vector__(void);  \
	__attribute__((section(".app_header")))								\
	const appHeader_t ApplicationHeader = {								\
		__PLATFORM_BOARD,												\
		__PLATFORM_EXTBOARD,											\
		appName,														\
		{ appVerH , appVerL },											\
		(unsigned int) (&_reset_vector__),								\
		__COMPILE_TIME ,												\
		( appVerH + appVerL) % 0xFF										\
	}

		
#endif


#endif /*PROGRAMMING_API_H_*/


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
