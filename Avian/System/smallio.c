#include <stdlib.h>
#include <stdarg.h>
#include <sys/inttypes.h>
#include <sys/cdefs.h>
#include <string.h>

int small_sscanf(const char * input, const char * format, ...) {
	int readEntries = 0;
	
	//prepare arguments
	va_list args;
	va_start(args, format);	
	
	//iterate over the format, and confirm whether the input matches the format
	//break condition: one of the strings is empty or we reached the end of the string
	while (*input != '\0' && *input != '\r' && *input != '\n' && *format != '\0') {
		if (*format == '%') { //one of following format specifiers is allowed %lu, %u and %s
			//read data properly, write to selected argument and update selection, update pointer in input
			
			format++;								// skip the '%' and go to the format specifier
			switch (*format) {
				case 'l': {							// processing the %lu
					format++;						// skip the 'l', we assume there was an 'l'
					if (*format == 'u') {			// make sure, the next one is 'u'
						uint32_t* placeholder =  (uint32_t*) (va_arg(args, uint32_t*));
						
						if (placeholder != NULL) {
							uint32_t v = atol(input);	// read the data
							*placeholder = v;			// assign the data
						}
					
						while (*input >= '0' && *input <= '9')	// skip the data read in the input
							input++;
						
						readEntries++;
					}
					
					break;
				}
				case '8': {							// processing the %8u
					format++;						// skip the '9', we assume there was an '9'
					if (*format == 'u') {			// make sure, the next one is 'u'
						uint8_t* placeholder =  (uint8_t*) (va_arg(args, uint8_t*));
						
						if (placeholder != NULL) {
							uint8_t v = atoi(input);	// read the data
							*placeholder = v;			// assign the data
						}
					
						while (*input >= '0' && *input <= '9')	// skip the data read in the input
							input++;
						
						readEntries++;
					}
					
					break;
				}
				case 'u': {							// processing the %u
					uint16_t* placeholder =  (uint16_t*) (va_arg(args, uint16_t*));
					
					if (placeholder != NULL) {
						uint16_t v16 = atoi(input);		// read the data
						*placeholder = v16;				// assign the data
					}

					while (*input >= '0' && *input <= '9')	// skip the data read in the input
						input++;
					
					readEntries++;

					break;
				}
				case 'c': {							// processing the %c
					
					char* placeholder =  (char*) (va_arg(args, char*));
					
					if (placeholder != NULL)
						*placeholder = *input;		// assign the data
	
					input++;	// skip the data read in the input
					
					readEntries++;

					break;
				}
				case '.' : {						// expecting the %.XXs where XX specifies the maximal length of the string. string ends with ',' or '\0'
					format++;						// skip the '.'

					uint8_t length = atoi(format);	// read the string length

					while (*format != '\0' && *format != 's')	// skip the string length in the format input 
						format++;

					// expecting 's'
					if (*format == 's') { //now, the format should point to 's' character, and the variable length should store the maximum length of the target array for the string to be read
					
						char* p = va_arg(args, char*);	// prepare the target pointer
						
						if (p != NULL)
							memset(p, 0, length);			// clear the target string buffer
						
						//copy the input string 
						while (length > 0) {
							if (*input == '\0' || *input == '\r' || *input == '\n' || *input == ',')
								break;		// break the while loop when the abort condition is met for the input string (it might the end of the input string or the end of the field - terminated by ',')

							if (p != NULL) {
								*p = *input; 	// copy the character
								p++;
							}
							input++; 	// increment target and source pointers
							length--;		// reduce the length counter so that the procdure aborts when the end of the target buffer is reached
						}
						
						readEntries++;
					}

					break;
				}
				default : {					//the format string doesn't match the expected format  
					va_end(args);
					return readEntries;	// unsupported format: break the loop here, return.
				}
			}
			
			format++;						// skip to the next character in the format string
			
		}
		else 										//there is a regular character in the format string. compare with input. break on missmatch
			if (*input == *format) {
				input++;
				format++;
			} else {
				break;								// there was a missmatch. break the while loop
			}
	} //while
	
	va_end(args);
	
	return readEntries;
}




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
