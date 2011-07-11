/**
 * @file		RTC.API.h
 */

#ifndef DOXYGEN_PUBLIC_DOC

#include <msp430x16x.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include "Commands.h"
#include "Timers.h"

#include "RTC.h"
#include "RTC.API.h"


#define RTC_START_OF_EPOCH 			1262304000L		// RTC starts with 946684800 seconds == 01.01.2010 00:00:00
#define RTC_END_OF_EPOCH 		 	1577836800UL	// RTC valid time ends with 1577836800 seconds == 01.01.2020 00:00:00

/**
 * __rtc_state
 * NEVER DIRECTLY ACCESS THIS VARIABLE. USE THE API LISTED IN TIMERS.H
 * This variable is used internally by the RTC and consists of a 32 bit second counter and a
 * 16 bit millisecond counter.
 *
 * __rtc_state gets incremented by the Timer_A1 Interrupt in Timers.c every 8 seconds.
 * IT DOESN'T REPRESENT THE _CURRENT_ REAL TIME
 * ==> additional operations are required to get the current time ==> USE THE API
 * For that reason this variable isn't declared public in the header file.
 */
extern volatile rawtime_t __rtc_state;

// Days per Month
#define JAN   (31)
#define FEB   (28)
#define MAR   (31)
#define APR   (30)
#define MAY   (31)
#define JUN   (30)
#define JUL   (31)
#define AUG   (31)
#define SEP   (30)
#define OCT   (31)
#define NOV   (30)
#define DEC   (31)

static uint8_t days_per_month[12] = {JAN, FEB, MAR, APR, MAY, JUN, JUL, AUG, SEP, OCT, NOV, DEC};

void RTC_Init() {
	/**
	 * the __rtc_state is carried on across resets of the device.
	 * this means, it's difficult to tell, whether the state is valid as there might be random
	 * numbers stored in this area.
	 * a simple check will be applied, only time > year 2000 and < 2020 will be accepted as valid
	 * if a value outside of this boundaries is detected, the device will set it's date to the year 2000
	 */
	if (__rtc_state.seconds < RTC_START_OF_EPOCH || __rtc_state.seconds > RTC_END_OF_EPOCH)
		__rtc_state.seconds = RTC_START_OF_EPOCH;
	
	__rtc_state.miliseconds = 0;
}


void inline RTC_ProcessTAI() {
	//extern volatile rawtime_t __rtc_state;		// access the __rtc_state variable
	__rtc_state.seconds += 8;					// Update the __rtc_state, increment the seconds counter by 8 seconds since the overflow of TimerA happens every 8 seconds
}

long dysize(y)
{
	if((y%4) == 0)
		return(366);
	return(365);
}


#endif /* DOXYGEN_PUBLIC_DOC */









void RTC_GetRawTime(rawtime_t* time) {
	uint16_t _TAR_Copy;
	
	// critial section starts here
		//		the _SystemTime and current TAR have to be added
		//		in the meantime the TimerA could overflow and the SAR adds 8 seconds
		//		to the _SystemTime and the TAR resets --> the addition will give false results if this happenes
		//		==> Interrupts HAVE to be disabled during this operation.
		
		_DINT();											// disable interrupts
		
		time->seconds = __rtc_state.seconds;				// copy the __rtc_state
		time->miliseconds = __rtc_state.miliseconds;
		
		_TAR_Copy = COPY_CURRENT_TAR;						// copy the current value of TAR
		
		_EINT();											// enable interrupts
	// end of the critical section
	
	// general idea:
	// time->miliseconds += (_TAR_Copy mod TICKS_PER_SECOND) % MILISECONDS_PER_TICK
	// we would use 8 ticks per milisecond and not 8.192 per milisecond.
	// we would not be accurate close to end of a second because we would count quicker
	// 		in the 976ms we would assume it is already 1000 ms and...
	// 		in the (976,1000)ms we have results > 1000 ...
	// soulution: we reduce the accuracy to 1/8th of a second == 125 ms and will round to the lowest
	// 125 ms == 1024 ticks
	// 1024 dec == 100 0000 0000 bin == 0x0400 hex
	//
	// (_TAR_Copy & 0x1FFF) == ticks count mod 8192
	// ((_TAR_Copy & 0x1FFF) >> 10) == (ticks count mod 8192) / 1024
	// this tells me how many 1024 ticks we had in this second
	// ((_TAR_Copy & 0x1FFF) >> 10) * 125 == ((ticks count mod 8192) / 1024) * 125
	// this tells me the number of ms with the resoultion of 125 ms

	// version with 125 ms accuracy
	#if RTC_RESOLUTION_125_MILISECONDS
		time->miliseconds += ((_TAR_Copy & 0x1FFF) >> 10) * 125;
	#endif
		
	// version with highest accuracy (8.192 ms), slow
	#if RTC_RESOLUTION_8_MILISECONDS
		time->miliseconds += ((_TAR_Copy & 0x1FFF) / TICKS_PER_MILISECOND);
	#endif

	// version with highest accuracy and speed but not in real ms anymore
	#if RTC_RESOLUTION_IN_TICKS
		time->miliseconds += (_TAR_Copy & 0x1FFF);
	#endif
		
	// check for overflows only when miliseconds are used.
#if RTC_RESOLUTION_125_MILISECONDS || RTC_RESOLUTION_8_MILISECONDS
	while (time->miliseconds >= 1000) {
		time->miliseconds -= 1000;
		time->seconds++;
	}
#endif

#if RTC_RESOLUTION_IN_TICKS
	while (time->miliseconds >= 0xFFFF) {
		time->miliseconds -= 0xFFFF;
		time->seconds++;
	}
#endif

	time->seconds += (_TAR_Copy >> 13);					// quicker
}

void RTC_SetRawTime(const rawtime_t* time) {
	uint16_t _TAR_Copy;	
	// critial section starts here
		//		the _SystemTime has to be modified
		//		in the meantime the TimerA could overflow and the SAR adds 8 seconds
		//		to the _SystemTime
		//		==> Interrupts HAVE to be disabled during this operation.
		//
		// the _SystemTime doesn't store the CURRENT time, the current time is a result of the addition of the TAR
		// the TAR stores up to 8 seconds:
		// use the current value of TAR and store the difference as _SystemTime
		// this is safe for _SystemTime.seconds but not safe for _SystemTime.miliseconds:
		//   the number of miliseconds is used for computations during GetRawTime
		//   miliseconds are converted to seconds if an overflow of miliseconds is detected
		//   this doesn't happend for _SystemTime.seconds
		//   since we work with unsigned numbers:
		//     small number of miliseconds - TAR(miliseconds) < 0 ==> overflow with a value around 65000
		//     this could be solved by logics but is not required.
		//     number of miliseconds will be ignored
		
		_DINT();											// disable interrupts
		
		_TAR_Copy = COPY_CURRENT_TAR;						// copy the current value of TAR
		
		__rtc_state.seconds = time->seconds - (_TAR_Copy >> 13);	// copies the number of seconds stored in TAR
		__rtc_state.miliseconds = 0;

		_EINT();											// enable interrupts
	// end of the critical section
}

void RTC_ConvertToTime(const rawtime_t* from, time_t* to) {
		int temp1, temp2;
		long secsInLastDay, day;
		
		long totalSeconds = (from->seconds);

		secsInLastDay = totalSeconds % 86400;
		day = totalSeconds / 86400;

		to->second = secsInLastDay % 60;
		temp2 = secsInLastDay / 60;
		to->minute = temp2 % 60;
		temp2 = temp2 / 60;
		to->hour= temp2;

		if (day>=0) {
			for(temp2=70; day >= dysize(temp2); temp2++)
				day -= dysize(temp2);
		}
		else {
			for (temp2=70; day<0; temp2--)
				day += dysize(temp2-1);
		}

		to->year = temp2 - 70;
		temp1 = day;
		
		if (dysize(temp2)==366)
			days_per_month[1] = 29;
		
		for(temp2=0; temp1 >= days_per_month[temp2]; temp2++)
			temp1 -= days_per_month[temp2];
		
		days_per_month[1] = 28;
		
		to->day = temp1+1;
		to->month = temp2 + 1;

		to->ms = from->miliseconds;
}

void RTC_ConverToRawTime(const time_t* from, rawtime_t* to) {
	  uint32_t seconds;
	  uint16_t feb;
	  uint16_t i;
	  uint32_t mult;
	  
	  //84400 == 24 * 60 * 60 == seconds a day
	  seconds = (86400*365) * (from->year); 	//add years to seconds

	  seconds += (86400*((from->year+1)/4));    //add leap year days

	  feb = FEB;                           		//choose feb 28 or 29
	  if(((from->year+2) % 4) == 0) feb++;
	  
	  switch(from->month) {                  	// add month and day
	    case  1:
	    	mult = 0;
	    	break;
	    case  2:
	    	mult = JAN;
	    	break;
	    default:
	    	mult = JAN + feb;
	    	for( i = 2; i < (from->month - 1); i++ )
	    		mult += days_per_month[i];
	  }
	  mult += from->day - 1;
	  seconds += 86400 * mult;
	  // add hour minute second
	  seconds += ((((uint32_t)from->hour * 60) + from->minute) * 60) + from->second;
	  // write dest
	  to->seconds = seconds;
	  to->miliseconds = from->ms;
}


void RTC_SetTime(const time_t* time) {
	rawtime_t rawtime;
	RTC_ConverToRawTime(time, &rawtime);
	RTC_SetRawTime(&rawtime);
}

void RTC_FromString(const char* input, time_t* time) {
	char* pointer = (char*) input;
	
	time->hour = strtoul(pointer, &pointer, 10);
	pointer++;
	time->minute = strtoul(pointer, &pointer, 10);
	pointer++;
	time->second = strtoul(pointer, &pointer, 10);
	
	if(*pointer == '.') {							// detected format with miliseconds "hh-mm-ss.mss dd-mm-yyyy"
		pointer++;
		time->ms = strtoul(pointer, &pointer, 10);
	}
	pointer++;
	
	time->day= strtoul(pointer, &pointer, 10);
	pointer++;
	time->month = strtoul(pointer, &pointer, 10);
	pointer++;
	
	uint16_t year = strtoul(pointer, NULL, 10);
	//we store only the years after 1970 and the variable is a byte
	if (year < 1970)
		year = 1970;
	
	year -= 1970;
	
	if (year > 255)						// 0xFF ist the maximum
		year = 255;

	time->year = year;
	
	time->ms = 0;
}

void RTC_GetTime(time_t* time) {
	rawtime_t rawtime;

	RTC_GetRawTime(&rawtime);
	RTC_ConvertToTime(&rawtime, time);
}

void RTC_ToString(const time_t* time, char* buffer) {
#if RTC_RESOLUTION_125_MILISECONDS || RTC_RESOLUTION_8_MILISECONDS
	snprintf(
			buffer, RTC_TIME_STRING_LENGTH,
			"%.2u:%.2u:%.2u.%.3u %.2u-%.2u-%.4u",
			time->hour,
			time->minute,
			time->second,
			time->ms,
			time->day,
			time->month,
			time->year + 1970
			);
#elif  RTC_RESOLUTION_IN_TICKS
	snprintf(
			buffer, RTC_TIME_STRING_LENGTH,
			"%.2u:%.2u:%.2u.%.4u %.2u-%.2u-%.4u",
			time->hour,
			time->minute,
			time->second,
			time->ms,
			time->day,
			time->month,
			time->year + 1970
			);
#else
	snprintf(
			buffer, RTC_TIME_STRING_LENGTH,
			"%.2u:%.2u:%.2u %.2u-%.2u-%.4u",
			time->hour,
			time->minute,
			time->second,
			time->day,
			time->month,
			time->year + 1970
			);
#endif
}

bool RTC_ValidTime(const rawtime_t* time) {
	return ((time->seconds > RTC_START_OF_EPOCH) && (time->seconds < RTC_END_OF_EPOCH));
}


/**
 * @ingroup Commands
 * @brief	@b time Manage local time
 * 
 * This commands allows to query the current local time of the device. It can be also used to set the time.
 * Example:
 * @code
 * time 12:30:00 14.02.2011
 * @endcode
 */
COMMAND(time, "Manage local time.", cmd_args)
{
	time_t time;

	// check whether the time change was requested
	if (cmd_args->ArgumentCount > 0) {
		RTC_FromString(cmd_args->ArgumentString, &time);
		RTC_SetTime(&time);
	} else {
		RTC_GetTime(&time);
	}
	
	char buffer[RTC_TIME_STRING_LENGTH];
	RTC_ToString(&time, buffer);
	snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength, "%s", buffer);
}

/**
 * @ingroup Commands
 * @brief	@b uptime Provides the uptime of the device since the last reset
 * 
 * Allows to query the uptime of the device since the last reset. The data is converted to a human readable format where number of days,
 * hours, minutes and seconds are provided.
 */
COMMAND(uptime, "Uptime since last reset", cmd_args) {
	rawtime_t rt; time_t bt;
	char buffer[RTC_TIME_STRING_LENGTH];
	
	rt.miliseconds = 0;
	rt.seconds = (*TICKS_SINCE_POR() >> 13);  //10000000000000
	
	RTC_ConvertToTime(&rt, &bt);
	RTC_ToString(&bt, buffer);			// the string contains also the date, which is not required
	buffer[8] = '\0';					// get ride of the date
	
	snprintf(
			cmd_args->ResponseString,
			cmd_args->ResponseStringMaxLength,
			"%.2ud %s",
			bt.day - 1,
			buffer
		);
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
