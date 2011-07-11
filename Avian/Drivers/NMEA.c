/**
 * @brief		Driver: Processing NMEA sentences
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#include <stdlib.h>
#include <string.h>
#include <msp430x16x.h>

#include "../System/Types.h"
#include "../System/smallio.h"

#include "LogEngine.API.h"
#include "NMEA.h"

/**
 * Converts hexadecimal number formatted as two characters to byte
 * Increments the input pointer by 2
 */
uint8_t NMEA_hex2byte(uint8_t** cs) {
	register unsigned int a;
	register unsigned int b;
	
	a = (*(*cs)++) - '0';
	if( a > 9 )
		a -= 7;	
	b = (*(*cs)++) - '0';
	if( b > 9 )
		b -= 7;
		
	a<<=1; a<<=1; a<<=1; a<<=1;		// smaller by 2 bytes from "a <<= 4;"...
	
	b |= a;
	return( b );
}

bool NMEA_ValidSentence(const char* input) {
	uint8_t checksum = 0x00;
	uint8_t index;
	
	//iterate over each character in the input string until the end of string is reached
	//the loop has an additional break condition built in
	for(index = 0; input[index] != 0x00 && index < 120; index++) {
		if (input[index] == '$') {
			// the start condition was met, but the '$' should be skipped
			// and then, all characters until '*' should be processed
			continue;
		}
		if (input[index] == '*') {
			// the end condition was met, break the loop
			break;
		}
		
		checksum ^= input[index];
	}

	// now, it's possible, the checksum was computed and the loop terminated at the '*'
	// this needs to be confirmed now:
	
	//if (index == 120) {
	//	LOG("EOL");
	//}
	
	if (input[index] != '*') {
	//	LOG("NO *");
		return false; // the sentece was not valid the loop exited earlier, or no '*' was found and the input string ended.
	}
	
	// now, the success path: the next two characters after the '*' (index+1 and index+2) represent the checksum

	// are they set?
	if (input[index + 1] == 0x00 || input[index + 2] == 0x00) {
	//	LOG("NO CS");
		return false;
	}
	
	// success path, convert and compare
	char* p = (char*) &(input[index + 1]);
	uint8_t nmeaChecksum = NMEA_hex2byte((uint8_t**) &p);
	
	return (checksum == nmeaChecksum);
}

bool NMEA_Read_GPGGA(const char* input, logentry_t* entry) {
	if (input == NULL || entry == NULL)
		return false;
	
	LogEngine_PrepareLogEntry(entry);
	entry->TypeId = GPGGA;
	
	/*
	 * $GPGGA, <time>, <latitude>, N, <longitude>, E, <quality>, <numsat>, <hdop>, <alt>, M, <geoalt>, M,,0000*<checksum>
	 * <time>,<lat>,<long>,<numsat>,<hdop>,<alt>
	 * 
	 * example: 
	 * $GPGGA,162808.806,5227.3506,N,01317.8474,E,1,05,2.7,0.0,M,41.9,M,,0000*52
	 *        tiiiiiimee,laaaaaaaaat,loooooooooon,q,ns,hdo,alt,M,geoa,M....
	 */

	/**
	 * time is encoded in the GGA string as follows: 162808.806 --> HHMMSS.MS
	 * fist, both tokens need to be converted in numbers
	 * then, both numbers will be combined into one uint32_t variable:
	 * max value possible 235959999 is less than the max uint32_t
	 * and since storing HHMMSS required the uint32_t anyway. they can be combined as follows
	 * 
	 * encoded_timestamp = HHMMSSasNumber * 1000 + MSasNumber
	 */
	
	uint32_t timestampHHMMSS; //HHMMSS
	uint16_t timestampMiliseconds; // milliseconds
	
	/**
	 * N/S and E/W are used to determine whether the latitudeIntegral/longitudeIntegral should be positive or negative
	 * it's not requireed to store the N/S/E/W character
	 * this saves 2 bytes
	 */
	uint8_t latitude;
	uint8_t longitude;
	
	/**
	 * hdop is encoded similar to the timestamp:
	 * encoded_hdop = hdopintegral * 10 + hdopfractional
	 */
	
	uint16_t hdop_integral;
	uint16_t hdop_fractional;
	
	//											   timeee,latttttt,longiiit,fix,sat,hdoooop,allllt
	int fieldsFilled = small_sscanf(input, "$GPGGA,%lu.%u,%u.%u,%c,%u.%u,%c,%8u,%8u,%u.%u,%u.%8u,",
			&timestampHHMMSS, //time max: 235959 uint32: 4294967295
			&timestampMiliseconds, // ms max 999				 0235959999 --> time could be completely in uint32_t with MS. but: diffs will be big.. will they?
			&(entry->Payload.GPGGA.LatitudeIntegral), // max 9060
			&(entry->Payload.GPGGA.LatitudeFractional), // max ?
			&latitude, //N or S N or S should be a sign at LatHigh
			&(entry->Payload.GPGGA.LongitudeIntegral), // max 18060
			&(entry->Payload.GPGGA.LongitudeFractional), // max ?
			&longitude, // E or W E or W should be a sign within LongHigh
			&(entry->Payload.GPGGA.FixQuality), // 0..8
			&(entry->Payload.GPGGA.TrackedSatellites), // 000..99
			&hdop_integral, // should be uint16_t
			&hdop_fractional, // 8 --> merge into one uint16_t (multiply by 10 and add hdoplow)
			&(entry->Payload.GPGGA.AltitudeIntegral), // 16 !! could be negative! improve the code so it does not crash when a '-' appears
			&(entry->Payload.GPGGA.AltitudeFractional) // 8 --> combine into one int16? max height +/-32xx.9 or if <0 ignored, then uint16, 65xx.x 
		);

	// managed to fill data entry
	// 14 == the number of fileds extracted from the GGA sentence
	// taken from the list of parameters of the small_sscanf function above.
	if (fieldsFilled == 14) {
		
		// is there a fix? DISABLED for DEV
		if (entry->Payload.GPGGA.FixQuality == 0) {
			return false; // no fix
		}
		
		// encode data:
		entry->Payload.GPGGA.EncodedTimestamp = timestampHHMMSS * 1000 + timestampMiliseconds;
		
		entry->Payload.GPGGA.EncodedHDop = hdop_integral * 10 + hdop_fractional;
		
		if (latitude == 'S')
			entry->Payload.GPGGA.LatitudeIntegral *= -1;
		
		if (longitude== 'W')
			entry->Payload.GPGGA.LongitudeIntegral *= -1;
		
		return true;
	}
	else {
		// fields were not filled
		return false;
	}
}

/**
 * Input in the format:
 * 	time "hhmmss.msms"
 *  date "ddmmyy"
 * 
 * please remember: the aviangps firmware uses ticks as milliseconds. this ensures high time resolution with the smallest
 * computation cost. this function takes it into account and converts milisecods to ticks
 * 
 * 1000 ms = 8192 ticks
 * 1 ms = approx 8 ticks
 * 
 * the conversion for the data will be realized as follws
 * 
 * the ascii code from the input string will be reduced by 48 because '0' in the ascii table is represented as decimal 48.
 * the '1' is 49, the '2' is 50 and so on. we assume, the data is in correct format.
 * then, the first digit from the hh mm ss dd mm and yy will be multiplied with 10 and the second digit will be added
 * 
 * 2 ticks at 4MHz 
 */
void NMEA_TimeDateFromNMEAString(const char* timeString, const char* dateString, time_t* time) {
	// process the time
	time->hour		= (timeString[0] - 48) * 10 + (timeString[1] - 48);
	time->minute	= (timeString[2] - 48) * 10 + (timeString[3] - 48);
	time->second	= (timeString[4] - 48) * 10 + (timeString[5] - 48);

	uint32_t ms		= atoi(&(timeString[7]));
	
	// convert the input to ticks
	ms = ms * 8192;
	ms = ms / 1000;
	
	time->ms		= (uint16_t) ms; 
	
	// process the date
	time->day		= (dateString[0] - 48) * 10 + (dateString[1] - 48);
	time->month 	= (dateString[2] - 48) * 10 + (dateString[3] - 48);
	
	//time_t stores only the years after 1970 and the variable is a byte
	uint16_t year 	= (dateString[4] - 48) * 10 + (dateString[5] - 48);
	year = 2000 + year; // this was from the GPS

	year -= 1970;		// now compute the difference
	if (year > 255)		// 0xFF ist the maximum
		year = 255;

	time->year = year;
}

bool NMEA_Read_GPRMC(const char* input, time_t* timeOptional, rawtime_t* rawtimeOptional) {
	char rmc_time[12];
	char rmc_date[12];
	memset(rmc_time, 0, sizeof(rmc_time));
	memset(rmc_date, 0, sizeof(rmc_date));

	/*
	 * example: 
	 * $GPRMC,104636.701,A,5157.3834,N,01443.3920,E,001.9,248.6,170810,,,A*61
	 * 		 ,%s       ,%c,%s      ,%c,%s       ,%c,%s   ,%s   ,%s,
	 *       ,tiiiiiimee,......................................,dateee,......
	 * 
	 * Important update: the RMC sentence contains information about the state of the RMC information.
	 * 	It can be Active (A) or Void (V). Only the Active (A) RMC sentence should be processed because
	 * 	it's generated with valid data from the sattelite. Void (V) RMC sentence should be ignored.
	 * 
	 * 	This rule can be implemented directly in the sscanf format string since (A) can be treated as a constant.
	 */
	
	/**
	 * now, the support for field skipping will be used. the field is skipped by the small_sscanf function
	 * when the target is specified as NULL. correct formats need to be specified in the format string though.
	 * 
	 * in order to skip data like ,22222.333, %.20s will be used as format.
	 * this means, the data will be a string, with maximal length of 20.
	 * small_scanf terminates to fill the target, when the maximal length is reached or a ',' is found in the input string.
	 * when the target is NULL, no data will be read, only skipped, the same termination conditions are used.
	 */
	int fieldsFilled = small_sscanf(input, "$GPRMC,%.12s,A,%.20s,%c,%.20s,%c,%.20s,%.20s,%.12s,",
			rmc_time, // read the time
			NULL,
			NULL,
			NULL,
			NULL,
			NULL,
			NULL,
			rmc_date // read the date
		);

	// managed to fill data entry
	// 8 == the number of fileds extracted from the RMC sentence
	// taken from the list of parameters of the small_sscanf function above.
	if (fieldsFilled == 8) {
		// and now, try to parse
		time_t gpsTime;
		NMEA_TimeDateFromNMEAString(rmc_time, rmc_date, &gpsTime);

		if (timeOptional != NULL)
			memcpy(timeOptional, &gpsTime, sizeof(time_t));
		
		if (rawtimeOptional != NULL) {
			RTC_ConverToRawTime(&gpsTime, rawtimeOptional);
		}
		
		return true;
		
	} // if managed to read the RMC sentence
	
	return false;
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
