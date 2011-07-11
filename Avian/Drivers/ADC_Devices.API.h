/**
 * @file
 * @ingroup		DriverAPI
 * @brief		@b Driver: Support for ADC
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * Enables access to data from devices attached to the Analog-Digital-Converter (ADC).
 * 
 * Currently following measurements are supported:
 *  - Measurement of the battery level
 *  - Measurement of the light level
 */

#ifndef ADC_DEVICES_API_H_
#define ADC_DEVICES_API_H_

#include <stdint.h>

#include "Device.h"

/**
 * @brief	Configures the ADC device.
 */
void ADC_Devices_Init();

/**
 * @brief	Defines the length of string representation of the battery level. You will need this value when requesting a string representation of the battery level.
 */
#define BATTERY_READ_STRING_LENGTH		8

/**
 * @brief	Enables the light sensor.
 */
#define LIGHT_ENABLE		{SET(LIGHT_COLL_OUT, LIGHT_COLL_PIN);}

/**
 * @brief	Disables the light sensor.
 */
#define LIGHT_DISABLE		{CLEAR(LIGHT_COLL_OUT, LIGHT_COLL_PIN);}


/**
 * @name		Battery Level Measurements
 * @{
 */

/**
 * @brief		Perfoms measurement of the battery level
 * 
 * Example:
 * @code
 * // prepare a buffer for the string representation of the battery level
 * char asString[BATTERY_READ_STRING_LENGTH];
 * // the GPS sensor must be enabled
 * Venus_PowerOn();
 * // ignoring the raw and floating point format
 * ADC_Devices_Battery_Read(NULL, asString);
 * // the GPS sensor may be disabled now
 * Venus_PowerOff();
 * // print the results
 * LOG("Battery Level: %s", asString);
 * @endcode
 * 
 * @param[out]	asFloatOutOptional	Result of the measurement as a floating point type. Set to NULL if that type is not required.
 * @param[out]	asStringOutOptional	Result of the measurement in a string representation. Set to NULL if that type is not required.
 * @returns							Result of the measurement in an internal raw format. Use \ref ADC_Devices_Battery_Convert to convert.
 * 
 * @remarks Battery voltage measurement is only possble when the GPS sensor is enabled.
 */
uint16_t ADC_Devices_Battery_Read(float* asFloatOutOptional, char* asStringOutOptional);

/**
 * @brief		Converts results of a battery level measurement
 *
 * Benchmarks:
 * 	- asStringOptional: 29 ACLK ticks
 * 	- asFlowatOptional: 11 ACLK ticks
 * 	- with both: 29 ACLK ticks
 * 	- with none: 5 ACLK ticks
 *
 * @param[in]	voltage_raw			Result of the measurement provided by the \ref ADC_Devices_Battery_Read function.
 * @param[out]	asFloatOutOptional	Result of the conversion as a floating point type. Set to NULL if that type is not required.
 * @param[out]	asStringOutOptional	Result of the conversion in a string representation. Set to NULL if that type is not required.
 */
void ADC_Devices_Battery_Convert(uint16_t voltage_raw, float* asFloatOutOptional, char* asStringOutOptional);

/** @} */

/**
 * @name		Light Level Measurements
 * @{
 */


/**
 * @brief		Perfoms measurement of the light level
 * 
 * @returns		Result of the measurement in an internal raw format [from 0 for "dark" to 4096 for "bright"]
 */
uint16_t ADC_Devices_Light_Read();

/** @} */

#endif /*ADC_DEVICES_API_H_*/

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
