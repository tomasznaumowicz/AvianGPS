/**
 * @file
 * @ingroup		DriverAPI
 * @brief		@b Driver: SCP pressure and temperature sensor
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 * 
 * The SCP is an absolute pressure sensor developed by VTI (http://www.vti.fi/en/support/obsolete_products/pressure_sensors). It also provides results of a temperature measurement.
 * By processing the pressure and tempreature information it's possible to compute the altitude. VTI provides
 * the required formulas.
 * 
 * This driver enables access to pressure and temperature data in various formats.
 * 
 * @note This device was discontinued by VTI.
 */

#ifndef SCP1000D01_API_H_
#define SCP1000D01_API_H_

#include <stdint.h>
#include "../System/Types.h"
#include "../System/RTC.API.h"



/**
 * @brief @b Handler: Definition of the handler for processing of results of a temperature and pressure measurement
 */
typedef void(*fp_scp_data_handler_t) (void);

/**
 * @ingroup	Handlers

 * @brief @b Handler: Access to results of a temperature and pressure measurement (see \ref Handlers for more handlers).
 * 
 * You need to process notifications about available results and load them from the sensor.
 * Any subsequent requests for a measurement will fail if previous results aren't loaded first.
 *  
 * Example:
 * @code
 * // sample data handler
 * void MyHandler() {
 *	float temperature;
 *	uint32_t pressure;
 *	uint16_t rawTemperature;
 *	uint32_t rawPressure;
 *	rawtime_t rt;
 *	time_t t;
 * 
 *	if (SCP_Read(&temperature, &pressure, &rawTemperature, &rawPressure, &rt)) {
 *		// here you can access the data and log it or transmit it over the radio
 *	}
 * }
 * 	
 * // somewhere in your application code, e.g. in the Application_Init() function
 * // ..
 * Handler_SCP_Results = MyHandler;
 * // ...
 * @endcode
 * 
 * @remarks		Only one handler can be registered.
 */
fp_scp_data_handler_t Handler_SCP_Results;


/**
 * @name		Managing the SCP device
 * 
 * @{
 */
 
/**
 * @brief 	Enables the SCP device
 * 
 * Enables the SCP sensor: brings it back from the power down mode (200-500nA) into the ultra low power mode (3.5uA).
 * 
 * @returns true on success and false on failure
 */

bool SCP_Enable(bool printProgress);

/**
 * @brief	Disables the SCP device
 * 
 * Puts the SCP sensor into the power down mode (200-500nA).
 */
void SCP_Disable();

/** @} */

/**
 * @name	Performing measurements
 * 
 * @{
 */

/**
 * @brief 	Triggers the pressure and temperature measurement and returns.
 * 
 * The measurement is executed in background by the SCP sensor and takes around 500ms.
 * The results can be read later using the \ref SCP_Read function.
 * 
 * @remarks To driver developers: the application will be notified by a ISR registered on the DRDY pin.
 * This ISR should also store the timestamp of the measurement.
 */
bool SCP_TriggerMeasurement();

/**
 * There are 4 measurement modes availble.
 * for the AvianGPS the "low power" mode was selected":
 * 
 * In low power measurement mode SCP1000 stays in standby mode until measurement
 * is externally triggered. The measurement is triggered with rising edge of TRIG
 * pin or by writing 0x0C to OPERATION register. Low power measurement can be
 * triggered after start-up and power down. If some other measurement mode is
 * activated, see section 2.2.2.1 for details of switching between measurement modes.
 * 
 * The default measurement resolution for low power mode is 17 bits.
 * Resolution can be configured to 15 bit or 17 bit mode through indirect CFG register
 * (see section 3.3).
 * 
 * This mode is the only one that can be triggered by software and doesn't execute
 * continous measurements. That's why this mode was selected.
 * 
 * returns true on success and false on failure
 */
bool SCP_MeasureBlocking(float* temperatureOut, uint32_t* pressureOut, uint16_t* rawTemperatureOut, uint32_t* rawPressureOut);

/** @} */



/**
 * @name	Working with measurement results
 * 
 * @{
 */

/**
 * @brief	Reads results from a recent pressure and temperature measurement.
 *
 * Results of a pressure and temperature measurement are available after such measurement
 * was triggered (by a call to the \ref SCP_TriggerMeasurement function). You can either
 * call this function after a delay of approx. 500ms after calling the \ref SCP_TriggerMeasurement function
 * or within an action handler \ref Handler_SCP_Results 
 * 
 * It's possible that the measurement was executed long time ago, that's why also a timestamp
 * of the measurement is stored by the software and made available here.
 * 
 * Every parameter can be set to @b NULL, this should be done in the case when that particular data
 * is not relevant to the user. Under some conditions it will reduce the time required to execute
 * this function since some conversions won't be performed.
 * 
 * Example:
 * @code
 * // sample data handler
 * void MyHandler() {
 *	float temperature;
 *	uint32_t pressure;
 *	uint16_t rawTemperature;
 *	uint32_t rawPressure;
 *	rawtime_t rt;
 *	time_t t;
 * 
 *	if (SCP_Read(&temperature, &pressure, &rawTemperature, &rawPressure, &rt)) {
 *		// here you can access the data and log it or transmit it over the radio
 *	}
 * }
 * 	
 * // somewhere in your application code, e.g. in the Application_Init() function
 * // ..
 * Handler_SCP_Results = MyHandler;
 * // ...
 * @endcode
 *  
 * @param[out]	temperatureOutOptional		Result of the measurement, converted to floating point type. Set to NULL if that type is not required.
 * @param[out]	pressureOutOptional			Result of the measurement, converted. Set to NULL if that type is not required.
 * @param[out]	rawTemperatureOutOptional	Result of the measurement, converted to floating point type. Set to NULL if that type is not required.
 * @param[out]	rawPressureOutOptional		Result of the measurement in internal raw format. Set to NULL if that type is not required.
 * @param[out]	rawPressureOutOptional		Result of the measurement in internal raw format. Set to NULL if that type is not required.
 * @param[out]	timestampOutOptional		Timestamp of the measurement. Set to NULL if that type is not required.
 * 
 * @returns true on success, false on failure
 */
bool SCP_Read(float* temperatureOutOptional, uint32_t* pressureOutOptional, uint16_t* rawTemperatureOutOptional, uint32_t* rawPressureOutOptional, rawtime_t* timestampOutOptional);


/**
 * @brief		Converts results of the pressure and temperature measurement
 * 
 * Conversion should only be processed when the data needs to be displayed to the user. In most cases, e.g. logging, the 
 * raw pressure and raw temperature data should be processed.
 * 
 * @param[out]	temperatureOutOptional	Result of the conversion as a floating point type. Set to NULL if that type is not required.
 * @param[out]	pressureOutOptional		Result of the conversion. Set to NULL if that type is not required.
 * @param[in]	rawTemperatureIn		Result of the measurement provided by the \ref SCP_Read function.
 * @param[in]	rawPressureIn			Result of the measurement provided by the \ref SCP_Read function.
 */
void SCP_Convert(float* temperatureOutOptional, uint32_t* pressureOutOptional, uint16_t rawTemperatureIn, uint32_t rawPressureIn);

/** @} */

#endif /*SCP1000D01_API_H_*/

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
