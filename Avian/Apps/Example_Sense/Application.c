/**
 * @brief		User Application
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#include "Application.API.h"

#include "Application.h"

APP(__APP_NAME, __APPVER_MAJOR, __APPVER_MINOR);

/**
 * This function performs periodic sensing and registeres a software timer that calls the function again after 2 seconds
 * 
 * @remark
 * The "data" parameter is not used in this application.
 */
void PeriodicSensing(uint16_t data) {
	
	{
		// measure the battery level and log the results
		uint16_t battery = ADC_Devices_Battery_Read(NULL, NULL);
		
		// Save the voltage information
		if (battery > 0) {
			logentry_t entry;
			LogEngine_PrepareLogEntry(&entry);
			entry.TypeId = Battery;
			entry.Payload.Battery.Value = battery;
			
			LogEngine_Save(&entry);
		}
	}
	
	SCP_TriggerMeasurement();

	Timers_Add(PeriodicSensing, TICKS_10SECONDS, 0);
}

/**
 * This function handles data from the SCP sensor.
 * The data is logged and not displayed.
 */
void Handler_SCP() {
	uint32_t pressure;
	uint16_t temperature;
	rawtime_t rt;
	
    if (SCP_Read(NULL, NULL, &temperature, &pressure, &rt)) {
    	logentry_t scp;
    	LogEngine_PrepareLogEntry(&scp);
    	scp.TypeId = SCP;
    	
    	scp.Payload.SCP.Pressure = pressure;
    	scp.Payload.SCP.Temperature = temperature;
    	scp.Timestamp = rt;
    	
    	LogEngine_Save(&scp);
    }
}

/**
 * This funciont handles position updates from the GPS receiver
 */
void Handler_Venus_GGA(const char* nmea, const logentry_t* entry) {
	// log every gga received
	LogEngine_Save(entry);
}


void Application_Init() {
	if (Init_Success != Avian_InitDrivers()) {
		// stop the device
		Avian_CriticalLPM4(1);
	}
	
	// reduce the position update rate for this experiment
	Venus_PowerOn();
	Venus_ChangePositionUpdateRate(1);

	// register handlers for GGA
	Handler_GPS_NMEA_GGA = Handler_Venus_GGA;
	
	Venus_StartProcessingNMEA();

	
	
	
	// enable the SCP (temperature and pressure) sensor
	SCP_Enable(false);
	
	// register a handler for SCP data
	Handler_SCP_Results = Handler_SCP;
	
	
	
	
	// register a software timer: call the function "PeriodicSensing" in 10 seconds, pass 0 as data
	Timers_Add(PeriodicSensing, TICKS_10SECONDS, 0);
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
