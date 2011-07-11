/**
 * @file		ADC_Devices.API.h
 * @ingroup		DriverAPI
 */

#include <stdint.h>
#include <msp430x16x.h>
#include <sys/cdefs.h>
#include <stdio.h>
#include <stdlib.h>

#include "Device.h"

#include "../System/BitOperations.h"

#include "../System/Commands.API.h"
#include "../System/Timers.API.h"

#include "ADC_Devices.h"
#include "ADC_Devices.API.h"


void ADC_Devices_Init() {
	//configure the ADC
	ADC12CTL0 = ADC12ON;
	ADC12CTL1 = ADC12SSEL_1 | SHP | CONSEQ_0; //ACLK as clock source and Pulse Sample Mode (SHP)
	
	// for the battery measurement
	ADC12MCTL1 = INCH_1 | EOS; 	// select P6.1 (A1) as input channel and mark this MCTL as end of sequence. VRef+ = VCC, VRef- = VSS
	
	// for the light level measurement
	ADC12MCTL0 = INCH_7 | EOS; 	// select P6.7 (A7) as input channel and mark this MCTL as end of sequence
}


void ADC_Devices_Battery_Convert(uint16_t voltage_raw, float* asFloatOutOptional, char* asStringOutOptional) {
	float voltage;

	if (asFloatOutOptional != NULL || asStringOutOptional != NULL) {
		// make this computation only if requested or requred for the optional string representation
		voltage = voltage_raw;
		voltage = (voltage * 6.6) / 4095.0;
		
		if (asFloatOutOptional != NULL)
			*asFloatOutOptional = voltage;
	}
	
	if (asStringOutOptional != NULL) {
		// produce optional string representation
		int high, low;
		
		high = voltage;
		low = abs((voltage - high) * 100);
		
		snprintf(asStringOutOptional, BATTERY_READ_STRING_LENGTH,
				"%d.%02dV", high, low);
	}
}

uint16_t ADC_Devices_Battery_Read(float* asFloatOutOptional, char* asStringOutOptional) {
	uint16_t voltage_raw = 0;

	// perform the measurement only if the powersupply for the GPS is enabled as this is how the hardware is designed
	if (GPS_PWR_OUT & GPS_PWR_PIN) {
		// configure the ADC to start the sequence at MCTL1
		ADC12CTL1 &= ~CSTARTADD_15;			//sets CSSTARTADDto MEM0, basicallly sets CSTARTADD_0 which is 0000
		ADC12CTL1 |= CSTARTADD_1;
	
		ADC12CTL0 |= ADC12SC | ENC;
		ADC12CTL0 &= ~ADC12SC;
	
		while (ADC12CTL1 & ADC12BUSY) {
			_NOP();							//todo: sleep here + allow ADC to raise interrupts when done
		}
				
		ADC12CTL0 &= ~ENC;
			
		voltage_raw = ADC12MEM1;
	}

	ADC_Devices_Battery_Convert(voltage_raw, asFloatOutOptional, asStringOutOptional);
	
	return voltage_raw;
}

/**
 * @ingroup Commands
 * @brief	@b batt Measures the battery voltage
 */
COMMAND(batt, "Battery voltage (valid when GPS is enabled)", cmd_args) {
	char asString[BATTERY_READ_STRING_LENGTH];
	
	uint16_t t1, t2;
	t1 = COPY_CURRENT_TAR;
	
	uint16_t raw = ADC_Devices_Battery_Read(NULL, asString);
	
	t2 = COPY_CURRENT_TAR;
	
	snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength,
			"%s (raw: %u)(%u ticks)", asString, raw, t2-t1);
}

uint16_t ADC_Devices_Light_Read() {
	uint16_t result;
	
			ADC12CTL1 &= ~CSTARTADD_15;			//sets CSSTARTADDto MEM0, basicallly sets CSTARTADD_0 which is 0000

			ADC12CTL0 |= ADC12SC | ENC;
			ADC12CTL0 &= ~ADC12SC;

			while (ADC12CTL1 & ADC12BUSY) {
				_NOP();							//todo: sleep here + allow ADC to raise interrupts when done
			}
			
			ADC12CTL0 &= ~ENC;
			
			result = ADC12MEM0;	
	
	return result;
}

COMMAND(light, "Advanced: for firmware developers", cmd_args) {
	while (true) {
		uint16_t light = ADC_Devices_Light_Read();
		printf("%u\r\n", light);
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
