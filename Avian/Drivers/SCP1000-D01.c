/**
 * @file		SCP1000-D01.API.h
 * @ingroup		DriverAPI
 * 
 * @remarks To driver devlopers: SCL frequency max is 500kHz.
 */

#include <msp430x16x.h>
#include <string.h>

#include "Device.h"

#include "../System/BitOperations.h"
#include "../System/Logging.h"
#include "../System/Interrupts.h"

#include "../System/System.API.h"
#include "../System/USART.API.h"
#include "../System/RTC.API.h"
#include "../System/Timers.API.h"
#include "../System/Commands.API.h"

#include "LogEngine.h"
#include "SCP1000-D01.h"
#include "SCP1000-D01.API.h"


rawtime_t SCP_MeasurementTimestamp;

/**
 * List of SCP1000 direct and indirect access registers and EEPROM registers.
 */
#define REVID		0x00 //ASIC revision number R Direct 8bits
#define DATAWR		0x01 // Indirect register access data RW Direct 8bits
#define ADDPTR		0x02 //Indirect register access pointer RW Direct 8bits
#define OPERATION	0x03 //Operation register RW Direct 8bits
#define OPSTATUS	0x04 //Operation status R Direct 8bits
#define RSTR		0x06 //ASIC software reset W Direct 8bits
#define STATUS		0x07 //ASIC top-level status R Direct 8bits
#define DATARD8		0x1F //Pressure output data (MSB) or 8 bit data read from EEPROM R Direct 8bits
#define DATARD16	0x20 //Pressure output data (LSB) or 8-bit data read from indirect register R Direct 16bits
#define TEMPOUT		0x21 // 14-bit temperature output data R Direct 16bits
#define CFG			0x00 //Configuration register RW Indirect 8 bits
#define TWIADD		0x05 //TWI address W Indirect 8bits
#define USERDATA1	0x29 //User data RW EEPROM 8 bits
#define USERDATA2	0x2A //User data RW EEPROM 8 bits
#define USERDATA3	0x2B //User data RW EEPROM 8 bits
#define USERDATA4	0x2C //User data RW EEPROM 8 bits

#define RW			BIT1 //read write bit (used for direct read and direct write)

/**
 * Measurement modes
 */
#define MM_HIGHRESOLUTION	0X0A
#define MM_HIGHSPEED		0X09
#define	MM_ULTRALOWPOWER	0X0B
#define MM_LOWPOWER			0X0C

/**
 * Low power measurement modes (to be written to the CFG register)
 */
#define LPMM_17BITS			0x05
#define LPMM_15BITS			0x0D

/**
 * Operations:
 */

#define ASIC_SELFTEST		0x0F

	/**
	 * SCP1000-D01 SPI frame format:
	 * -----------------------------
	 *  
	 * Each SPI communication frame contains two or three 8 bit words:
	 * 
	 * 		1)
	 * 		the first word defines the register address (6 bits wide, bits [A5:A0] in Figure 9)
	 * 		followed by the type of access (‘0’ = Read or ‘1’ = Write) and one zero bit (bit 0, LSB).
	 * 		
	 * 		2)
	 * 		The following word(s) contain the data being read or written.
	 * 		The MSB of the words are sent first. Bits from MOSI line are sampled in on
	 * 		the rising edge of SCK and bits to MISO line are latched out on falling edge of SCK.
	 * 
	 * The CSB line must stay low during the entire frame accesses,
	 * i.e. between the bytes. If the CSB line state changes to high,
	 * the access is terminated. The CSB has to be pulled up after each
	 * communication frame.
	 */

/**
 * DIRECT READ, inputs: register address, number of bytes to be read
 * ------------
 * 1. 	Convert the SCP1000 register address to real address by shifting the
 * 		bit pattern to left by 2 bits ([xxAAAAAA] --> [A A A A A A RW 0], A = address bit):
 * 		(MSB) A A A A A A A RW 0 (LSB)
 * 			  |           |  | *-- allways zero
 * 			  |           |  *---- Read/Write Bit (Read: RW=0, Write: RW=1)
 * 			  *-----------*------- Register address bits
 * 2.	Set Read/Write bit to '0' at the address byte:
 * 		[A A A A A A RW 0] --> [A A A A A A 0 0]
 * 		The RW bit is zero automatically after the register address is shifted to left by 2
 * 		3. Set CSB to low
 * 		4. Send address byte (register)
 * 		5. Send SCK (clock cycles) and read data bytes
 *		6. Set CSB to HIGH
 */
static uint16_t _SCP_Read_via_Direct_Access(uint8_t address, uint8_t number_of_bytes) {
	uint16_t result;
	
	// 1. Convert the address:
	address = address << 2;		// shift the address
	CLEAR(address, RW);			// set the RW bit to 0 (read)
	
	SCP_SELECT;
		USART0_SPI_Comm(address);		// send the address
		
		// read number_of_bytes bytes
		result = 0;
		while (number_of_bytes > 0) {
			uint8_t justread;
			justread = USART0_SPI_Comm(0x00);
			
			result = result << 8;	// shift 8 bits left to make space for the new byte
			result = result | justread;
			
			number_of_bytes--;		// reduce the number of bytes to read
		}
	SCP_DESELECT;
	
	return result;
}

 /**
 * DIRECT WRITE, inputs: register address, register data
 * 1. 	Convert the SCP1000 register address to real address by shifting the
 * 		bit pattern to left by 2 bits ([xxAAAAAA] --> [A A A A A A RW 0], A = address bit):
 * 		(MSB) A A A A A A A RW 0 (LSB)
 * 			  |           |  | *-- allways zero
 * 			  |           |  *---- Read/Write Bit (Read: RW=0, Write: RW=1)
 * 			  *-----------*------- Register address bits
 * 2.	Set Read/Write bit to '1' at the address byte:
 * 		[A A A A A A RW 0] --> [A A A A A A 1 0]
 * 		The RW bit is zero automatically after the register address is shifted to left by 2
 * 		3. Set CSB to low
 * 		4. Send address byte (register)
 * 		5. Send register content (data byte)
 *		6. Set CSB to HIGH
 */
static void _SCP_Write_via_Direct_Access(uint8_t address, uint8_t data) {
	// 1. Convert the address:
	address = address << 2;		// shift the address
	SET(address, RW);			// set the RW bit to 1 (RW and 0);
	
	SCP_SELECT;
		USART0_SPI_Comm(address);		// send the address
		USART0_SPI_Comm(data);			// send thedata		
	SCP_DESELECT;
}


/**
* INDIRECT WRITE, input: indirect register address, register data
*	Indirect write requires three DIRECT WRITE actions:
*		1. Use DIRECT WRITE to write register address to ADDPTR (0x02)
*		2. Use DIRECT WRITE to write data to DATAWR (0x01)
*		3. Use DIRECT WRITE to write "indirect write" command 0x02 to OPERATION (0x03)
*		4. Wait 50ms
*/
/* not used, disabled.
static void _SCP_Write_Indirect_Access(uint8_t address, uint8_t data) {
	_SCP_Write_via_Direct_Access(0x02, address);	// #1, write reg address to ADDPTR (0x02)
	_SCP_Write_via_Direct_Access(0x01, data);		// #2, write reg data to DATAWR (0x01)
	_SCP_Write_via_Direct_Access(0x03, 0x02);		// #3, write 0x02 to OPERATION (0x03)

	TIMERS_BLOCK_MS(50);									// #4, wait 50ms
}
*/

 /****************************************************************
 * INDIRECT READ, input: indirect register address
 *	Indirect read requires three DIRECT read/write actions:
 *		1. Use DIRECT WRITE to write the register address to ADDPTR (0x02)
 *		2. Use DIRECT WRITE to write the "read indirect" command 0x01 to OPERATION (0x03)
 *		3. Wait 10ms
 *		4. Use DIRECT READ to read 2 bytes from DATARD16 (0x20), the register content
 *		   is in bits [7:0], bits [15:8] should be treated as zeros
 */
static uint16_t _SCP_Read_Indirect_Access_SPI(uint8_t address)
{
	uint16_t data;
	
	_SCP_Write_via_Direct_Access(ADDPTR, address);	// #1, write reg address to ADDPTR (0x02)
	_SCP_Write_via_Direct_Access(OPERATION, 0x01);	// #2, write 0x01 to OPERATION (0x03)

	TIMERS_BLOCK_MS(10);									// #3, wait 10ms

	data = _SCP_Read_via_Direct_Access(DATARD16, 2); // #4, read two bytes from DATARD16 (0x20)
	
	//clear bits [15:8] == clear '1111111100000000' == AND operation with 0000000011111111 (0x00FF)
	data = data & ((uint16_t) 0x00FF);

	return data;
}


bool SCP_TriggerMeasurement() {
	// check the preconditions:
	
	// 1. check whether the device is active
	if (SCP_PD_POUT & SCP_PD_PIN) {
		// the device is in the power down mode, it wasn't activated or the activation has failed
		LOGDBG("SCPt!PD");

		return false;
	}

	// 2. the request can only be issued when the DRDY line is low
	if (SCP_DRDY_PINPUT & SCP_DRDY_PIN) {
		LOGDBG("SCPt!DRDY");

		return false; // a dummy read could be executed here but it's actually user's resposibility
	}
	
	// the measurement is triggered by the low-to-high edge of the TRIG pin
	// the TRIG is set to low during reading of the last measrurement, so it's safe to assume
	// the TRIG is low 
	
	// 3. the request can only be issued when the TRIG is low
	if (SCP_TRIG_POUT & SCP_TRIG_PIN) {
		LOGDBG("SCPt!TRIG");
		
		return false;
	}
	
	//
	// preconditions check passed
	//
	
	// store the current timestamp
	RTC_GetRawTime(&SCP_MeasurementTimestamp);

	// trigger the conversion
	SET(SCP_TRIG_POUT, SCP_TRIG_PIN);
	
	return true;
}

void SCP_Convert(float* temperatureOutOptional, uint32_t* pressureOutOptional, uint16_t rawTemperatureIn, uint32_t rawPressureIn) {
	//convert the readings
	if (temperatureOutOptional != NULL) {
		bool isNegative = (rawTemperatureIn & BITD) == BITD;
		if (isNegative) {
			/**
			 * the temperature was negative, special treatment is required.
			 * See scp1000_product_family_specification_rev_0.08-3.pdf, 2.2.3.1 Examples of temperature conversion to [°C] for details
			 */
			
			rawTemperatureIn = ~rawTemperatureIn;				// invert bits
			rawTemperatureIn = rawTemperatureIn & 0x3FFF; 		// mask bits [13:0]
			rawTemperatureIn++;									// add 1
		}
		
		*temperatureOutOptional = rawTemperatureIn;
		*temperatureOutOptional = *temperatureOutOptional / 20;
		
		if (isNegative) {
			*temperatureOutOptional =  0.0f - *temperatureOutOptional;
		}
	}

	if (pressureOutOptional != NULL) {
		uint32_t rawPressureInCopy = rawPressureIn;
		rawPressureInCopy = rawPressureInCopy >> 2;
		*pressureOutOptional = rawPressureInCopy;
	}
}

bool SCP_Read(float* temperatureOutOptional, uint32_t* pressureOutOptional, uint16_t* rawTemperatureOutOptional, uint32_t* rawPressureOutOptional, rawtime_t* timestampOutOptional) {
	// check the preconditions:
	
	// 1. check whether the device is active
	if (SCP_PD_POUT & SCP_PD_PIN) {
		LOGDBG("SCPr!PD");
		// the device is in the power down mode, it wasn't activated or the activation has failed
		return false;
	}
	
	// 2. check whether the SCP sensor signals data being available (DRDY pin)
	if ((SCP_DRDY_PINPUT & SCP_DRDY_PIN) != SCP_DRDY_PIN) {
		LOGDBG("SCPr!DRDY");
		return false; // data isn't avilable yet
	}
	
	// 3. check whether the request was previously triggered
	if ((SCP_TRIG_POUT & SCP_TRIG_PIN) != SCP_TRIG_PIN) {
		LOGDBG("SCPr!TRIG");
		// the request wasn't triggered by the software
		return false;
	}
	// clear the TRIG so another conversion can be requested
	CLEAR(SCP_TRIG_POUT, SCP_TRIG_PIN);
	
	// 
	// preconditions check passed.
	//
	
	if (!USART0_SPI_Lock(0x08, 0x00, 0x00)) {
		LOGDBG("SCP!SPI");
		return false; // failed to accquire a lock on the SPI
	}
	
	uint16_t temperature;
	uint32_t pressure;
	
	temperature = _SCP_Read_via_Direct_Access(TEMPOUT, 2);
	temperature = temperature & 0x3FFF; // maxk bits [13:0]

	pressure = (uint32_t) _SCP_Read_via_Direct_Access(DATARD8, 1);
	pressure = pressure & 0x00000007;
	pressure = pressure << 16;
	pressure = pressure | _SCP_Read_via_Direct_Access(DATARD16, 2);
	
	USART0_SPI_Release();

	//publish the raw data
	if (rawTemperatureOutOptional != NULL)
		*rawTemperatureOutOptional = (uint16_t) temperature;

	if (rawPressureOutOptional != NULL)
		*rawPressureOutOptional = (uint32_t) pressure;
	
	SCP_Convert(temperatureOutOptional, pressureOutOptional, (uint16_t) temperature, (uint32_t) pressure);
	
	// publish the timestamp
	if (timestampOutOptional != NULL) {
		memcpy(
			timestampOutOptional,
			&SCP_MeasurementTimestamp,
			sizeof(rawtime_t)
		);
	}
	
	return true;	
}

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
bool SCP_MeasureBlocking(float* temperatureOut, uint32_t* pressureOut, uint16_t* rawTemperatureOut, uint32_t* rawPressureOut) {
	
	if (!SCP_TriggerMeasurement())
		return false;

	//wait until measugrement is ready (could go to sleep now and wait for an interrupt)
	//this takes around 580ms
	while (!(SCP_DRDY_PINPUT & SCP_DRDY_PIN)) {		// while the DRDY is low
		TIMERS_BLOCK_MS(10);
	}

	return SCP_Read(temperatureOut, pressureOut, rawTemperatureOut, rawPressureOut, NULL);
}


void _SCP_OnDataAvailable() {
	// even though we are only forwarding the call, it wouldn't be good
	// to registerd the Handler_SCP_Results directly with the Handler_AppLayer_Task
	// since the user might have changed the handler after the SCP device was enabled/initialized.
	if (Handler_SCP_Results != NULL) {
		Handler_SCP_Results();
	}
}

/**
 * This function will be executed inside of an interrupt.
 * The interrupt is raised when a measurement completes.
 */
bool _SCP_InterruptHandler() {
	if (P2IFG & SCP_DRDY_PIN) {
		CLEAR( P2IFG, SCP_DRDY_PIN );
		// interrupt was raised by the SCP pressure and temperature sensor
		// it notifies the application that a measurement was executed properly
		
		// register the AppTask handler and request invocation of this handler
		Handler_AppLayer_Task = _SCP_OnDataAvailable;
		System_Trigger_AppLayer_Task();
		
		return true; // return true to wake up the device
	}
	
	return false;
}

bool SCP_Enable(bool printProgress) {
	uint16_t i = 0;
	uint16_t data = 0;
	uint16_t revid = 0;
	uint16_t operation = 0;
	uint16_t cfg = 0;

	//turn on power
	CLEAR(SCP_PD_POUT, SCP_PD_PIN);
	
	// this is required by the specification of the SCP sensor.
	// it's possible to turn it earlier, execute other tasks, or wait in LPM
	TIMERS_BLOCK_MS(60);
	
	if (printProgress) {
		LOGDBG("SCP Enable...");
	}

	if (!USART0_SPI_Lock(0x08, 0x00, 0x00)) {
		LOGDBG("SCP Enable failed (SPI lock rejected)");

		//turn off the device
		SET(SCP_PD_POUT, SCP_PD_PIN);
		
		return false;
	}
	
	for(i = 10; i > 0; i--) {
		data = _SCP_Read_via_Direct_Access(STATUS, 1);
		
		TIMERS_BLOCK_MS(10);

		//if LSB == 0, the startup procedure is finished
		if (!(data & BIT0))
			break;
	}
	
	if (i == 0) {
		USART0_SPI_Release();

		LOGDBG("SCP Enable fail!");
		
		//turn off the device
		SET(SCP_PD_POUT, SCP_PD_PIN);
		
		return false;
	}
	
	data = _SCP_Read_via_Direct_Access(DATARD8, 1);
	
	// the DATARD8 should have the LSB == 1 , this means, the EEPROM checksum check was OK
	
	if (!(data & BIT0)) {
		USART0_SPI_Release();
		LOGDBG("SCP EEPROM check fail!");

		//turn off the device
		SET(SCP_PD_POUT, SCP_PD_PIN);
		
		return false;
	}
	
	/**
	 * SCP boot procedure was performed.
	 */

	/**
	 * display basic information about the chip
	 */
	if (printProgress) {
		revid = _SCP_Read_via_Direct_Access(REVID, 1);
		operation = _SCP_Read_via_Direct_Access(OPERATION, 1);
		cfg = _SCP_Read_Indirect_Access_SPI(CFG);
	}
	
	USART0_SPI_Release();

	if (printProgress) {
		LOGDBG("SCP ASIC revision number: %u", revid);
		LOGDBG("SCP operation register: 0x%02X", operation);
		LOGDBG("SCP CFG: 0x%.2X (0x05=17bits, 0x0D=15bits)", cfg);
		LOG("SCP Enable: OK");
	}
	
	// enable DRDY interrupt support
	CLEAR(SCP_DRDY_IES, SCP_DRDY_PIN); 		// raise interrupt on the raising edge
	CLEAR(P2IFG, SCP_DRDY_PIN); 			// make sure the interrupt request was cleared
	SET(SCP_DRDY_IE, SCP_DRDY_PIN);
	
	// register for Port2 Interrupts (since this is the only device that needs this on the board, the access can be exclusive)
	Handler_Interrupt_Port2 = _SCP_InterruptHandler;
	
	return true;
}

void SCP_Disable() {
	// turn off power
	SET(SCP_PD_POUT, SCP_PD_PIN);
	
	// and disable the interrupt
	CLEAR(SCP_DRDY_IE, SCP_DRDY_PIN);
	
	// remove the handler
	Handler_Interrupt_Port2 = NULL;
}

/**
 * triggers measurement
 */
COMMAND(scpreq, "Triggers temperature and pressure measurment", cmd_args) {
	bool success = SCP_TriggerMeasurement();
	
	snprintf(cmd_args->ResponseString, cmd_args->ResponseStringMaxLength,
			"success: %u",
			success);	
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
