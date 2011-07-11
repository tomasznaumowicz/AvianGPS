/**
 * @file
 * @ingroup		DriverAPI
 * @brief		@b Driver: Support for the AvianGPS V4 device (used by Applications, Drivers and System)
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz
 */

#ifndef AVIAN_BOARD_H_
#define AVIAN_BOARD_H_

#include <msp430x16x.h>
#include "../System/Types.h"


/**
 * @name		Default configuration.
 * 
 * You can overwrite this configuration by putting the same defines with adjusted values in your \ref AppConfig.h file.
 * 
 * @{
 */


/**
 * @brief		Controls the LED1 feature on the device.
 * 
 * You can enable LED1 usage in the software by setting this switch to 1. Setting
 * this switch to 0 will disable the usage of the LED1 by the software.
 * 
 * @remarks		The periodic "alive" blink signal that happens every 8 seconds will execute independent of this setting. 
 */
#ifndef LED1_ENABLED
	#define LED1_ENABLED		1
#endif

/**
 * @brief		Controls the LED2 feature on the device.
 * 
 * You can enable LED2 usage in the software by setting this switch to 1. Setting
 * this switch to 0 will disable the usage of the LED2 by the software.
 */
#ifndef LED2_ENABLED
	#define LED2_ENABLED		1
#endif

/** @} */

/**
 * @brief Describes the results of driver init function
 */
typedef enum {
	Init_Success			= 0,		///< No Failure
	LogEngine_Failure		= 1,		///< Failure while enabling the LogEngine
	VenusGPS_Failure		= 2,		///< Failure while enabling the VenusGPS device
	SCP_Failure				= 3			///< Failure while enabling the pressure and temperature sensor
} init_drivers_result_t;

/**
 * @brief		Initializes all drivers on the AvianGPS board
 * 
 * @returns		Status as \ref init_drivers_result_t
 * 
 * @remarks		All sensors are disabled after the initialization. You need to enable them if you need to use them.
 */
init_drivers_result_t Avian_InitDrivers();

/**
 * @brief		Stops the device and puts it in the LPM4 mode. Provides visual feedback via LED1 and LED2 (if LEDs are enabled)
 */
void Avian_CriticalLPM4(uint8_t code);



#ifndef DOXYGEN_PUBLIC_DOC


void Avian_InitPorts();


#if LED1_ENABLED
	#define LED1_ON			HW_LED1_ON
	#define LED1_STATE		HW_LED1_STATE
	#define LED1_OFF		HW_LED1_OFF
	#define LED1_TOGGLE		HW_LED1_TOGGLE
#else
	#define LED1_ON			
	#define LED1_STATE		
	#define LED1_OFF		
	#define LED1_TOGGLE		
#endif


#if LED2_ENABLED
	#define LED2_ON			HW_LED2_ON
	#define LED2_STATE		HW_LED2_STATE
	#define LED2_OFF		HW_LED2_OFF
	#define LED2_TOGGLE		HW_LED2_TOGGLE
#else
	#define LED2_ON			
	#define LED2_STATE		
	#define LED2_OFF		
	#define LED2_TOGGLE		
#endif


#ifndef LED_WHEN_IN_ACTIVEMODE_TURN_ON_MACRO
	#define LED_WHEN_IN_ACTIVEMODE_TURN_ON_MACRO
#endif

#ifndef LED_WHEN_IN_ACTIVEMODE_TURN_OFF_MACRO
	#define LED_WHEN_IN_ACTIVEMODE_TURN_OFF_MACRO
#endif

/*-------------AVIAN BORD - PORT DESCRIPTION-------------*/

// light
#define LIGHT_COLL_PIN			BIT2
#define LIGHT_COLL_DIR			P6DIR
#define LIGHT_COLL_OUT			P6OUT

#define LIGHT_EMM_PIN			BIT7
#define LIGHT_EMM_DIR			P6DIR
#define LIGHT_EMM_OUT			P6OUT
#define LIGHT_EMM_INPUT			P6OUT
#define LIGHT_EMM_SEL			P6SEL



/*---------GPS Module Venus634FLPx - MSP1612---------------
	
								Port/Function		Pin
			
	GPIO24 <G24>				P5.6/TA1 - Input	8
	P1PPS						P1.1/TA0 - Input	40
	
	GPS_PWR is a voltage regulator pin. The regulator
	ouput powers on the GPS module.
	
	GPS_PWR_PDIR				P5.5 - Output		2/36
	
	RXD							P3.6/UTXD1 - Output	42
	TXD							P3.7/URXD1 -Input	44
	
	
-----------GPS #define-----------------------------------*/

#define GPS_GPIO24_PIN			BIT6
#define GPS_GPIO24_DIR			P5DIR
#define GPS_GPIO24_OUT			P5OUT
#define GPS_GPIO24_IN			P5IN

#define GPS_G1PPS_PIN			BIT1
#define GPS_G1PPS_DIR			P1DIR
#define GPS_G1PPS_IN			P1IN
#define GPS_G1PPS_IE			P1IE
#define GPS_G1PPS_IES			P1IES
#define GPS_G1PPS_IFG			P1IFG

#define GPS_PWR_PIN				BIT5
#define GPS_PWR_DIR				P5DIR
#define GPS_PWR_OUT				P5OUT

#define GPS_RX_PIN				BIT6
#define GPS_TX_PIN				BIT7
#define GPS_RXTX_DIR			P3DIR
#define GPS_RXTX_SEL			P3SEL

/*SPI for Tranceiver, FLash Mem and Pressure Pressure Sensor
	
								Port/Function		Pin
	
	MOSI						P3.1/SIMO0/SDA		29
	MISO						P3.2/SOMI0			30
	SCK							P3.3/UCLK0/SCL		31

-----------SPI #define-----------------------------------*/

#define SPI_MOSI_PIN			BIT1
#define SPI_MISO_PIN			BIT2
#define SPI_SCLK_PIN			BIT3
#define SPI_DIR					P3DIR
#define SPI_SEL					P3SEL
#define SPI_IN					P3IN

/*-----------Pressure Sensor/SCP1000-D01-------------------
	
								Port/Function		Pin
	
	nCS							P1.7/TA2 - Output	19
	PWR_DN						P1.4/SMCLK - Output	16
	DRDY						P2.2/TA0 - Input	22
	TRIG						P6.4/Output			3	
	

-----------Pressure Sensor/SCP1000-D01 #define-----------*/

//Port 1 chip select

#define SCP_nCS_PIN				BIT7
#define SCP_CS_OUT         		P1OUT//when high, device is deselected
#define SCP_CS_DIR         		P1DIR

//Port 1 power down
#define SCP_PWR_PIN				BIT4
#define SCP_PWR_OUT         	P1OUT//when high, device is deselected
#define SCP_PWR_DIR         	P1DIR

//Trigger input, connect to GND if not used (digital input)

#define SCP_TRIG_PIN    		BIT4
#define SCP_TRIG_OUT   			P6OUT
#define SCP_TRIG_DIR   			P6DIR

//Interrupt signal (data ready) (digital output)

#define SCP_DRDY_PIN    		BIT2
#define SCP_DRDY_OUT   			P2OUT
#define SCP_DRDY_DIR   			P2DIR
#define SCP_DRDY_INPUT			P2IN
#define SCP_DRDY_IE         	P2IE
#define SCP_DRDY_IES        	P2IES

/*----------Memory/M25P80----------------------------------
	
								Port/Function		Pin

	nHOLD						P5.0/TB3 - Output	44
	nCS							P5.4/MCLK - Out		48
	nWP							P4.2/TB2 - Out		38

------------Memory/M25P80 #define------------------------*/

//Mem hold and chip select

#define HW_LED1_ON			{ P1OUT &= ~BIT3; }
#define HW_LED1_STATE		( (P1OUT & BIT3) != BIT3 )
#define HW_LED1_OFF			{ P1OUT |= BIT3; }
#define HW_LED1_TOGGLE		{ P1OUT ^= BIT3; }

#define HW_LED2_ON			{ P1OUT &= ~BIT6; }
#define HW_LED2_STATE		( (P1OUT & BIT6) != BIT6 )
#define HW_LED2_OFF			{ P1OUT |= BIT6; }
#define HW_LED2_TOGGLE		{ P1OUT ^= BIT6; }

#define TP1_OFF			{ P2OUT &= ~BIT1; }
#define TP1_STATE		( (P2OUT & BIT1) == BIT1 )
#define TP1_ON			{ P2OUT |= BIT1; }
#define TP1_TOGGLE		{ P2OUT ^= BIT1; }

#define TP2_OFF			{ P1OUT &= ~BIT5; }
#define TP2_STATE		( (P1OUT & BIT5) == BIT5 )
#define TP2_ON			{ P1OUT |= BIT5; }
#define TP2_TOGGLE		{ P1OUT ^= BIT5; }


#define FLASH_MEM_HOLD_PIN      BIT0
#define FLASH_MEM_HOLD_POUT     P5OUT
#define FLASH_MEM_HOLD_PDIR     P5DIR

#define FLASH_CS_PIN          	BIT4
#define FLASH_CS_OUT         	P5OUT //when high, device is deselected (MEMS)
#define FLASH_CS_DIR         	P5DIR

//write protect
#define FLASH_MEM_WP_PIN        BIT2
#define FLASH_MEM_WP_OUT       	P4OUT
#define FLASH_MEM_WP_DIR       	P4DIR

/*---------Radio/CC1101------------------------------------
	
								Port/Function		Pin

	GDO2						P2.6/ADC12CLK/DMAE0	26
	GDO0						P2.7/TA0			27
	nCS							P1.7/ACLK			19

-----------CC1100 #define--------------------------------*/

#define CC_GDO0_PIN          	BIT7
#define CC_GDO0_OUT        		P2OUT
#define CC_GDO0_IN         		P2IN
#define CC_GDO0_DIR        		P2DIR
#define CC_GDO0_IE         		P2IE
#define CC_GDO0_IES        		P2IES
#define CC_GDO0_IFG        		P2IFG

#define CC_GDO2_PIN          	BIT6
#define CC_GDO2_OUT        		P2OUT
#define CC_GDO2_IN         		P2IN
#define CC_GDO2_DIR        		P2DIR
#define CC_GDO2_IE         		P2IE
#define CC_GDO2_IES        		P2IES
#define CC_GDO2_IFG        		P2IFG

#define CC_CSn_PIN           	BIT0
#define CC_CSn_OUT         		P2OUT
#define CC_CSn_DIR         		P2DIR



#endif // DOXYGEN_PUBLIC_DOC



#endif /*AVIAN_BOARD_H_*/

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
