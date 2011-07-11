#ifndef SCP1000D01_H_
#define SCP1000D01_H_

#include <stdint.h>
#include "../System/Types.h"
#include "../System/RTC.h"

#define SCP_CS_POUT		SCP_CS_OUT //when high, device is deselected
#define SCP_CS_PDIR		SCP_CS_DIR
#define SCP_CS_PIN		SCP_nCS_PIN

//Trigger input, connect to GND if not used (digital input)
#define SCP_TRIG_POUT	SCP_TRIG_OUT
#define SCP_TRIG_PDIR	SCP_TRIG_DIR
//#define SCP_TRIG_PIN	SCP_TRIG_PIN

//Interrupt signal (data ready) (digital output)
#define SCP_DRDY_POUT	SCP_DRDY_OUT
#define SCP_DRDY_PINPUT	SCP_DRDY_INPUT
#define SCP_DRDY_PDIR	SCP_DRDY_DIR
//#define SCP_DRDY_PIN	SCP_DRDY_PIN

//Power down, connect to GND if not used (Input to force the chip in power down mode)
#define SCP_PD_POUT		SCP_PWR_OUT
#define SCP_PD_PDIR		SCP_PWR_DIR
#define SCP_PD_PIN		SCP_PWR_PIN

#define SCP_SELECT		CLEAR(SCP_CS_POUT, SCP_CS_PIN)
#define SCP_DESELECT	SET(SCP_CS_POUT, SCP_CS_PIN)

/**
 * currently not in use
 * 

enum scp_response_code {
	SCP_RESPONSE_OK			= 0,
	SCP_RESPONSE_POWERDOWN	= 1,
	SCP_RESPONSE_DRDY		= 2,
	SCP_RESPONSE_TRIG		= 3
};

*/

#endif /*SCP1000D01_H_*/

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
