/**
 * @ingroup		SystemAPI
 * @brief		Support for radio communication
 * @author		Freie Universitaet Berlin, Tomasz Naumowicz, Hans-Peter Heitzmann
 * 
 * This is a modified file that was provided by Texas Instruments. Details below:
 */


//----------------------------------------------------------------------------
//  Description:  This file contains functions that configure the CC1100/2500 
//  device.
// 
//  Demo Application for MSP430/CC1100-2500 Interface Code Library v1.0
//
//  K. Quiring
//  Texas Instruments, Inc.
//  July 2006
//  IAR Embedded Workbench v3.41
//----------------------------------------------------------------------------
//
//	
//	H. P. Heitzmann
//	Freie Universität Berlin
//	Institut für Informatik
//	CST (Computer Systems & Telematics) 
//
//----------------------------------------------------------------------------

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "../../Drivers/Device.h"
#include "../Logging.h"
#include "../Configuration.h"
#include "../USART.h"
#include "../Commands.h"
#include "../Timers.h"

#include "cc1100.h"

static uint8_t _radio_channel; 

void CC1100_Prepare_RFSettings() {
	unsigned long temp_long;
	unsigned char bw_e, bw_m;
	
	temp_long = (unsigned long)( ( base_freq / freq_osz ) * 65536.0 + 0.5 );
	
	base_freq = ( freq_osz / 65536.0 ) * (double)temp_long; // 1000000.0; computes the actual base frequency
	
	freq0 = ( unsigned char )( temp_long & 0xFF );
	temp_long >>= 8;
	freq1 = ( unsigned char )( temp_long & 0xFF );
	temp_long >>= 8;
	freq2 = ( unsigned char )( temp_long & 0xFF );

	temp_long = 1;
	temp_long <<= 20;
	mdmcfg4 = (unsigned char )( log10f( (float)temp_long *\
					 (float)baud_rate / (float) freq_osz ) / log10f( 2.0 ) );
	temp_long = 1;
	temp_long <<= 28;
	mdmcfg3 = (unsigned char )( (double)temp_long * (double)baud_rate /\
					 ( freq_osz * (double)( 1 << mdmcfg4 ) ) - 256.0 );

	temp_long = 1;
	temp_long <<= 28;
	
	// computes the achieved baud rate
	baud_rate = (unsigned long)( (double)( ( 256L + (unsigned long)mdmcfg3 ) *\
					   (unsigned long)( 1 << mdmcfg4 ) ) * freq_osz / (double)temp_long + 0.5 );

	
	temp_long = (unsigned long)( (double)baud_rate * 1.18 + 0.5 );
	
	for( bw_e = 3;; bw_e-- )
	{
		for( bw_m = 3;; bw_m-- )
		{
			bandwith =\
				(unsigned long)( ( freq_osz / (double)( ( 1 << bw_e ) * 8 * ( 4 + bw_m ) ) ) + 0.5 ); //0.5 um aufzurunden
			
			if( bandwith >= temp_long )
			{
				bw_e <<= 6;
				bw_m <<= 4;
				mdmcfg4 |= ( bw_e + bw_m );
				bw_e = 0;
				break;
			}
			if( !bw_m ) break;
		}
		if( !bw_e ) break;
	}

	temp_long = 1;
	temp_long <<= 18;
	for( bw_e = 0; bw_e < 4; bw_e++ )
	{
		for( bw_m = 0;; bw_m++ )
		{
			channel_spacing =\
				(unsigned long)( (double)( ( 256 + bw_m ) * ( 1 << bw_e ) ) * freq_osz / (double)temp_long );
			
			if( ( channel_spacing ) >= bandwith )
			{

				mdmcfg0 = bw_m;
				mdmcfg1 = bw_e;
				bw_e = 0xAA; // added so that the outer loop exits
				break;
			}
			if( bw_m == 255 ) break;
		}
	}
	
}

void CC1100_LogConfiguration() {
	LOGDBG("CC.Carrier: %lu Hz", (uint32_t) base_freq);
	LOGDBG("CC.BaudRate: %lu Baud", baud_rate );

	LOGDBG("CC.Bandwith: %lu Hz", bandwith);
	LOGDBG("CC.ChannelSpace: %lu Hz", channel_spacing );
	LOGDBG("CC.Channel: %u", channel );
	
    LOGDBG("CCxxx0_FREQ2 0x%.2X", freq2); // Freq control word, high byte
    LOGDBG("CCxxx0_FREQ1 0x%.2X",freq1); // Freq control word, mid byte.
    LOGDBG("CCxxx0_FREQ0 0x%.2X",freq0); // Freq control word, low byte.
    LOGDBG("CCxxx0_MDMCFG4 0x%.2X",mdmcfg4); // Modem configuration, default 0x2D, 500 kBaud 0x2E
    LOGDBG("CCxxx0_MDMCFG3 0x%.2X",mdmcfg3); // Modem configuration.
    LOGDBG("CCxxx0_MDMCFG1 0x%.2X",mdmcfg1 + 0x20); // Modem configuration.
    LOGDBG("CCxxx0_MDMCFG0 0x%.2X",mdmcfg0); // Modem configuration.
}

static void writeRFSettings() {
    // Write register settings
    CC_SPIWriteReg(CCxxx0_IOCFG2,   0x2E); // GDO2 output pin config
    CC_SPIWriteReg(CCxxx0_IOCFG0,   0x06); // GDO0 output pin config, 0x07 wasn't sufficient: it notified the mcu about packets with CRC OK, but the CC goes to idle when CRC is not OK (because of the RXOFF_MODE setting). 0x06 is more detailed setting.  
    CC_SPIWriteReg(CCxxx0_FIFOTHR,	0x0F); // Threshold TX=1, RX=64
    CC_SPIWriteReg(CCxxx0_PKTLEN,   0x3D); // Packet length. configured to 61 bytes: so that the RX buffer can hold the packet length, the payload and the CRC+LQI. See data sheet, page 37+38
    CC_SPIWriteReg(CCxxx0_PKTCTRL1, 0x0F); // Packet automation control, crc_autoflush + appendstatus +addchk11
    CC_SPIWriteReg(CCxxx0_PKTCTRL0, 0x05); // Packet automation control, CRC calc in TX + CRC check in RX, var length
    CC_SPIWriteReg(CCxxx0_ADDR,     Configuration.NodeID); // Device address.
    CC_SPIWriteReg(CCxxx0_CHANNR,   channel);//( ptrce->channel << 1 ) ); // Channel number. //0x00
    CC_SPIWriteReg(CCxxx0_FSCTRL1,  0x0B); // Freq synthesizer control.
    CC_SPIWriteReg(CCxxx0_FSCTRL0,  0x00); // Freq synthesizer control.
    CC_SPIWriteReg(CCxxx0_FREQ2,    freq2);// 0x21 ptrce->freq2); // Freq control word, high byte
    CC_SPIWriteReg(CCxxx0_FREQ1,    freq1);// 0x62ptrce->freq1); // Freq control word, mid byte.
    CC_SPIWriteReg(CCxxx0_FREQ0,    freq0); //0x76 ptrce->freq0); // Freq control word, low byte.
    CC_SPIWriteReg(CCxxx0_MDMCFG4,  mdmcfg4); //0x5D ptrce->mdmcfg4); // Modem configuration, default 0x2D, 500 kBaud 0x2E
    CC_SPIWriteReg(CCxxx0_MDMCFG3,  mdmcfg3); //0x3B ptrce->mdmcfg3); // Modem configuration.
    CC_SPIWriteReg(CCxxx0_MDMCFG2,  0x73); // Modem configuration.	// TI_CC = 0x73);
    CC_SPIWriteReg(CCxxx0_MDMCFG1,  mdmcfg1 + 0x20); //0x4E ( ptrce->mdmcfg1 + 0x20 ) ); // Modem configuration.
    CC_SPIWriteReg(CCxxx0_MDMCFG0,  mdmcfg0); //0x9A ptrce->mdmcfg0); // Modem configuration.
    CC_SPIWriteReg(CCxxx0_DEVIATN,  0x00); // Modem dev (when FSK mod en)
    CC_SPIWriteReg(CCxxx0_MCSM1 ,   0x33); // 11 0011 ((00)rxoff_mode) 11 			MainRadio Cntrl State Machine
    CC_SPIWriteReg(CCxxx0_MCSM0 ,   0x18); // 00[01]autocal_01_on_idle_to_rx_or_tx 1000 MainRadio Cntrl State Machine
    CC_SPIWriteReg(CCxxx0_FOCCFG,   0x1D); // Freq Offset Compens. Config
    CC_SPIWriteReg(CCxxx0_BSCFG,    0x1C); // Bit synchronization config.
    CC_SPIWriteReg(CCxxx0_AGCCTRL2, 0xC7); // AGC control.
    CC_SPIWriteReg(CCxxx0_AGCCTRL1, 0x00); // AGC control.
    CC_SPIWriteReg(CCxxx0_AGCCTRL0, 0xB2); // AGC control.
    CC_SPIWriteReg(CCxxx0_FREND1,   0xB6); // Front end RX configuration.
    CC_SPIWriteReg(CCxxx0_FREND0,   0x10); // Front end RX configuration.
    CC_SPIWriteReg(CCxxx0_FSCAL3,   0xEA); // Frequency synthesizer cal.
    CC_SPIWriteReg(CCxxx0_FSCAL2,   0x0A); // Frequency synthesizer cal.
    CC_SPIWriteReg(CCxxx0_FSCAL1,   0x00); // Frequency synthesizer cal.
    CC_SPIWriteReg(CCxxx0_FSCAL0,   0x11); // Frequency synthesizer cal.
    CC_SPIWriteReg(CCxxx0_FSTEST,   0x59); // Frequency synthesizer cal.
    CC_SPIWriteReg(CCxxx0_TEST2,    0x88); // Various test settings.
    CC_SPIWriteReg(CCxxx0_TEST1,    0x31); // Various test settings.
    CC_SPIWriteReg(CCxxx0_TEST0,    0x0B); // Various test settings.
}

//-----------------------------------------------------------------------------
//  void RFSendPacket(char *txBuffer, char size)
//
//  DESCRIPTION:
//  This function transmits a packet with length up to 63 bytes.  To use this
//  function, GD00 must be configured to be asserted when sync word is sent and
//  de-asserted at the end of the packet, which is accomplished by setting the
//  IOCFG0 register to 0x06, per the CCxxxx datasheet.  GDO0 goes high at
//  packet start and returns low when complete.  The function polls GDO0 to
//  ensure packet completion before returning.
//
//  ARGUMENTS:
//      char *txBuffer
//          Pointer to a buffer containing the data to be transmitted
//
//      char size
//          The size of the txBuffer
//-----------------------------------------------------------------------------
void RFSendPacket(const uint8_t *txBuffer, uint8_t size) {
	//LOGDBG("RFSendPacket got %u (%u) bytes for %u", size, txBuffer[0], txBuffer[1]);
	
	/**
	 * in the case the CCxxx0_IOCFG0 is set to 0x06 (which is the default setting in this firmware!)
	 * 
	 * the GDO0 is asserted when a packet is beeing received or sent (simplified description, see CC's manual for details).
	 * It's OK to submit an TX during RX, the CC will handle this properly. --> although: there is a forced IDLE in this code. Needs to be tested during radio-coding.
	 * 
	 * But it's not OK to submit an RX during RX.
	 * 
	 * WRONG: TX will be ignored when state transition is not possible
	 * 
	 * GDO0 will be used to BLOCK in this function, but not after sending a packet (in order to keep the overhead low when having rare communication)
	 * but before sending one, if the GDO0 is asserted.
	 */
	
	
		CC_SPIStrobe( CCxxx0_SIDLE );							// Change state to Idle
		CC_SPIStrobe( CCxxx0_SFTX);								// Flush the TX FIFO buffer

		CC_SPIWriteBurstReg( CCxxx0_TXFIFO, txBuffer, size );	// Write data to the TX FIFO buffer

		CC_SPIStrobe( CCxxx0_STX );

		
		// now, the CC will callibrate and after this is done, the packet will be sent (and the GDO0 will be asserted, wait for this)
		while ((CC_GDO0_IN & CC_GDO0_PIN) == 0) {}

		// now the TX is in progress. wait until it's done.
		while (CC_GDO0_IN & CC_GDO0_PIN) {}
}

//-----------------------------------------------------------------------------
//  char RFReceivePacket(char *rxBuffer, char *length)
//
//  DESCRIPTION:
//  Receives a packet of variable length (first byte in the packet must be the
//  length byte).  The packet length should not exceed the RXFIFO size.  To use
//  this function, APPEND_STATUS in the PKTCTRL1 register must be enabled.  It
//  is assumed that the function is called after it is known that a packet has
//  been received; for example, in response to GDO0 going low when it is
//  configured to output packet reception status.
//
//  The RXBYTES register is first read to ensure there are bytes in the FIFO.
//  This is done because the GDO signal will go high even if the FIFO is flushed
//  due to address filtering, CRC filtering, or packet length filtering.
//
//  ARGUMENTS:
//      char *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      char *length
//          Pointer to a variable containing the size of the buffer where the
//          incoming data should be stored. After this function returns, that
//          variable holds the packet length.
//
//  The packet contains the length of the data in the first byte (provided by the CC hardware)
//
//  500us for 62 bytes
//
//  RETURN VALUE:
//      char
//          0x80:  CRC OK
//          0x00:  CRC NOT OK (or no pkt was put in the RXFIFO due to filtering)
//-----------------------------------------------------------------------------

bool RFReceivePacket(uint8_t* rxBuffer, uint8_t rxBufferLength) {
	// initialize the length entry for the case, the function returns early
	rxBuffer[0] = 0;
	
	// get status of the CC RX FIFO
    // This status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
	uint8_t cc_rxbytes_register = CC_SPIReadStatus(CCxxx0_RXBYTES);	// access the CC register
	
	// extract information from the cc_rxbytes_register
		uint8_t	num_rxbytes 			= cc_rxbytes_register & CCxxx0_NUM_RXBYTES; // extract the number of bytes in the fifo
	
		// extract information about a overflow of the fifo
		bool	rxfifo_overflow 		= false;
		if (cc_rxbytes_register & CCxxx0_RXFIFO_OVERFLOW)
			rxfifo_overflow = true;

	if ( num_rxbytes && (rxfifo_overflow == false))
	{
		// read the length byte
		uint8_t cc_receivedPacketLength = CC_SPIReadReg( CCxxx0_RXFIFO );

		// receve the data only when the provided buffer is large enough. Remember, the first byte of the rxBuffer is used to store the size of this buffer
		if ( cc_receivedPacketLength < rxBufferLength ) 
		{
			uint8_t cc_statusBytes[2];

			rxBuffer[0] = cc_receivedPacketLength;
			// read the data
			CC_SPIReadBurstReg(CCxxx0_RXFIFO, &(rxBuffer[1]), cc_receivedPacketLength); // Pull data

			// read appended status bytes
			CC_SPIReadBurstReg(CCxxx0_RXFIFO, cc_statusBytes, 2);

			CC_LastRSSI = cc_statusBytes[0];
			
			/**
			 * Convert the RSSI value (user guide page 43)
			 */
			if (CC_LastRSSI >= 128)
				CC_LastRSSI = CC_LastRSSI - 256;
			CC_LastRSSI = CC_LastRSSI / 2 - 74;
			
			CC_LastLQI = (cc_statusBytes[CCxxx0_LQI_RX]) & CCxxx0_LQI_MASK;
			
			// it's possible, more packets were received. for some reason, when the CC didn't change IDLE mode e.g.
			// detect and discard them..
			if ( num_rxbytes != (cc_receivedPacketLength + 3)) {//3 == 1 (length) + 2 (RSSI+LQI)
				// Make sure that the radio is in IDLE state before flushing the FIFO
				// (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
				CC_SPIStrobe(CCxxx0_SIDLE);
				CC_SPIStrobe(CCxxx0_SFRX);    		// Flush RXFIFO
				LOGERR("cc.sfrx");
			}
			
			return (cc_statusBytes[CCxxx0_LQI_RX] & CCxxx0_CRC_OK); // Return CRC_OK bit
		}                                       
	}
	
	/*
	 * this is the error path:
	 * 
	 * possible reasons:
	 * 	- there was RX FIFO overflow -> data should be discarded
	 *  - the provided buffer was too small -> data should be disarded
	 *  - maybe there was no data at all (num_rxbytes == 0)... -> easier to use the same error path and disard the data (although the FIFO is empty anyway)
	 */

	// Force the IDLE state before flushing the FIFO
	// (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point anyway)
	CC_SPIStrobe(CCxxx0_SIDLE);
	CC_SPIStrobe(CCxxx0_SFRX);    		// Flush RXFIFO
	
	return false;
}

void CC1100_PowerOff()
{
	USART0_SPI_Lock(0x04, 0x00, 0x00);

		CC_SPIStrobe(CCxxx0_SIDLE);	// in den IDLE Mode setzen
		CC_Wait(12);
		CC_SPIStrobe(CCxxx0_SPWD);		// in den PowerDown Mode setzen
		CC_Wait(12);
	
	USART0_SPI_Release();
	
	LOG("CC1101 power down: OK");
}	

void CC1100_PrintInformation() {
	uint8_t cc1101_partnum = CC_SPIReadStatus(CCxxx0_PARTNUM);
	uint8_t cc1101_version = CC_SPIReadStatus(CCxxx0_VERSION);
	
	LOGDBG("CC1101: PartNumber %u, Version %u", cc1101_partnum, cc1101_version);
}

void CC1100_Init_PowerOn_RX()
{
	freq_osz = CRYSTAL_CLOCK;
	base_freq = DEFAULT_RF_FREQUENCY; // + 107500.0;	// default Frequency in Hz
	baud_rate = DEFAULT_CC1101_BAUD_RATE;
	channel = 0;
	
	LOGDBG("Computing RF Configuration");
	
	channel = 0;
	
	CC1100_Prepare_RFSettings();
	
	CC1100_LogConfiguration();
	
	
	CC_GDO0_IE &= ~CC_GDO0_PIN;		// Disable int on end of packet
	CC_GDO0_IFG &= ~CC_GDO0_PIN;		// Clear flag

	//CC_GDO0_IFG |= CC_GDO0_PIN;		// Clear flag, old!

	//CC_SPISetup();						// Initialize SPI port
	
	CC_PowerupResetCCxxxx();			// Reset CCxxxx
	//calc_RFSettings(); // not available yet
	writeRFSettings();					// Write RF settings to config reg

	//uint8_t paTable[8]; uint8_t paTableLen = 1;
	//paTable[0] = 0x60;
	//CC_SPIWriteBurstReg( CCxxx0_PATABLE, paTable, paTableLen);	//Write PATABLE
	CC_SPIWriteReg(CCxxx0_PATABLE, 0xC2); //max power. C2
	
			
	//CC_GDO0_IES &= ~CC_GDO0_PIN;		// Int on rising edge (end of pkt) GDO0 = 7
	CC_GDO0_IES |= CC_GDO0_PIN;		// Int on falling edge (end of pkt) GDO0 = 6
	CC_GDO0_IFG &= ~CC_GDO0_PIN;		// Clear flag
	CC_GDO0_IE |= CC_GDO0_PIN;		// Enable int on end of packet

	CC_SPIStrobe(CCxxx0_SRX);			// Initialize CCxxxx in RX mode.
										// When a pkt is received, it will signal on GDO0 and wake CPU
	
	LOG("CC1101 is in RX");
}

void CC110x_SetAddress(uint8_t nodeid) {
	CC_SPIWriteReg(CCxxx0_ADDR, nodeid);
}

void CC110x_BlockUntilIdleState() {
	/**
	 * Remark from the errata sheet:	Reading status byte can produce corrupted results
	 * Workaround: 						Majority voting was added
	 */
	
	uint8_t currentState;
	do {
		do {
			currentState = CC_SPIReadStatus(CCxxx0_MARCSTATE);
		} while (currentState != CC_SPIReadStatus(CCxxx0_MARCSTATE));
	} while (currentState != CC110x_MARC_STATE_IDLE);
}

/**
 * Channels can be set in the CHANNR register, there are 256 channels available.
 * The spacing between channels is already set in the other freqency configuration registers.
 * Channels don't overlap but if the spacing is not small enought, there could be some interferences.
 * In order to aviod problems now, the channel number specified by the user will be multiplied by two
 * and as such, will double the channel spacing computed by the Chipcon algorithm.
 */
void CC110x_SetChannel(uint8_t channel) {
	/**
	 * If any frequency programming register is
	 * altered when the frequency synthesizer is
	 * running, the synthesizer may give an
	 * undesired response. Hence, the frequency
	 * programming should only be updated when
	 * the radio is in the IDLE state.
	 * 
	 * Source:
	 * 	http://focus.ti.com/general/docs/lit/getliterature.tsp?genericPartNumber=cc1101&fileType=pdf
	 * 	Chapter 21
	 * 	Page 55
	 */
	
	// 0. Check whether the channel number is valid (256/2 = 127)
	if (channel > 127)
		return;
	
	uint8_t physicalChannel = channel * 2; // modify user's input to increase the channel spacing
	
	// 1. check the current channel
	if (_radio_channel == physicalChannel)
		return; // the channel was already set.
	
	// 2. Switch to idle
	CC_SPIStrobe(CCxxx0_SIDLE);
	
	/**
	 * Note: An SIDLE strobe will clear all
	 * pending command strobes until IDLE
	 * state is reached. This means that if for
	 * example an SIDLE strobe is issued
	 * while the radio is in RX state, any other
	 * command strobes issued before the
	 * radio reaches IDLE state will be
	 * ignored.
	 * 
	 * Source:
	 * 	http://focus.ti.com/general/docs/lit/getliterature.tsp?genericPartNumber=cc1101&fileType=pdf
	 * 	Chapter 10.4
	 * 	Page 31
	 * 
	 * Conclusion: It's important to wait until the IDLE state is reached.
	 */
	
	CC110x_BlockUntilIdleState();
	
	// 3. Update current channel configuration
	_radio_channel = channel;
	CC_SPIWriteReg(CCxxx0_CHANNR, physicalChannel);
	
	// 4. Force callibration
	CC_SPIStrobe(CCxxx0_SCAL);
	Timers_Block(7);				// 721us are required for the calibration to complete

	CC110x_BlockUntilIdleState(); 	// only as safety check
}

uint8_t CC110x_GetChannel() {
	return _radio_channel;
}

void CC110x_PowerOn_RX() {
	CC_SPIStrobe(CCxxx0_SRX);			// Initialize CCxxxx in RX mode.
}

