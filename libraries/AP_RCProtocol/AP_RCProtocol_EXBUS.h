/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 /*
    EXBUS protocol imolementation based on 
        nichtgedacht/JetiExBus 
        chiefenne/JETI_EX_BUS
        betaflight/betaflight
    Code by David B. Bitton
*/
#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_EXBUS_ENABLED

#include "AP_RCProtocol.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "SoftSerial.h"

#define EXBUS_MAX_CHANNELS   24U      // Maximum number of channels from EXBUS datastream
#define EXBUS_FRAMELEN_MAX   14U      // maximum possible framelength
#define EXBUS_HEADER_LEN     2U       // header length
#define EXBUS_FRAME_PAYLOAD_MAX (EXBUS_FRAMELEN_MAX - EXBUS_HEADER_LEN)     // maximum size of the frame length field in a packet
#define EXBUS_BAUDRATE      250000U //125000U
#define EXBUS_TX_TIMEOUT    500000U   // the period after which the transmitter is considered disconnected (matches copters failsafe)
#define EXBUS_RX_TIMEOUT    150000U   // the period after which the receiver is considered disconnected (>ping frequency)

#define JETIEXBUS_COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x]))))) // number of elements in an array

class AP_RCProtocol_EXBUS : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_EXBUS(AP_RCProtocol &_frontend);
    virtual ~AP_RCProtocol_EXBUS();

    void process_byte(uint8_t byte, uint32_t baudrate) override;
    // void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_handshake(uint32_t baudrate) override;
    void update(void) override;

    // is the receiver active, used to detect power loss and baudrate changes
    bool is_rx_active() const override {
        // later versions of CRSFv3 will send link rate frames every 200ms
        // but only before an initial failsafe
        return _last_rx_frame_time_us != 0 && AP_HAL::micros() - _last_rx_frame_time_us < EXBUS_RX_TIMEOUT;
    }

    // is the transmitter active, used to adjust telemetry data
    bool is_tx_active() const {
        // this is the same as the Copter failsafe timeout
        // return AP_HAL::micros() < _last_tx_frame_time_us + EXBUS_TX_TIMEOUT;
        return false;
    }

    // get singleton instance
    static AP_RCProtocol_EXBUS* get_singleton() {
        return _singleton;
    }

        // return the protocol string
    const char* get_protocol_string() const;

protected:
	// packet processing
	enum enPacketState
	{
		WAIT_HDR_START = 0,
		WAIT_HDR_TYPE = 1,
		WAIT_LEN = 2,
		WAIT_END = 3,
	};

	uint8_t		  m_exFrameCnt;

	enPacketState m_state;
	bool          m_bChannelData;
	bool          m_bReleaseBus;
	uint8_t       m_nPacketLen;
	uint8_t       m_nBytes;
	uint8_t       m_nPacketId;
	uint8_t       m_exBusBuffer[64];  // 7 bytes header + 2 bytes crc + 24*2bytes channels
	void ResetPacket() { m_state = WAIT_HDR_START; m_bChannelData = false;  m_bReleaseBus = false;  m_nPacketLen = 0; m_nBytes = 0; m_nPacketId = 0; }

	// channel and button data
	uint8_t  m_nButtons;
	bool     m_bNewChannelData;
	uint8_t  m_nNumChannels;
	uint16_t m_channelValues[24];

    uint32_t _last_frame_time_us;
    uint32_t _last_tx_frame_time_us;
    uint32_t _last_uart_start_time_ms;
    uint32_t _last_rx_frame_time_us;
    uint32_t _start_frame_time_us;

	// jetibox text buffer
	char m_textBuffer[32];

	// lib is now sending on bus
	bool m_bBusReleased;

	// helpers
	void DecodeChannelData();
	void SendJetiBoxData();
	void SendTelemetryData();
	bool ReceiveCRCCheck();
	uint16_t crc_ccitt_update(uint16_t crc, uint8_t data);

	// debug 
	void DumpPacket();
    void DumpChar(char c);

    uint32_t HasNewChannelData() { bool b = m_bNewChannelData; m_bNewChannelData = false; return b; }
	uint8_t  GetNumChannels() { return m_nNumChannels; }
	uint16_t GetChannel(uint8_t nChannel);

private:
    const uint8_t MAX_CHANNELS = MIN((uint8_t)EXBUS_MAX_CHANNELS, (uint8_t)MAX_RCIN_CHANNELS);

    static AP_RCProtocol_EXBUS* _singleton;

    void _process_byte(uint8_t byte);
    // SoftSerial ss{EXBUS_BAUDRATE, SoftSerial::SERIAL_CONFIG_8N1};
    uint32_t saved_width;
    const bool inverted = false;

    AP_HAL::UARTDriver* get_current_UART() { return (_uart ? _uart : get_available_UART()); }
    
    AP_HAL::UARTDriver *_uart;

    void start_uart();
};

namespace AP {
    AP_RCProtocol_EXBUS* exbus();
};













#endif  // AP_RCPROTOCOL_EXBUS_ENABLED