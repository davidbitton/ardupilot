#include "AP_RCProtocol_EXBUS.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_RCTelemetry/AP_RCTelemetry_config.h>
#include <AP_SerialManager/AP_SerialManager.h>

AP_RCProtocol_EXBUS* AP_RCProtocol_EXBUS::_singleton;

AP_RCProtocol_EXBUS::AP_RCProtocol_EXBUS(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend)
{
#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    if (_singleton != nullptr) {
        AP_HAL::panic("Duplicate EXBUS handler");
    }

    _singleton = this;
#else
    if (_singleton == nullptr) {
        _singleton = this;
    }
#endif

#if HAL_EXBUS_TELEM_ENABLED && !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_RCIN, 0);
    if (_uart) {
        start_uart();
    }
#endif

}

AP_RCProtocol_EXBUS::~AP_RCProtocol_EXBUS() {
    _singleton = nullptr;
}

// get the protocol string
const char* AP_RCProtocol_EXBUS::get_protocol_string() const {
    return "EXBUS";
}

void AP_RCProtocol_EXBUS::process_handshake(uint32_t baudrate) {

    AP_HAL::UARTDriver *uart = get_current_UART();

    // // only change the baudrate if we are specifically bootstrapping EXBUS
    if (uart == nullptr
        || baudrate != EXBUS_BAUDRATE
        || uart->get_baud_rate() == EXBUS_BAUDRATE
        || !protocol_enabled(AP_RCProtocol::EXBUS)) {
        return;
    }

    uart->begin(EXBUS_BAUDRATE);    
}

void AP_RCProtocol_EXBUS::update(void) {

	    // if we are in standalone mode, process data from the uart
    if (_uart) {
        uint32_t now = AP_HAL::millis();

        // for some reason it's necessary to keep trying to start the uart until we get data
        if (now - _last_uart_start_time_ms > 1000U && _last_frame_time_us == 0) {
            start_uart();
            _last_uart_start_time_ms = now;
        }
        uint32_t n = _uart->available();
        n = MIN(n, 255U);
        for (uint8_t i = 0; i < n; i++) {
            int16_t b = _uart->read();
            if (b >= 0) {
                _process_byte(uint8_t(b));
            }
        }
    }
}

void AP_RCProtocol_EXBUS::process_byte(uint8_t byte, uint32_t baudrate) {
	// if (baudrate != EXBUS_BAUDRATE) {
    //     return;
    // }
	
	_process_byte(byte);
}

void AP_RCProtocol_EXBUS::_process_byte(uint8_t c) {

	const uint32_t now = AP_HAL::micros();
    // int c = m_pSerial->read();
		 // DumpChar( (char)c );

		if (m_state == WAIT_HDR_START)
		{
			if (c == 0x3d || c == 0x3e)
			{
				m_state = WAIT_HDR_TYPE;
				m_bChannelData = (c == 0x3e) ? true : false;
				m_exBusBuffer[0] = c;
				// Serial.println("start");
			}
		}
		else if (m_state == WAIT_HDR_TYPE)
		{
			if (c == 0x01 || c == 0x03 )
			{
				m_state = WAIT_LEN;
				m_bReleaseBus = (c == 0x01) ? true : false;
				m_exBusBuffer[1] = c;
				// Serial.println("type");
			}
			else
			{
				m_state = WAIT_HDR_START; // --> Error
			}
		}
		else if ( m_state == WAIT_LEN)
		{
			m_state = WAIT_END;
			m_nPacketLen = (uint8_t)c;
			m_exBusBuffer[2] = c;
			m_nBytes = 3;
			if( m_nPacketLen > sizeof( m_exBusBuffer) )
			{
				m_state = WAIT_HDR_START; // --> Error
			}
			// Serial.print("len - "); Serial.println( m_nPacketLen );
		}
		else if ( m_state == WAIT_END )
		{
			m_exBusBuffer[m_nBytes++] = c;
			if ( m_nBytes >= m_nPacketLen )
			{
				if (ReceiveCRCCheck() )
				{
					//  DumpPacket();
					m_nPacketId = m_exBusBuffer[3];
					
					// packet contains channel data 
					if (m_bChannelData && ( m_exBusBuffer[4] == 0x31 ) )
					{
						DecodeChannelData();

						if(HasNewChannelData()) {
							// uint8_t num_channels = GetNumChannels();
							// uint16_t data[num_channels];

							// for (uint8_t i = 0; i < num_channels; i++) {
							// 	data[i] = GetChannel(i);// / 8;
							// }

							// int16_t rssi = 100;
							// int16_t rx_link_quality = 100;
							add_input(m_nNumChannels, (uint16_t*)m_channelValues, false);	//TODO: need to add RSSI/LQ
						}
					}
					// packet is a telemetry request
					else if( m_exBusBuffer[ 4 ] == 0x3a && m_bReleaseBus )
					{
//						digitalWrite(PIN_A0, HIGH);
						SendTelemetryData();
						m_bBusReleased = true;
//						digitalWrite(PIN_A0, LOW);
					}
					// packet is a Jetibox request
					else if (m_exBusBuffer[4] == 0x3b && m_bReleaseBus )
					{
						SendJetiBoxData();
						m_bBusReleased = true;
					}
				}
				m_state = WAIT_HDR_START;
			}
		}
		
		_last_frame_time_us = _last_rx_frame_time_us = now;

}

// start the uart if we have one
void AP_RCProtocol_EXBUS::start_uart()
{

    _uart->configure_parity(0);
    _uart->set_stop_bits(1);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    // _uart->set_options(uart->get_options() & ~AP_HAL::UARTDriver::OPTION_RXINV);
    _uart->begin(EXBUS_BAUDRATE);
}

// void AP_RCProtocol_EXBUS::process_pulse(uint32_t width_s0, uint32_t width_s1) {
// 	    if (have_UART()) {
//         // if we can use a UART we would much prefer to, as it allows
//         // us to send SPORT data out
//         return;
//     }
//     uint32_t w0 = width_s0;
//     uint32_t w1 = width_s1;
//     if (inverted) {
//         w0 = saved_width;
//         w1 = width_s0;
//         saved_width = width_s1;
//     }
//     uint8_t b;
//     if (ss.process_pulse(w0, w1, b)) {
//         _process_byte(ss.get_byte_timestamp_us(), b);
//     }
// }

void AP_RCProtocol_EXBUS::DecodeChannelData()
{
	// ::printf("%s::DecodeChannelData\n", __FILE__);

	m_nNumChannels = m_exBusBuffer[5] / 2;  // number of channels

	for (int i = 0; i < m_nNumChannels; i++)
		m_channelValues[i] = ((m_exBusBuffer[6 + (i << 1)]) | (m_exBusBuffer[7 + (i << 1)] << 8 )) >> 3;   

	m_bNewChannelData = true;
}

void AP_RCProtocol_EXBUS::SendJetiBoxData() {}

uint16_t AP_RCProtocol_EXBUS::GetChannel(uint8_t nChannel)
{
	if( nChannel < JETIEXBUS_COUNT_OF( m_channelValues ) )
	  return m_channelValues[ nChannel ];
	return 0;
}

void AP_RCProtocol_EXBUS::SendTelemetryData() {

}

bool AP_RCProtocol_EXBUS::ReceiveCRCCheck()
{
	// calc crc...
	uint16_t crcCalc = 0;
	crcCalc = crc16_ccitt_r(m_exBusBuffer, m_nBytes, crcCalc, 0);

	return (crcCalc == 0);
}

// debug 
//////////////////////
#ifdef JEXTIEXBUS_PROTOCOL_DEBUG
	void JetiExBusProtocol::DumpPacket()
	{
		Serial.println("");
		Serial.println("--- dump start ---");
		for (int i = 0; i < m_nBytes; i++)
			DumpChar(m_exBusBuffer[i]);
		Serial.println("");
		Serial.println("--- dump end ---");
	}

	void JetiExBusProtocol::DumpChar(char c)
	{
		char buf[5];
		int idx = 0;

		buf[idx++] = '0';
		buf[idx++] = 'x';
		itoa(((int)c) & 0x00FF, &buf[idx], 16);
		idx += 2;
		buf[idx++] = ' ';

		if (idx > 0 && idx < (int)sizeof( buf ) )
			buf[idx++] = '\0';

		Serial.println(buf);
	}
#else
void AP_RCProtocol_EXBUS::DumpPacket() {}
void AP_RCProtocol_EXBUS::DumpChar(char c) {}
#endif // JEXTIEXBUS_PROTOCOL_DEBUG

namespace AP {
    AP_RCProtocol_EXBUS* exbus() {
        return AP_RCProtocol_EXBUS::get_singleton();
    }
};