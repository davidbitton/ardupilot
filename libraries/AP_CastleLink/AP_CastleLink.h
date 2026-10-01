/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#pragma once

#include "AP_CastleLink_config.h"

#if AP_CASTLELINK_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include "AP_CastleLink_Decoder.h"
#include "AP_CastleLink_Serial.h"

/*
  Castle ESC telemetry.

  Primary: Castle Serial Link (010-0121-00) on a UART
    (SerialProtocol_CastleLink). Pad A (TX) -> FC RX, pad B (RX) ->
    FC TX, pad D = PWM pass-through from a normal servo output, 3-pin
    = ESC, common GND, +BUS from ESC BEC. Do not steal the throttle
    pin. Default baud 115200, no flow control.

  Also: inverted PWM + tick on the throttle wire when no Serial Link
  UART is configured. See AP_CastleLink_Decoder.h. Pulse path cannot
  use IOMCU MAIN outputs (pins 101-108).
 */

class AP_CastleLink : public AP_ESC_Telem_Backend {
public:
    AP_CastleLink();

    CLASS_NO_COPY(AP_CastleLink);

    static const struct AP_Param::GroupInfo var_info[];

    static AP_CastleLink *get_singleton() {
        return _singleton;
    }

    // called from SRV_Channels::push()
    void update();

private:
    static AP_CastleLink *_singleton;

    AP_Int8 _enable;
    AP_Int8 _channel;     // 1-based servo: pulse GPIO, or SERTH PWM source (0 = k_throttle)
    AP_Int8 _esc_index;   // AP_ESC_Telem index
    AP_Int8 _poles;
    AP_Int8 _device_id;   // Serial Link device ID 0-63
    AP_Int8 _serial_thr;  // write register 128 (default off)

    bool _initialised;
    bool _gpio_ok;
    uint8_t _gpio_pin;
    uint8_t _chan0;       // 0-based servo channel

    AP_HAL::UARTDriver *_uart;  // nullptr unless SerialProtocol_CastleLink is configured
    uint8_t _poll_idx;
    bool _awaiting;
    uint32_t _await_ms;
    uint8_t _pending_reg;

    float _ticks_ms[AP_CastleLink_Decoder::PACKET_ITEMS];
    uint8_t _item;
    bool _have_packet;
    AP_CastleLink_Decoder::Packet _packet;
    HAL_Semaphore _sem;

    void init();
    bool init_uart();
    void uart_update();
    bool send_cmd(uint8_t reg, uint16_t data);
    void handle_register(uint8_t reg, uint16_t raw);
    bool lookup_serth_pwm(uint16_t &pwm);
    bool resolve_gpio_pin();
    void disable_hw_pwm();
    void thread();
    void emit_pulse_and_capture(uint16_t pulse_us, float &tick_ms);
    void handle_packet(const AP_CastleLink_Decoder::Packet &pkt);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    void sitl_update();
#endif
};

#endif  // AP_CASTLELINK_ENABLED
