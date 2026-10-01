/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#pragma once

#include <stdint.h>

/*
  Castle Serial Link TTL register protocol (v1.5).

  HAL-free codec. The Serial Link board speaks Castle Link Live to the
  ESC and this 5/3-byte register protocol on UART.

  Wiring (Serial Link 010-0121-00):
    pad A (TX) -> FC RX
    pad B (RX) -> FC TX
    pad D      -> FC throttle PWM (pass-through; do not steal this pin)
    3-pin      -> ESC throttle/RX
    GND common; +BUS from ESC BEC (not FC 5V if BEC is already connected)

  Command (5 bytes): start+device_id, register, data_hi, data_lo, checksum
    first byte = 0x80 | (device_id & 0x3F)   // start bit always 1, ID 0-63
    checksum   = (uint8_t)(0 - (b0+b1+b2+b3))  // five bytes sum to 0

  Response (3 bytes): data_hi, data_lo, checksum
  Error response: 0xFFFF (reject). Flush the command buffer with 5+ 0x00.

  Conversion: value = register_u16 / 2042.0 * scale
 */

class AP_CastleLink_Serial {
public:
    static constexpr uint8_t CMD_LEN = 5;
    static constexpr uint8_t RSP_LEN = 3;
    static constexpr uint8_t FLUSH_ZEROS = 8;
    static constexpr uint8_t DEVICE_ID_MAX = 63;
    static constexpr uint16_t ERROR_VALUE = 0xFFFF;
    static constexpr float DIVISOR = 2042.0f;

    enum Reg : uint8_t {
        REG_VOLTAGE     = 0,
        REG_RIPPLE      = 1,
        REG_CURRENT     = 2,
        REG_THROTTLE    = 3,
        REG_POWER       = 4,
        REG_SPEED       = 5,
        REG_TEMP        = 6,
        REG_BEC_V       = 7,
        REG_BEC_I       = 8,
        REG_RAW_NTC     = 9,
        REG_RAW_LINEAR  = 10,
        REG_THROTTLE_W  = 128,
    };

    static constexpr float SCALE_VOLTAGE = 20.0f;
    static constexpr float SCALE_RIPPLE = 4.0f;
    static constexpr float SCALE_CURRENT = 50.0f;
    static constexpr float SCALE_THROTTLE = 1.0f;
    static constexpr float SCALE_POWER = 0.2502f;
    static constexpr float SCALE_RPM = 20416.66f;
    static constexpr float SCALE_BEC_V = 4.0f;
    static constexpr float SCALE_BEC_I = 4.0f;
    static constexpr float SCALE_TEMP = 30.0f;
    static constexpr float SCALE_RAW_NTC = 63.8125f;
    static constexpr float SCALE_RAW_LINEAR = 30.0f;

    static constexpr uint8_t NUM_POLL_REGS = 6;
    static constexpr uint8_t POLL_REGS[NUM_POLL_REGS] = {
        REG_VOLTAGE, REG_CURRENT, REG_SPEED, REG_TEMP, REG_POWER, REG_THROTTLE
    };
    static constexpr uint32_t RX_TIMEOUT_MS = 20;

    static uint8_t pack_device_id(uint8_t device_id);
    static uint8_t checksum(const uint8_t *bytes, uint8_t n);
    static void encode_command(uint8_t device_id, uint8_t reg, uint16_t data,
                               uint8_t out[CMD_LEN]);

    enum class DecodeResult : uint8_t {
        Ok = 0,
        BadChecksum,
        DeviceError,
    };
    // leaves value unchanged unless Ok
    static DecodeResult decode_result(const uint8_t in[RSP_LEN], uint16_t &value);
    static bool decode_response(const uint8_t in[RSP_LEN], uint16_t &value);
    static float convert(uint16_t raw, float scale);

    // 1000 us -> 0, 2000 us -> 65535
    static uint16_t pwm_to_throttle_reg(uint16_t pwm_us);

    // slot in a poll period of NUM_POLL_REGS reads, plus one write if serth
    static bool poll_is_throttle_write(uint8_t poll_idx, bool serth);
    static uint8_t poll_read_reg(uint8_t poll_idx, bool serth);

    struct RxAction {
        bool waiting;
        bool discard;
        bool flush_zeros;
        bool have_value;
        uint16_t value;
    };
    // rsp used only when available >= RSP_LEN.
    // pending_reg: 0xFFFF is DeviceError/flush for reads; REG_THROTTLE_W echo of 65535 is full throttle.
    static RxAction consider_rx(uint32_t available, uint32_t elapsed_ms, const uint8_t *rsp,
                                uint8_t pending_reg = 0);

    struct MappedTelem {
        bool has_voltage;
        bool has_current;
        bool has_temperature;
        bool has_erpm;
        bool has_throttle;
        bool has_power;
        float voltage_v;
        float current_a;
        float temperature_c;
        int16_t temperature_cdeg;
        float erpm;
        float mech_rpm;
        float throttle_ms;
        uint8_t input_duty;
        float power;
        uint8_t power_pct;
    };
    static bool map_register(uint8_t reg, uint16_t raw, int8_t poles, MappedTelem &out);
};
