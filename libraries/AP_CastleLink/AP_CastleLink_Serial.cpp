/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#include "AP_CastleLink_config.h"

#if AP_CASTLELINK_ENABLED

#include "AP_CastleLink_Serial.h"

constexpr uint8_t AP_CastleLink_Serial::NUM_POLL_REGS;
constexpr uint8_t AP_CastleLink_Serial::POLL_REGS[AP_CastleLink_Serial::NUM_POLL_REGS];

static_assert(sizeof(AP_CastleLink_Serial::POLL_REGS) == AP_CastleLink_Serial::NUM_POLL_REGS,
              "POLL_REGS size");

uint8_t AP_CastleLink_Serial::pack_device_id(uint8_t device_id)
{
    return uint8_t(0x80u | (device_id & DEVICE_ID_MAX));
}

uint8_t AP_CastleLink_Serial::checksum(const uint8_t *bytes, uint8_t n)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum = uint8_t(sum + bytes[i]);
    }
    return uint8_t(0u - sum);
}

void AP_CastleLink_Serial::encode_command(uint8_t device_id, uint8_t reg, uint16_t data,
                                          uint8_t out[CMD_LEN])
{
    out[0] = pack_device_id(device_id);
    out[1] = reg;
    out[2] = uint8_t(data >> 8);
    out[3] = uint8_t(data);
    out[4] = checksum(out, 4);
}

AP_CastleLink_Serial::DecodeResult AP_CastleLink_Serial::decode_result(const uint8_t in[RSP_LEN], uint16_t &value)
{
    if (checksum(in, RSP_LEN) != 0) {
        return DecodeResult::BadChecksum;
    }
    const uint16_t raw = uint16_t((uint16_t(in[0]) << 8) | in[1]);
    if (raw == ERROR_VALUE) {
        return DecodeResult::DeviceError;
    }
    value = raw;
    return DecodeResult::Ok;
}

bool AP_CastleLink_Serial::decode_response(const uint8_t in[RSP_LEN], uint16_t &value)
{
    return decode_result(in, value) == DecodeResult::Ok;
}

float AP_CastleLink_Serial::convert(uint16_t raw, float scale)
{
    return (float(raw) / DIVISOR) * scale;
}

uint16_t AP_CastleLink_Serial::pwm_to_throttle_reg(uint16_t pwm_us)
{
    if (pwm_us < 1000) {
        pwm_us = 1000;
    } else if (pwm_us > 2000) {
        pwm_us = 2000;
    }
    return uint16_t(((uint32_t(pwm_us) - 1000U) * 65535U) / 1000U);
}

bool AP_CastleLink_Serial::poll_is_throttle_write(uint8_t poll_idx, bool serth)
{
    if (!serth) {
        return false;
    }
    const uint8_t period = NUM_POLL_REGS + 1;
    return (poll_idx % period) == NUM_POLL_REGS;
}

uint8_t AP_CastleLink_Serial::poll_read_reg(uint8_t poll_idx, bool serth)
{
    if (poll_is_throttle_write(poll_idx, serth)) {
        return POLL_REGS[0];
    }
    const uint8_t period = serth ? (NUM_POLL_REGS + 1) : NUM_POLL_REGS;
    return POLL_REGS[poll_idx % period];
}

AP_CastleLink_Serial::RxAction AP_CastleLink_Serial::consider_rx(uint32_t available, uint32_t elapsed_ms, const uint8_t *rsp,
                                                                uint8_t pending_reg)
{
    RxAction act{};
    if (available >= RSP_LEN) {
        act.discard = true;
        const DecodeResult dr = decode_result(rsp, act.value);
        if (dr == DecodeResult::Ok) {
            act.have_value = true;
        } else if (dr == DecodeResult::DeviceError && pending_reg == REG_THROTTLE_W) {
            // write echo of 65535 is 2.0 ms full throttle, not a protocol error
            act.value = ERROR_VALUE;
            act.have_value = true;
        } else if (dr == DecodeResult::DeviceError) {
            act.flush_zeros = true;
        }
        return act;
    }
    if (elapsed_ms > RX_TIMEOUT_MS) {
        act.discard = true;
        return act;
    }
    act.waiting = true;
    return act;
}

bool AP_CastleLink_Serial::map_register(uint8_t reg, uint16_t raw, int8_t poles, MappedTelem &out)
{
    out = MappedTelem{};
    switch (reg) {
    case REG_VOLTAGE:
        out.voltage_v = convert(raw, SCALE_VOLTAGE);
        out.has_voltage = true;
        return true;
    case REG_CURRENT:
        out.current_a = convert(raw, SCALE_CURRENT);
        out.has_current = true;
        return true;
    case REG_TEMP:
        out.temperature_c = convert(raw, SCALE_TEMP);
        out.temperature_cdeg = int16_t(out.temperature_c * 100.0f);
        out.has_temperature = true;
        return true;
    case REG_SPEED:
        out.erpm = convert(raw, SCALE_RPM);
        out.mech_rpm = (poles >= 2) ? (out.erpm * 2.0f / float(poles)) : out.erpm;
        out.has_erpm = true;
        return true;
    case REG_THROTTLE: {
        out.throttle_ms = convert(raw, SCALE_THROTTLE);
        float duty = (out.throttle_ms - 1.0f) * 100.0f;
        if (duty < 0.0f) {
            duty = 0.0f;
        } else if (duty > 100.0f) {
            duty = 100.0f;
        }
        out.input_duty = uint8_t(duty);
        out.has_throttle = true;
        return true;
    }
    case REG_POWER: {
        out.power = convert(raw, SCALE_POWER);
        float pct = out.power * 100.0f + 0.5f;
        if (pct < 0.0f) {
            pct = 0.0f;
        } else if (pct > 100.0f) {
            pct = 100.0f;
        }
        out.power_pct = uint8_t(pct);
        out.has_power = true;
        return true;
    }
    default:
        return false;
    }
}

#endif  // AP_CASTLELINK_ENABLED
