/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#pragma once

#include <stdint.h>

/*
  Castle Link Live 2.0 protocol decoder.

  Pure math: no HAL. Feed 12 tick times in milliseconds (one per
  inverted-PWM frame). A missing tick (RESET, or no capture) is
  TICK_NONE / NaN / any non-finite or negative value.

  Packet order (spec v2.0):
    0 RESET (no tick)
    1 1.0 ms calibration
    2 Voltage
    3 Ripple voltage
    4 Current
    5 Throttle (ms as measured by the ESC)
    6 Output power (0..1)
    7 Electrical RPM
    8 BEC voltage
    9 BEC current
   10 Linear temperature, or 0.5 ms calibration
   11 NTC temperature, or 0.5 ms calibration

  Two-point calibration uses item 1 as 1.0 ms and the smaller of
  items 10/11 as 0.5 ms.

  Wiring (driver, not this decoder): inverted RC PWM on the throttle
  wire, line idles high, go high-Z after the rising edge so the ESC
  can pull a ~1 us tick. External pull-up required (ESC has 6.65 kOhm
  pull-down): <=2 kOhm at 3.3 V.
 */

class AP_CastleLink_Decoder {
public:
    static constexpr uint8_t PACKET_ITEMS = 12;
    static constexpr float TICK_NONE = -1.0f;

    enum Item : uint8_t {
        ITEM_RESET = 0,
        ITEM_CAL_1MS = 1,
        ITEM_VOLTAGE = 2,
        ITEM_RIPPLE = 3,
        ITEM_CURRENT = 4,
        ITEM_THROTTLE = 5,
        ITEM_POWER = 6,
        ITEM_RPM = 7,
        ITEM_BEC_V = 8,
        ITEM_BEC_I = 9,
        ITEM_TEMP_LINEAR = 10,
        ITEM_TEMP_NTC = 11,
    };

    // scale: live_value = (calibrated_ms - 0.5) * scale
    static constexpr float SCALE_VOLTAGE = 20.0f;        // V/ms
    static constexpr float SCALE_RIPPLE = 4.0f;          // V/ms
    static constexpr float SCALE_CURRENT = 50.0f;        // A/ms
    static constexpr float SCALE_THROTTLE = 1.0f;        // ms/ms
    static constexpr float SCALE_POWER = 0.2502f;        // fraction/ms
    static constexpr float SCALE_RPM = 20416.7f;         // eRPM/ms
    static constexpr float SCALE_BEC_V = 4.0f;           // V/ms
    static constexpr float SCALE_BEC_I = 4.0f;           // A/ms
    static constexpr float SCALE_TEMP_LINEAR = 30.0f;    // degC/ms
    static constexpr float SCALE_TEMP_NTC = 63.8125f;    // units/ms (0..255)

    // max pulse length (ms) after the 0.5 ms zero-offset
    static constexpr float MAX_PULSE_VOLTAGE = 5.0f;
    static constexpr float MAX_PULSE_RIPPLE = 5.0f;
    static constexpr float MAX_PULSE_CURRENT = 5.0f;
    static constexpr float MAX_PULSE_THROTTLE = 2.5f;
    static constexpr float MAX_PULSE_POWER = 4.0f;
    static constexpr float MAX_PULSE_RPM = 5.0f;
    static constexpr float MAX_PULSE_BEC_V = 5.0f;
    static constexpr float MAX_PULSE_BEC_I = 5.0f;
    static constexpr float MAX_PULSE_TEMP_LINEAR = 5.0f;
    static constexpr float MAX_PULSE_TEMP_NTC = 4.0f;

    struct Packet {
        float voltage_v;
        float ripple_v;
        float current_a;
        float throttle_ms;
        float power;          // 0..1
        float erpm;
        float bec_v;
        float bec_a;
        float temperature_c;

        bool has_voltage;
        bool has_ripple;
        bool has_current;
        bool has_throttle;
        bool has_power;
        bool has_erpm;
        bool has_bec_v;
        bool has_bec_a;
        bool has_temperature;
    };

    // true if a tick slot has no capture (RESET / timeout)
    static bool is_missing(float tick_ms);

    // Steinhart-style NTC conversion. Returns false if value is not in (0, 255)
    // or the result is not finite.
    static bool ntc_to_degC(float ntc_value, float &deg_c);

    // Decode one 12-item packet. Returns false if RESET/calibration is unusable.
    // Implausible per-item ticks clear that field's has_* flag rather than
    // failing the whole packet.
    static bool decode(const float ticks_ms[PACKET_ITEMS], Packet &out);

private:
    static constexpr float ZERO_OFFSET_MS = 0.5f;
    static constexpr float CAL_1MS = 1.0f;
    static constexpr float TICK_FLOOR_MS = 0.45f;   // calibrated ticks below this are rejected
    static constexpr float TICK_SLACK_MS = 0.1f;    // allowed overshoot of table max pulse
    static constexpr float TEMP_VALID_MARGIN_MS = 0.01f;

    static constexpr float NTC_R0 = 10000.0f;
    static constexpr float NTC_R2 = 10200.0f;
    static constexpr float NTC_B = 3455.0f;
    static constexpr float NTC_T0 = 298.0f;

    static bool calibrate(const float ticks_ms[PACKET_ITEMS],
                          float &t05_meas, float &t1_meas);
    static bool to_calibrated(float tick_ms, float t05_meas, float t1_meas,
                              float &calibrated_ms);
    static bool convert_field(float tick_ms, float t05_meas, float t1_meas,
                              float scale, float max_pulse_ms,
                              float &value);
};
