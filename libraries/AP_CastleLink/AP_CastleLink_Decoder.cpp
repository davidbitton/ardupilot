/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#include "AP_CastleLink_config.h"

#if AP_CASTLELINK_ENABLED

#include "AP_CastleLink_Decoder.h"

#include <cmath>

bool AP_CastleLink_Decoder::is_missing(float tick_ms)
{
    return !std::isfinite(tick_ms) || tick_ms < 0.0f;
}

bool AP_CastleLink_Decoder::ntc_to_degC(float ntc_value, float &deg_c)
{
    // value==255 divides by zero; value<=0 is not a valid thermistor reading
    if (!std::isfinite(ntc_value) || ntc_value <= 0.0f || ntc_value >= 255.0f) {
        return false;
    }

    const float ratio = (ntc_value * NTC_R2) / ((255.0f - ntc_value) * NTC_R0);
    if (!(ratio > 0.0f)) {
        return false;
    }

    const float inv_t = (std::log(ratio) / NTC_B) + (1.0f / NTC_T0);
    if (!std::isfinite(inv_t) || inv_t == 0.0f) {
        return false;
    }

    deg_c = (1.0f / inv_t) - 273.0f;
    return std::isfinite(deg_c);
}

bool AP_CastleLink_Decoder::calibrate(const float ticks_ms[PACKET_ITEMS],
                                      float &t05_meas, float &t1_meas)
{
    if (is_missing(ticks_ms[ITEM_CAL_1MS])) {
        return false;
    }
    t1_meas = ticks_ms[ITEM_CAL_1MS];

    // 1.0 ms reference should land near 1 ms on a sane local clock
    if (t1_meas < 0.5f || t1_meas > 2.0f) {
        return false;
    }

    const bool have_lin = !is_missing(ticks_ms[ITEM_TEMP_LINEAR]);
    const bool have_ntc = !is_missing(ticks_ms[ITEM_TEMP_NTC]);
    if (!have_lin && !have_ntc) {
        return false;
    }

    if (have_lin && have_ntc) {
        t05_meas = (ticks_ms[ITEM_TEMP_LINEAR] < ticks_ms[ITEM_TEMP_NTC]) ?
                   ticks_ms[ITEM_TEMP_LINEAR] : ticks_ms[ITEM_TEMP_NTC];
    } else if (have_lin) {
        t05_meas = ticks_ms[ITEM_TEMP_LINEAR];
    } else {
        t05_meas = ticks_ms[ITEM_TEMP_NTC];
    }

    // 0.5 ms reference must be shorter than the 1.0 ms reference
    if (!(t05_meas > 0.2f) || !(t05_meas < t1_meas)) {
        return false;
    }

    return true;
}

bool AP_CastleLink_Decoder::to_calibrated(float tick_ms, float t05_meas, float t1_meas,
                                          float &calibrated_ms)
{
    if (is_missing(tick_ms)) {
        return false;
    }
    const float span = t1_meas - t05_meas;
    if (!(span > 0.0f)) {
        return false;
    }
    calibrated_ms = ZERO_OFFSET_MS + ZERO_OFFSET_MS * (tick_ms - t05_meas) / span;
    return std::isfinite(calibrated_ms);
}

bool AP_CastleLink_Decoder::convert_field(float tick_ms, float t05_meas, float t1_meas,
                                          float scale, float max_pulse_ms,
                                          float &value)
{
    float calibrated_ms;
    if (!to_calibrated(tick_ms, t05_meas, t1_meas, calibrated_ms)) {
        return false;
    }
    if (calibrated_ms < TICK_FLOOR_MS) {
        return false;
    }
    if (calibrated_ms > (ZERO_OFFSET_MS + max_pulse_ms + TICK_SLACK_MS)) {
        return false;
    }

    float corrected_ms = calibrated_ms - ZERO_OFFSET_MS;
    if (corrected_ms < 0.0f) {
        corrected_ms = 0.0f;
    }
    value = corrected_ms * scale;
    return std::isfinite(value);
}

bool AP_CastleLink_Decoder::decode(const float ticks_ms[PACKET_ITEMS], Packet &out)
{
    out = Packet{};

    // item 0 is RESET: a tick here means we are not packet-aligned
    if (!is_missing(ticks_ms[ITEM_RESET])) {
        return false;
    }

    float t05_meas, t1_meas;
    if (!calibrate(ticks_ms, t05_meas, t1_meas)) {
        return false;
    }

    out.has_voltage = convert_field(ticks_ms[ITEM_VOLTAGE], t05_meas, t1_meas,
                                    SCALE_VOLTAGE, MAX_PULSE_VOLTAGE, out.voltage_v);
    out.has_ripple = convert_field(ticks_ms[ITEM_RIPPLE], t05_meas, t1_meas,
                                   SCALE_RIPPLE, MAX_PULSE_RIPPLE, out.ripple_v);
    out.has_current = convert_field(ticks_ms[ITEM_CURRENT], t05_meas, t1_meas,
                                    SCALE_CURRENT, MAX_PULSE_CURRENT, out.current_a);
    out.has_throttle = convert_field(ticks_ms[ITEM_THROTTLE], t05_meas, t1_meas,
                                     SCALE_THROTTLE, MAX_PULSE_THROTTLE, out.throttle_ms);
    out.has_power = convert_field(ticks_ms[ITEM_POWER], t05_meas, t1_meas,
                                  SCALE_POWER, MAX_PULSE_POWER, out.power);
    out.has_erpm = convert_field(ticks_ms[ITEM_RPM], t05_meas, t1_meas,
                                 SCALE_RPM, MAX_PULSE_RPM, out.erpm);
    out.has_bec_v = convert_field(ticks_ms[ITEM_BEC_V], t05_meas, t1_meas,
                                  SCALE_BEC_V, MAX_PULSE_BEC_V, out.bec_v);
    out.has_bec_a = convert_field(ticks_ms[ITEM_BEC_I], t05_meas, t1_meas,
                                  SCALE_BEC_I, MAX_PULSE_BEC_I, out.bec_a);

    // valid temperature is the field whose calibrated tick is > 0.5 ms;
    // the other is the 0.5 ms calibration pulse
    float lin_cal = 0.0f, ntc_cal = 0.0f;
    const bool have_lin = to_calibrated(ticks_ms[ITEM_TEMP_LINEAR], t05_meas, t1_meas, lin_cal);
    const bool have_ntc = to_calibrated(ticks_ms[ITEM_TEMP_NTC], t05_meas, t1_meas, ntc_cal);
    const bool lin_valid = have_lin && (lin_cal > (ZERO_OFFSET_MS + TEMP_VALID_MARGIN_MS));
    const bool ntc_valid = have_ntc && (ntc_cal > (ZERO_OFFSET_MS + TEMP_VALID_MARGIN_MS));

    if (lin_valid && (!ntc_valid || lin_cal >= ntc_cal)) {
        float temp;
        if (convert_field(ticks_ms[ITEM_TEMP_LINEAR], t05_meas, t1_meas,
                          SCALE_TEMP_LINEAR, MAX_PULSE_TEMP_LINEAR, temp)) {
            out.temperature_c = temp;
            out.has_temperature = true;
        }
    } else if (ntc_valid) {
        float ntc_units;
        if (convert_field(ticks_ms[ITEM_TEMP_NTC], t05_meas, t1_meas,
                          SCALE_TEMP_NTC, MAX_PULSE_TEMP_NTC, ntc_units) &&
            ntc_to_degC(ntc_units, out.temperature_c)) {
            out.has_temperature = true;
        }
    }

    return true;
}

#endif  // AP_CASTLELINK_ENABLED
