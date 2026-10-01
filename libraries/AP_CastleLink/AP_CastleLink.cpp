/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#include "AP_CastleLink.h"

#if AP_CASTLELINK_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_SerialManager/AP_SerialManager_config.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

extern const AP_HAL::HAL& hal;

using Dec = AP_CastleLink_Decoder;

const AP_Param::GroupInfo AP_CastleLink::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Castle Link enable
    // @Description: Enable Castle ESC telemetry. Preferred path is a Castle Serial Link on a UART (set SERIALx_PROTOCOL to CastleLink). Rover keeps sending normal PWM to Serial Link pad D. The pulse/tick path (SERVO_CLL_CHAN) is only for a bare Link Live throttle wire.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_CastleLink, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CHAN
    // @DisplayName: Castle Link Live servo channel
    // @Description: 1-based servo channel. Pulse path: throttle/telemetry pin (0 disables pulse path; cannot be an IOMCU MAIN output). When SERVO_CLL_SERTH=1, this channel supplies the PWM for register 128; 0 means the throttle function. PWM pass-through to Serial Link pad D uses a normal servo output and does not need this param.
    // @Range: 0 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("CHAN", 2, AP_CastleLink, _channel, 0),

    // @Param: IDX
    // @DisplayName: Castle Link ESC index
    // @Description: ESC telemetry index (0-based) used when publishing voltage, current, temperature and RPM to AP_ESC_Telem.
    // @Range: 0 31
    // @User: Advanced
    AP_GROUPINFO("IDX", 3, AP_CastleLink, _esc_index, 0),

    // @Param: POLES
    // @DisplayName: Castle Link motor pole count
    // @Description: Number of motor electrical poles. Mechanical RPM = eRPM * 2 / POLES.
    // @Range: 2 50
    // @User: Standard
    AP_GROUPINFO("POLES", 4, AP_CastleLink, _poles, 14),

    // @Param: DEVID
    // @DisplayName: Castle Serial Link device ID
    // @Description: Serial Link device ID (0-63) packed into the command start byte.
    // @Range: 0 63
    // @User: Advanced
    AP_GROUPINFO("DEVID", 5, AP_CastleLink, _device_id, 0),

    // @Param: SERTH
    // @DisplayName: Castle Serial Link throttle write
    // @Description: If 1, write register 128 from throttle PWM (0=1.0ms, 65535=2.0ms) once per 7-slot poll (six telem reads + one write; about loop_rate/7). Not 100 Hz. Requires TTL Serial without PPM/analog pass-through; do not also drive pad D. Default 0: UART is telemetry only and pad D takes normal PWM.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SERTH", 6, AP_CastleLink, _serial_thr, 0),

    AP_GROUPEND
};

AP_CastleLink *AP_CastleLink::_singleton;

AP_CastleLink::AP_CastleLink()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_CastleLink must be singleton");
    }
#endif
    _singleton = this;
    _uart = nullptr;
}

bool AP_CastleLink::init_uart()
{
#if AP_SERIALMANAGER_ENABLED
    auto &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CastleLink, 0);
    if (_uart == nullptr) {
        return false;
    }
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    const uint32_t baud = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_CastleLink, 0);
    const uint32_t use_baud = (baud > 0) ? baud : AP_SERIALMANAGER_CASTLELINK_BAUD;
    _uart->begin(use_baud, AP_SERIALMANAGER_CASTLELINK_BUFSIZE_RX, AP_SERIALMANAGER_CASTLELINK_BUFSIZE_TX);
    const uint8_t zeros[AP_CastleLink_Serial::FLUSH_ZEROS] {};
    _uart->write(zeros, sizeof(zeros));
    _uart->discard_input();
    return true;
#else
    return false;
#endif
}

void AP_CastleLink::init()
{
    if (_initialised) {
        return;
    }
    _initialised = true;

    if (_enable.get() <= 0) {
        return;
    }

    if (init_uart()) {
        return;
    }

    const int8_t chan1 = _channel.get();
    if (chan1 < 1 || chan1 > NUM_SERVO_CHANNELS) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CastleLink: no UART and SERVO_CLL_CHAN=0");
        return;
    }
    _chan0 = uint8_t(chan1 - 1);

    for (uint8_t i = 0; i < Dec::PACKET_ITEMS; i++) {
        _ticks_ms[i] = Dec::TICK_NONE;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    return;
#else
    if (!resolve_gpio_pin()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CastleLink: no GPIO for SERVO%u", unsigned(chan1));
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CastleLink::thread, void),
                                      "castle",
                                      2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CastleLink: thread failed");
        return;
    }
    disable_hw_pwm();
    _gpio_ok = true;
#endif
}

bool AP_CastleLink::resolve_gpio_pin()
{
    for (uint16_t pin = 1; pin < 256; pin++) {
        uint8_t servo_ch;
        if (!hal.gpio->pin_to_servo_channel(uint8_t(pin), servo_ch) || servo_ch != _chan0) {
            continue;
        }
        // IOMCU MAIN 101-108 cannot pinMode/wait_pin
        if (pin >= 101 && pin <= 108) {
            return false;
        }
        if (!hal.gpio->valid_pin(uint8_t(pin))) {
            return false;
        }
        _gpio_pin = uint8_t(pin);
        return true;
    }
    return false;
}

void AP_CastleLink::disable_hw_pwm()
{
    uint32_t mask = SRV_Channels::get_disabled_channel_mask();
    mask |= (1U << _chan0);
    SRV_Channels::set_disabled_channel_mask(mask);
    hal.rcout->disable_ch(_chan0);
}

void AP_CastleLink::emit_pulse_and_capture(uint16_t pulse_us, float &tick_ms)
{
    tick_ms = Dec::TICK_NONE;
    if (pulse_us < 800) {
        pulse_us = 800;
    } else if (pulse_us > 2200) {
        pulse_us = 2200;
    }

    // inverted PWM: idle high, drive low for the throttle width, then
    // totem-pole high and immediately high-Z for the ESC tick
    hal.gpio->pinMode(_gpio_pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(_gpio_pin, 1);
    const uint32_t t_end = AP_HAL::micros() + pulse_us;
    hal.gpio->write(_gpio_pin, 0);
    while ((int32_t)(AP_HAL::micros() - t_end) < 0) {
    }
    hal.gpio->write(_gpio_pin, 1);
    const uint32_t t_edge = AP_HAL::micros() + 2;
    while ((int32_t)(AP_HAL::micros() - t_edge) < 0) {
    }
    hal.gpio->pinMode(_gpio_pin, HAL_GPIO_INPUT);

    const uint32_t t0 = AP_HAL::micros();
    // longest table entry is 5 ms + 0.5 ms offset; 7 ms covers RESET (no tick)
    const bool got = hal.gpio->wait_pin(_gpio_pin, AP_HAL::GPIO::INTERRUPT_FALLING, 7000);
    const uint32_t dt = AP_HAL::micros() - t0;
    if (got && dt >= 200 && dt <= 7000) {
        tick_ms = dt * 1.0e-3f;
    }

    // idle high for the rest of the 20 ms frame
    hal.gpio->pinMode(_gpio_pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(_gpio_pin, 1);
}

void AP_CastleLink::thread()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(10);
    }

    while (true) {
        const uint32_t t_frame = AP_HAL::micros();

        uint16_t pwm = 1500;
        SRV_Channels::get_output_pwm_chan(_chan0, pwm);

        float tick_ms;
        emit_pulse_and_capture(pwm, tick_ms);

        if (Dec::is_missing(tick_ms)) {
            _ticks_ms[0] = Dec::TICK_NONE;
            _item = 1;
        } else if (_item != 0) {
            _ticks_ms[_item] = tick_ms;
            _item++;
            if (_item >= Dec::PACKET_ITEMS) {
                Dec::Packet pkt{};
                if (Dec::decode(_ticks_ms, pkt)) {
                    WITH_SEMAPHORE(_sem);
                    _packet = pkt;
                    _have_packet = true;
                }
                _item = 0;
                for (uint8_t i = 0; i < Dec::PACKET_ITEMS; i++) {
                    _ticks_ms[i] = Dec::TICK_NONE;
                }
            }
        }

        const uint32_t elapsed = AP_HAL::micros() - t_frame;
        if (elapsed < 20000) {
            hal.scheduler->delay_microseconds(20000 - elapsed);
        }
    }
}

bool AP_CastleLink::send_cmd(uint8_t reg, uint16_t data)
{
    uint8_t cmd[AP_CastleLink_Serial::CMD_LEN];
    AP_CastleLink_Serial::encode_command(uint8_t(_device_id.get()), reg, data, cmd);
    if (_uart->write(cmd, sizeof(cmd)) != sizeof(cmd)) {
        const uint8_t zeros[AP_CastleLink_Serial::FLUSH_ZEROS] {};
        _uart->write(zeros, sizeof(zeros));
        _uart->discard_input();
        return false;
    }
    _pending_reg = reg;
    _awaiting = true;
    _await_ms = AP_HAL::millis();
    return true;
}

bool AP_CastleLink::lookup_serth_pwm(uint16_t &pwm)
{
    const int8_t chan1 = _channel.get();
    if (chan1 >= 1 && chan1 <= NUM_SERVO_CHANNELS) {
        return SRV_Channels::get_output_pwm_chan(uint8_t(chan1 - 1), pwm);
    }
    return SRV_Channels::get_output_pwm(SRV_Channel::k_throttle, pwm);
}

void AP_CastleLink::handle_register(uint8_t reg, uint16_t raw)
{
    const uint8_t idx = uint8_t(_esc_index.get());
    if (idx >= ESC_TELEM_MAX_ESCS) {
        return;
    }

    AP_CastleLink_Serial::MappedTelem m{};
    if (!AP_CastleLink_Serial::map_register(reg, raw, _poles.get(), m)) {
        return;
    }

    if (m.has_erpm) {
        update_rpm(idx, m.mech_rpm);
        return;
    }

    TelemetryData t {};
    uint16_t mask = 0;
    if (m.has_voltage) {
        t.voltage = m.voltage_v;
        mask |= TelemetryType::VOLTAGE;
    }
    if (m.has_current) {
        t.current = m.current_a;
        mask |= TelemetryType::CURRENT;
    }
    if (m.has_temperature) {
        t.temperature_cdeg = m.temperature_cdeg;
        mask |= TelemetryType::TEMPERATURE;
    }
#if AP_EXTENDED_ESC_TELEM_ENABLED
    if (m.has_throttle) {
        t.input_duty = m.input_duty;
        mask |= TelemetryType::INPUT_DUTY;
    }
    if (m.has_power) {
        t.power_percentage = m.power_pct;
        t.output_duty = m.power_pct;
        mask |= TelemetryType::POWER_PERCENTAGE | TelemetryType::OUTPUT_DUTY;
    }
#endif
    if (mask != 0) {
        update_telem_data(idx, t, mask);
    }
}

void AP_CastleLink::uart_update()
{
    using Ser = AP_CastleLink_Serial;
    const bool serth = _serial_thr.get() > 0;

    if (_awaiting) {
        uint8_t rsp[Ser::RSP_LEN] {};
        const uint32_t avail = _uart->available();
        if (avail >= Ser::RSP_LEN) {
            (void)_uart->read(rsp, sizeof(rsp));
        }
        const Ser::RxAction act = Ser::consider_rx(avail, AP_HAL::millis() - _await_ms, rsp, _pending_reg);
        if (act.waiting) {
            return;
        }
        if (act.discard) {
            _uart->discard_input();
        }
        if (act.flush_zeros) {
            const uint8_t zeros[Ser::FLUSH_ZEROS] {};
            _uart->write(zeros, sizeof(zeros));
        }
        if (act.have_value) {
            handle_register(_pending_reg, act.value);
        }
        _awaiting = false;
    }

    if (Ser::poll_is_throttle_write(_poll_idx, serth)) {
        uint16_t pwm;
        if (!lookup_serth_pwm(pwm)) {
            _poll_idx++;
        } else if (send_cmd(Ser::REG_THROTTLE_W, Ser::pwm_to_throttle_reg(pwm))) {
            _poll_idx++;
            return;
        } else {
            return;
        }
    }

    if (send_cmd(Ser::poll_read_reg(_poll_idx, serth), 0)) {
        _poll_idx++;
    }
}

void AP_CastleLink::handle_packet(const AP_CastleLink_Decoder::Packet &pkt)
{
    const uint8_t idx = uint8_t(_esc_index.get());
    if (idx >= ESC_TELEM_MAX_ESCS) {
        return;
    }

    TelemetryData t {};
    uint16_t mask = 0;

    if (pkt.has_voltage) {
        t.voltage = pkt.voltage_v;
        mask |= TelemetryType::VOLTAGE;
    }
    if (pkt.has_current) {
        t.current = pkt.current_a;
        mask |= TelemetryType::CURRENT;
    }
    if (pkt.has_temperature) {
        t.temperature_cdeg = int16_t(constrain_float(pkt.temperature_c * 100.0f, -32768.0f, 32767.0f));
        mask |= TelemetryType::TEMPERATURE;
    }
#if AP_EXTENDED_ESC_TELEM_ENABLED
    if (pkt.has_throttle) {
        const float duty = (pkt.throttle_ms - 1.0f) * 100.0f;
        t.input_duty = uint8_t(constrain_int16(int16_t(duty), 0, 100));
        mask |= TelemetryType::INPUT_DUTY;
    }
    if (pkt.has_power) {
        const uint8_t pct = uint8_t(constrain_int16(int16_t(pkt.power * 100.0f + 0.5f), 0, 100));
        t.power_percentage = pct;
        t.output_duty = pct;
        mask |= TelemetryType::POWER_PERCENTAGE | TelemetryType::OUTPUT_DUTY;
    }
#endif

    if (mask != 0) {
        update_telem_data(idx, t, mask);
    }

    if (pkt.has_erpm) {
        const int8_t poles = _poles.get();
        float rpm = pkt.erpm;
        if (poles >= 2) {
            rpm = pkt.erpm * 2.0f / float(poles);
        }
        update_rpm(idx, rpm);
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AP_CastleLink::sitl_update()
{
    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }

    float ticks[Dec::PACKET_ITEMS];
    for (uint8_t i = 0; i < Dec::PACKET_ITEMS; i++) {
        ticks[i] = 0.5f;
    }
    ticks[Dec::ITEM_RESET] = Dec::TICK_NONE;
    ticks[Dec::ITEM_CAL_1MS] = 1.0f;

    ticks[Dec::ITEM_VOLTAGE] = 0.5f + constrain_float(float(sitl->state.battery_voltage), 0.0f, 100.0f) / Dec::SCALE_VOLTAGE;
    ticks[Dec::ITEM_CURRENT] = 0.5f + constrain_float(float(sitl->state.battery_current), 0.0f, 250.0f) / Dec::SCALE_CURRENT;

    uint16_t pwm = 1500;
    SRV_Channels::get_output_pwm_chan(_chan0, pwm);
    ticks[Dec::ITEM_THROTTLE] = 0.5f + constrain_float(pwm * 0.001f, 0.0f, 2.5f) / Dec::SCALE_THROTTLE;

    const int8_t poles = MAX(_poles.get(), 2);
    const uint8_t motor = (_chan0 < 32) ? _chan0 : 31;
    const float erpm = constrain_float(sitl->state.rpm[motor] * (float(poles) * 0.5f), 0.0f, 100000.0f);
    ticks[Dec::ITEM_RPM] = 0.5f + erpm / Dec::SCALE_RPM;

    // 32 degC linear sensor; item 11 is the 0.5 ms calibration pulse
    ticks[Dec::ITEM_TEMP_LINEAR] = 0.5f + 32.0f / Dec::SCALE_TEMP_LINEAR;
    ticks[Dec::ITEM_TEMP_NTC] = 0.5f;

    ticks[Dec::ITEM_POWER] = 0.5f + 0.25f / Dec::SCALE_POWER;

    Dec::Packet pkt{};
    if (Dec::decode(ticks, pkt)) {
        handle_packet(pkt);
    }
}
#endif

void AP_CastleLink::update()
{
    if (_enable.get() <= 0) {
        return;
    }

    if (!_initialised) {
        init();
        return;
    }

    if (_uart != nullptr) {
        uart_update();
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_channel.get() >= 1) {
        sitl_update();
    }
    return;
#endif

    if (!_gpio_ok) {
        return;
    }

    Dec::Packet pkt{};
    bool have = false;
    {
        WITH_SEMAPHORE(_sem);
        if (_have_packet) {
            pkt = _packet;
            _have_packet = false;
            have = true;
        }
    }
    if (have) {
        handle_packet(pkt);
    }
}

#endif  // AP_CASTLELINK_ENABLED
