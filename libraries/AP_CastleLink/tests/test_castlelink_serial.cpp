/*
  tests for Castle Serial Link TTL register codec
  to build, use:
    ./waf configure --board sitl --debug
    ./waf --target tests/test_castlelink_serial
 */

#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_CastleLink/AP_CastleLink_Serial.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using Ser = AP_CastleLink_Serial;

TEST(CastleLinkSerial, device_id_packing)
{
    EXPECT_EQ(0x80, Ser::pack_device_id(0));
    EXPECT_EQ(0x81, Ser::pack_device_id(1));
    EXPECT_EQ(0xBF, Ser::pack_device_id(63));
    // IDs above 63 wrap into 6 bits
    EXPECT_EQ(0x80, Ser::pack_device_id(64));
}

TEST(CastleLinkSerial, checksum_encode)
{
    uint8_t cmd[Ser::CMD_LEN];
    Ser::encode_command(0, Ser::REG_VOLTAGE, 0, cmd);
    EXPECT_EQ(0x80, cmd[0]);
    EXPECT_EQ(0, cmd[1]);
    EXPECT_EQ(0, cmd[2]);
    EXPECT_EQ(0, cmd[3]);
    EXPECT_EQ(0x80, cmd[4]);
    EXPECT_EQ(0, Ser::checksum(cmd, Ser::CMD_LEN));

    Ser::encode_command(1, Ser::REG_CURRENT, 0, cmd);
    EXPECT_EQ(0x81, cmd[0]);
    EXPECT_EQ(2, cmd[1]);
    EXPECT_EQ(0, Ser::checksum(cmd, Ser::CMD_LEN));
}

TEST(CastleLinkSerial, checksum_write_payload)
{
    uint8_t cmd[Ser::CMD_LEN];
    Ser::encode_command(0, Ser::REG_THROTTLE_W, 0x1234, cmd);
    EXPECT_EQ(0x12, cmd[2]);
    EXPECT_EQ(0x34, cmd[3]);
    EXPECT_EQ(0, Ser::checksum(cmd, Ser::CMD_LEN));
}

TEST(CastleLinkSerial, decode_good_response)
{
    // 4084 = 0x0FF4, checksum = 0 - (0x0F+0xF4) = 0xFD
    const uint8_t rsp[3] = { 0x0F, 0xF4, 0xFD };
    uint16_t value = 0;
    ASSERT_TRUE(Ser::decode_response(rsp, value));
    EXPECT_EQ(4084, value);
}

TEST(CastleLinkSerial, spec_example_voltage_40V)
{
    EXPECT_NEAR(Ser::convert(4084, Ser::SCALE_VOLTAGE), 40.0f, 1e-4);
}

TEST(CastleLinkSerial, scale_factors)
{
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_VOLTAGE), 20.0f, 1e-3);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_RIPPLE), 4.0f, 1e-3);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_CURRENT), 50.0f, 1e-3);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_THROTTLE), 1.0f, 1e-3);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_POWER), 0.2502f, 1e-4);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_RPM), 20416.66f, 0.1f);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_BEC_V), 4.0f, 1e-3);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_BEC_I), 4.0f, 1e-3);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_TEMP), 30.0f, 1e-3);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_RAW_NTC), 63.8125f, 1e-3);
    EXPECT_NEAR(Ser::convert(2042, Ser::SCALE_RAW_LINEAR), 30.0f, 1e-3);
    EXPECT_FLOAT_EQ(Ser::convert(0, Ser::SCALE_VOLTAGE), 0.0f);
}

TEST(CastleLinkSerial, bad_checksum_rejected)
{
    const uint8_t rsp[3] = { 0x0F, 0xF4, 0x00 };
    uint16_t value = 0xABCD;
    EXPECT_FALSE(Ser::decode_response(rsp, value));
    EXPECT_EQ(0xABCD, value);
}

TEST(CastleLinkSerial, error_ffff_rejected)
{
    const uint8_t rsp[3] = { 0xFF, 0xFF, 0x02 };
    EXPECT_EQ(0, Ser::checksum(rsp, 3));
    uint16_t value = 0xABCD;
    EXPECT_FALSE(Ser::decode_response(rsp, value));
    EXPECT_EQ(0xABCD, value);
}

TEST(CastleLinkSerial, decode_zero_accepted)
{
    const uint8_t rsp[3] = { 0x00, 0x00, 0x00 };
    uint16_t value = 0xABCD;
    ASSERT_TRUE(Ser::decode_response(rsp, value));
    EXPECT_EQ(0, value);
}

TEST(CastleLinkSerial, decode_fffe_accepted)
{
    // 0xFFFE, checksum = 0 - (0xFF+0xFE) = 0x03
    const uint8_t rsp[3] = { 0xFF, 0xFE, 0x03 };
    uint16_t value = 0;
    ASSERT_TRUE(Ser::decode_response(rsp, value));
    EXPECT_EQ(0xFFFE, value);
}

TEST(CastleLinkSerial, pwm_to_throttle_reg)
{
    EXPECT_EQ(0, Ser::pwm_to_throttle_reg(1000));
    EXPECT_EQ(32767, Ser::pwm_to_throttle_reg(1500));
    EXPECT_EQ(65535, Ser::pwm_to_throttle_reg(2000));
    EXPECT_EQ(0, Ser::pwm_to_throttle_reg(999));
    EXPECT_EQ(65535, Ser::pwm_to_throttle_reg(2001));
}

TEST(CastleLinkSerial, poll_slots_no_serth)
{
    uint8_t seen[Ser::NUM_POLL_REGS] {};
    for (uint8_t i = 0; i < Ser::NUM_POLL_REGS; i++) {
        EXPECT_FALSE(Ser::poll_is_throttle_write(i, false));
        const uint8_t reg = Ser::poll_read_reg(i, false);
        EXPECT_EQ(Ser::POLL_REGS[i], reg);
        seen[i] = reg;
    }
    EXPECT_FALSE(Ser::poll_is_throttle_write(Ser::NUM_POLL_REGS, false));
    EXPECT_EQ(Ser::POLL_REGS[0], Ser::poll_read_reg(Ser::NUM_POLL_REGS, false));
    EXPECT_EQ(Ser::REG_VOLTAGE, seen[0]);
    EXPECT_EQ(Ser::REG_THROTTLE, seen[Ser::NUM_POLL_REGS - 1]);
}

TEST(CastleLinkSerial, poll_slots_serth)
{
    uint8_t writes = 0;
    const uint8_t period = Ser::NUM_POLL_REGS + 1;
    for (uint8_t i = 0; i < 2 * period; i++) {
        if (Ser::poll_is_throttle_write(i, true)) {
            writes++;
            EXPECT_EQ(Ser::NUM_POLL_REGS, i % period);
        } else {
            EXPECT_EQ(Ser::POLL_REGS[i % period], Ser::poll_read_reg(i, true));
        }
    }
    EXPECT_EQ(2, writes);
    EXPECT_EQ(Ser::REG_VOLTAGE, Ser::poll_read_reg(7, true));
    EXPECT_FALSE(Ser::poll_is_throttle_write(7, true));
    EXPECT_EQ(Ser::POLL_REGS[0], Ser::poll_read_reg(6, true));
}

TEST(CastleLinkSerial, consider_rx_partial_then_timeout)
{
    const uint8_t dummy[3] {};
    Ser::RxAction a = Ser::consider_rx(0, 5, dummy);
    EXPECT_TRUE(a.waiting);
    EXPECT_FALSE(a.discard);

    a = Ser::consider_rx(1, 5, dummy);
    EXPECT_TRUE(a.waiting);

    a = Ser::consider_rx(2, 5, dummy);
    EXPECT_TRUE(a.waiting);

    a = Ser::consider_rx(1, 25, dummy);
    EXPECT_FALSE(a.waiting);
    EXPECT_TRUE(a.discard);
    EXPECT_FALSE(a.have_value);
}

TEST(CastleLinkSerial, consider_rx_good_and_leftover)
{
    const uint8_t rsp[3] = { 0x0F, 0xF4, 0xFD };
    Ser::RxAction a = Ser::consider_rx(3, 5, rsp);
    EXPECT_FALSE(a.waiting);
    EXPECT_TRUE(a.discard);
    EXPECT_TRUE(a.have_value);
    EXPECT_EQ(4084, a.value);
    EXPECT_FALSE(a.flush_zeros);

    a = Ser::consider_rx(4, 5, rsp);
    EXPECT_TRUE(a.discard);
    EXPECT_TRUE(a.have_value);
}

TEST(CastleLinkSerial, consider_rx_device_error_flushes)
{
    const uint8_t rsp[3] = { 0xFF, 0xFF, 0x02 };
    Ser::RxAction a = Ser::consider_rx(3, 5, rsp);
    EXPECT_TRUE(a.discard);
    EXPECT_TRUE(a.flush_zeros);
    EXPECT_FALSE(a.have_value);
}

TEST(CastleLinkSerial, consider_rx_bad_checksum)
{
    const uint8_t rsp[3] = { 0x0F, 0xF4, 0x00 };
    Ser::RxAction a = Ser::consider_rx(3, 5, rsp);
    EXPECT_FALSE(a.waiting);
    EXPECT_TRUE(a.discard);
    EXPECT_FALSE(a.have_value);
    EXPECT_FALSE(a.flush_zeros);
}

TEST(CastleLinkSerial, consider_rx_throttle_write_ffff_is_full)
{
    const uint8_t rsp[3] = { 0xFF, 0xFF, 0x02 };
    Ser::RxAction a = Ser::consider_rx(3, 5, rsp, Ser::REG_THROTTLE_W);
    EXPECT_TRUE(a.discard);
    EXPECT_TRUE(a.have_value);
    EXPECT_EQ(0xFFFF, a.value);
    EXPECT_FALSE(a.flush_zeros);
}

TEST(CastleLinkSerial, map_register)
{
    Ser::MappedTelem m{};
    ASSERT_TRUE(Ser::map_register(Ser::REG_VOLTAGE, 4084, 14, m));
    EXPECT_TRUE(m.has_voltage);
    EXPECT_NEAR(m.voltage_v, 40.0f, 1e-4);

    ASSERT_TRUE(Ser::map_register(Ser::REG_VOLTAGE, 2042, 14, m));
    EXPECT_NEAR(m.voltage_v, 20.0f, 1e-3);

    ASSERT_TRUE(Ser::map_register(Ser::REG_TEMP, 2042, 14, m));
    EXPECT_TRUE(m.has_temperature);
    EXPECT_NEAR(m.temperature_c, 30.0f, 1e-3);
    EXPECT_EQ(3000, m.temperature_cdeg);

    ASSERT_TRUE(Ser::map_register(Ser::REG_SPEED, 2042, 14, m));
    EXPECT_TRUE(m.has_erpm);
    EXPECT_NEAR(m.erpm, 20416.66f, 0.1f);
    EXPECT_NEAR(m.mech_rpm, 20416.66f * 2.0f / 14.0f, 0.1f);

    ASSERT_TRUE(Ser::map_register(Ser::REG_SPEED, 2042, 0, m));
    EXPECT_NEAR(m.mech_rpm, m.erpm, 0.1f);

    ASSERT_TRUE(Ser::map_register(Ser::REG_THROTTLE, 2042, 14, m));
    EXPECT_TRUE(m.has_throttle);
    EXPECT_NEAR(m.throttle_ms, 1.0f, 1e-3);
    EXPECT_EQ(0, m.input_duty);

    ASSERT_TRUE(Ser::map_register(Ser::REG_POWER, 2042, 14, m));
    EXPECT_TRUE(m.has_power);
    EXPECT_NEAR(m.power, 0.2502f, 1e-4);
    EXPECT_EQ(25, m.power_pct);

    EXPECT_FALSE(Ser::map_register(99, 2042, 14, m));
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
