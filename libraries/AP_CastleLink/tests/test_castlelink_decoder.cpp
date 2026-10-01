/*
  tests for Castle Link Live 2.0 decoder
  to build, use:
    ./waf configure --board sitl --debug
    ./waf --target tests/test_castlelink_decoder
 */

#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_CastleLink/AP_CastleLink_Decoder.h>
#include <cmath>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using Dec = AP_CastleLink_Decoder;

static void fill_zero_packet(float ticks[Dec::PACKET_ITEMS])
{
    ticks[Dec::ITEM_RESET] = Dec::TICK_NONE;
    ticks[Dec::ITEM_CAL_1MS] = 1.0f;
    ticks[Dec::ITEM_VOLTAGE] = 0.5f;
    ticks[Dec::ITEM_RIPPLE] = 0.5f;
    ticks[Dec::ITEM_CURRENT] = 0.5f;
    ticks[Dec::ITEM_THROTTLE] = 0.5f;
    ticks[Dec::ITEM_POWER] = 0.5f;
    ticks[Dec::ITEM_RPM] = 0.5f;
    ticks[Dec::ITEM_BEC_V] = 0.5f;
    ticks[Dec::ITEM_BEC_I] = 0.5f;
    ticks[Dec::ITEM_TEMP_LINEAR] = 0.5f;
    ticks[Dec::ITEM_TEMP_NTC] = 0.5f;
}

TEST(CastleLinkDecoder, missing_tick)
{
    EXPECT_TRUE(Dec::is_missing(Dec::TICK_NONE));
    EXPECT_TRUE(Dec::is_missing(NAN));
    EXPECT_TRUE(Dec::is_missing(-0.1f));
    EXPECT_FALSE(Dec::is_missing(0.0f));
    EXPECT_FALSE(Dec::is_missing(0.5f));
    EXPECT_FALSE(Dec::is_missing(1.5f));
}

TEST(CastleLinkDecoder, reset_required)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_RESET] = 0.5f;  // a tick on RESET means we are not synced

    Dec::Packet pkt{};
    EXPECT_FALSE(Dec::decode(ticks, pkt));
}

TEST(CastleLinkDecoder, reset_missing_ok)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);

    Dec::Packet pkt{};
    EXPECT_TRUE(Dec::decode(ticks, pkt));
}

TEST(CastleLinkDecoder, missing_calibration_rejected)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_CAL_1MS] = Dec::TICK_NONE;

    Dec::Packet pkt{};
    EXPECT_FALSE(Dec::decode(ticks, pkt));
}

TEST(CastleLinkDecoder, missing_half_ms_cal_rejected)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_TEMP_LINEAR] = Dec::TICK_NONE;
    ticks[Dec::ITEM_TEMP_NTC] = Dec::TICK_NONE;

    Dec::Packet pkt{};
    EXPECT_FALSE(Dec::decode(ticks, pkt));
}

TEST(CastleLinkDecoder, spec_example_voltage_20V)
{
    // calibrated 1.5 ms voltage tick → (1.5 - 0.5) * 20 = 20.0 V
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_VOLTAGE] = 1.5f;

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    ASSERT_TRUE(pkt.has_voltage);
    EXPECT_NEAR(pkt.voltage_v, 20.0f, 1e-4);
}

TEST(CastleLinkDecoder, scale_factors)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_VOLTAGE] = 1.5f;     // 20 V
    ticks[Dec::ITEM_RIPPLE] = 1.5f;      // 4 V
    ticks[Dec::ITEM_CURRENT] = 1.5f;     // 50 A
    ticks[Dec::ITEM_THROTTLE] = 1.5f;    // 1.0 ms
    ticks[Dec::ITEM_POWER] = 1.5f;       // 0.2502
    ticks[Dec::ITEM_RPM] = 1.5f;         // 20416.7 eRPM
    ticks[Dec::ITEM_BEC_V] = 1.5f;       // 4 V
    ticks[Dec::ITEM_BEC_I] = 1.5f;       // 4 A

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    EXPECT_TRUE(pkt.has_voltage);
    EXPECT_NEAR(pkt.voltage_v, 20.0f, 1e-3);
    EXPECT_TRUE(pkt.has_ripple);
    EXPECT_NEAR(pkt.ripple_v, 4.0f, 1e-3);
    EXPECT_TRUE(pkt.has_current);
    EXPECT_NEAR(pkt.current_a, 50.0f, 1e-3);
    EXPECT_TRUE(pkt.has_throttle);
    EXPECT_NEAR(pkt.throttle_ms, 1.0f, 1e-3);
    EXPECT_TRUE(pkt.has_power);
    EXPECT_NEAR(pkt.power, 0.2502f, 1e-4);
    EXPECT_TRUE(pkt.has_erpm);
    EXPECT_NEAR(pkt.erpm, 20416.7f, 0.1f);
    EXPECT_TRUE(pkt.has_bec_v);
    EXPECT_NEAR(pkt.bec_v, 4.0f, 1e-3);
    EXPECT_TRUE(pkt.has_bec_a);
    EXPECT_NEAR(pkt.bec_a, 4.0f, 1e-3);
}

TEST(CastleLinkDecoder, zero_offset_is_zero)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    EXPECT_TRUE(pkt.has_voltage);
    EXPECT_NEAR(pkt.voltage_v, 0.0f, 1e-4);
    EXPECT_TRUE(pkt.has_current);
    EXPECT_NEAR(pkt.current_a, 0.0f, 1e-4);
    EXPECT_TRUE(pkt.has_erpm);
    EXPECT_NEAR(pkt.erpm, 0.0f, 1e-2);
}

TEST(CastleLinkDecoder, two_point_calibration_clock_skew)
{
    // local clock 10% fast: every ESC millisecond measures as 1.1 ms
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_CAL_1MS] = 1.1f;
    ticks[Dec::ITEM_TEMP_LINEAR] = 0.55f;
    ticks[Dec::ITEM_TEMP_NTC] = 0.55f;
    ticks[Dec::ITEM_VOLTAGE] = 1.65f;  // 1.5 ms on the ESC clock

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    ASSERT_TRUE(pkt.has_voltage);
    EXPECT_NEAR(pkt.voltage_v, 20.0f, 1e-3);
}

TEST(CastleLinkDecoder, linear_temperature)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_TEMP_LINEAR] = 1.5f;  // (1.5-0.5)*30 = 30 degC
    ticks[Dec::ITEM_TEMP_NTC] = 0.5f;

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    ASSERT_TRUE(pkt.has_temperature);
    EXPECT_NEAR(pkt.temperature_c, 30.0f, 1e-3);
}

TEST(CastleLinkDecoder, ntc_vs_linear_selection)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    // NTC is the larger tick → NTC is the data field
    ticks[Dec::ITEM_TEMP_LINEAR] = 0.5f;
    ticks[Dec::ITEM_TEMP_NTC] = 0.5f + (100.0f / Dec::SCALE_TEMP_NTC);  // 100 units

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    ASSERT_TRUE(pkt.has_temperature);

    float expected;
    ASSERT_TRUE(Dec::ntc_to_degC(100.0f, expected));
    EXPECT_NEAR(pkt.temperature_c, expected, 1e-3);
}

TEST(CastleLinkDecoder, both_temp_cal_means_no_temperature)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_TEMP_LINEAR] = 0.5f;
    ticks[Dec::ITEM_TEMP_NTC] = 0.5f;

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    EXPECT_FALSE(pkt.has_temperature);
}

TEST(CastleLinkDecoder, ntc_formula)
{
    float deg_c;
    ASSERT_TRUE(Dec::ntc_to_degC(100.0f, deg_c));
    EXPECT_NEAR(deg_c, 36.15822143f, 1e-4);

    ASSERT_TRUE(Dec::ntc_to_degC(127.0f, deg_c));
    EXPECT_NEAR(deg_c, 24.69292255f, 1e-4);

    ASSERT_TRUE(Dec::ntc_to_degC(1.0f, deg_c));
    EXPECT_NEAR(deg_c, 295.59061735f, 1e-3);
}

TEST(CastleLinkDecoder, ntc_rejects_out_of_range)
{
    float deg_c;
    EXPECT_FALSE(Dec::ntc_to_degC(0.0f, deg_c));
    EXPECT_FALSE(Dec::ntc_to_degC(-1.0f, deg_c));
    EXPECT_FALSE(Dec::ntc_to_degC(255.0f, deg_c));
    EXPECT_FALSE(Dec::ntc_to_degC(256.0f, deg_c));
    EXPECT_FALSE(Dec::ntc_to_degC(NAN, deg_c));
}

TEST(CastleLinkDecoder, reject_tick_below_half_ms)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_VOLTAGE] = 0.2f;  // well below the 0.5 ms zero-offset

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    EXPECT_FALSE(pkt.has_voltage);
}

TEST(CastleLinkDecoder, reject_tick_above_table_max)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    // voltage max pulse is 5 ms after the 0.5 ms offset → 5.5 ms calibrated
    ticks[Dec::ITEM_VOLTAGE] = 6.5f;

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    EXPECT_FALSE(pkt.has_voltage);
}

TEST(CastleLinkDecoder, reject_throttle_above_max)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_THROTTLE] = 4.0f;  // max is 0.5+2.5 = 3.0 ms

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    EXPECT_FALSE(pkt.has_throttle);
    // other fields still decoded
    EXPECT_TRUE(pkt.has_voltage);
}

TEST(CastleLinkDecoder, ntc_units_outside_range_rejected)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_TEMP_LINEAR] = 0.5f;
    // 255 units / 63.8125 + 0.5 = 4.496 ms - within the 4.5 ms NTC max+offset
    // but ntc_to_degC must reject value == 255
    ticks[Dec::ITEM_TEMP_NTC] = 0.5f + (255.0f / Dec::SCALE_TEMP_NTC);

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    EXPECT_FALSE(pkt.has_temperature);
}

TEST(CastleLinkDecoder, implausible_1ms_cal_rejected)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_CAL_1MS] = 3.0f;  // not a plausible 1.0 ms reference

    Dec::Packet pkt{};
    EXPECT_FALSE(Dec::decode(ticks, pkt));
}

TEST(CastleLinkDecoder, max_in_range_voltage)
{
    float ticks[Dec::PACKET_ITEMS];
    fill_zero_packet(ticks);
    ticks[Dec::ITEM_VOLTAGE] = 5.5f;  // (5.5-0.5)*20 = 100 V, table max

    Dec::Packet pkt{};
    ASSERT_TRUE(Dec::decode(ticks, pkt));
    ASSERT_TRUE(pkt.has_voltage);
    EXPECT_NEAR(pkt.voltage_v, 100.0f, 1e-2);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
