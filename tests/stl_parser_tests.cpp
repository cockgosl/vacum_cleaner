#include <gtest/gtest.h>

#include "io/stl_parser.hpp"
#include "mapping/lidar.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <span>
#include <vector>

namespace {

// Fixed protocol fixtures include CRC bytes; tests do not reuse the parser's CRC routine.
const std::array<std::uint8_t, 47> north_clockwise{{0x54, 0x2C, 0x10, 0x0E, 0x28, 0x23, 0xD2, 0x04, 0x64, 0xD2, 0x04, 0x65,
                                                    0xD2, 0x04, 0x66, 0xD2, 0x04, 0x67, 0xD2, 0x04, 0x68, 0xD2, 0x04, 0x69,
                                                    0xD2, 0x04, 0x6A, 0xD2, 0x04, 0x6B, 0xD2, 0x04, 0x6C, 0xD2, 0x04, 0x6D,
                                                    0xD2, 0x04, 0x6E, 0xD2, 0x04, 0x6F, 0x28, 0x23, 0x34, 0x12, 0xB7}};

const std::array<std::uint8_t, 47> interpolated{{0x54, 0x2C, 0x10, 0x0E, 0x00, 0x00, 0xE8, 0x03, 0x64, 0x4C, 0x04, 0x65,
                                                 0xB0, 0x04, 0x66, 0x14, 0x05, 0x67, 0x78, 0x05, 0x68, 0xDC, 0x05, 0x69,
                                                 0x40, 0x06, 0x6A, 0xA4, 0x06, 0x6B, 0x08, 0x07, 0x6C, 0x6C, 0x07, 0x6D,
                                                 0xD0, 0x07, 0x6E, 0x34, 0x08, 0x6F, 0x4C, 0x04, 0x34, 0x12, 0xC1}};

const std::array<std::uint8_t, 47> wrapped{{0x54, 0x2C, 0x10, 0x0E, 0x3C, 0x8C, 0xE8, 0x03, 0x64, 0xE8, 0x03, 0x65,
                                            0xE8, 0x03, 0x66, 0xE8, 0x03, 0x67, 0xE8, 0x03, 0x68, 0xE8, 0x03, 0x69,
                                            0xE8, 0x03, 0x6A, 0xE8, 0x03, 0x6B, 0xE8, 0x03, 0x6C, 0xE8, 0x03, 0x6D,
                                            0xE8, 0x03, 0x6E, 0xE8, 0x03, 0x6F, 0x64, 0x00, 0x34, 0x12, 0x05}};

const std::array<std::uint8_t, 47> no_return{{0x54, 0x2C, 0x10, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x65,
                                              0x00, 0x00, 0x66, 0x00, 0x00, 0x67, 0x00, 0x00, 0x68, 0x00, 0x00, 0x69,
                                              0x00, 0x00, 0x6A, 0x00, 0x00, 0x6B, 0x00, 0x00, 0x6C, 0x00, 0x00, 0x6D,
                                              0x00, 0x00, 0x6E, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x34, 0x12, 0xEB}};

const std::array<std::uint8_t, 47> invalid_angle{{0x54, 0x2C, 0x10, 0x0E, 0xA0, 0x8C, 0xE8, 0x03, 0x64, 0xE8, 0x03, 0x65,
                                                  0xE8, 0x03, 0x66, 0xE8, 0x03, 0x67, 0xE8, 0x03, 0x68, 0xE8, 0x03, 0x69,
                                                  0xE8, 0x03, 0x6A, 0xE8, 0x03, 0x6B, 0xE8, 0x03, 0x6C, 0xE8, 0x03, 0x6D,
                                                  0xE8, 0x03, 0x6E, 0xE8, 0x03, 0x6F, 0x00, 0x00, 0x34, 0x12, 0x6D}};

std::vector<io::ScanPacket> feed(io::StlParser& parser, const auto& bytes) {
    std::vector<io::ScanPacket> packets;
    for (const auto byte : bytes)
        if (auto packet = parser.push(byte))
            packets.push_back(*packet);
    return packets;
}

TEST(StlParserTest, ConvertsClockwiseDegreesAndMillimetres) {
    io::StlParser parser;
    const auto packets = feed(parser, north_clockwise);
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets[0].timestamp, 0x1234);
    for (std::size_t i = 0; i < 12; ++i) {
        EXPECT_NEAR(packets[0].measurements[i].angle, -std::numbers::pi / 2, 1e-12);
        EXPECT_DOUBLE_EQ(packets[0].measurements[i].distance, 1.234);
        EXPECT_EQ(packets[0].measurements[i].intensity, 100 + i);
    }
}

TEST(StlParserTest, InterpolatesAllTwelveSamplesIncludingEndpoints) {
    io::StlParser parser;
    const auto packets = feed(parser, interpolated);
    ASSERT_EQ(packets.size(), 1U);
    for (std::size_t i = 0; i < 12; ++i) {
        EXPECT_NEAR(packets[0].measurements[i].angle, -static_cast<double>(i) * std::numbers::pi / 180.0, 1e-12);
        EXPECT_NEAR(packets[0].measurements[i].distance, 1.0 + i * 0.1, 1e-12);
    }
}

TEST(StlParserTest, InterpolatesForwardAcrossZeroDegrees) {
    io::StlParser parser;
    const auto packets = feed(parser, wrapped);
    ASSERT_EQ(packets.size(), 1U);
    for (std::size_t i = 0; i < 12; ++i) {
        const double degrees = std::fmod(359.0 + 2.0 * i / 11.0, 360.0);
        EXPECT_NEAR(packets[0].measurements[i].angle, -degrees * std::numbers::pi / 180, 1e-12);
    }
}

TEST(StlParserTest, PreservesPartialFramesUntilTheCrcArrives) {
    io::StlParser parser;
    for (std::size_t i = 0; i < interpolated.size() - 1; ++i)
        EXPECT_FALSE(parser.push(interpolated[i]));
    ASSERT_TRUE(parser.push(interpolated.back()));
}

TEST(StlParserTest, IgnoresNoiseAndRepeatedFirstHeaderBytes) {
    io::StlParser parser;
    EXPECT_TRUE(feed(parser, std::array<std::uint8_t, 6>{0, 0x54, 0, 0x2C, 0x54, 0x54}).empty());
    EXPECT_EQ(feed(parser, interpolated).size(), 1U);
}

TEST(StlParserTest, DecodesConsecutivePackets) {
    io::StlParser parser;
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(feed(parser, interpolated).size(), 1U);
}

TEST(StlParserTest, RejectsCorruptedPayloadAndRecovers) {
    io::StlParser parser;
    auto damaged = interpolated;
    damaged[6] ^= 1;
    EXPECT_TRUE(feed(parser, damaged).empty());
    EXPECT_EQ(feed(parser, interpolated).size(), 1U);
}

TEST(StlParserTest, RetainsAValidHeaderInsideADamagedFrame) {
    io::StlParser parser;
    // Simulate a truncated packet immediately followed by a complete packet.
    EXPECT_TRUE(feed(parser, std::span{interpolated}.first(10)).empty());
    const auto packets = feed(parser, north_clockwise);
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_DOUBLE_EQ(packets[0].measurements[0].distance, 1.234);
}

TEST(StlParserTest, PreservesZeroDistanceWithoutInventingAReturn) {
    io::StlParser parser;
    const auto packets = feed(parser, no_return);
    ASSERT_EQ(packets.size(), 1U);
    for (const auto& measurement : packets[0].measurements)
        EXPECT_DOUBLE_EQ(measurement.distance, 0.0);
}

TEST(StlParserTest, RejectsOutOfRangeAnglesEvenWithValidCrc) {
    io::StlParser parser;
    EXPECT_TRUE(feed(parser, invalid_angle).empty());
    EXPECT_EQ(feed(parser, interpolated).size(), 1U);
}

TEST(StlParserTest, MapsAClockwiseQuarterTurnBelowTheSensor) {
    io::StlParser parser;
    auto sensor = lidar::Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor);
    const auto packets = feed(parser, north_clockwise);
    ASSERT_EQ(packets.size(), 1U);
    const auto& point = packets[0].measurements[0];
    ASSERT_TRUE(sensor->update_measurement({4.5, 4.5, 0.0}, point.angle, point.distance));
    EXPECT_EQ(sensor->grid().at(4, 3), lidar::CellState::Occupied);
    EXPECT_EQ(sensor->grid().at(4, 4), lidar::CellState::Free);
    EXPECT_EQ(sensor->grid().at(4, 5), lidar::CellState::Unknown);
}

} // namespace
