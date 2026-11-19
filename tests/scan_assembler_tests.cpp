#include <gtest/gtest.h>

#include "io/scan_assembler.hpp"

#include <array>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <vector>

namespace {

io::Measurement point(double clockwise_degrees, double distance = 1.0, std::uint8_t intensity = 42) {
    return {-clockwise_degrees * std::numbers::pi / 180.0, distance, intensity};
}

void expect_pending(io::ScanAssembler& assembler, std::initializer_list<double> degrees) {
    for (const double angle : degrees)
        EXPECT_FALSE(assembler.push(point(angle))) << "clockwise degrees=" << angle;
}

TEST(ScanAssemblerTest, DiscardsTheInitialPartialRevolution) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {135.0, 180.0, 270.0, 359.0});
}

TEST(ScanAssemblerTest, FirstCrossingStartsButDoesNotEmitAScan) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {135.0, 359.0, 0.0, 90.0});
}

TEST(ScanAssemblerTest, SecondCrossingEmitsTheFirstScan) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {135.0, 359.0, 0.0, 90.0, 180.0, 270.0, 359.0});
    const auto scan = assembler.push(point(0.0));
    ASSERT_TRUE(scan);
    ASSERT_EQ(scan->measurements.size(), 5U);
    EXPECT_DOUBLE_EQ(scan->measurements.front().angle, point(0.0).angle);
    EXPECT_DOUBLE_EQ(scan->measurements.back().angle, point(359.0).angle);
}

TEST(ScanAssemblerTest, CrossingMeasurementBelongsToTheNextScan) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {359.0, 0.0, 90.0, 359.0});
    const auto first = assembler.push(point(1.0));
    ASSERT_TRUE(first);
    ASSERT_EQ(first->measurements.size(), 3U);
    EXPECT_DOUBLE_EQ(first->measurements.back().angle, point(359.0).angle);

    expect_pending(assembler, {2.0, 180.0, 359.0});
    const auto second = assembler.push(point(0.0));
    ASSERT_TRUE(second);
    ASSERT_EQ(second->measurements.size(), 4U);
    EXPECT_DOUBLE_EQ(second->measurements.front().angle, point(1.0).angle);
}

TEST(ScanAssemblerTest, PreservesAllMeasurementFieldsInArrivalOrder) {
    io::ScanAssembler assembler;
    ASSERT_FALSE(assembler.push(point(359.0)));
    const std::array<io::Measurement, 3> expected{point(0.0, 2.0, 10), point(90.0, 3.0, 20), point(359.0, 4.0, 30)};
    for (const auto& measurement : expected)
        ASSERT_FALSE(assembler.push(measurement));
    const auto scan = assembler.push(point(0.0, 5.0, 40));
    ASSERT_TRUE(scan);
    ASSERT_EQ(scan->measurements.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_DOUBLE_EQ(scan->measurements[i].angle, expected[i].angle);
        EXPECT_DOUBLE_EQ(scan->measurements[i].distance, expected[i].distance);
        EXPECT_EQ(scan->measurements[i].intensity, expected[i].intensity);
    }
}

TEST(ScanAssemblerTest, RetainsZeroReturnsAndUsesTheirAnglesForCrossings) {
    io::ScanAssembler assembler;
    for (const double degrees : {359.0, 0.0, 90.0, 180.0, 270.0, 359.0})
        EXPECT_FALSE(assembler.push(point(degrees, 0.0)));
    const auto scan = assembler.push(point(0.0, 0.0));
    ASSERT_TRUE(scan);
    ASSERT_EQ(scan->measurements.size(), 5U);
    for (const auto& measurement : scan->measurements)
        EXPECT_DOUBLE_EQ(measurement.distance, 0.0);
}

TEST(ScanAssemblerTest, SmallReverseJitterDoesNotStartANewRevolution) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {359.0, 0.0, 90.0, 89.9, 180.0, 359.0});
    const auto scan = assembler.push(point(0.0));
    ASSERT_TRUE(scan);
    EXPECT_EQ(scan->measurements.size(), 5U);
}

TEST(ScanAssemblerTest, RepeatedAnglesDoNotEmitExtraScans) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {359.0, 0.0, 0.0, 90.0, 90.0, 180.0, 270.0, 359.0});
    const auto scan = assembler.push(point(0.0));
    ASSERT_TRUE(scan);
    EXPECT_EQ(scan->measurements.size(), 7U);
}

TEST(ScanAssemblerTest, AnInitialZeroAngleStillRequiresTwoObservedCrossings) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {0.0, 90.0, 180.0, 270.0, 359.0, 0.0, 90.0, 180.0, 270.0, 359.0});
    const auto scan = assembler.push(point(0.0));
    ASSERT_TRUE(scan);
    EXPECT_EQ(scan->measurements.size(), 5U);
}

TEST(ScanAssemblerTest, InvalidAnglesDoNotChangeAssemblyState) {
    for (const double angle : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity(), 0.1, -2.0 * std::numbers::pi, -7.0}) {
        io::ScanAssembler assembler;
        expect_pending(assembler, {359.0, 0.0, 90.0, 180.0, 359.0});
        EXPECT_FALSE(assembler.push({angle, 1.0, 42}));
        const auto scan = assembler.push(point(0.0));
        ASSERT_TRUE(scan);
        EXPECT_EQ(scan->measurements.size(), 4U);
    }
}

TEST(ScanAssemblerTest, InvalidDistancesDoNotCauseCrossingsOrDropPendingPoints) {
    for (const double distance : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity(), -0.1}) {
        io::ScanAssembler assembler;
        expect_pending(assembler, {359.0, 0.0, 90.0, 180.0, 359.0});
        EXPECT_FALSE(assembler.push(point(0.0, distance)));
        const auto scan = assembler.push(point(0.0));
        ASSERT_TRUE(scan);
        EXPECT_EQ(scan->measurements.size(), 4U);
    }
}

TEST(ScanAssemblerTest, CompletedScansOwnTheirMeasurementsIndependently) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {359.0, 0.0, 90.0, 359.0});
    const auto first = assembler.push(point(1.0));
    ASSERT_TRUE(first);
    expect_pending(assembler, {180.0, 270.0, 359.0});
    const auto second = assembler.push(point(0.0));
    ASSERT_TRUE(second);
    ASSERT_EQ(first->measurements.size(), 3U);
    EXPECT_DOUBLE_EQ(first->measurements.front().angle, point(0.0).angle);
    EXPECT_DOUBLE_EQ(first->measurements[1].angle, point(90.0).angle);
    EXPECT_EQ(second->measurements.size(), 4U);
}

TEST(ScanAssemblerTest, AScanMayContainExactlyTheMaximumNumberOfMeasurements) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {359.0, 0.0});
    for (std::size_t i = 0; i < io::ScanAssembler::max_measurements - 2; ++i)
        ASSERT_FALSE(assembler.push(point(0.0)));
    ASSERT_FALSE(assembler.push(point(359.0)));
    const auto scan = assembler.push(point(0.0));
    ASSERT_TRUE(scan);
    EXPECT_EQ(scan->measurements.size(), io::ScanAssembler::max_measurements);
}

TEST(ScanAssemblerTest, OverflowDiscardsTheScanAndResynchronizesAtTheNextCrossing) {
    io::ScanAssembler assembler;
    expect_pending(assembler, {359.0, 0.0});
    for (std::size_t i = 0; i < io::ScanAssembler::max_measurements; ++i)
        ASSERT_FALSE(assembler.push(point(0.0)));
    // The overfilled revolution must not be emitted at this crossing.
    expect_pending(assembler, {90.0, 180.0, 270.0, 359.0, 0.0, 90.0, 180.0, 359.0});
    const auto scan = assembler.push(point(0.0));
    ASSERT_TRUE(scan);
    EXPECT_EQ(scan->measurements.size(), 4U);
}

TEST(ScanAssemblerTest, RevolutionBoundariesMayOccurInsidePackets) {
    io::ScanAssembler assembler;
    const std::array<double, 12> first_angles{350, 352, 354, 356, 358, 0, 10, 20, 30, 40, 50, 60};
    const std::array<double, 12> second_angles{70, 90, 120, 150, 180, 210, 240, 270, 300, 330, 359, 0};
    std::vector<io::Scan> scans;
    for (const auto& angles : {first_angles, second_angles}) {
        io::ScanPacket packet{};
        for (std::size_t i = 0; i < angles.size(); ++i)
            packet.measurements[i] = point(angles[i]);
        for (const auto& measurement : packet.measurements)
            if (auto scan = assembler.push(measurement))
                scans.push_back(std::move(*scan));
    }
    ASSERT_EQ(scans.size(), 1U);
    ASSERT_EQ(scans[0].measurements.size(), 18U);
    EXPECT_DOUBLE_EQ(scans[0].measurements.front().angle, point(0.0).angle);
    EXPECT_DOUBLE_EQ(scans[0].measurements.back().angle, point(359.0).angle);
}

} // namespace
