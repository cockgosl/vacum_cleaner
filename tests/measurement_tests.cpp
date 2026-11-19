#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <vector>

#include "mapping/lidar.hpp"

using lidar::CellIndex;
using lidar::CellState;
using lidar::Grid;
using lidar::Lidar;
using lidar::SensorPose;

namespace {

void expect_map(const Grid& grid, const std::vector<CellIndex>& free_cells, CellIndex occupied) {
    for (std::size_t y = 0; y < grid.height(); ++y) {
        for (std::size_t x = 0; x < grid.width(); ++x) {
            auto expected = CellState::Unknown;
            for (const auto cell : free_cells)
                if (cell.x == x && cell.y == y)
                    expected = CellState::Free;
            if (occupied.x == x && occupied.y == y)
                expected = CellState::Occupied;
            EXPECT_EQ(grid.at(x, y), expected) << "x=" << x << ", y=" << y;
        }
    }
}

TEST(MeasurementTest, MapsFourCardinalAngles) {
    struct Case {
        const char* name;
        double angle;
        CellIndex hit;
        std::vector<CellIndex> free_cells;
    };
    const std::array<Case, 4> cases{{{"East", 0.0, {7, 4}, {{4, 4}, {5, 4}, {6, 4}}},
                                     {"North", std::numbers::pi / 2.0, {4, 7}, {{4, 4}, {4, 5}, {4, 6}}},
                                     {"West", std::numbers::pi, {1, 4}, {{4, 4}, {3, 4}, {2, 4}}},
                                     {"South", -std::numbers::pi / 2.0, {4, 1}, {{4, 4}, {4, 3}, {4, 2}}}}};
    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        auto sensor = Lidar::Create(10, 10, 1.0);
        ASSERT_TRUE(sensor.has_value());
        ASSERT_TRUE(sensor->update_measurement({4.5, 4.5, 0.0}, test.angle, 3.0));
        expect_map(sensor->grid(), test.free_cells, test.hit);
    }
}

TEST(MeasurementTest, CombinesSensorYawAndRelativeAngle) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor.has_value());

    ASSERT_TRUE(sensor->update_measurement({4.5, 4.5, std::numbers::pi / 2.0}, std::numbers::pi / 2.0, 3.0));
    expect_map(sensor->grid(), {{4, 4}, {3, 4}, {2, 4}}, {1, 4});
}

TEST(MeasurementTest, UsesMetresAndTranslatedSensorPosition) {
    auto sensor = Lidar::Create(10, 8, 0.25);
    ASSERT_TRUE(sensor.has_value());

    ASSERT_TRUE(sensor->update_measurement({1.125, 0.625, 0.0}, 0.0, 0.5));
    expect_map(sensor->grid(), {{4, 2}, {5, 2}}, {6, 2});
}

TEST(MeasurementTest, MapsObliqueMeasurementThroughCrossedCells) {
    auto sensor = Lidar::Create(8, 5, 1.0);
    ASSERT_TRUE(sensor.has_value());

    ASSERT_TRUE(sensor->update_measurement({1.25, 1.25, 0.0}, std::atan2(1.0, 3.5), std::hypot(3.5, 1.0)));
    expect_map(sensor->grid(), {{1, 1}, {2, 1}, {3, 1}, {3, 2}}, {4, 2});
}

TEST(MeasurementTest, HandlesAnglesBeyondOneRevolution) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor.has_value());

    ASSERT_TRUE(sensor->update_measurement({4.5, 4.5, -2.0 * std::numbers::pi}, 2.5 * std::numbers::pi, 3.0));
    expect_map(sensor->grid(), {{4, 4}, {4, 5}, {4, 6}}, {4, 7});
}

TEST(MeasurementTest, ReducesLargeFiniteAnglesBeforeAddingThem) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor.has_value());
    const double largest = std::numeric_limits<double>::max();

    EXPECT_TRUE(sensor->update_measurement({4.5, 4.5, largest}, largest, 1.0));
}

TEST(MeasurementTest, MarksHitInSensorCellOccupied) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor.has_value());

    ASSERT_TRUE(sensor->update_measurement({4.5, 4.5, 0.0}, 0.0, 0.1));
    expect_map(sensor->grid(), {}, {4, 4});
}

TEST(MeasurementTest, RejectsNonFiniteInputsWithoutChangingMap) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor.has_value());
    ASSERT_TRUE(sensor->update_ray(9.5, 9.5, 9.5, 9.5));

    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
        for (std::size_t field = 0; field < 5; ++field) {
            SCOPED_TRACE(field);
            SensorPose pose{4.5, 4.5, 0.0};
            double angle = 0.0;
            double distance = 1.0;
            switch (field) {
            case 0:
                pose.x = invalid;
                break;
            case 1:
                pose.y = invalid;
                break;
            case 2:
                pose.yaw = invalid;
                break;
            case 3:
                angle = invalid;
                break;
            case 4:
                distance = invalid;
                break;
            }
            EXPECT_FALSE(sensor->update_measurement(pose, angle, distance));
            expect_map(sensor->grid(), {}, {9, 9});
        }
    }
}

TEST(MeasurementTest, RejectsNonPositiveDistanceWithoutChangingMap) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor.has_value());
    ASSERT_TRUE(sensor->update_ray(9.5, 9.5, 9.5, 9.5));

    for (const double distance : {0.0, -0.1}) {
        EXPECT_FALSE(sensor->update_measurement({4.5, 4.5, 0.0}, 0.0, distance));
        expect_map(sensor->grid(), {}, {9, 9});
    }
}

TEST(MeasurementTest, RejectsSensorOutsideMapWithoutChangingMap) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor.has_value());
    ASSERT_TRUE(sensor->update_ray(9.5, 9.5, 9.5, 9.5));
    const std::array<SensorPose, 4> poses{{{-0.5, 4.5, 0.0},
                                           {10.0, 4.5, std::numbers::pi},
                                           {4.5, -0.5, std::numbers::pi / 2.0},
                                           {4.5, 10.0, -std::numbers::pi / 2.0}}};
    for (const auto& pose : poses) {
        EXPECT_FALSE(sensor->update_measurement(pose, 0.0, 3.0));
        expect_map(sensor->grid(), {}, {9, 9});
    }
}

TEST(MeasurementTest, RejectsHitsOutsideMapWithoutChangingMap) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor.has_value());
    ASSERT_TRUE(sensor->update_ray(9.5, 9.5, 9.5, 9.5));

    for (const double angle : {0.0, std::numbers::pi / 2.0, std::numbers::pi, -std::numbers::pi / 2.0}) {
        EXPECT_FALSE(sensor->update_measurement({4.5, 4.5, 0.0}, angle, 10.0));
        expect_map(sensor->grid(), {}, {9, 9});
    }
    EXPECT_FALSE(sensor->update_measurement({4.5, 4.5, 0.0}, 0.0, std::numeric_limits<double>::max()));
    expect_map(sensor->grid(), {}, {9, 9});
}


TEST(ScanMappingTest, MapsAllCardinalRaysAtOnePose) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor);
    const std::array<io::Measurement, 4> points{
        {{0.0, 3.0, 100}, {std::numbers::pi / 2.0, 3.0, 100}, {std::numbers::pi, 3.0, 100}, {-std::numbers::pi / 2.0, 3.0, 100}}};
    const auto update = sensor->update_scan({4.5, 4.5, 0.0}, points);
    ASSERT_TRUE(update);
    EXPECT_EQ(update->mapped, 4U);
    EXPECT_EQ(update->skipped, 0U);
    EXPECT_EQ(update->rejected, 0U);
    for (const auto cell : {CellIndex{7, 4}, {4, 7}, {1, 4}, {4, 1}})
        EXPECT_EQ(sensor->grid().at(cell.x, cell.y), CellState::Occupied);
    EXPECT_EQ(sensor->grid().at(4, 4), CellState::Free);
    EXPECT_EQ(sensor->grid().at(5, 4), CellState::Free);
    EXPECT_EQ(sensor->grid().at(5, 5), CellState::Unknown);
}

TEST(ScanMappingTest, KeepsHitsOccupiedWhenAnotherRayCrossesThem) {
    auto sensor = Lidar::Create(5, 5, 1.0);
    ASSERT_TRUE(sensor);
    const std::array<io::Measurement, 2> points{{{0.0, 1.0, 100}, {0.0, 3.0, 100}}};
    ASSERT_TRUE(sensor->update_scan({0.5, 2.5, 0.0}, points));
    EXPECT_EQ(sensor->grid().at(0, 2), CellState::Free);
    EXPECT_EQ(sensor->grid().at(1, 2), CellState::Occupied);
    EXPECT_EQ(sensor->grid().at(2, 2), CellState::Free);
    EXPECT_EQ(sensor->grid().at(3, 2), CellState::Occupied);
    EXPECT_EQ(sensor->grid().at(4, 2), CellState::Unknown);
}

TEST(ScanMappingTest, FinalCellsDoNotDependOnRayOrder) {
    auto forward = Lidar::Create(10, 10, 1.0);
    auto backward = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(forward);
    ASSERT_TRUE(backward);
    std::array<io::Measurement, 5> points{{{0.0, 1.0, 100},
                                           {0.0, 3.0, 100},
                                           {std::numbers::pi / 4.0, 3.0, 100},
                                           {std::numbers::pi / 2.0, 3.0, 100},
                                           {0.0, 0.0, 0}}};
    ASSERT_TRUE(forward->update_scan({4.5, 4.5, 0.0}, points));
    std::reverse(points.begin(), points.end());
    ASSERT_TRUE(backward->update_scan({4.5, 4.5, 0.0}, points));
    for (std::size_t y = 0; y < 10; ++y)
        for (std::size_t x = 0; x < 10; ++x)
            EXPECT_EQ(forward->grid().at(x, y), backward->grid().at(x, y));
}

TEST(ScanMappingTest, ANewScanCanClearAnOldHit) {
    auto sensor = Lidar::Create(5, 5, 1.0);
    ASSERT_TRUE(sensor);
    const std::array<io::Measurement, 2> initial{{{0.0, 1.0, 100}, {0.0, 3.0, 100}}};
    ASSERT_TRUE(sensor->update_scan({0.5, 2.5, 0.0}, initial));
    const std::array<io::Measurement, 1> current{{{0.0, 3.0, 100}}};
    ASSERT_TRUE(sensor->update_scan({0.5, 2.5, 0.0}, current));
    EXPECT_EQ(sensor->grid().at(1, 2), CellState::Free);
    EXPECT_EQ(sensor->grid().at(3, 2), CellState::Occupied);
}

TEST(ScanMappingTest, CountsSkippedAndRejectedMeasurementsIndividually) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor);
    const std::array<io::Measurement, 6> points{{{0.0, 1.0, 100},
                                                 {0.0, 0.0, 0},
                                                 {std::numeric_limits<double>::quiet_NaN(), 1.0, 100},
                                                 {0.0, -1.0, 100},
                                                 {0.0, 100.0, 100},
                                                 {0.0, std::numeric_limits<double>::infinity(), 100}}};
    const auto update = sensor->update_scan({4.5, 4.5, 0.0}, points);
    ASSERT_TRUE(update);
    EXPECT_EQ(update->mapped, 1U);
    EXPECT_EQ(update->skipped, 1U);
    EXPECT_EQ(update->rejected, 4U);
    expect_map(sensor->grid(), {{4, 4}}, {5, 4});
}

TEST(ScanMappingTest, RejectsInvalidPosesWithoutChangingAnyCell) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor);
    ASSERT_TRUE(sensor->update_ray(9.5, 9.5, 9.5, 9.5));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::array<SensorPose, 7> poses{{{-0.1, 4.5, 0.0},
                                           {10.0, 4.5, 0.0},
                                           {4.5, -0.1, 0.0},
                                           {4.5, 10.0, 0.0},
                                           {nan, 4.5, 0.0},
                                           {4.5, inf, 0.0},
                                           {4.5, 4.5, nan}}};
    const std::array<io::Measurement, 1> points{{{0.0, 1.0, 100}}};
    for (const auto pose : poses) {
        EXPECT_FALSE(sensor->update_scan(pose, points));
        expect_map(sensor->grid(), {}, {9, 9});
    }
}

TEST(ScanMappingTest, EmptyAndZeroReturnScansLeaveTheMapUnchanged) {
    auto sensor = Lidar::Create(10, 10, 1.0);
    ASSERT_TRUE(sensor);
    ASSERT_TRUE(sensor->update_ray(9.5, 9.5, 9.5, 9.5));
    const auto empty = sensor->update_scan({4.5, 4.5, 0.0}, {});
    ASSERT_TRUE(empty);
    EXPECT_EQ(empty->mapped + empty->skipped + empty->rejected, 0U);
    const std::array<io::Measurement, 2> zero{{{0.0, 0.0, 0}, {std::numbers::pi / 2.0, 0.0, 0}}};
    const auto update = sensor->update_scan({4.5, 4.5, 0.0}, zero);
    ASSERT_TRUE(update);
    EXPECT_EQ(update->mapped, 0U);
    EXPECT_EQ(update->skipped, 2U);
    EXPECT_EQ(update->rejected, 0U);
    expect_map(sensor->grid(), {}, {9, 9});
}

TEST(ScanMappingTest, AppliesTheSensorHeadingToEveryRay) {
    auto sensor = Lidar::Create(6, 6, 1.0);
    ASSERT_TRUE(sensor);
    const std::array<io::Measurement, 2> points{{{0.0, 2.0, 100}, {std::numbers::pi / 2.0, 1.0, 100}}};
    const auto update = sensor->update_scan({2.5, 2.5, std::numbers::pi / 2.0}, points);
    ASSERT_TRUE(update);
    EXPECT_EQ(update->mapped, 2U);
    EXPECT_EQ(sensor->grid().at(2, 4), CellState::Occupied);
    EXPECT_EQ(sensor->grid().at(1, 2), CellState::Occupied);
    EXPECT_EQ(sensor->grid().at(4, 2), CellState::Unknown);
}

} // namespace
