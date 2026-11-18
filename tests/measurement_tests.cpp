#include <gtest/gtest.h>

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

} // namespace
