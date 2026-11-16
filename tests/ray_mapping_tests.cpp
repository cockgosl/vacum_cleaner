#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <ostream>
#include <vector>

#include "mapping/grid.hpp"

using lidar::CellIndex;
using lidar::CellState;
using lidar::Grid;
using lidar::update_ray;

namespace {

void expect_map(const Grid& grid, const std::vector<CellIndex>& free_cells, CellIndex hit) {
    for (std::size_t y = 0; y < grid.height(); ++y) {
        for (std::size_t x = 0; x < grid.width(); ++x) {
            const bool free =
                std::any_of(free_cells.begin(), free_cells.end(), [x, y](CellIndex cell) { return cell.x == x && cell.y == y; });
            const auto expected =
                (x == hit.x && y == hit.y) ? CellState::Occupied : (free ? CellState::Free : CellState::Unknown);
            EXPECT_EQ(grid.at(x, y), expected) << "x=" << x << ", y=" << y;
        }
    }
}

struct RayCase {
    const char* name;
    double start_x;
    double start_y;
    double hit_x;
    double hit_y;
    std::vector<CellIndex> free_cells;
    CellIndex hit;
};

void PrintTo(const RayCase& ray, std::ostream* stream) {
    *stream << ray.name;
}

class RayDirectionTest : public testing::TestWithParam<RayCase> {};

TEST_P(RayDirectionTest, UpdatesOnlyTraversedCells) {
    auto grid = Grid::Create(6, 6, 1.0);
    ASSERT_TRUE(grid.has_value());
    const auto& ray = GetParam();

    ASSERT_TRUE(update_ray(*grid, ray.start_x, ray.start_y, ray.hit_x, ray.hit_y));
    expect_map(*grid, ray.free_cells, ray.hit);
}

INSTANTIATE_TEST_SUITE_P(
    Directions, RayDirectionTest,
    testing::Values(RayCase{"HorizontalForward", 0.5, 2.5, 4.5, 2.5, {{0, 2}, {1, 2}, {2, 2}, {3, 2}}, {4, 2}},
                    RayCase{"HorizontalReverse", 4.5, 2.5, 0.5, 2.5, {{4, 2}, {3, 2}, {2, 2}, {1, 2}}, {0, 2}},
                    RayCase{"VerticalForward", 2.5, 0.5, 2.5, 4.5, {{2, 0}, {2, 1}, {2, 2}, {2, 3}}, {2, 4}},
                    RayCase{"VerticalReverse", 2.5, 4.5, 2.5, 0.5, {{2, 4}, {2, 3}, {2, 2}, {2, 1}}, {2, 0}},
                    RayCase{"DiagonalForward", 0.5, 0.5, 3.5, 3.5, {{0, 0}, {1, 1}, {2, 2}}, {3, 3}},
                    RayCase{"DiagonalReverse", 3.5, 3.5, 0.5, 0.5, {{3, 3}, {2, 2}, {1, 1}}, {0, 0}},
                    RayCase{"DiagonalDown", 0.5, 3.5, 3.5, 0.5, {{0, 3}, {1, 2}, {2, 1}}, {3, 0}},
                    RayCase{"DiagonalUp", 3.5, 0.5, 0.5, 3.5, {{3, 0}, {2, 1}, {1, 2}}, {0, 3}},
                    RayCase{"ShallowForward", 0.25, 0.25, 3.75, 1.25, {{0, 0}, {1, 0}, {2, 0}, {2, 1}}, {3, 1}},
                    RayCase{"ShallowReverse", 3.75, 1.25, 0.25, 0.25, {{3, 1}, {2, 1}, {2, 0}, {1, 0}}, {0, 0}},
                    RayCase{"SteepForward", 0.25, 0.25, 1.25, 3.75, {{0, 0}, {0, 1}, {0, 2}, {1, 2}}, {1, 3}},
                    RayCase{"SteepReverse", 1.25, 3.75, 0.25, 0.25, {{1, 3}, {1, 2}, {0, 2}, {0, 1}}, {0, 0}}),
    [](const testing::TestParamInfo<RayCase>& info) { return info.param.name; });

TEST(RayMappingTest, MarksSameCellOccupied) {
    auto grid = Grid::Create(4, 4, 1.0);
    ASSERT_TRUE(grid.has_value());

    ASSERT_TRUE(update_ray(*grid, 0.1, 0.1, 0.9, 0.9));
    expect_map(*grid, {}, {0, 0});
}

TEST(RayMappingTest, HandlesCoincidentEndpoints) {
    auto grid = Grid::Create(4, 4, 1.0);
    ASSERT_TRUE(grid.has_value());

    ASSERT_TRUE(update_ray(*grid, 1.5, 1.5, 1.5, 1.5));
    expect_map(*grid, {}, {1, 1});
}

TEST(RayMappingTest, HandlesEndpointsOnCellBoundaries) {
    auto grid = Grid::Create(4, 4, 1.0);
    ASSERT_TRUE(grid.has_value());

    ASSERT_TRUE(update_ray(*grid, 1.0, 1.0, 3.0, 1.0));
    expect_map(*grid, {{1, 1}, {2, 1}}, {3, 1});
}

TEST(RayMappingTest, HandlesReverseRayStartingOnBoundary) {
    auto grid = Grid::Create(4, 4, 1.0);
    ASSERT_TRUE(grid.has_value());

    ASSERT_TRUE(update_ray(*grid, 3.0, 1.0, 1.0, 1.0));
    expect_map(*grid, {{3, 1}, {2, 1}}, {1, 1});
}

TEST(RayMappingTest, UsesGridResolutionForMetreCoordinates) {
    auto grid = Grid::Create(6, 6, 0.5);
    ASSERT_TRUE(grid.has_value());

    ASSERT_TRUE(update_ray(*grid, 0.25, 0.75, 1.75, 0.75));
    expect_map(*grid, {{0, 1}, {1, 1}, {2, 1}}, {3, 1});
}

TEST(RayMappingTest, PreservesPreviousObservationsOutsideRay) {
    auto grid = Grid::Create(6, 6, 1.0);
    ASSERT_TRUE(grid.has_value());
    grid->set(5, 5, CellState::Free);
    grid->set(0, 5, CellState::Occupied);

    ASSERT_TRUE(update_ray(*grid, 0.5, 2.5, 4.5, 2.5));
    EXPECT_EQ(grid->at(5, 5), CellState::Free);
    EXPECT_EQ(grid->at(0, 5), CellState::Occupied);
    EXPECT_EQ(grid->at(5, 2), CellState::Unknown);
}

TEST(RayMappingTest, ReplacesPreviousStatesAlongRay) {
    auto grid = Grid::Create(6, 6, 1.0);
    ASSERT_TRUE(grid.has_value());
    grid->set(1, 2, CellState::Occupied);
    grid->set(4, 2, CellState::Free);

    ASSERT_TRUE(update_ray(*grid, 0.5, 2.5, 4.5, 2.5));
    expect_map(*grid, {{0, 2}, {1, 2}, {2, 2}, {3, 2}}, {4, 2});
}

TEST(RayMappingTest, RejectsInvalidEndpointsWithoutChangingGrid) {
    auto grid = Grid::Create(4, 4, 1.0);
    ASSERT_TRUE(grid.has_value());
    grid->set(1, 1, CellState::Occupied);

    for (const double invalid : {-0.1, 4.0, 10.0, std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        EXPECT_FALSE(update_ray(*grid, invalid, 0.5, 2.5, 2.5));
        expect_map(*grid, {}, {1, 1});
        EXPECT_FALSE(update_ray(*grid, 0.5, invalid, 2.5, 2.5));
        expect_map(*grid, {}, {1, 1});
        EXPECT_FALSE(update_ray(*grid, 0.5, 0.5, invalid, 2.5));
        expect_map(*grid, {}, {1, 1});
        EXPECT_FALSE(update_ray(*grid, 0.5, 0.5, 2.5, invalid));
        expect_map(*grid, {}, {1, 1});
    }
}

} // namespace
