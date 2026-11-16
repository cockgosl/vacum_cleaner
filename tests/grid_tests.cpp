#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "lidar/grid.hpp"

using lidar::CellState;
using lidar::Grid;

TEST(GridTest, CreatesUnknownGridWithRequestedParameters) {
    const auto grid = Grid::Create(3, 2, 0.05);
    ASSERT_TRUE(grid.has_value());

    EXPECT_EQ(grid->width(), 3U);
    EXPECT_EQ(grid->height(), 2U);
    EXPECT_DOUBLE_EQ(grid->resolution(), 0.05);
    for (std::size_t y = 0; y < grid->height(); ++y)
        for (std::size_t x = 0; x < grid->width(); ++x)
            EXPECT_EQ(grid->at(x, y), CellState::Unknown) << "x=" << x << ", y=" << y;
}

TEST(GridTest, RejectsZeroDimensions) {
    EXPECT_FALSE(Grid::Create(0, 2, 0.05).has_value());
    EXPECT_FALSE(Grid::Create(3, 0, 0.05).has_value());
    EXPECT_FALSE(Grid::Create(0, 0, 0.05).has_value());
}

TEST(GridTest, RejectsInvalidResolution) {
    for (const double resolution : {0.0, -0.05, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                    -std::numeric_limits<double>::infinity()}) {
        EXPECT_FALSE(Grid::Create(3, 2, resolution).has_value()) << "resolution=" << resolution;
    }
}

TEST(GridTest, RejectsUnrepresentableDimensions) {
    constexpr auto largest = std::numeric_limits<std::size_t>::max();

    EXPECT_FALSE(Grid::Create(largest, 2, 0.05).has_value());
    EXPECT_FALSE(Grid::Create(2, largest, 0.05).has_value());
    EXPECT_FALSE(Grid::Create(largest, 1, 0.05).has_value());
}

TEST(GridTest, SupportsSingleCellAndRestoringUnknownState) {
    auto grid = Grid::Create(1, 1, 0.05);
    ASSERT_TRUE(grid.has_value());

    EXPECT_EQ(grid->at(0, 0), CellState::Unknown);
    grid->set(0, 0, CellState::Occupied);
    EXPECT_EQ(grid->at(0, 0), CellState::Occupied);
    grid->set(0, 0, CellState::Free);
    EXPECT_EQ(grid->at(0, 0), CellState::Free);
    grid->set(0, 0, CellState::Unknown);
    EXPECT_EQ(grid->at(0, 0), CellState::Unknown);
}

TEST(GridTest, UpdatesCellsIndependentlyOnRectangularGrid) {
    auto grid = Grid::Create(3, 2, 0.05);
    ASSERT_TRUE(grid.has_value());

    grid->set(1, 0, CellState::Free);
    grid->set(2, 1, CellState::Occupied);

    const std::array<std::array<CellState, 3>, 2> expected{{{CellState::Unknown, CellState::Free, CellState::Unknown},
                                                            {CellState::Unknown, CellState::Unknown, CellState::Occupied}}};
    for (std::size_t y = 0; y < expected.size(); ++y)
        for (std::size_t x = 0; x < expected[y].size(); ++x)
            EXPECT_EQ(grid->at(x, y), expected[y][x]) << "x=" << x << ", y=" << y;
}

TEST(GridTest, ThrowsWhenReadingOutsideGrid) {
    const auto grid = Grid::Create(3, 2, 0.05);
    ASSERT_TRUE(grid.has_value());

    EXPECT_THROW(grid->at(3, 0), std::out_of_range);
    EXPECT_THROW(grid->at(0, 2), std::out_of_range);
    EXPECT_THROW(grid->at(std::numeric_limits<std::size_t>::max(), 0), std::out_of_range);
    EXPECT_THROW(grid->at(0, std::numeric_limits<std::size_t>::max()), std::out_of_range);
}

TEST(GridTest, InvalidWritesPreserveExistingCells) {
    auto grid = Grid::Create(3, 2, 0.05);
    ASSERT_TRUE(grid.has_value());
    grid->set(0, 1, CellState::Free);

    // An invalid column must not alias the first cell of the next row.
    EXPECT_THROW(grid->set(3, 0, CellState::Occupied), std::out_of_range);
    EXPECT_THROW(grid->set(0, 2, CellState::Occupied), std::out_of_range);
    EXPECT_THROW(grid->set(std::numeric_limits<std::size_t>::max(), 0, CellState::Occupied), std::out_of_range);
    EXPECT_THROW(grid->set(0, std::numeric_limits<std::size_t>::max(), CellState::Occupied), std::out_of_range);

    for (std::size_t y = 0; y < grid->height(); ++y) {
        for (std::size_t x = 0; x < grid->width(); ++x) {
            const auto expected = (x == 0 && y == 1) ? CellState::Free : CellState::Unknown;
            EXPECT_EQ(grid->at(x, y), expected) << "x=" << x << ", y=" << y;
        }
    }
}

TEST(GridTest, ConvertsMetresToCellIndices) {
    const auto grid = Grid::Create(10, 6, 0.05);
    ASSERT_TRUE(grid.has_value());

    const auto cell = grid->world_to_cell(0.12, 0.07);
    ASSERT_TRUE(cell.has_value());
    EXPECT_EQ(cell->x, 2U);
    EXPECT_EQ(cell->y, 1U);
}

TEST(GridTest, MapsOriginToFirstCell) {
    const auto grid = Grid::Create(3, 2, 0.05);
    ASSERT_TRUE(grid.has_value());

    const auto cell = grid->world_to_cell(0.0, 0.0);
    ASSERT_TRUE(cell.has_value());
    EXPECT_EQ(cell->x, 0U);
    EXPECT_EQ(cell->y, 0U);
}

TEST(GridTest, InternalEdgesBelongToFollowingCell) {
    const auto grid = Grid::Create(4, 3, 0.125);
    ASSERT_TRUE(grid.has_value());

    const auto cell = grid->world_to_cell(0.125, 0.25);
    ASSERT_TRUE(cell.has_value());
    EXPECT_EQ(cell->x, 1U);
    EXPECT_EQ(cell->y, 2U);

    const auto before = grid->world_to_cell(std::nextafter(0.125, 0.0), std::nextafter(0.25, 0.0));
    ASSERT_TRUE(before.has_value());
    EXPECT_EQ(before->x, 0U);
    EXPECT_EQ(before->y, 1U);
}

TEST(GridTest, RejectsNegativeWorldCoordinates) {
    const auto grid = Grid::Create(3, 2, 0.05);
    ASSERT_TRUE(grid.has_value());

    EXPECT_FALSE(grid->world_to_cell(-0.01, 0.0).has_value());
    EXPECT_FALSE(grid->world_to_cell(0.0, -0.01).has_value());
}

TEST(GridTest, RejectsNonFiniteWorldCoordinates) {
    const auto grid = Grid::Create(3, 2, 0.05);
    ASSERT_TRUE(grid.has_value());

    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
        EXPECT_FALSE(grid->world_to_cell(invalid, 0.0).has_value());
        EXPECT_FALSE(grid->world_to_cell(0.0, invalid).has_value());
    }
}

TEST(GridTest, RejectsUpperEdgesAndPointsBeyondThem) {
    const auto grid = Grid::Create(17, 43, 0.1);
    ASSERT_TRUE(grid.has_value());

    const double right = grid->width() * grid->resolution();
    const double top = grid->height() * grid->resolution();
    EXPECT_FALSE(grid->world_to_cell(right, 0.0).has_value());
    EXPECT_FALSE(grid->world_to_cell(0.0, top).has_value());
    EXPECT_FALSE(grid->world_to_cell(right + 1.0, 0.0).has_value());
    EXPECT_FALSE(grid->world_to_cell(0.0, top + 1.0).has_value());
}

TEST(GridTest, AcceptsPointsImmediatelyInsideUpperEdges) {
    const auto grid = Grid::Create(17, 43, 0.1);
    ASSERT_TRUE(grid.has_value());

    const double right = grid->width() * grid->resolution();
    const double top = grid->height() * grid->resolution();
    const auto cell = grid->world_to_cell(std::nextafter(right, 0.0), std::nextafter(top, 0.0));
    ASSERT_TRUE(cell.has_value());
    EXPECT_EQ(cell->x, 16U);
    EXPECT_EQ(cell->y, 42U);
}

TEST(GridTest, RejectsHugeWorldCoordinatesAtSmallResolution) {
    const auto grid = Grid::Create(3, 2, std::numeric_limits<double>::min());
    ASSERT_TRUE(grid.has_value());

    const double largest = std::numeric_limits<double>::max();
    EXPECT_FALSE(grid->world_to_cell(largest, 0.0).has_value());
    EXPECT_FALSE(grid->world_to_cell(0.0, largest).has_value());
}
