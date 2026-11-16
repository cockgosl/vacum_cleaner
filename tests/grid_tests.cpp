#include <gtest/gtest.h>

#include <array>
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
