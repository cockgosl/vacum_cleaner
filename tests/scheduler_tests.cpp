#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <vector>

#include "rooting/scheduler.hpp"

using lidar::CellIndex;
using lidar::CellState;
using rooting::PlanStatus;
using rooting::Scheduler;

namespace {

lidar::Grid free_grid(std::size_t width, std::size_t height, double resolution = 1.0) {
    auto grid = lidar::Grid::Create(width, height, resolution).value();
    for (std::size_t y = 0; y < height; ++y)
        for (std::size_t x = 0; x < width; ++x)
            grid.set(x, y, CellState::Free);
    return grid;
}

void expect_path(const rooting::PlanResult& result, CellIndex start, CellIndex goal, std::size_t size) {
    ASSERT_EQ(result.status, PlanStatus::Success);
    ASSERT_EQ(result.path.size(), size);
    ASSERT_FALSE(result.path.empty());
    EXPECT_EQ(result.path.front().x, start.x);
    EXPECT_EQ(result.path.front().y, start.y);
    EXPECT_EQ(result.path.back().x, goal.x);
    EXPECT_EQ(result.path.back().y, goal.y);
    for (std::size_t i = 1; i < result.path.size(); ++i) {
        const auto a = result.path[i - 1];
        const auto b = result.path[i];
        const auto dx = a.x > b.x ? a.x - b.x : b.x - a.x;
        const auto dy = a.y > b.y ? a.y - b.y : b.y - a.y;
        EXPECT_EQ(dx + dy, 1U);
    }
}

TEST(SchedulerTest, FindsShortestFourNeighbourPathOnOpenMap) {
    const auto grid = free_grid(7, 6);
    expect_path(Scheduler{}.plan(grid, {1, 1}, {4, 3}, 0.2), {1, 1}, {4, 3}, 6);
}

TEST(SchedulerTest, FindsShortestDetourAroundWall) {
    auto grid = free_grid(7, 7);
    for (std::size_t y = 0; y < 5; ++y)
        grid.set(3, y, CellState::Occupied);
    const auto result = Scheduler{}.plan(grid, {1, 2}, {5, 2}, 0.2);
    expect_path(result, {1, 2}, {5, 2}, 11);
    for (const auto cell : result.path)
        EXPECT_EQ(grid.at(cell.x, cell.y), CellState::Free);
}

TEST(SchedulerTest, CannotCrossOccupiedWall) {
    auto grid = free_grid(7, 7);
    for (std::size_t y = 0; y < grid.height(); ++y)
        grid.set(3, y, CellState::Occupied);
    const auto result = Scheduler{}.plan(grid, {1, 3}, {5, 3}, 0.2);
    EXPECT_EQ(result.status, PlanStatus::NoPath);
    EXPECT_TRUE(result.path.empty());
}

TEST(SchedulerTest, CannotCrossUnknownWall) {
    auto grid = free_grid(7, 7);
    for (std::size_t y = 0; y < grid.height(); ++y)
        grid.set(3, y, CellState::Unknown);
    EXPECT_EQ(Scheduler{}.plan(grid, {1, 3}, {5, 3}, 0.2).status, PlanStatus::NoPath);
}

TEST(SchedulerTest, RadiusDeterminesWhetherNarrowPassageIsUsable) {
    auto grid = free_grid(9, 7);
    for (std::size_t x = 3; x <= 5; ++x)
        for (std::size_t y = 0; y < grid.height(); ++y)
            if (y != 3)
                grid.set(x, y, CellState::Occupied);
    expect_path(Scheduler{}.plan(grid, {1, 3}, {7, 3}, 0.49), {1, 3}, {7, 3}, 7);
    EXPECT_EQ(Scheduler{}.plan(grid, {1, 3}, {7, 3}, 0.5).status, PlanStatus::NoPath);
    EXPECT_EQ(Scheduler{}.plan(grid, {1, 3}, {7, 3}, 0.51).status, PlanStatus::NoPath);
}

TEST(SchedulerTest, ChecksEntireObstacleCellIncludingCorners) {
    auto grid = free_grid(6, 6);
    grid.set(3, 3, CellState::Occupied);
    expect_path(Scheduler{}.plan(grid, {2, 2}, {2, 2}, 0.7), {2, 2}, {2, 2}, 1);
    EXPECT_EQ(Scheduler{}.plan(grid, {2, 2}, {2, 2}, 0.71).status, PlanStatus::InvalidStart);
}

TEST(SchedulerTest, TreatsTouchingUnknownCellAsCollision) {
    auto grid = free_grid(5, 5);
    grid.set(3, 2, CellState::Unknown);
    expect_path(Scheduler{}.plan(grid, {2, 2}, {2, 2}, 0.49), {2, 2}, {2, 2}, 1);
    EXPECT_EQ(Scheduler{}.plan(grid, {2, 2}, {2, 2}, 0.5).status, PlanStatus::InvalidStart);
}

TEST(SchedulerTest, RejectsStartOrGoalTouchingMapBoundary) {
    const auto grid = free_grid(5, 5);
    EXPECT_EQ(Scheduler{}.plan(grid, {0, 2}, {2, 2}, 0.5).status, PlanStatus::InvalidStart);
    EXPECT_EQ(Scheduler{}.plan(grid, {2, 2}, {4, 2}, 0.5).status, PlanStatus::InvalidGoal);
}

TEST(SchedulerTest, RejectsInvalidOrBlockedEndpoints) {
    auto grid = free_grid(5, 5);
    grid.set(2, 2, CellState::Occupied);
    EXPECT_EQ(Scheduler{}.plan(grid, {5, 1}, {1, 1}, 0.2).status, PlanStatus::InvalidStart);
    EXPECT_EQ(Scheduler{}.plan(grid, {1, 1}, {1, 5}, 0.2).status, PlanStatus::InvalidGoal);
    EXPECT_EQ(Scheduler{}.plan(grid, {2, 2}, {1, 1}, 0.2).status, PlanStatus::InvalidStart);
    EXPECT_EQ(Scheduler{}.plan(grid, {1, 1}, {2, 2}, 0.2).status, PlanStatus::InvalidGoal);
}

TEST(SchedulerTest, RejectsInvalidRadius) {
    const auto grid = free_grid(5, 5);
    for (const double radius : {0.0, -0.2, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity()}) {
        const auto result = Scheduler{}.plan(grid, {1, 1}, {3, 3}, radius);
        EXPECT_EQ(result.status, PlanStatus::InvalidParameters);
        EXPECT_TRUE(result.path.empty());
    }
}

TEST(SchedulerTest, ConvertsRadiusFromMetresUsingResolution) {
    const auto grid = free_grid(5, 5, 0.1);
    expect_path(Scheduler{}.plan(grid, {1, 1}, {3, 3}, 0.04), {1, 1}, {3, 3}, 5);
    EXPECT_EQ(Scheduler{}.plan(grid, {1, 1}, {3, 3}, 0.16).status, PlanStatus::InvalidStart);
}

TEST(SchedulerTest, LeavesMapUnchanged) {
    auto grid = free_grid(5, 5);
    grid.set(2, 2, CellState::Occupied);
    std::vector<CellState> before;
    for (std::size_t y = 0; y < grid.height(); ++y)
        for (std::size_t x = 0; x < grid.width(); ++x)
            before.push_back(grid.at(x, y));
    ASSERT_EQ(Scheduler{}.plan(grid, {1, 1}, {3, 3}, 0.2).status, PlanStatus::Success);
    for (std::size_t y = 0; y < grid.height(); ++y)
        for (std::size_t x = 0; x < grid.width(); ++x)
            EXPECT_EQ(grid.at(x, y), before[y * grid.width() + x]);
}

} // namespace
