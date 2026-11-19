#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <limits>
#include <vector>

#include "rooting/scheduler.hpp"

using lidar::CellIndex;
using lidar::CellState;
using rooting::Direction;
using rooting::PlanStatus;
using rooting::Pose;
using rooting::Scheduler;

namespace {

lidar::Grid free_grid(std::size_t width, std::size_t height, double resolution = 1.0) {
    auto grid = lidar::Grid::Create(width, height, resolution).value();
    for (std::size_t y = 0; y < height; ++y)
        for (std::size_t x = 0; x < width; ++x)
            grid.set(x, y, CellState::Free);
    return grid;
}

void expect_path(const rooting::PlanResult& result, Pose start, CellIndex goal, std::size_t cost) {
    ASSERT_EQ(result.status, PlanStatus::Success);
    EXPECT_EQ(result.cost, cost);
    ASSERT_EQ(result.path.size(), cost + 1);
    ASSERT_FALSE(result.path.empty());
    EXPECT_EQ(result.path.front().cell.x, start.cell.x);
    EXPECT_EQ(result.path.front().cell.y, start.cell.y);
    EXPECT_EQ(result.path.front().direction, start.direction);
    EXPECT_EQ(result.path.back().cell.x, goal.x);
    EXPECT_EQ(result.path.back().cell.y, goal.y);
    for (std::size_t i = 1; i < result.path.size(); ++i) {
        const auto before = result.path[i - 1];
        const auto after = result.path[i];
        if (before.cell.x == after.cell.x && before.cell.y == after.cell.y) {
            const auto a = static_cast<unsigned>(before.direction);
            const auto b = static_cast<unsigned>(after.direction);
            EXPECT_TRUE((a + 1) % 4 == b || (a + 3) % 4 == b);
        } else {
            EXPECT_EQ(after.direction, before.direction);
            switch (before.direction) {
            case Direction::East:
                EXPECT_EQ(after.cell.x, before.cell.x + 1);
                EXPECT_EQ(after.cell.y, before.cell.y);
                break;
            case Direction::North:
                EXPECT_EQ(after.cell.x, before.cell.x);
                EXPECT_EQ(after.cell.y, before.cell.y + 1);
                break;
            case Direction::West:
                EXPECT_EQ(after.cell.x + 1, before.cell.x);
                EXPECT_EQ(after.cell.y, before.cell.y);
                break;
            case Direction::South:
                EXPECT_EQ(after.cell.x, before.cell.x);
                EXPECT_EQ(after.cell.y + 1, before.cell.y);
                break;
            }
        }
    }
}

TEST(SchedulerTest, FindsShortestFourNeighbourPathOnOpenMap) {
    const auto grid = free_grid(7, 6);
    expect_path(Scheduler{}.plan(grid, {{1, 1}, Direction::East}, {4, 3}, 0.2), {{1, 1}, Direction::East}, {4, 3}, 6);
}

TEST(SchedulerTest, FindsShortestDetourAroundWall) {
    auto grid = free_grid(7, 7);
    for (std::size_t y = 0; y < 5; ++y)
        grid.set(3, y, CellState::Occupied);
    const auto result = Scheduler{}.plan(grid, {{1, 2}, Direction::East}, {5, 2}, 0.2);
    expect_path(result, {{1, 2}, Direction::East}, {5, 2}, 13);
    for (const auto pose : result.path)
        EXPECT_EQ(grid.at(pose.cell.x, pose.cell.y), CellState::Free);
}

TEST(SchedulerTest, CannotCrossOccupiedWall) {
    auto grid = free_grid(7, 7);
    for (std::size_t y = 0; y < grid.height(); ++y)
        grid.set(3, y, CellState::Occupied);
    const auto result = Scheduler{}.plan(grid, {{1, 3}, Direction::East}, {5, 3}, 0.2);
    EXPECT_EQ(result.status, PlanStatus::NoPath);
    EXPECT_TRUE(result.path.empty());
}

TEST(SchedulerTest, CannotCrossUnknownWall) {
    auto grid = free_grid(7, 7);
    for (std::size_t y = 0; y < grid.height(); ++y)
        grid.set(3, y, CellState::Unknown);
    EXPECT_EQ(Scheduler{}.plan(grid, {{1, 3}, Direction::East}, {5, 3}, 0.2).status, PlanStatus::NoPath);
}

TEST(SchedulerTest, RadiusDeterminesWhetherNarrowPassageIsUsable) {
    auto grid = free_grid(9, 7);
    for (std::size_t x = 3; x <= 5; ++x)
        for (std::size_t y = 0; y < grid.height(); ++y)
            if (y != 3)
                grid.set(x, y, CellState::Occupied);
    expect_path(Scheduler{}.plan(grid, {{1, 3}, Direction::East}, {7, 3}, 0.49), {{1, 3}, Direction::East}, {7, 3}, 6);
    EXPECT_EQ(Scheduler{}.plan(grid, {{1, 3}, Direction::East}, {7, 3}, 0.5).status, PlanStatus::NoPath);
    EXPECT_EQ(Scheduler{}.plan(grid, {{1, 3}, Direction::East}, {7, 3}, 0.51).status, PlanStatus::NoPath);
}

TEST(SchedulerTest, ChecksEntireObstacleCellIncludingCorners) {
    auto grid = free_grid(6, 6);
    grid.set(3, 3, CellState::Occupied);
    expect_path(Scheduler{}.plan(grid, {{2, 2}, Direction::East}, {2, 2}, 0.7), {{2, 2}, Direction::East}, {2, 2}, 0);
    EXPECT_EQ(Scheduler{}.plan(grid, {{2, 2}, Direction::East}, {2, 2}, 0.71).status, PlanStatus::InvalidStart);
}

TEST(SchedulerTest, TreatsTouchingUnknownCellAsCollision) {
    auto grid = free_grid(5, 5);
    grid.set(3, 2, CellState::Unknown);
    expect_path(Scheduler{}.plan(grid, {{2, 2}, Direction::East}, {2, 2}, 0.49), {{2, 2}, Direction::East}, {2, 2}, 0);
    EXPECT_EQ(Scheduler{}.plan(grid, {{2, 2}, Direction::East}, {2, 2}, 0.5).status, PlanStatus::InvalidStart);
}

TEST(SchedulerTest, RejectsStartOrGoalTouchingMapBoundary) {
    const auto grid = free_grid(5, 5);
    EXPECT_EQ(Scheduler{}.plan(grid, {{0, 2}, Direction::East}, {2, 2}, 0.5).status, PlanStatus::InvalidStart);
    EXPECT_EQ(Scheduler{}.plan(grid, {{2, 2}, Direction::East}, {4, 2}, 0.5).status, PlanStatus::InvalidGoal);
}

TEST(SchedulerTest, RejectsInvalidOrBlockedEndpoints) {
    auto grid = free_grid(5, 5);
    grid.set(2, 2, CellState::Occupied);
    EXPECT_EQ(Scheduler{}.plan(grid, {{5, 1}, Direction::East}, {1, 1}, 0.2).status, PlanStatus::InvalidStart);
    EXPECT_EQ(Scheduler{}.plan(grid, {{1, 1}, Direction::East}, {1, 5}, 0.2).status, PlanStatus::InvalidGoal);
    EXPECT_EQ(Scheduler{}.plan(grid, {{2, 2}, Direction::East}, {1, 1}, 0.2).status, PlanStatus::InvalidStart);
    EXPECT_EQ(Scheduler{}.plan(grid, {{1, 1}, Direction::East}, {2, 2}, 0.2).status, PlanStatus::InvalidGoal);
}

TEST(SchedulerTest, RejectsInvalidRadius) {
    const auto grid = free_grid(5, 5);
    for (const double radius : {0.0, -0.2, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity()}) {
        const auto result = Scheduler{}.plan(grid, {{1, 1}, Direction::East}, {3, 3}, radius);
        EXPECT_EQ(result.status, PlanStatus::InvalidParameters);
        EXPECT_TRUE(result.path.empty());
    }
}

TEST(SchedulerTest, ConvertsRadiusFromMetresUsingResolution) {
    const auto grid = free_grid(5, 5, 0.1);
    expect_path(Scheduler{}.plan(grid, {{1, 1}, Direction::East}, {3, 3}, 0.04), {{1, 1}, Direction::East}, {3, 3}, 5);
    EXPECT_EQ(Scheduler{}.plan(grid, {{1, 1}, Direction::East}, {3, 3}, 0.16).status, PlanStatus::InvalidStart);
}

TEST(SchedulerTest, LeavesMapUnchanged) {
    auto grid = free_grid(5, 5);
    grid.set(2, 2, CellState::Occupied);
    std::vector<CellState> before;
    for (std::size_t y = 0; y < grid.height(); ++y)
        for (std::size_t x = 0; x < grid.width(); ++x)
            before.push_back(grid.at(x, y));
    ASSERT_EQ(Scheduler{}.plan(grid, {{1, 1}, Direction::East}, {3, 3}, 0.2).status, PlanStatus::Success);
    for (std::size_t y = 0; y < grid.height(); ++y)
        for (std::size_t x = 0; x < grid.width(); ++x)
            EXPECT_EQ(grid.at(x, y), before[y * grid.width() + x]);
}


TEST(SchedulerTest, InitialHeadingChangesRouteCost) {
    const auto grid = free_grid(5, 5);
    expect_path(Scheduler{}.plan(grid, {{1, 2}, Direction::East}, {3, 2}, 0.2), {{1, 2}, Direction::East}, {3, 2}, 2);
    expect_path(Scheduler{}.plan(grid, {{1, 2}, Direction::North}, {3, 2}, 0.2), {{1, 2}, Direction::North}, {3, 2}, 3);
    expect_path(Scheduler{}.plan(grid, {{1, 2}, Direction::West}, {3, 2}, 0.2), {{1, 2}, Direction::West}, {3, 2}, 4);
}

TEST(SchedulerTest, PrefersFewerTurnsAmongEqualLengthRoutes) {
    const auto grid = free_grid(5, 5);
    const auto result = Scheduler{}.plan(grid, {{1, 1}, Direction::East}, {3, 3}, 0.2);
    expect_path(result, {{1, 1}, Direction::East}, {3, 3}, 5);
    std::size_t turns = 0;
    for (std::size_t i = 1; i < result.path.size(); ++i)
        if (result.path[i].direction != result.path[i - 1].direction)
            ++turns;
    EXPECT_EQ(turns, 1U);
}

TEST(SchedulerTest, CanTurnAroundInSingleCellWideCorridor) {
    auto grid = free_grid(5, 3);
    for (std::size_t x = 0; x < grid.width(); ++x) {
        grid.set(x, 0, CellState::Occupied);
        grid.set(x, 2, CellState::Occupied);
    }
    const auto result = Scheduler{}.plan(grid, {{3, 1}, Direction::East}, {1, 1}, 0.2);
    expect_path(result, {{3, 1}, Direction::East}, {1, 1}, 4);
}

TEST(SchedulerTest, AlreadyAtGoalPreservesHeadingWithoutTurning) {
    const auto grid = free_grid(3, 3);
    expect_path(Scheduler{}.plan(grid, {{1, 1}, Direction::South}, {1, 1}, 0.2), {{1, 1}, Direction::South}, {1, 1}, 0);
}

TEST(SchedulerTest, PlansForwardInAllFourDirections) {
    const auto grid = free_grid(5, 5);
    expect_path(Scheduler{}.plan(grid, {{2, 2}, Direction::East}, {3, 2}, 0.2), {{2, 2}, Direction::East}, {3, 2}, 1);
    expect_path(Scheduler{}.plan(grid, {{2, 2}, Direction::North}, {2, 3}, 0.2), {{2, 2}, Direction::North}, {2, 3}, 1);
    expect_path(Scheduler{}.plan(grid, {{2, 2}, Direction::West}, {1, 2}, 0.2), {{2, 2}, Direction::West}, {1, 2}, 1);
    expect_path(Scheduler{}.plan(grid, {{2, 2}, Direction::South}, {2, 1}, 0.2), {{2, 2}, Direction::South}, {2, 1}, 1);
}

TEST(SchedulerTest, RejectsInvalidHeading) {
    const auto grid = free_grid(5, 5);
    const auto result = Scheduler{}.plan(grid, {{1, 1}, static_cast<Direction>(255)}, {3, 3}, 0.2);
    EXPECT_EQ(result.status, PlanStatus::InvalidStart);
    EXPECT_TRUE(result.path.empty());
    EXPECT_EQ(result.cost, 0U);
}


TEST(ContinuousMotionTest, AllowsClearDiagonalSegmentsInBothDirections) {
    const auto grid = free_grid(7, 7);
    EXPECT_TRUE(Scheduler{}.can_move(grid, 1.25, 1.75, 4.5, 5.0, 0.2));
    EXPECT_TRUE(Scheduler{}.can_move(grid, 4.5, 5.0, 1.25, 1.75, 0.2));
}

TEST(ContinuousMotionTest, DetectsAnObstacleBetweenSafeEndpoints) {
    auto grid = free_grid(7, 7);
    grid.set(3, 3, CellState::Occupied);
    const Scheduler scheduler;
    ASSERT_TRUE(scheduler.can_move(grid, 1.5, 1.5, 1.5, 1.5, 0.1));
    ASSERT_TRUE(scheduler.can_move(grid, 5.5, 5.5, 5.5, 5.5, 0.1));
    EXPECT_FALSE(scheduler.can_move(grid, 1.5, 1.5, 5.5, 5.5, 0.1));
}

TEST(ContinuousMotionTest, ChecksRoundedCornerClearanceInsteadOfOnlyBoundingBoxes) {
    auto grid = free_grid(7, 7);
    grid.set(3, 3, CellState::Occupied);
    // The segment lies on x+y=5.5, at sqrt(0.125) from the corner (3,3).
    EXPECT_TRUE(Scheduler{}.can_move(grid, 2.0, 3.5, 3.5, 2.0, 0.35));
    EXPECT_FALSE(Scheduler{}.can_move(grid, 2.0, 3.5, 3.5, 2.0, 0.36));
    EXPECT_FALSE(Scheduler{}.can_move(grid, 3.5, 2.0, 2.0, 3.5, 0.36));
}

TEST(ContinuousMotionTest, TreatsSideTangencyAsCollision) {
    auto grid = free_grid(7, 7);
    grid.set(3, 3, CellState::Occupied);
    EXPECT_TRUE(Scheduler{}.can_move(grid, 1.5, 2.5, 5.5, 2.5, 0.49));
    EXPECT_FALSE(Scheduler{}.can_move(grid, 1.5, 2.5, 5.5, 2.5, 0.5));
}

TEST(ContinuousMotionTest, ChecksTheActualFootprintAwayFromTheCellCentre) {
    auto grid = free_grid(7, 7);
    grid.set(3, 2, CellState::Occupied);
    EXPECT_TRUE(Scheduler{}.can_move(grid, 2.5, 2.5, 2.5, 2.5, 0.2));
    EXPECT_FALSE(Scheduler{}.can_move(grid, 2.9, 2.5, 2.5, 2.5, 0.2));
}

TEST(ContinuousMotionTest, HandlesZeroLengthSegments) {
    auto grid = free_grid(7, 7);
    grid.set(3, 2, CellState::Occupied);
    EXPECT_TRUE(Scheduler{}.can_move(grid, 2.75, 2.5, 2.75, 2.5, 0.24));
    EXPECT_FALSE(Scheduler{}.can_move(grid, 2.75, 2.5, 2.75, 2.5, 0.25));
    EXPECT_FALSE(Scheduler{}.can_move(grid, 3.5, 2.5, 3.5, 2.5, 0.1));
}

TEST(ContinuousMotionTest, UnknownCellsBlockTheSweptFootprint) {
    auto grid = free_grid(7, 7);
    grid.set(3, 3, CellState::Unknown);
    EXPECT_FALSE(Scheduler{}.can_move(grid, 2.5, 2.5, 4.5, 4.5, 0.1));
}

TEST(ContinuousMotionTest, RejectsTouchingAndOutsideMapFootprints) {
    const auto grid = free_grid(5, 5);
    EXPECT_FALSE(Scheduler{}.can_move(grid, 0.5, 2.5, 2.5, 2.5, 0.5));
    EXPECT_FALSE(Scheduler{}.can_move(grid, 2.5, 2.5, 4.5, 2.5, 0.5));
    EXPECT_FALSE(Scheduler{}.can_move(grid, -0.1, 2.5, 2.5, 2.5, 0.1));
    EXPECT_FALSE(Scheduler{}.can_move(grid, 2.5, 2.5, 5.1, 2.5, 0.1));
    EXPECT_TRUE(Scheduler{}.can_move(grid, 0.75, 2.5, 2.5, 2.5, 0.5));
}

TEST(ContinuousMotionTest, ConvertsWorldCoordinatesAndRadiusFromMetres) {
    auto grid = free_grid(10, 10, 0.25);
    EXPECT_TRUE(Scheduler{}.can_move(grid, 0.625, 0.625, 1.125, 1.125, 0.05));
    grid.set(3, 3, CellState::Occupied);
    EXPECT_FALSE(Scheduler{}.can_move(grid, 0.625, 0.625, 1.125, 1.125, 0.05));
}

TEST(ContinuousMotionTest, RejectsInvalidNumbersAndNonPositiveRadius) {
    const auto grid = free_grid(7, 7);
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
        EXPECT_FALSE(Scheduler{}.can_move(grid, invalid, 1.5, 4.5, 4.5, 0.2));
        EXPECT_FALSE(Scheduler{}.can_move(grid, 1.5, invalid, 4.5, 4.5, 0.2));
        EXPECT_FALSE(Scheduler{}.can_move(grid, 1.5, 1.5, invalid, 4.5, 0.2));
        EXPECT_FALSE(Scheduler{}.can_move(grid, 1.5, 1.5, 4.5, invalid, 0.2));
        EXPECT_FALSE(Scheduler{}.can_move(grid, 1.5, 1.5, 4.5, 4.5, invalid));
    }
    EXPECT_FALSE(Scheduler{}.can_move(grid, 1.5, 1.5, 4.5, 4.5, 0.0));
    EXPECT_FALSE(Scheduler{}.can_move(grid, 1.5, 1.5, 4.5, 4.5, -0.2));
}

TEST(ContinuousMotionTest, AcceptsEverySegmentOfAReturnedGridRoute) {
    auto grid = free_grid(7, 7);
    for (std::size_t y = 0; y < 5; ++y)
        grid.set(3, y, CellState::Occupied);
    const Scheduler scheduler;
    const auto route = scheduler.plan(grid, {{1, 2}, Direction::East}, {5, 2}, 0.2);
    ASSERT_EQ(route.status, PlanStatus::Success);
    for (std::size_t i = 1; i < route.path.size(); ++i) {
        const auto from = route.path[i - 1].cell;
        const auto to = route.path[i].cell;
        EXPECT_TRUE(scheduler.can_move(grid, from.x + 0.5, from.y + 0.5, to.x + 0.5, to.y + 0.5, 0.2));
    }
}

} // namespace
