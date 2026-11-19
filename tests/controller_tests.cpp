#include "motion/controller.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <numbers>

namespace {

using namespace std::chrono_literals;
using motion::ControlStatus;
using rooting::Direction;

class ControllerTest : public testing::Test {
protected:
    lidar::Grid grid = *lidar::Grid::Create(7, 7, 1.0);
    motion::Controller controller = *motion::Controller::Create();
    lidar::SensorPose pose{2.5, 2.5, 0.0};
    static constexpr double radius = 0.2;

    void SetUp() override {
        for (std::size_t y = 0; y < grid.height(); ++y)
            for (std::size_t x = 0; x < grid.width(); ++x)
                grid.set(x, y, lidar::CellState::Free);
    }

    rooting::PlanResult route(lidar::CellIndex goal) const {
        return rooting::Scheduler{}.plan(grid, {{2, 2}, Direction::East}, goal, radius);
    }
};

void expect_stop(const motion::ControlResult& result, ControlStatus status) {
    EXPECT_EQ(result.status, status);
    EXPECT_EQ(result.command.linear_mps, 0.0);
    EXPECT_EQ(result.command.angular_radps, 0.0);
    EXPECT_EQ(result.wheels.left_mps, 0.0);
    EXPECT_EQ(result.wheels.right_mps, 0.0);
}

TEST_F(ControllerTest, DrivesForwardWithoutReturningToTheStartCentre) {
    pose.x += 0.25;
    const auto result = controller.compute(grid, pose, route({4, 2}), radius, 499ms);
    EXPECT_EQ(result.status, ControlStatus::Moving);
    EXPECT_GT(result.command.linear_mps, 0.0);
    EXPECT_EQ(result.command.angular_radps, 0.0);
    EXPECT_DOUBLE_EQ(result.wheels.left_mps, result.command.linear_mps);
    EXPECT_DOUBLE_EQ(result.wheels.right_mps, result.command.linear_mps);
}

TEST_F(ControllerTest, TurnsInPlaceInEitherDirection) {
    for (const auto goal : {lidar::CellIndex{2, 4}, lidar::CellIndex{2, 1}}) {
        const auto result = controller.compute(grid, pose, route(goal), radius, 0ms);
        EXPECT_EQ(result.status, ControlStatus::Turning);
        EXPECT_EQ(result.command.linear_mps, 0.0);
        EXPECT_GT(result.command.angular_radps * (goal.y > 2 ? 1 : -1), 0.0);
        EXPECT_DOUBLE_EQ(result.wheels.left_mps, -result.wheels.right_mps);
    }
}

TEST_F(ControllerTest, WheelLimitPreservesRequestedTurnGeometry) {
    motion::ControllerOptions options;
    options.wheel_separation = 0.8;
    options.max_wheel_speed = 0.05;
    const auto limited = motion::Controller::Create(options);
    ASSERT_TRUE(limited);
    const auto result = limited->compute(grid, pose, route({2, 4}), radius, 0ms);
    ASSERT_EQ(result.status, ControlStatus::Turning);
    EXPECT_NEAR(result.wheels.right_mps, 0.05, 1e-12);
    EXPECT_NEAR(result.command.angular_radps, (result.wheels.right_mps - result.wheels.left_mps) / 0.8, 1e-12);
}

TEST_F(ControllerTest, StopsAtGoalAndOnPlanningFailure) {
    expect_stop(controller.compute(grid, pose, route({2, 2}), radius, 0ms), ControlStatus::GoalReached);
    const rooting::PlanResult failed{rooting::PlanStatus::NoPath, {}};
    expect_stop(controller.compute(grid, pose, failed, radius, 0ms), ControlStatus::Stopped);
}

TEST_F(ControllerTest, RejectsExpiredAndNegativeScanAges) {
    const auto plan = route({4, 2});
    expect_stop(controller.compute(grid, pose, plan, radius, 500ms), ControlStatus::StaleScan);
    expect_stop(controller.compute(grid, pose, plan, radius, -1ms), ControlStatus::InvalidInput);
}

TEST_F(ControllerTest, RejectsNonAdjacentRouteAndWrongStart) {
    const rooting::PlanResult jump{rooting::PlanStatus::Success, {{{2, 2}, Direction::East}, {{4, 2}, Direction::East}}};
    expect_stop(controller.compute(grid, pose, jump, radius, 0ms), ControlStatus::InvalidInput);
    auto plan = route({4, 2});
    pose.x = 1.5;
    expect_stop(controller.compute(grid, pose, plan, radius, 0ms), ControlStatus::InvalidInput);
}

TEST_F(ControllerTest, RechecksExistingRouteAgainstUpdatedMap) {
    const auto plan = route({4, 2});
    for (const auto state : {lidar::CellState::Unknown, lidar::CellState::Occupied}) {
        grid.set(3, 2, state);
        expect_stop(controller.compute(grid, pose, plan, radius, 0ms), ControlStatus::Blocked);
    }
}

TEST_F(ControllerTest, StopsWhenActualFootprintTouchesAnAdjacentObstacle) {
    const auto plan = route({4, 2});
    grid.set(2, 3, lidar::CellState::Occupied);
    pose.y = 2.9;
    expect_stop(controller.compute(grid, pose, plan, radius, 0ms), ControlStatus::Blocked);
}

TEST(ControllerOptionsTest, RejectsInvalidGeometryAndThresholds) {
    motion::ControllerOptions options;
    options.wheel_separation = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(motion::Controller::Create(options));
    options = {};
    options.heading_tolerance = std::numbers::pi / 2.0;
    EXPECT_FALSE(motion::Controller::Create(options));
    options = {};
    options.max_scan_age = 0ms;
    EXPECT_FALSE(motion::Controller::Create(options));
}

} // namespace
