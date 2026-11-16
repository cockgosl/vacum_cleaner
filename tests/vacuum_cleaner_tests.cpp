#include <gtest/gtest.h>

#include <chrono>
#include <limits>

#include "cleaner/vacuum_cleaner.hpp"

using cleaner::RobotSpecs;
using cleaner::StepStatus;
using cleaner::VacuumCleaner;
using rooting::PlanStatus;
using Seconds = std::chrono::seconds;

namespace {

TEST(VacuumCleanerTest, CreatesWithRequestedSpecsAndUnknownMap) {
    const auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(robot.has_value());
    EXPECT_DOUBLE_EQ(robot->specs().radius, 0.2);
    EXPECT_EQ(robot->specs().autonomy, Seconds{60});
    EXPECT_EQ(robot->grid().width(), 4U);
    EXPECT_EQ(robot->grid().height(), 3U);
    EXPECT_EQ(robot->grid().at(0, 0), lidar::CellState::Unknown);
}

TEST(VacuumCleanerTest, RejectsInvalidSpecsAndMap) {
    for (const double radius : {0.0, -0.2, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
        EXPECT_FALSE(VacuumCleaner::Create({radius, Seconds{60}}, 4, 3, 1.0).has_value());
    EXPECT_FALSE(VacuumCleaner::Create({0.2, Seconds{0}}, 4, 3, 1.0).has_value());
    EXPECT_FALSE(VacuumCleaner::Create({0.2, Seconds{-1}}, 4, 3, 1.0).has_value());
    EXPECT_FALSE(VacuumCleaner::Create({0.2, Seconds{60}}, 0, 3, 1.0).has_value());
    EXPECT_FALSE(VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 0.0).has_value());
}

TEST(VacuumCleanerTest, PlansOverObservedMapWithoutChangingIt) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(robot.has_value());
    ASSERT_TRUE(robot->update_ray(0.5, 1.5, 2.5, 1.5));

    const auto step = robot->run_step({0, 1}, {1, 1}, Seconds{0});
    EXPECT_EQ(step.status, StepStatus::Planned);
    ASSERT_TRUE(step.plan.has_value());
    EXPECT_EQ(step.plan->status, PlanStatus::Success);
    ASSERT_EQ(step.plan->path.size(), 2U);
    EXPECT_EQ(step.plan->path.front().x, 0U);
    EXPECT_EQ(step.plan->path.back().x, 1U);
    EXPECT_EQ(robot->grid().at(0, 1), lidar::CellState::Free);
    EXPECT_EQ(robot->grid().at(1, 1), lidar::CellState::Free);
    EXPECT_EQ(robot->grid().at(2, 1), lidar::CellState::Occupied);
    EXPECT_EQ(robot->grid().at(0, 0), lidar::CellState::Unknown);
}

TEST(VacuumCleanerTest, ReportsPlannerFailure) {
    const auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(robot.has_value());
    const auto step = robot->run_step({0, 1}, {1, 1}, Seconds{0});
    EXPECT_EQ(step.status, StepStatus::PlanningFailed);
    ASSERT_TRUE(step.plan.has_value());
    EXPECT_EQ(step.plan->status, PlanStatus::InvalidStart);
    EXPECT_TRUE(step.plan->path.empty());
}

TEST(VacuumCleanerTest, StopsAtOrAfterAutonomyLimit) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(robot.has_value());
    ASSERT_TRUE(robot->update_ray(0.5, 1.5, 2.5, 1.5));
    EXPECT_EQ(robot->run_step({0, 1}, {1, 1}, Seconds{59}).status, StepStatus::Planned);
    for (const auto elapsed : {Seconds{60}, Seconds{61}}) {
        const auto step = robot->run_step({0, 1}, {1, 1}, elapsed);
        EXPECT_EQ(step.status, StepStatus::TimeLimitReached);
        EXPECT_FALSE(step.plan.has_value());
    }
}

TEST(VacuumCleanerTest, RejectsNegativeElapsedTime) {
    const auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(robot.has_value());
    const auto step = robot->run_step({0, 1}, {1, 1}, Seconds{-1});
    EXPECT_EQ(step.status, StepStatus::InvalidElapsedTime);
    EXPECT_FALSE(step.plan.has_value());
}

TEST(VacuumCleanerTest, PassesRobotRadiusToScheduler) {
    auto small = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    auto large = VacuumCleaner::Create({0.6, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(small.has_value());
    ASSERT_TRUE(large.has_value());
    ASSERT_TRUE(small->update_ray(0.5, 1.5, 2.5, 1.5));
    ASSERT_TRUE(large->update_ray(0.5, 1.5, 2.5, 1.5));
    EXPECT_EQ(small->run_step({1, 1}, {1, 1}, Seconds{0}).status, StepStatus::Planned);
    const auto step = large->run_step({1, 1}, {1, 1}, Seconds{0});
    EXPECT_EQ(step.status, StepStatus::PlanningFailed);
    ASSERT_TRUE(step.plan.has_value());
    EXPECT_EQ(step.plan->status, PlanStatus::InvalidStart);
}

} // namespace
