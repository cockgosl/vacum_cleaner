#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "cleaner/vacuum_cleaner.hpp"

using cleaner::RobotSpecs;
using cleaner::StepStatus;
using cleaner::VacuumCleaner;
using rooting::Direction;
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

    const auto step = robot->run_step({{0, 1}, Direction::East}, {1, 1}, Seconds{0});
    EXPECT_EQ(step.status, StepStatus::Planned);
    ASSERT_TRUE(step.plan.has_value());
    EXPECT_EQ(step.plan->status, PlanStatus::Success);
    ASSERT_EQ(step.plan->path.size(), 2U);
    EXPECT_EQ(step.plan->path.front().cell.x, 0U);
    EXPECT_EQ(step.plan->path.back().cell.x, 1U);
    EXPECT_EQ(robot->grid().at(0, 1), lidar::CellState::Free);
    EXPECT_EQ(robot->grid().at(1, 1), lidar::CellState::Free);
    EXPECT_EQ(robot->grid().at(2, 1), lidar::CellState::Occupied);
    EXPECT_EQ(robot->grid().at(0, 0), lidar::CellState::Unknown);
}

TEST(VacuumCleanerTest, ReportsPlannerFailure) {
    const auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(robot.has_value());
    const auto step = robot->run_step({{0, 1}, Direction::East}, {1, 1}, Seconds{0});
    EXPECT_EQ(step.status, StepStatus::PlanningFailed);
    ASSERT_TRUE(step.plan.has_value());
    EXPECT_EQ(step.plan->status, PlanStatus::InvalidStart);
    EXPECT_TRUE(step.plan->path.empty());
}

TEST(VacuumCleanerTest, StopsAtOrAfterAutonomyLimit) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(robot.has_value());
    ASSERT_TRUE(robot->update_ray(0.5, 1.5, 2.5, 1.5));
    EXPECT_EQ(robot->run_step({{0, 1}, Direction::East}, {1, 1}, Seconds{59}).status, StepStatus::Planned);
    for (const auto elapsed : {Seconds{60}, Seconds{61}}) {
        const auto step = robot->run_step({{0, 1}, Direction::East}, {1, 1}, elapsed);
        EXPECT_EQ(step.status, StepStatus::TimeLimitReached);
        EXPECT_FALSE(step.plan.has_value());
    }
}

TEST(VacuumCleanerTest, RejectsNegativeElapsedTime) {
    const auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(robot.has_value());
    const auto step = robot->run_step({{0, 1}, Direction::East}, {1, 1}, Seconds{-1});
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
    EXPECT_EQ(small->run_step({{1, 1}, Direction::East}, {1, 1}, Seconds{0}).status, StepStatus::Planned);
    const auto step = large->run_step({{1, 1}, Direction::East}, {1, 1}, Seconds{0});
    EXPECT_EQ(step.status, StepStatus::PlanningFailed);
    ASSERT_TRUE(step.plan.has_value());
    EXPECT_EQ(step.plan->status, PlanStatus::InvalidStart);
}


TEST(VacuumCleanerTest, PassesInitialHeadingToScheduler) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 4, 3, 1.0);
    ASSERT_TRUE(robot.has_value());
    ASSERT_TRUE(robot->update_ray(0.5, 1.5, 2.5, 1.5));

    const auto step = robot->run_step({{0, 1}, Direction::North}, {1, 1}, Seconds{0});
    ASSERT_EQ(step.status, StepStatus::Planned);
    ASSERT_TRUE(step.plan.has_value());
    EXPECT_EQ(step.plan->cost, 2U);
    ASSERT_EQ(step.plan->path.size(), 3U);
    EXPECT_EQ(step.plan->path.front().direction, Direction::North);
    EXPECT_EQ(step.plan->path.back().direction, Direction::East);
}


struct Landmark {
    double x;
    double y;
};

const std::array<Landmark, 16> mapping_landmarks{{{1.0, 0.5},
                                                  {2.0, 0.2},
                                                  {3.2, 1.1},
                                                  {1.6, 2.4},
                                                  {-0.8, 2.3},
                                                  {-2.2, 1.6},
                                                  {-3.1, 0.2},
                                                  {-2.4, -1.3},
                                                  {-1.0, -2.6},
                                                  {0.8, -2.3},
                                                  {2.6, -1.7},
                                                  {3.4, -0.8},
                                                  {-1.5, 0.4},
                                                  {0.3, 1.4},
                                                  {-0.4, -1.0},
                                                  {1.5, -0.6}}};

io::Scan mapping_scan_at(localization::RelativePose sensor = {0.0, 0.0, 0.0}) {
    io::Scan scan;
    const double c = std::cos(sensor.dyaw);
    const double s = std::sin(sensor.dyaw);
    for (const auto point : mapping_landmarks) {
        const double dx = point.x - sensor.dx;
        const double dy = point.y - sensor.dy;
        const double x = c * dx + s * dy;
        const double y = -s * dx + c * dy;
        double angle = std::atan2(y, x);
        if (angle > 0.0)
            angle -= 2.0 * std::numbers::pi;
        scan.measurements.push_back({angle, std::hypot(x, y), 100});
    }
    std::sort(scan.measurements.begin(), scan.measurements.end(), [](const auto& a, const auto& b) { return a.angle > b.angle; });
    return scan;
}

void expect_same_grid(const lidar::Grid& actual, const lidar::Grid& expected) {
    ASSERT_EQ(actual.width(), expected.width());
    ASSERT_EQ(actual.height(), expected.height());
    for (std::size_t y = 0; y < actual.height(); ++y)
        for (std::size_t x = 0; x < actual.width(); ++x)
            EXPECT_EQ(actual.at(x, y), expected.at(x, y)) << "x=" << x << ", y=" << y;
}

void expect_pose(const lidar::SensorPose& pose, const lidar::SensorPose& expected) {
    EXPECT_NEAR(pose.x, expected.x, 1e-8);
    EXPECT_NEAR(pose.y, expected.y, 1e-8);
    EXPECT_NEAR(pose.yaw, expected.yaw, 1e-8);
}

TEST(ScanTrackingTest, DefaultsToTheMapCentreAndWaitsForTheFirstScan) {
    const auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    expect_pose(robot->pose(), {7.625, 7.625, 0.0});
    EXPECT_EQ(robot->tracking_state(), cleaner::TrackingState::WaitingForScan);
}

TEST(ScanTrackingTest, ValidatesTheInitialPoseAndMatcherOptions) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const auto pose : {lidar::SensorPose{-0.1, 5.0, 0.0},
                            {10.0, 5.0, 0.0},
                            {5.0, -0.1, 0.0},
                            {5.0, 10.0, 0.0},
                            {nan, 5.0, 0.0},
                            {5.0, inf, 0.0},
                            {5.0, 5.0, nan}})
        EXPECT_FALSE(VacuumCleaner::Create({0.2, Seconds{60}}, 10, 10, 1.0, pose));
    localization::MatcherOptions options;
    options.min_pairs = 0;
    EXPECT_FALSE(VacuumCleaner::Create({0.2, Seconds{60}}, 10, 10, 1.0, std::nullopt, options));
    const auto robot =
        VacuumCleaner::Create({0.2, Seconds{60}}, 10, 10, 1.0, lidar::SensorPose{2.5, 3.5, 4.0 * std::numbers::pi + 0.25});
    ASSERT_TRUE(robot);
    expect_pose(robot->pose(), {2.5, 3.5, 0.25});
}

TEST(ScanTrackingTest, RejectsAnEmptyFirstScanAndCanInitializeFromALaterScan) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    const auto before = robot->grid();
    const auto rejected = robot->process_scan({});
    EXPECT_EQ(rejected.status, cleaner::ScanStatus::Rejected);
    EXPECT_FALSE(rejected.pose);
    EXPECT_FALSE(rejected.map_update);
    ASSERT_TRUE(rejected.match);
    EXPECT_EQ(rejected.match->status, localization::MatchStatus::InsufficientPoints);
    EXPECT_EQ(robot->tracking_state(), cleaner::TrackingState::WaitingForScan);
    expect_same_grid(robot->grid(), before);

    const auto initialized = robot->process_scan(mapping_scan_at());
    EXPECT_EQ(initialized.status, cleaner::ScanStatus::Initialized);
    ASSERT_TRUE(initialized.pose);
    expect_pose(*initialized.pose, {7.625, 7.625, 0.0});
    ASSERT_TRUE(initialized.map_update);
    EXPECT_EQ(initialized.map_update->mapped, mapping_landmarks.size());
    EXPECT_EQ(robot->tracking_state(), cleaner::TrackingState::Tracking);
}

TEST(ScanTrackingTest, RejectsADegenerateFirstScanWithoutDrawingIt) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    const auto before = robot->grid();
    io::Scan wall;
    for (int i = 0; i < 20; ++i) {
        const double y = -2.0 + i * 0.2;
        wall.measurements.push_back({std::atan2(y, 2.0), std::hypot(2.0, y), 100});
    }
    const auto result = robot->process_scan(wall);
    EXPECT_EQ(result.status, cleaner::ScanStatus::Rejected);
    ASSERT_TRUE(result.match);
    EXPECT_EQ(result.match->status, localization::MatchStatus::DegenerateGeometry);
    expect_same_grid(robot->grid(), before);
}

TEST(ScanTrackingTest, ComposesMotionInMapCoordinatesAcrossSuccessiveScans) {
    const lidar::SensorPose initial{7.625, 7.625, 0.7};
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25, initial);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    for (const auto motion : {localization::RelativePose{0.06, -0.03, 0.025}, {0.12, -0.01, 0.05}}) {
        const auto result = robot->process_scan(mapping_scan_at(motion));
        ASSERT_EQ(result.status, cleaner::ScanStatus::Updated);
        ASSERT_TRUE(result.pose);
        const double c = std::cos(initial.yaw);
        const double s = std::sin(initial.yaw);
        expect_pose(*result.pose, {initial.x + c * motion.dx - s * motion.dy, initial.y + s * motion.dx + c * motion.dy,
                                   initial.yaw + motion.dyaw});
    }
}

TEST(ScanTrackingTest, KeepsStationaryLandmarksInTheSameMapCellsDuringMotion) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    for (const auto motion : {localization::RelativePose{0.06, -0.03, 0.025}, {0.12, -0.01, 0.05}}) {
        ASSERT_EQ(robot->process_scan(mapping_scan_at(motion)).status, cleaner::ScanStatus::Updated);
        for (const auto point : mapping_landmarks) {
            const auto cell = robot->grid().world_to_cell(7.625 + point.x, 7.625 + point.y);
            ASSERT_TRUE(cell);
            EXPECT_EQ(robot->grid().at(cell->x, cell->y), lidar::CellState::Occupied);
        }
        std::size_t occupied = 0;
        for (std::size_t y = 0; y < robot->grid().height(); ++y)
            for (std::size_t x = 0; x < robot->grid().width(); ++x)
                occupied += robot->grid().at(x, y) == lidar::CellState::Occupied;
        EXPECT_EQ(occupied, mapping_landmarks.size());
    }
}

TEST(ScanTrackingTest, NormalizesTheAccumulatedHeadingAcrossPi) {
    auto robot =
        VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25, lidar::SensorPose{7.625, 7.625, std::numbers::pi - 0.01});
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    const auto result = robot->process_scan(mapping_scan_at({0.0, 0.0, 0.04}));
    ASSERT_EQ(result.status, cleaner::ScanStatus::Updated);
    ASSERT_TRUE(result.pose);
    expect_pose(*result.pose, {7.625, 7.625, -std::numbers::pi + 0.03});
}

TEST(ScanTrackingTest, FailedTrackingPreservesThePoseAndMapAndDoesNotAutomaticallyResume) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    ASSERT_EQ(robot->process_scan(mapping_scan_at({0.06, -0.03, 0.025})).status, cleaner::ScanStatus::Updated);
    const auto before = robot->grid();
    const auto pose = robot->pose();
    const auto lost = robot->process_scan({});
    EXPECT_EQ(lost.status, cleaner::ScanStatus::TrackingLost);
    EXPECT_FALSE(lost.pose);
    EXPECT_FALSE(lost.map_update);
    ASSERT_TRUE(lost.match);
    EXPECT_EQ(robot->tracking_state(), cleaner::TrackingState::TrackingLost);
    expect_pose(robot->pose(), pose);
    expect_same_grid(robot->grid(), before);

    const auto retry = robot->process_scan(mapping_scan_at());
    EXPECT_EQ(retry.status, cleaner::ScanStatus::TrackingLost);
    EXPECT_FALSE(retry.match);
    EXPECT_FALSE(retry.pose);
    EXPECT_FALSE(retry.map_update);
    EXPECT_FALSE(robot->update_ray(0.5, 0.5, 1.5, 0.5));
    const auto plan = robot->run_step({{10, 10}, Direction::East}, {11, 10}, Seconds{0});
    EXPECT_EQ(plan.status, StepStatus::TrackingLost);
    EXPECT_FALSE(plan.plan);
    expect_pose(robot->pose(), pose);
    expect_same_grid(robot->grid(), before);
}

TEST(ScanTrackingTest, OutsidePoseFreezesMappingBeforeAnyCellsAreChanged) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 10, 10, 1.0, lidar::SensorPose{9.98, 5.0, 0.0});
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    const auto before = robot->grid();
    const auto result = robot->process_scan(mapping_scan_at({0.08, 0.0, 0.0}));
    EXPECT_EQ(result.status, cleaner::ScanStatus::PoseOutsideMap);
    ASSERT_TRUE(result.match);
    EXPECT_EQ(result.match->status, localization::MatchStatus::Success);
    EXPECT_FALSE(result.pose);
    EXPECT_FALSE(result.map_update);
    EXPECT_EQ(robot->tracking_state(), cleaner::TrackingState::TrackingLost);
    expect_pose(robot->pose(), {9.98, 5.0, 0.0});
    expect_same_grid(robot->grid(), before);
}

TEST(ScanTrackingTest, OwnsTheReferenceScanAfterTheCallerChangesItsInput) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    auto first = mapping_scan_at();
    ASSERT_EQ(robot->process_scan(first).status, cleaner::ScanStatus::Initialized);
    first.measurements.clear();
    const auto result = robot->process_scan(mapping_scan_at({0.06, -0.03, 0.025}));
    ASSERT_EQ(result.status, cleaner::ScanStatus::Updated);
    ASSERT_TRUE(result.pose);
    expect_pose(*result.pose, {7.685, 7.595, 0.025});
}

TEST(ScanTrackingTest, ReportsZeroReturnsWithoutDrawingThemAsObstacles) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    auto scan = mapping_scan_at();
    for (const double angle : {0.0, -1.0, -2.0})
        scan.measurements.push_back({angle, 0.0, 0});
    const auto first = robot->process_scan(scan);
    ASSERT_EQ(first.status, cleaner::ScanStatus::Initialized);
    ASSERT_TRUE(first.map_update);
    EXPECT_EQ(first.map_update->mapped, mapping_landmarks.size());
    EXPECT_EQ(first.map_update->skipped, 3U);
    const auto before = robot->grid();
    const auto second = robot->process_scan(scan);
    ASSERT_EQ(second.status, cleaner::ScanStatus::Updated);
    expect_same_grid(robot->grid(), before);
}

TEST(ScanTrackingTest, DoesNotDrawAChangedSceneWhenMatchingFails) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    const auto before = robot->grid();
    io::Scan unrelated;
    for (int i = 0; i < 180; ++i)
        unrelated.measurements.push_back({-2.0 * std::numbers::pi * i / 180.0, 2.0, 100});
    const auto result = robot->process_scan(unrelated);
    EXPECT_EQ(result.status, cleaner::ScanStatus::TrackingLost);
    expect_same_grid(robot->grid(), before);
    expect_pose(robot->pose(), {7.625, 7.625, 0.0});
}


void known_planning_map(VacuumCleaner& robot) {
    // Build a known planning map independently of the sparse localization landmarks.
    const auto& grid = robot.grid();
    const double resolution = grid.resolution();
    for (std::size_t y = 0; y < grid.height(); ++y) {
        const double centre_y = (y + 0.5) * resolution;
        ASSERT_TRUE(robot.update_ray(0.5 * resolution, centre_y, (grid.width() - 0.5) * resolution, centre_y));
    }
}

void planning_obstacle(VacuumCleaner& robot, lidar::CellIndex cell) {
    const double x = (cell.x + 0.5) * robot.grid().resolution();
    const double y = (cell.y + 0.5) * robot.grid().resolution();
    ASSERT_TRUE(robot.update_ray(x, y, x, y));
}

TEST(TrackedPlanningTest, WaitsForTrackingEvenWhenAManualMapIsAvailable) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    known_planning_map(*robot);
    const auto waiting = robot->run_step(lidar::CellIndex{30, 30}, Seconds{0});
    EXPECT_EQ(waiting.status, StepStatus::WaitingForScan);
    EXPECT_FALSE(waiting.plan);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    const auto planned = robot->run_step(lidar::CellIndex{30, 30}, Seconds{0});
    EXPECT_EQ(planned.status, StepStatus::Planned);
}

TEST(TrackedPlanningTest, UsesTheUpdatedStartAndKeepsTheGoalFixed) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25, lidar::SensorPose{7.74, 7.625, 0.0});
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    known_planning_map(*robot);
    const lidar::CellIndex goal{38, 30};
    const auto first = robot->run_step(goal, Seconds{0});
    ASSERT_EQ(first.status, StepStatus::Planned);
    ASSERT_TRUE(first.plan);
    ASSERT_FALSE(first.plan->path.empty());
    EXPECT_EQ(first.plan->path.front().cell.x, 30U);

    ASSERT_EQ(robot->process_scan(mapping_scan_at({0.06, 0.0, 0.0})).status, cleaner::ScanStatus::Updated);
    known_planning_map(*robot);
    const auto second = robot->run_step(goal, Seconds{1});
    ASSERT_EQ(second.status, StepStatus::Planned);
    ASSERT_TRUE(second.plan);
    ASSERT_FALSE(second.plan->path.empty());
    EXPECT_EQ(second.plan->path.front().cell.x, 31U);
    EXPECT_EQ(second.plan->path.front().cell.y, 30U);
    EXPECT_EQ(second.plan->path.back().cell.x, goal.x);
    EXPECT_EQ(second.plan->path.back().cell.y, goal.y);
}

TEST(TrackedPlanningTest, RoundsHeadingToTheNearestCardinalDirectionWithCounterclockwiseTies) {
    struct Case {
        double yaw;
        Direction direction;
    };
    const double pi = std::numbers::pi;
    const std::array<Case, 13> cases{{{0.0, Direction::East},
                                      {pi / 4 - 1e-6, Direction::East},
                                      {pi / 4, Direction::North},
                                      {pi / 4 + 1e-6, Direction::North},
                                      {pi / 2, Direction::North},
                                      {3 * pi / 4 + 1e-6, Direction::West},
                                      {pi, Direction::West},
                                      {-pi, Direction::West},
                                      {-3 * pi / 4 + 1e-6, Direction::South},
                                      {-pi / 2, Direction::South},
                                      {-pi / 4 - 1e-6, Direction::South},
                                      {-pi / 4, Direction::East},
                                      {-pi / 4 + 1e-6, Direction::East}}};
    for (const auto test : cases) {
        SCOPED_TRACE(test.yaw);
        auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25, lidar::SensorPose{7.625, 7.625, test.yaw});
        ASSERT_TRUE(robot);
        ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
        known_planning_map(*robot);
        const auto result = robot->run_step(lidar::CellIndex{30, 30}, Seconds{0});
        ASSERT_EQ(result.status, StepStatus::Planned);
        ASSERT_TRUE(result.plan);
        ASSERT_FALSE(result.plan->path.empty());
        EXPECT_EQ(result.plan->path.front().direction, test.direction);
    }
}

TEST(TrackedPlanningTest, ReplansAroundANewObstacle) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    known_planning_map(*robot);
    const lidar::CellIndex goal{38, 30};
    const auto initial = robot->run_step(goal, Seconds{0});
    ASSERT_EQ(initial.status, StepStatus::Planned);
    ASSERT_TRUE(initial.plan);
    const auto contains = [](const auto& path, lidar::CellIndex cell) {
        return std::any_of(path.begin(), path.end(),
                           [&](const auto& pose) { return pose.cell.x == cell.x && pose.cell.y == cell.y; });
    };
    ASSERT_TRUE(contains(initial.plan->path, {34, 30}));
    planning_obstacle(*robot, {34, 30});
    const auto updated = robot->run_step(goal, Seconds{1});
    ASSERT_EQ(updated.status, StepStatus::Planned);
    ASSERT_TRUE(updated.plan);
    EXPECT_FALSE(contains(updated.plan->path, {34, 30}));
    EXPECT_GT(updated.plan->cost, initial.plan->cost);
}

TEST(TrackedPlanningTest, ReplacesASuccessfulRouteWithAnInvalidGoalResult) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    known_planning_map(*robot);
    const lidar::CellIndex goal{38, 30};
    ASSERT_EQ(robot->run_step(goal, Seconds{0}).status, StepStatus::Planned);
    planning_obstacle(*robot, goal);
    for (const auto target : {goal, lidar::CellIndex{61, 30}}) {
        const auto result = robot->run_step(target, Seconds{1});
        EXPECT_EQ(result.status, StepStatus::PlanningFailed);
        ASSERT_TRUE(result.plan);
        EXPECT_EQ(result.plan->status, PlanStatus::InvalidGoal);
        EXPECT_TRUE(result.plan->path.empty());
    }
}

TEST(TrackedPlanningTest, RefusesToPlanToAnUnknownGoal) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    for (std::size_t y = 29; y <= 31; ++y)
        ASSERT_TRUE(robot->update_ray(7.375, (y + 0.5) * 0.25, 8.125, (y + 0.5) * 0.25));
    ASSERT_EQ(robot->grid().at(55, 55), lidar::CellState::Unknown);
    const auto result = robot->run_step(lidar::CellIndex{55, 55}, Seconds{0});
    EXPECT_EQ(result.status, StepStatus::PlanningFailed);
    ASSERT_TRUE(result.plan);
    EXPECT_EQ(result.plan->status, PlanStatus::InvalidGoal);
    EXPECT_TRUE(result.plan->path.empty());
}

TEST(TrackedPlanningTest, ReportsNoPathAfterACompleteWallAppears) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    known_planning_map(*robot);
    ASSERT_EQ(robot->run_step(lidar::CellIndex{38, 30}, Seconds{0}).status, StepStatus::Planned);
    for (std::size_t y = 0; y < robot->grid().height(); ++y)
        planning_obstacle(*robot, {34, y});
    const auto result = robot->run_step(lidar::CellIndex{38, 30}, Seconds{1});
    EXPECT_EQ(result.status, StepStatus::PlanningFailed);
    ASSERT_TRUE(result.plan);
    EXPECT_EQ(result.plan->status, PlanStatus::NoPath);
    EXPECT_TRUE(result.plan->path.empty());
}

TEST(TrackedPlanningTest, RejectsActualFootprintCollisionEvenWhenTheCellCentreHasAPath) {
    auto robot = VacuumCleaner::Create({0.1, Seconds{60}}, 61, 61, 0.25, lidar::SensorPose{7.74, 7.625, 0.0});
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    known_planning_map(*robot);
    planning_obstacle(*robot, {31, 30});
    const lidar::CellIndex goal{34, 30};
    ASSERT_EQ(robot->run_step({{30, 30}, Direction::East}, goal, Seconds{0}).status, StepStatus::Planned);
    const auto result = robot->run_step(goal, Seconds{0});
    EXPECT_EQ(result.status, StepStatus::PlanningFailed);
    ASSERT_TRUE(result.plan);
    EXPECT_EQ(result.plan->status, PlanStatus::InvalidStart);
    EXPECT_TRUE(result.plan->path.empty());
}

TEST(TrackedPlanningTest, ChecksTheEntireConnectorEvenWhenBothEndpointFootprintsAreClear) {
    constexpr double radius = 0.1725;
    auto robot = VacuumCleaner::Create({radius, Seconds{60}}, 61, 61, 0.25, lidar::SensorPose{7.7475, 7.55, 0.0});
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    known_planning_map(*robot);
    planning_obstacle(*robot, {31, 31});
    const rooting::Scheduler scheduler;
    const auto& grid = robot->grid();
    ASSERT_TRUE(scheduler.can_move(grid, 7.7475, 7.55, 7.7475, 7.55, radius));
    ASSERT_TRUE(scheduler.can_move(grid, 7.625, 7.625, 7.625, 7.625, radius));
    const auto result = robot->run_step(lidar::CellIndex{30, 30}, Seconds{0});
    EXPECT_EQ(result.status, StepStatus::PlanningFailed);
    ASSERT_TRUE(result.plan);
    EXPECT_EQ(result.plan->status, PlanStatus::InvalidStart);
}

TEST(TrackedPlanningTest, RemovesTheRouteAfterTrackingIsLost) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    known_planning_map(*robot);
    ASSERT_EQ(robot->run_step(lidar::CellIndex{38, 30}, Seconds{0}).status, StepStatus::Planned);
    ASSERT_EQ(robot->process_scan({}).status, cleaner::ScanStatus::TrackingLost);
    const auto lost = robot->run_step(lidar::CellIndex{38, 30}, Seconds{1});
    EXPECT_EQ(lost.status, StepStatus::TrackingLost);
    EXPECT_FALSE(lost.plan);
}

TEST(TrackedPlanningTest, ValidatesElapsedTimeAndHonoursAutonomy) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    known_planning_map(*robot);
    const auto invalid = robot->run_step(lidar::CellIndex{38, 30}, Seconds{-1});
    EXPECT_EQ(invalid.status, StepStatus::InvalidElapsedTime);
    EXPECT_FALSE(invalid.plan);
    for (const auto elapsed : {Seconds{60}, Seconds{61}}) {
        const auto expired = robot->run_step(lidar::CellIndex{38, 30}, elapsed);
        EXPECT_EQ(expired.status, StepStatus::TimeLimitReached);
        EXPECT_FALSE(expired.plan);
    }
}

TEST(TrackedPlanningTest, AlreadyAtTheGoalCellReturnsOneGridPose) {
    auto robot = VacuumCleaner::Create({0.2, Seconds{60}}, 61, 61, 0.25);
    ASSERT_TRUE(robot);
    ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
    known_planning_map(*robot);
    const auto result = robot->run_step(lidar::CellIndex{30, 30}, Seconds{0});
    ASSERT_EQ(result.status, StepStatus::Planned);
    ASSERT_TRUE(result.plan);
    EXPECT_EQ(result.plan->cost, 0U);
    ASSERT_EQ(result.plan->path.size(), 1U);
    EXPECT_EQ(result.plan->path[0].cell.x, 30U);
    EXPECT_EQ(result.plan->path[0].cell.y, 30U);
}

TEST(TrackedPlanningTest, UsesRobotRadiusWhenCheckingANarrowOpening) {
    for (const double radius : {0.1, 0.15}) {
        auto robot = VacuumCleaner::Create({radius, Seconds{60}}, 61, 61, 0.25);
        ASSERT_TRUE(robot);
        ASSERT_EQ(robot->process_scan(mapping_scan_at()).status, cleaner::ScanStatus::Initialized);
        known_planning_map(*robot);
        for (std::size_t y = 0; y < robot->grid().height(); ++y)
            if (y != 30)
                planning_obstacle(*robot, {34, y});
        const auto result = robot->run_step(lidar::CellIndex{38, 30}, Seconds{0});
        ASSERT_TRUE(result.plan);
        EXPECT_EQ(result.plan->status, radius < 0.125 ? PlanStatus::Success : PlanStatus::NoPath);
    }
}

} // namespace
