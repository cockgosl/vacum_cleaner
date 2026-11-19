#include <gtest/gtest.h>

#include "localization/scan_matcher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace {

using localization::MatcherOptions;
using localization::MatchStatus;
using localization::RelativePose;
using localization::ScanMatcher;

struct Point {
    double x;
    double y;
};

const std::vector<Point> landmarks{{1.0, 0.5},  {2.0, 0.2},   {3.2, 1.1},   {1.6, 2.4},  {-0.8, 2.3}, {-2.2, 1.6},
                                   {-3.1, 0.2}, {-2.4, -1.3}, {-1.0, -2.6}, {0.8, -2.3}, {2.6, -1.7}, {3.4, -0.8},
                                   {-1.5, 0.4}, {0.3, 1.4},   {-0.4, -1.0}, {1.5, -0.6}};

io::Scan scan_at(const std::vector<Point>& world, RelativePose sensor = {0.0, 0.0, 0.0}) {
    io::Scan scan;
    const double c = std::cos(sensor.dyaw);
    const double s = std::sin(sensor.dyaw);
    for (const auto point : world) {
        // Generate sensor observations with the inverse pose, independently of ICP.
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

io::Scan room_scan(RelativePose sensor, std::size_t count) {
    io::Scan scan;
    for (std::size_t i = 0; i < count; ++i) {
        const double angle = -2.0 * std::numbers::pi * i / count;
        const double dx = std::cos(sensor.dyaw + angle);
        const double dy = std::sin(sensor.dyaw + angle);
        double distance = std::numeric_limits<double>::infinity();
        const auto vertical = [&](double x, double bottom, double top) {
            if (std::abs(dx) < 1e-12)
                return;
            const double t = (x - sensor.dx) / dx;
            const double y = sensor.dy + t * dy;
            if (t > 0.0 && y >= bottom && y <= top)
                distance = std::min(distance, t);
        };
        const auto horizontal = [&](double y, double left, double right) {
            if (std::abs(dy) < 1e-12)
                return;
            const double t = (y - sensor.dy) / dy;
            const double x = sensor.dx + t * dx;
            if (t > 0.0 && x >= left && x <= right)
                distance = std::min(distance, t);
        };
        // An asymmetric room and a rectangular obstacle produce real ray samples,
        // whose correspondences change when the sensor moves or sampling changes.
        vertical(-3.0, -2.0, 3.0);
        vertical(4.0, -2.0, 3.0);
        horizontal(-2.0, -3.0, 4.0);
        horizontal(3.0, -3.0, 4.0);
        vertical(1.5, -0.8, 0.2);
        vertical(2.5, -0.8, 0.2);
        horizontal(-0.8, 1.5, 2.5);
        horizontal(0.2, 1.5, 2.5);
        scan.measurements.push_back({angle, distance, 100});
    }
    return scan;
}

void expect_motion(const localization::MatchResult& result, RelativePose expected, double position_tolerance = 1e-8,
                   double rotation_tolerance = 1e-8) {
    ASSERT_EQ(result.status, MatchStatus::Success) << "rmse=" << result.rmse << ", pairs=" << result.correspondences;
    ASSERT_TRUE(result.motion);
    EXPECT_NEAR(result.motion->dx, expected.dx, position_tolerance);
    EXPECT_NEAR(result.motion->dy, expected.dy, position_tolerance);
    EXPECT_NEAR(result.motion->dyaw, expected.dyaw, rotation_tolerance);
}

TEST(ScanMatcherTest, IdenticalScansProduceZeroMotion) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const auto scan = scan_at(landmarks);
    const auto result = matcher->match(scan, scan);
    expect_motion(result, {0.0, 0.0, 0.0});
    EXPECT_LT(result.rmse, 1e-10);
    EXPECT_EQ(result.correspondences, landmarks.size());
}

TEST(ScanMatcherTest, RecoversSensorTranslationWithTheCorrectSign) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const RelativePose expected{0.08, -0.04, 0.0};
    expect_motion(matcher->match(scan_at(landmarks), scan_at(landmarks, expected)), expected);
}

TEST(ScanMatcherTest, RecoversCounterclockwiseSensorRotation) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const RelativePose expected{0.0, 0.0, 0.04};
    expect_motion(matcher->match(scan_at(landmarks), scan_at(landmarks, expected)), expected);
}

TEST(ScanMatcherTest, RecoversCombinedTranslationAndRotation) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const RelativePose expected{0.06, -0.03, 0.025};
    expect_motion(matcher->match(scan_at(landmarks), scan_at(landmarks, expected)), expected);
}

TEST(ScanMatcherTest, ExpressesTranslationInThePreviousSensorFrame) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const RelativePose previous{0.2, -0.1, 0.4};
    const RelativePose expected{0.06, -0.03, 0.025};
    const double c = std::cos(previous.dyaw);
    const double s = std::sin(previous.dyaw);
    const RelativePose current{previous.dx + c * expected.dx - s * expected.dy, previous.dy + s * expected.dx + c * expected.dy,
                               previous.dyaw + expected.dyaw};
    expect_motion(matcher->match(scan_at(landmarks, previous), scan_at(landmarks, current)), expected);
}

TEST(ScanMatcherTest, ReversingTheScanOrderProducesTheInverseMotion) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const RelativePose motion{0.06, -0.03, 0.025};
    const double c = std::cos(motion.dyaw);
    const double s = std::sin(motion.dyaw);
    const RelativePose inverse{-c * motion.dx - s * motion.dy, s * motion.dx - c * motion.dy, -motion.dyaw};
    expect_motion(matcher->match(scan_at(landmarks, motion), scan_at(landmarks)), inverse);
}

TEST(ScanMatcherTest, FiltersZeroNegativeAndNonFiniteMeasurements) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const RelativePose expected{0.06, -0.03, 0.025};
    auto previous = scan_at(landmarks);
    auto current = scan_at(landmarks, expected);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (auto* scan : {&previous, &current})
        for (const auto measurement : {io::Measurement{0.0, 0.0, 0},
                                       {0.0, -1.0, 0},
                                       {nan, 1.0, 0},
                                       {inf, 1.0, 0},
                                       {-inf, 1.0, 0},
                                       {0.0, nan, 0},
                                       {0.0, inf, 0}})
            scan->measurements.push_back(measurement);
    expect_motion(matcher->match(previous, current), expected);
}

TEST(ScanMatcherTest, RejectsEmptyScansWithoutPublishingMotion) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const auto result = matcher->match({}, scan_at(landmarks));
    EXPECT_EQ(result.status, MatchStatus::InsufficientPoints);
    EXPECT_FALSE(result.motion);
    EXPECT_EQ(result.iterations, 0U);
    EXPECT_EQ(result.correspondences, 0U);
    EXPECT_TRUE(std::isinf(result.rmse));
}

TEST(ScanMatcherTest, RejectsScansWithTooFewValidReturns) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    auto scan = scan_at(landmarks);
    for (std::size_t i = 0; i < 6; ++i)
        scan.measurements[i].distance = 0.0;
    const auto result = matcher->match(scan_at(landmarks), scan);
    EXPECT_EQ(result.status, MatchStatus::InsufficientPoints);
    EXPECT_FALSE(result.motion);
}

TEST(ScanMatcherTest, RejectsScansWithoutOverlappingPoints) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    auto distant = landmarks;
    for (auto& point : distant) {
        point.x += 50.0;
        point.y += 50.0;
    }
    const auto result = matcher->match(scan_at(landmarks), scan_at(distant));
    EXPECT_EQ(result.status, MatchStatus::InsufficientOverlap);
    EXPECT_FALSE(result.motion);
    EXPECT_EQ(result.correspondences, 0U);
}

TEST(ScanMatcherTest, RequiresOverlapBeyondTheMinimumPairCount) {
    MatcherOptions options;
    options.min_pairs = 3;
    options.min_overlap = 0.75;
    const auto matcher = ScanMatcher::Create(options);
    ASSERT_TRUE(matcher);
    auto partial = landmarks;
    for (std::size_t i = 6; i < partial.size(); ++i)
        partial[i].x += 50.0;
    const auto result = matcher->match(scan_at(landmarks), scan_at(partial));
    EXPECT_EQ(result.status, MatchStatus::InsufficientOverlap);
    EXPECT_FALSE(result.motion);
    EXPECT_GE(result.correspondences, options.min_pairs);
}

TEST(ScanMatcherTest, RejectsCollinearGeometryEvenForAZeroErrorFit) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    std::vector<Point> wall;
    for (int i = 0; i < 20; ++i)
        wall.push_back({2.0, -2.0 + i * 0.2});
    const auto scan = scan_at(wall);
    const auto result = matcher->match(scan, scan);
    EXPECT_EQ(result.status, MatchStatus::DegenerateGeometry);
    EXPECT_FALSE(result.motion);
    EXPECT_LT(result.rmse, 1e-10);
}

TEST(ScanMatcherTest, ManyToOneMatchesCannotFakeSufficientOverlap) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const auto scan = scan_at(std::vector<Point>(20, Point{2.0, 1.0}));
    const auto result = matcher->match(scan, scan);
    EXPECT_EQ(result.status, MatchStatus::InsufficientOverlap);
    EXPECT_FALSE(result.motion);
    EXPECT_EQ(result.correspondences, 1U);
}

TEST(ScanMatcherTest, IgnoresDistantOutliersWhenEnoughScenePointsRemain) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const RelativePose expected{0.06, -0.03, 0.025};
    auto world = landmarks;
    for (int i = 0; i < 4; ++i)
        world.push_back({20.0 + i, 15.0 + i});
    expect_motion(matcher->match(scan_at(landmarks), scan_at(world, expected)), expected);
}

TEST(ScanMatcherTest, DoesNotPublishMotionBeyondTranslationLimit) {
    MatcherOptions options;
    options.max_translation = 0.02;
    const auto matcher = ScanMatcher::Create(options);
    ASSERT_TRUE(matcher);
    const auto result = matcher->match(scan_at(landmarks), scan_at(landmarks, {0.08, 0.0, 0.0}));
    EXPECT_EQ(result.status, MatchStatus::MotionLimitExceeded);
    EXPECT_FALSE(result.motion);
}

TEST(ScanMatcherTest, DoesNotPublishMotionBeyondRotationLimit) {
    MatcherOptions options;
    options.max_rotation = 0.01;
    const auto matcher = ScanMatcher::Create(options);
    ASSERT_TRUE(matcher);
    const auto result = matcher->match(scan_at(landmarks), scan_at(landmarks, {0.0, 0.0, 0.04}));
    EXPECT_EQ(result.status, MatchStatus::MotionLimitExceeded);
    EXPECT_FALSE(result.motion);
}

TEST(ScanMatcherTest, DoesNotPublishMotionWhenIterationBudgetIsExhausted) {
    MatcherOptions options;
    options.max_iterations = 1;
    const auto matcher = ScanMatcher::Create(options);
    ASSERT_TRUE(matcher);
    const auto result = matcher->match(scan_at(landmarks), scan_at(landmarks, {0.06, -0.03, 0.025}));
    EXPECT_EQ(result.status, MatchStatus::NotConverged);
    EXPECT_FALSE(result.motion);
    EXPECT_EQ(result.iterations, 1U);
}

TEST(ScanMatcherTest, RejectsConvergedButInaccurateFits) {
    MatcherOptions options;
    options.max_rmse = 1e-5;
    const auto matcher = ScanMatcher::Create(options);
    ASSERT_TRUE(matcher);
    auto distorted = landmarks;
    for (std::size_t i = 0; i < distorted.size(); ++i)
        distorted[i].x += i % 2 == 0 ? 0.02 : -0.02;
    const auto result = matcher->match(scan_at(landmarks), scan_at(distorted));
    EXPECT_EQ(result.status, MatchStatus::PoorFit);
    EXPECT_FALSE(result.motion);
    EXPECT_GT(result.rmse, options.max_rmse);
}

TEST(ScanMatcherTest, RespectsTheSampleLimit) {
    MatcherOptions options;
    options.max_points = 12;
    options.min_pairs = 12;
    const auto matcher = ScanMatcher::Create(options);
    ASSERT_TRUE(matcher);
    const auto scan = scan_at(landmarks);
    const auto result = matcher->match(scan, scan);
    expect_motion(result, {0.0, 0.0, 0.0});
    EXPECT_EQ(result.correspondences, 12U);
}

TEST(ScanMatcherTest, KeepsStationaryEstimatesSmallWithRangeNoise) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const auto reference = room_scan({0.0, 0.0, 0.0}, 720);
    auto noisy = reference;
    for (std::size_t i = 0; i < noisy.measurements.size(); ++i)
        noisy.measurements[i].distance += 0.002 * std::sin(i * 1.7);
    expect_motion(matcher->match(reference, noisy), {0.0, 0.0, 0.0}, 0.003, 0.002);
}

TEST(ScanMatcherTest, MatchesRoomScansWithDifferentAngularSampling) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    expect_motion(matcher->match(room_scan({0.0, 0.0, 0.0}, 500), room_scan({0.0, 0.0, 0.0}, 720)), {0.0, 0.0, 0.0}, 0.015,
                  0.005);
}

TEST(ScanMatcherTest, EstimatesMotionFromRaycastRoomObservations) {
    const auto matcher = ScanMatcher::Create();
    ASSERT_TRUE(matcher);
    const RelativePose expected{0.06, -0.03, 0.02};
    expect_motion(matcher->match(room_scan({0.0, 0.0, 0.0}, 720), room_scan(expected, 720)), expected, 0.02, 0.01);
}

TEST(ScanMatcherOptionsTest, AcceptsDefaultAndBoundaryOptions) {
    EXPECT_TRUE(ScanMatcher::Create());
    MatcherOptions options;
    options.max_points = options.min_pairs = 3;
    options.max_iterations = 1;
    options.min_overlap = 1.0;
    options.max_rmse = options.max_pair_distance;
    options.max_rotation = std::numbers::pi;
    EXPECT_TRUE(ScanMatcher::Create(options));
}

TEST(ScanMatcherOptionsTest, RejectsInvalidWorkBudgets) {
    for (const auto points : {0U, 2U, 2049U}) {
        MatcherOptions options;
        options.max_points = points;
        EXPECT_FALSE(ScanMatcher::Create(options));
    }
    for (const auto pairs : {0U, 2U, 181U}) {
        MatcherOptions options;
        options.min_pairs = pairs;
        EXPECT_FALSE(ScanMatcher::Create(options));
    }
    for (const auto iterations : {0U, 201U}) {
        MatcherOptions options;
        options.max_iterations = iterations;
        EXPECT_FALSE(ScanMatcher::Create(options));
    }
}

TEST(ScanMatcherOptionsTest, RejectsInvalidNumericThresholds) {
    const std::array<double MatcherOptions::*, 5> fields{&MatcherOptions::max_pair_distance, &MatcherOptions::min_overlap,
                                                         &MatcherOptions::max_rmse, &MatcherOptions::max_translation,
                                                         &MatcherOptions::max_rotation};
    for (const auto field : fields) {
        for (const double value : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                   -std::numeric_limits<double>::infinity()}) {
            MatcherOptions options;
            options.*field = value;
            EXPECT_FALSE(ScanMatcher::Create(options));
        }
    }
    MatcherOptions options;
    options.min_overlap = 1.01;
    EXPECT_FALSE(ScanMatcher::Create(options));
    options = {};
    options.max_rotation = std::numbers::pi + 0.01;
    EXPECT_FALSE(ScanMatcher::Create(options));
    options = {};
    options.max_rmse = options.max_pair_distance + 0.01;
    EXPECT_FALSE(ScanMatcher::Create(options));
}

} // namespace
