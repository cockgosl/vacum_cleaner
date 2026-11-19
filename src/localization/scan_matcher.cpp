#include "localization/scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <utility>
#include <vector>

namespace localization {
namespace {

struct Point {
    double x;
    double y;
};

struct Pair {
    Point source; // A current-scan point transformed into the previous frame.
    Point target; // Its nearest point in the previous scan.
    double distance;
};

struct Moments {
    Point centre{};
    double xx = 0.0;
    double xy = 0.0;
    double yy = 0.0;
};

bool valid_measurement(const io::Measurement& point) {
    return std::isfinite(point.angle) && std::isfinite(point.distance) && point.distance > 0.0;
}

std::vector<Point> sample(const io::Scan& scan, std::size_t limit) {
    const auto count =
        static_cast<std::size_t>(std::count_if(scan.measurements.begin(), scan.measurements.end(), valid_measurement));
    const auto kept = std::min(count, limit);
    std::vector<Point> points;
    points.reserve(kept);
    if (kept == 0)
        return points;

    std::size_t seen = 0;
    std::size_t next = 0;
    for (const auto& measurement : scan.measurements) {
        if (!valid_measurement(measurement))
            continue;
        if (seen == next) {
            const double angle = std::remainder(measurement.angle, 2.0 * std::numbers::pi);
            points.push_back({measurement.distance * std::cos(angle), measurement.distance * std::sin(angle)});
            if (points.size() == kept)
                break;
            // Round to the nearest evenly spaced index. Always rounding down
            // would bias the angular phase when scan sample counts differ.
            // Split the quotient to avoid overflowing selected * count.
            next = points.size() * (count / kept) + (points.size() * (count % kept) + kept / 2) / kept;
        }
        ++seen;
    }
    return points;
}

Point transform(Point point, const RelativePose& pose) {
    const double c = std::cos(pose.dyaw);
    const double s = std::sin(pose.dyaw);
    return {c * point.x - s * point.y + pose.dx, s * point.x + c * point.y + pose.dy};
}

std::vector<Pair> associate(std::span<const Point> reference, std::span<const Point> current, const RelativePose& pose,
                            double max_distance) {
    std::vector<Pair> pairs;
    pairs.reserve(current.size());
    const auto unused = reference.size();
    std::vector<std::size_t> pair_for_target(reference.size(), unused);
    for (const auto point : current) {
        const auto source = transform(point, pose);
        auto target_index = unused;
        double nearest = max_distance;
        for (std::size_t i = 0; i < reference.size(); ++i) {
            const double distance = std::hypot(source.x - reference[i].x, source.y - reference[i].y);
            if (distance <= nearest) {
                nearest = distance;
                target_index = i;
            }
        }
        if (target_index == unused)
            continue;
        auto& pair_index = pair_for_target[target_index];
        if (pair_index == unused) {
            pair_index = pairs.size();
            pairs.push_back({source, reference[target_index], nearest});
        } else if (nearest < pairs[pair_index].distance)
            pairs[pair_index] = {source, reference[target_index], nearest};
    }
    // A reference point can occur only once: many current points must not collapse
    // onto one target and create an artificially small fitting error.
    return pairs;
}

Moments moments(std::span<const Pair> pairs, bool target) {
    Moments result;
    const double count = static_cast<double>(pairs.size());
    for (const auto& pair : pairs) {
        const auto point = target ? pair.target : pair.source;
        result.centre.x += point.x / count;
        result.centre.y += point.y / count;
    }
    for (const auto& pair : pairs) {
        const auto point = target ? pair.target : pair.source;
        const double x = point.x - result.centre.x;
        const double y = point.y - result.centre.y;
        result.xx += x * x / count;
        result.xy += x * y / count;
        result.yy += y * y / count;
    }
    return result;
}

bool has_spread(const Moments& cloud) {
    const double trace = cloud.xx + cloud.yy;
    const double minor = (trace - std::hypot(cloud.xx - cloud.yy, 2.0 * cloud.xy)) / 2.0;
    // Conservatively reject point clusters and nearly straight walls: tangential
    // motion along a wall cannot be trusted from nearest-neighbour matching alone.
    return std::isfinite(trace) && std::isfinite(minor) && trace > 1e-8 && minor > trace * 1e-3;
}

std::optional<RelativePose> increment(std::span<const Pair> pairs) {
    const auto source = moments(pairs, false);
    const auto target = moments(pairs, true);
    if (!has_spread(source) || !has_spread(target))
        return std::nullopt;
    double dot = 0.0;
    double cross = 0.0;
    for (const auto& pair : pairs) {
        const double px = pair.source.x - source.centre.x;
        const double py = pair.source.y - source.centre.y;
        const double qx = pair.target.x - target.centre.x;
        const double qy = pair.target.y - target.centre.y;
        dot += px * qx + py * qy;
        cross += px * qy - py * qx;
    }
    if (!std::isfinite(dot) || !std::isfinite(cross) || std::hypot(dot, cross) < 1e-12)
        return std::nullopt;
    const double yaw = std::atan2(cross, dot);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return RelativePose{target.centre.x - (c * source.centre.x - s * source.centre.y),
                        target.centre.y - (s * source.centre.x + c * source.centre.y), yaw};
}

MatchResult report(MatchStatus status, std::span<const Pair> pairs, std::size_t iterations) {
    MatchResult result{status, std::nullopt};
    result.correspondences = pairs.size();
    result.iterations = iterations;
    if (!pairs.empty()) {
        double squared_error = 0.0;
        for (const auto& pair : pairs)
            squared_error += pair.distance * pair.distance / static_cast<double>(pairs.size());
        result.rmse = std::sqrt(squared_error);
    }
    return result;
}

bool enough_pairs(std::size_t count, std::size_t cloud_size, const MatcherOptions& options) {
    return count >= options.min_pairs && static_cast<double>(count) >= options.min_overlap * static_cast<double>(cloud_size);
}

} // namespace

std::optional<ScanMatcher> ScanMatcher::Create(MatcherOptions options) {
    if (options.min_pairs < 3 || options.max_points < options.min_pairs || options.max_points > 2048 ||
        options.max_iterations == 0 || options.max_iterations > 200 || !std::isfinite(options.max_pair_distance) ||
        options.max_pair_distance <= 0.0 || !std::isfinite(options.min_overlap) || options.min_overlap <= 0.0 ||
        options.min_overlap > 1.0 || !std::isfinite(options.max_rmse) || options.max_rmse <= 0.0 ||
        options.max_rmse > options.max_pair_distance || !std::isfinite(options.max_translation) ||
        options.max_translation <= 0.0 || !std::isfinite(options.max_rotation) || options.max_rotation <= 0.0 ||
        options.max_rotation > std::numbers::pi)
        return std::nullopt;
    return ScanMatcher(options);
}

ScanMatcher::ScanMatcher(MatcherOptions options) : options_(options) {}

MatchResult ScanMatcher::match(const io::Scan& previous, const io::Scan& current) const {
    const auto reference = sample(previous, options_.max_points);
    const auto reading = sample(current, options_.max_points);
    if (reference.size() < options_.min_pairs || reading.size() < options_.min_pairs)
        return {MatchStatus::InsufficientPoints, std::nullopt};

    RelativePose pose{0.0, 0.0, 0.0};
    auto pairs = associate(reference, reading, pose, options_.max_pair_distance);
    const auto cloud_size = std::max(reference.size(), reading.size());
    for (std::size_t iteration = 1; iteration <= options_.max_iterations; ++iteration) {
        if (!enough_pairs(pairs.size(), cloud_size, options_))
            return report(MatchStatus::InsufficientOverlap, pairs, iteration - 1);
        const auto delta = increment(pairs);
        if (!delta)
            return report(MatchStatus::DegenerateGeometry, pairs, iteration - 1);

        // Compose delta after the current estimate: T_next = delta * T_current.
        const auto position = transform({pose.dx, pose.dy}, *delta);
        pose = {position.x, position.y, std::remainder(pose.dyaw + delta->dyaw, 2.0 * std::numbers::pi)};
        if (!std::isfinite(pose.dx) || !std::isfinite(pose.dy) || !std::isfinite(pose.dyaw) ||
            std::hypot(pose.dx, pose.dy) > options_.max_translation || std::abs(pose.dyaw) > options_.max_rotation)
            return report(MatchStatus::MotionLimitExceeded, pairs, iteration);

        pairs = associate(reference, reading, pose, options_.max_pair_distance);
        if (!enough_pairs(pairs.size(), cloud_size, options_))
            return report(MatchStatus::InsufficientOverlap, pairs, iteration);
        // Correspondences may change after the last update; validate their geometry too.
        if (!has_spread(moments(pairs, false)) || !has_spread(moments(pairs, true)))
            return report(MatchStatus::DegenerateGeometry, pairs, iteration);
        if (std::hypot(delta->dx, delta->dy) <= 1e-4 && std::abs(delta->dyaw) <= 1e-4) {
            auto result = report(MatchStatus::Success, pairs, iteration);
            if (!std::isfinite(result.rmse) || result.rmse > options_.max_rmse) {
                result.status = MatchStatus::PoorFit;
                return result;
            }
            result.motion = pose;
            return result;
        }
    }
    return report(MatchStatus::NotConverged, pairs, options_.max_iterations);
}

} // namespace localization
