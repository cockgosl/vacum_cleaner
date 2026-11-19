#pragma once

#include "io/scan_assembler.hpp"

#include <cstddef>
#include <limits>
#include <optional>

namespace localization {

struct RelativePose {
    double dx;   // Sensor displacement in the previous sensor frame, in metres.
    double dy;   // Sensor displacement in the previous sensor frame, in metres.
    double dyaw; // Counterclockwise sensor rotation, in radians.
};

enum class MatchStatus {
    Success,
    InsufficientPoints,
    InsufficientOverlap,
    DegenerateGeometry,
    MotionLimitExceeded,
    NotConverged,
    PoorFit
};

struct MatchResult {
    MatchStatus status;
    // Present only for Success. It transforms current-frame points into the previous frame.
    std::optional<RelativePose> motion;
    double rmse = std::numeric_limits<double>::infinity(); // Metres; infinity when there are no pairs.
    std::size_t correspondences = 0;
    std::size_t iterations = 0;
};

struct MatcherOptions {
    std::size_t max_points = 180;
    std::size_t min_pairs = 12;
    std::size_t max_iterations = 40;
    double max_pair_distance = 0.4; // Metres; rejects distant nearest neighbours.
    double min_overlap = 0.6;       // Required fraction of each sampled cloud with unique correspondences.
    double max_rmse = 0.06;         // Metres; final fit threshold.
    double max_translation = 0.5;   // Metres per scan pair.
    double max_rotation = 0.35;     // Radians per scan pair.
};

class ScanMatcher {
public:
    // Validates finite positive thresholds, overlap in (0, 1], and max_rmse <= max_pair_distance.
    // Requires 3 <= min_pairs <= max_points <= 2048, 1 <= max_iterations <= 200,
    // and max_rotation <= pi. Returns nullopt for invalid options.
    static std::optional<ScanMatcher> Create(MatcherOptions options = {});

    // Stateless point-to-point 2D ICP, initialized with zero relative motion.
    // Filters zero, negative and non-finite distances and non-finite angles.
    // Samples valid returns evenly in arrival order, using at most max_points per cloud.
    // Neither input scan is changed. Matching assumes small motion and a mostly static scene.
    // Success is a local fit, not proof of correct localization in a symmetric environment.
    MatchResult match(const io::Scan& previous, const io::Scan& current) const;

private:
    explicit ScanMatcher(MatcherOptions options);

    MatcherOptions options_;
};

} // namespace localization
