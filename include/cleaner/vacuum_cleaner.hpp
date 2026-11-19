#pragma once

#include "localization/scan_matcher.hpp"
#include "mapping/lidar.hpp"
#include "rooting/scheduler.hpp"

#include <chrono>
#include <optional>

namespace cleaner {

struct RobotSpecs {
    double radius; // Metres; the first model has a circular footprint.
    std::chrono::seconds autonomy;
};

enum class StepStatus {
    Planned,
    PlanningFailed,
    TimeLimitReached,
    InvalidElapsedTime,
    TrackingLost,
    WaitingForScan
};

struct StepResult {
    StepStatus status;
    // Present when planning was attempted; includes endpoint or search failure reasons.
    std::optional<rooting::PlanResult> plan;
};

enum class TrackingState {
    WaitingForScan,
    Tracking,
    TrackingLost
};

enum class ScanStatus {
    Initialized,
    Updated,
    Rejected,
    TrackingLost,
    PoseOutsideMap
};

struct ScanResult {
    ScanStatus status;
    // A current accepted pose is present only for Initialized and Updated.
    std::optional<lidar::SensorPose> pose;
    std::optional<localization::MatchResult> match;
    std::optional<lidar::ScanUpdate> map_update;
};

class VacuumCleaner {
public:
    // Validates specs, matcher options and initial pose, then creates the owned map.
    // Defaults to the map centre with yaw=0. Explicit poses must lie inside the map.
    static std::optional<VacuumCleaner> Create(RobotSpecs specs, std::size_t width, std::size_t height, double resolution,
                                               std::optional<lidar::SensorPose> initial_pose = std::nullopt,
                                               localization::MatcherOptions matcher_options = {});

    // Manual ray updates are blocked after tracking is lost.
    bool update_ray(double start_x, double start_y, double hit_x, double hit_y);
    const lidar::Grid& grid() const noexcept;
    const RobotSpecs& specs() const noexcept;

    // Validates the first scan by self-matching, then matches against the last accepted scan.
    // Accepted motion is composed in map coordinates before the scan is mapped.
    // A bad first scan is rejected and can be retried. After initialization, a match
    // failure or outside pose latches TrackingLost; pose, reference and map stay unchanged.
    // Recreate the session to restart tracking. Scans are assumed rigid during acquisition.
    ScanResult process_scan(const io::Scan& scan);

    // Initial or last accepted pose. Check tracking_state() before using it as current.
    const lidar::SensorPose& pose() const noexcept;
    TrackingState tracking_state() const noexcept;

    // One planning tick, without movement or marking any area as cleaned.
    // The caller supplies elapsed time since the cleaning session started.
    StepResult run_step(rooting::Pose start, lidar::CellIndex goal, std::chrono::seconds elapsed) const;

    // Plans from the accepted tracked pose to the centre of a fixed goal cell.
    // Requires Tracking; validates the swept footprint from the actual position
    // to the first cell centre before searching. Heading is rounded to the nearest
    // cardinal direction, with counterclockwise ties. Grid cost excludes the connector.
    StepResult run_step(lidar::CellIndex goal, std::chrono::seconds elapsed) const;

private:
    VacuumCleaner(RobotSpecs specs, lidar::Lidar lidar, localization::ScanMatcher matcher, lidar::SensorPose pose);

    RobotSpecs specs_;
    rooting::Scheduler scheduler_;
    lidar::Lidar lidar_;
    localization::ScanMatcher matcher_;
    lidar::SensorPose pose_;
    std::optional<io::Scan> reference_;
    TrackingState tracking_state_ = TrackingState::WaitingForScan;
};

} // namespace cleaner
