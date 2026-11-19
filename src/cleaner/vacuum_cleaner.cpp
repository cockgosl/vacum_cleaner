#include "cleaner/vacuum_cleaner.hpp"

#include <cmath>
#include <numbers>
#include <utility>

namespace cleaner {

std::optional<VacuumCleaner> VacuumCleaner::Create(RobotSpecs specs, std::size_t width, std::size_t height, double resolution,
                                                   std::optional<lidar::SensorPose> initial_pose,
                                                   localization::MatcherOptions matcher_options) {
    if (!std::isfinite(specs.radius) || specs.radius <= 0.0 || specs.autonomy.count() <= 0)
        return std::nullopt;
    auto lidar = lidar::Lidar::Create(width, height, resolution);
    auto matcher = localization::ScanMatcher::Create(matcher_options);
    if (!lidar || !matcher)
        return std::nullopt;
    // Multiplying by 0.5 before resolution avoids an unnecessary overflow at the centre.
    auto pose = initial_pose.value_or(
        lidar::SensorPose{static_cast<double>(width) * 0.5 * resolution, static_cast<double>(height) * 0.5 * resolution, 0.0});
    if (!std::isfinite(pose.yaw) || !lidar->grid().world_to_cell(pose.x, pose.y))
        return std::nullopt;
    pose.yaw = std::remainder(pose.yaw, 2.0 * std::numbers::pi);
    return VacuumCleaner(specs, std::move(*lidar), std::move(*matcher), pose);
}

VacuumCleaner::VacuumCleaner(RobotSpecs specs, lidar::Lidar lidar, localization::ScanMatcher matcher, lidar::SensorPose pose)
    : specs_(specs), lidar_(std::move(lidar)), matcher_(std::move(matcher)), pose_(pose) {}

bool VacuumCleaner::update_ray(double start_x, double start_y, double hit_x, double hit_y) {
    if (tracking_state_ == TrackingState::TrackingLost)
        return false;
    return lidar_.update_ray(start_x, start_y, hit_x, hit_y);
}

const lidar::Grid& VacuumCleaner::grid() const noexcept {
    return lidar_.grid();
}

const RobotSpecs& VacuumCleaner::specs() const noexcept {
    return specs_;
}

const lidar::SensorPose& VacuumCleaner::pose() const noexcept {
    return pose_;
}

TrackingState VacuumCleaner::tracking_state() const noexcept {
    return tracking_state_;
}

ScanResult VacuumCleaner::process_scan(const io::Scan& scan) {
    if (tracking_state_ == TrackingState::TrackingLost)
        return {ScanStatus::TrackingLost, std::nullopt, std::nullopt, std::nullopt};

    const bool initializing = !reference_;
    auto match = matcher_.match(initializing ? scan : *reference_, scan);
    if (match.status != localization::MatchStatus::Success || !match.motion) {
        if (initializing)
            return {ScanStatus::Rejected, std::nullopt, std::move(match), std::nullopt};
        tracking_state_ = TrackingState::TrackingLost;
        return {ScanStatus::TrackingLost, std::nullopt, std::move(match), std::nullopt};
    }

    auto candidate = pose_;
    if (!initializing) {
        // Relative translation is expressed in the previous sensor axes, not map axes.
        const auto& motion = *match.motion;
        const double c = std::cos(pose_.yaw);
        const double s = std::sin(pose_.yaw);
        candidate.x += c * motion.dx - s * motion.dy;
        candidate.y += s * motion.dx + c * motion.dy;
        candidate.yaw = std::remainder(pose_.yaw + motion.dyaw, 2.0 * std::numbers::pi);
    }
    if (!std::isfinite(candidate.yaw) || !lidar_.grid().world_to_cell(candidate.x, candidate.y)) {
        tracking_state_ = TrackingState::TrackingLost;
        return {ScanStatus::PoseOutsideMap, std::nullopt, std::move(match), std::nullopt};
    }

    // Copy before mapping. Allocation failure leaves the accepted reference and map intact.
    auto next_reference = scan;
    const auto update = lidar_.update_scan(candidate, scan.measurements);
    if (!update) {
        tracking_state_ = TrackingState::TrackingLost;
        return {ScanStatus::PoseOutsideMap, std::nullopt, std::move(match), std::nullopt};
    }
    pose_ = candidate;
    reference_ = std::move(next_reference);
    tracking_state_ = TrackingState::Tracking;
    return {initializing ? ScanStatus::Initialized : ScanStatus::Updated, pose_, std::move(match), *update};
}

StepResult VacuumCleaner::run_step(rooting::Pose start, lidar::CellIndex goal, std::chrono::seconds elapsed) const {
    if (elapsed.count() < 0)
        return {StepStatus::InvalidElapsedTime, std::nullopt};
    if (elapsed >= specs_.autonomy)
        return {StepStatus::TimeLimitReached, std::nullopt};
    if (tracking_state_ == TrackingState::TrackingLost)
        return {StepStatus::TrackingLost, std::nullopt};
    auto plan = scheduler_.plan(lidar_.grid(), start, goal, specs_.radius);
    const auto status = plan.status == rooting::PlanStatus::Success ? StepStatus::Planned : StepStatus::PlanningFailed;
    return {status, std::move(plan)};
}

} // namespace cleaner
