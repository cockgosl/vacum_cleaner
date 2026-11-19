#pragma once

#include "mapping/lidar.hpp"
#include "rooting/scheduler.hpp"

#include <chrono>
#include <optional>

namespace motion {

struct MotionCommand {
    double linear_mps = 0.0;    // Positive is forward.
    double angular_radps = 0.0; // Positive is counterclockwise.
};

struct WheelSpeeds {
    double left_mps = 0.0; // Commanded ground speed, not a measured motor speed.
    double right_mps = 0.0;
};

enum class ControlStatus {
    Stopped,
    Moving,
    Turning,
    GoalReached,
    StaleScan,
    Blocked,
    InvalidInput
};

struct ControlResult {
    ControlStatus status = ControlStatus::Stopped;
    MotionCommand command{};
    WheelSpeeds wheels{};
};

struct ControllerOptions {
    double wheel_separation = 0.30;   // Metres; a demonstration value until the robot is measured.
    double max_linear_speed = 0.10;   // Metres per second.
    double max_angular_speed = 0.50;  // Radians per second.
    double max_wheel_speed = 0.15;    // Metres per second for either wheel.
    double position_tolerance = 0.03; // Metres from the goal cell centre.
    double heading_tolerance = 0.08;  // Radians, in (0, pi/4].
    std::chrono::milliseconds max_scan_age{500};
};

class Controller {
public:
    // All numeric options must be finite and positive; heading_tolerance <= pi/4.
    // max_scan_age must be positive. Invalid options return nullopt.
    static std::optional<Controller> Create(ControllerOptions options = {});

    const ControllerOptions& options() const noexcept;

    // Validates the grid route and swept footprint, turns in place, then drives forward.
    // Skips initial in-place turns and prefers the next cell centre when directly reachable.
    // This prevents repeated replanning from pulling the robot back to its current cell centre.
    // Rejects stale data, malformed paths, collisions and invalid inputs with zero commands.
    // The application must also stop commands when tracking or planning is unavailable.
    ControlResult compute(const lidar::Grid& grid, const lidar::SensorPose& pose, const rooting::PlanResult& plan, double radius,
                          std::chrono::milliseconds scan_age) const;

private:
    explicit Controller(ControllerOptions options);

    ControllerOptions options_;
};

} // namespace motion
