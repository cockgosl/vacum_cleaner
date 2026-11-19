#include "motion/controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace motion {
namespace {

struct Point {
    double x;
    double y;
};

Point centre(lidar::CellIndex cell, double resolution) {
    return {(static_cast<double>(cell.x) + 0.5) * resolution, (static_cast<double>(cell.y) + 0.5) * resolution};
}

bool same_cell(lidar::CellIndex a, lidar::CellIndex b) {
    return a.x == b.x && a.y == b.y;
}

ControlResult output(ControlStatus status, MotionCommand command, const ControllerOptions& options) {
    WheelSpeeds wheels{command.linear_mps - command.angular_radps * options.wheel_separation * 0.5,
                       command.linear_mps + command.angular_radps * options.wheel_separation * 0.5};
    if (!std::isfinite(wheels.left_mps) || !std::isfinite(wheels.right_mps))
        return {ControlStatus::InvalidInput};
    const double maximum = std::max(std::abs(wheels.left_mps), std::abs(wheels.right_mps));
    if (maximum > options.max_wheel_speed) {
        const double scale = options.max_wheel_speed / maximum;
        command.linear_mps *= scale;
        command.angular_radps *= scale;
        wheels.left_mps *= scale;
        wheels.right_mps *= scale;
    }
    return {status, command, wheels};
}

} // namespace

std::optional<Controller> Controller::Create(ControllerOptions options) {
    const std::array<double, 6> values{options.wheel_separation, options.max_linear_speed,   options.max_angular_speed,
                                       options.max_wheel_speed,  options.position_tolerance, options.heading_tolerance};
    if (std::any_of(values.begin(), values.end(), [](double value) { return !std::isfinite(value) || value <= 0.0; }) ||
        options.heading_tolerance > std::numbers::pi / 4.0 || options.max_scan_age.count() <= 0)
        return std::nullopt;
    return Controller(options);
}

Controller::Controller(ControllerOptions options) : options_(options) {}

const ControllerOptions& Controller::options() const noexcept {
    return options_;
}

ControlResult Controller::compute(const lidar::Grid& grid, const lidar::SensorPose& pose, const rooting::PlanResult& plan,
                                  double radius, std::chrono::milliseconds scan_age) const {
    if (scan_age.count() < 0 || !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
        !std::isfinite(radius) || radius <= 0.0)
        return {ControlStatus::InvalidInput};
    if (scan_age >= options_.max_scan_age)
        return {ControlStatus::StaleScan};
    if (plan.status != rooting::PlanStatus::Success)
        return {ControlStatus::Stopped};
    if (plan.path.empty())
        return {ControlStatus::InvalidInput};
    const auto current = grid.world_to_cell(pose.x, pose.y);
    if (!current || !same_cell(*current, plan.path.front().cell))
        return {ControlStatus::InvalidInput};

    const rooting::Scheduler scheduler;
    if (!scheduler.can_move(grid, pose.x, pose.y, pose.x, pose.y, radius))
        return {ControlStatus::Blocked};
    for (std::size_t i = 0; i < plan.path.size(); ++i) {
        const auto& waypoint = plan.path[i];
        if (waypoint.cell.x >= grid.width() || waypoint.cell.y >= grid.height() || static_cast<unsigned>(waypoint.direction) > 3)
            return {ControlStatus::InvalidInput};
        const auto to = centre(waypoint.cell, grid.resolution());
        if (i == 0) {
            if (!scheduler.can_move(grid, to.x, to.y, to.x, to.y, radius))
                return {ControlStatus::Blocked};
            continue;
        }
        const auto& before = plan.path[i - 1];
        if (same_cell(before.cell, waypoint.cell)) {
            const auto a = static_cast<unsigned>(before.direction);
            const auto b = static_cast<unsigned>(waypoint.direction);
            if (a != b && (a + 1) % 4 != b && (a + 3) % 4 != b)
                return {ControlStatus::InvalidInput};
        } else {
            bool valid = before.direction == waypoint.direction;
            switch (before.direction) {
            case rooting::Direction::East:
                valid = valid && waypoint.cell.x == before.cell.x + 1 && waypoint.cell.y == before.cell.y;
                break;
            case rooting::Direction::North:
                valid = valid && waypoint.cell.x == before.cell.x && waypoint.cell.y == before.cell.y + 1;
                break;
            case rooting::Direction::West:
                valid = valid && before.cell.x == waypoint.cell.x + 1 && waypoint.cell.y == before.cell.y;
                break;
            case rooting::Direction::South:
                valid = valid && waypoint.cell.x == before.cell.x && before.cell.y == waypoint.cell.y + 1;
                break;
            }
            if (!valid)
                return {ControlStatus::InvalidInput};
        }
        const auto from = centre(before.cell, grid.resolution());
        if (!scheduler.can_move(grid, from.x, from.y, to.x, to.y, radius))
            return {ControlStatus::Blocked};
    }
    const auto goal = centre(plan.path.back().cell, grid.resolution());
    if (std::hypot(goal.x - pose.x, goal.y - pose.y) <= options_.position_tolerance)
        return {ControlStatus::GoalReached};

    auto target = centre(plan.path.front().cell, grid.resolution());
    for (const auto& waypoint : plan.path) {
        if (!same_cell(waypoint.cell, plan.path.front().cell)) {
            const auto next = centre(waypoint.cell, grid.resolution());
            if (scheduler.can_move(grid, pose.x, pose.y, next.x, next.y, radius))
                target = next;
            break;
        }
    }
    if (!scheduler.can_move(grid, pose.x, pose.y, target.x, target.y, radius))
        return {ControlStatus::Blocked};
    const double distance = std::hypot(target.x - pose.x, target.y - pose.y);
    if (!std::isfinite(distance) || distance < 1e-9)
        return {ControlStatus::Blocked};
    const double yaw = std::remainder(pose.yaw, 2.0 * std::numbers::pi);
    const double error = std::remainder(std::atan2(target.y - pose.y, target.x - pose.x) - yaw, 2.0 * std::numbers::pi);
    if (std::abs(error) > options_.heading_tolerance) {
        const double angular = std::copysign(std::min(options_.max_angular_speed, std::abs(error)), error);
        return output(ControlStatus::Turning, {0.0, angular}, options_);
    }
    const double speed = std::min({options_.max_linear_speed, options_.max_wheel_speed, distance});
    // Check motion along the actual heading over the complete data timeout,
    // rather than assuming perfect alignment with the desired segment.
    const double horizon = std::chrono::duration<double>(options_.max_scan_age).count();
    const double travel = speed * horizon;
    if (!scheduler.can_move(grid, pose.x, pose.y, pose.x + travel * std::cos(yaw), pose.y + travel * std::sin(yaw), radius))
        return {ControlStatus::Blocked};
    return output(ControlStatus::Moving, {speed, 0.0}, options_);
}

} // namespace motion
