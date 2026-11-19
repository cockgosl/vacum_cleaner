#include "mapping/lidar.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

namespace lidar {

std::optional<Lidar> Lidar::Create(std::size_t width, std::size_t height, double resolution) {
    auto grid = Grid::Create(width, height, resolution);
    if (!grid)
        return std::nullopt;

    return Lidar(std::move(*grid));
}

Lidar::Lidar(Grid grid) : grid_(std::move(grid)) {}

namespace {

std::optional<std::pair<double, double>> measurement_hit(const SensorPose& pose, double angle, double distance) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) || !std::isfinite(angle) ||
        !std::isfinite(distance) || distance <= 0.0)
        return std::nullopt;

    // Reduce each angle before addition to avoid overflow for large finite angles.
    constexpr double full_turn = 2.0 * std::numbers::pi;
    const double direction = std::remainder(pose.yaw, full_turn) + std::remainder(angle, full_turn);
    const double x = pose.x + distance * std::cos(direction);
    const double y = pose.y + distance * std::sin(direction);
    if (!std::isfinite(x) || !std::isfinite(y))
        return std::nullopt;
    return std::pair{x, y};
}

} // namespace

bool Lidar::update_measurement(const SensorPose& pose, double angle, double distance) {
    const auto hit = measurement_hit(pose, angle, distance);
    return hit && update_ray(pose.x, pose.y, hit->first, hit->second);
}

std::optional<ScanUpdate> Lidar::update_scan(const SensorPose& pose, std::span<const io::Measurement> measurements) {
    if (!std::isfinite(pose.yaw) || !grid_.world_to_cell(pose.x, pose.y))
        return std::nullopt;

    // Reserve before writing cells so allocation failure cannot leave a partial scan.
    std::vector<CellIndex> endpoints;
    endpoints.reserve(measurements.size());
    ScanUpdate result;
    for (const auto& point : measurements) {
        if (!std::isfinite(point.angle) || !std::isfinite(point.distance) || point.distance < 0.0) {
            ++result.rejected;
            continue;
        }
        if (point.distance == 0.0) {
            ++result.skipped;
            continue;
        }
        const auto hit = measurement_hit(pose, point.angle, point.distance);
        const auto cell = hit ? grid_.world_to_cell(hit->first, hit->second) : std::nullopt;
        if (!cell) {
            ++result.rejected;
            continue;
        }
        update_ray(pose.x, pose.y, hit->first, hit->second);
        endpoints.push_back(*cell);
        ++result.mapped;
    }
    // A later ray can cross an earlier endpoint. Restore all hits after tracing
    // so the final map of this scan does not depend on measurement order.
    for (const auto cell : endpoints)
        grid_.set(cell.x, cell.y, CellState::Occupied);
    return result;
}

const Grid& Lidar::grid() const noexcept {
    return grid_;
}

namespace {

long double next_crossing(std::size_t current, std::size_t target, long double origin, long double delta, double resolution) {
    if (current == target)
        return std::numeric_limits<long double>::infinity();

    const auto boundary = current < target ? current + 1 : current;
    return (static_cast<long double>(boundary) * resolution - origin) / delta;
}

} // namespace

bool Lidar::update_ray(double start_x, double start_y, double hit_x, double hit_y) {
    const auto start = grid_.world_to_cell(start_x, start_y);
    const auto hit = grid_.world_to_cell(hit_x, hit_y);
    if (!start || !hit)
        return false;

    const long double delta_x = static_cast<long double>(hit_x) - start_x;
    const long double delta_y = static_cast<long double>(hit_y) - start_y;
    auto current = *start;

    while (current.x != hit->x || current.y != hit->y) {
        grid_.set(current.x, current.y, CellState::Free);

        // Compare the next vertical and horizontal crossings along the segment.
        const auto next_x = next_crossing(current.x, hit->x, start_x, delta_x, grid_.resolution());
        const auto next_y = next_crossing(current.y, hit->y, start_y, delta_y, grid_.resolution());

        // Equality means crossing a corner: both coordinates advance together.
        if (next_x <= next_y)
            current.x = current.x < hit->x ? current.x + 1 : current.x - 1;
        if (next_y <= next_x)
            current.y = current.y < hit->y ? current.y + 1 : current.y - 1;
    }

    grid_.set(hit->x, hit->y, CellState::Occupied);
    return true;
}

} // namespace lidar
