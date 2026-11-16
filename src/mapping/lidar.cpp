#include "mapping/lidar.hpp"

#include <limits>
#include <utility>

namespace lidar {

std::optional<Lidar> Lidar::Create(std::size_t width, std::size_t height, double resolution) {
    auto grid = Grid::Create(width, height, resolution);
    if (!grid)
        return std::nullopt;

    return Lidar(std::move(*grid));
}

Lidar::Lidar(Grid grid) : grid_(std::move(grid)) {}

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
