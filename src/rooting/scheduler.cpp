#include "rooting/scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <utility>

namespace rooting {
namespace {

bool inside(const lidar::Grid& grid, lidar::CellIndex cell) {
    return cell.x < grid.width() && cell.y < grid.height();
}

// For an axis-aligned segment, its bounding rectangle is the segment itself.
// The distance to a cell rectangle can therefore be computed independently per axis.
bool can_traverse(const lidar::Grid& grid, lidar::CellIndex from, lidar::CellIndex to, long double radius) {
    const long double min_x = static_cast<long double>(std::min(from.x, to.x)) + 0.5L;
    const long double max_x = static_cast<long double>(std::max(from.x, to.x)) + 0.5L;
    const long double min_y = static_cast<long double>(std::min(from.y, to.y)) + 0.5L;
    const long double max_y = static_cast<long double>(std::max(from.y, to.y)) + 0.5L;

    if (min_x - radius <= 0.0L || min_y - radius <= 0.0L || max_x + radius >= static_cast<long double>(grid.width()) ||
        max_y + radius >= static_cast<long double>(grid.height()))
        return false;

    // Include the cell before each lower bound to catch exact tangency.
    const auto left = static_cast<std::size_t>(std::floor(min_x - radius));
    const auto bottom = static_cast<std::size_t>(std::floor(min_y - radius));
    const auto right = static_cast<std::size_t>(std::floor(max_x + radius));
    const auto top = static_cast<std::size_t>(std::floor(max_y + radius));
    for (std::size_t y = bottom > 0 ? bottom - 1 : 0; y <= top; ++y) {
        for (std::size_t x = left > 0 ? left - 1 : 0; x <= right; ++x) {
            if (grid.at(x, y) == lidar::CellState::Free)
                continue;
            const long double cell_x = x;
            const long double cell_y = y;
            const auto dx = std::max({cell_x - max_x, min_x - (cell_x + 1.0L), 0.0L});
            const auto dy = std::max({cell_y - max_y, min_y - (cell_y + 1.0L), 0.0L});
            if (std::hypot(dx, dy) <= radius)
                return false;
        }
    }
    return true;
}

std::size_t heuristic(lidar::CellIndex cell, lidar::CellIndex goal) {
    const auto dx = cell.x < goal.x ? goal.x - cell.x : cell.x - goal.x;
    const auto dy = cell.y < goal.y ? goal.y - cell.y : cell.y - goal.y;
    return dx + dy;
}

struct Entry {
    std::size_t index;
    std::size_t cost;
    std::size_t priority;
};

struct LaterEntry {
    bool operator()(const Entry& left, const Entry& right) const {
        if (left.priority != right.priority)
            return left.priority > right.priority;
        return left.index > right.index;
    }
};

constexpr std::size_t direction_count = 4;

std::size_t state_index(Pose pose, std::size_t width) {
    return (pose.cell.y * width + pose.cell.x) * direction_count + static_cast<std::size_t>(pose.direction);
}

Pose state_pose(std::size_t index, std::size_t width) {
    const auto cell = index / direction_count;
    return {{cell % width, cell / width}, static_cast<Direction>(index % direction_count)};
}

std::optional<lidar::CellIndex> forward_cell(Pose pose, const lidar::Grid& grid) {
    auto cell = pose.cell;
    switch (pose.direction) {
    case Direction::East:
        if (cell.x + 1 < grid.width())
            return lidar::CellIndex{cell.x + 1, cell.y};
        break;
    case Direction::North:
        if (cell.y + 1 < grid.height())
            return lidar::CellIndex{cell.x, cell.y + 1};
        break;
    case Direction::West:
        if (cell.x > 0)
            return lidar::CellIndex{cell.x - 1, cell.y};
        break;
    case Direction::South:
        if (cell.y > 0)
            return lidar::CellIndex{cell.x, cell.y - 1};
        break;
    }
    return std::nullopt;
}

} // namespace

PlanResult Scheduler::plan(const lidar::Grid& grid, Pose start, lidar::CellIndex goal, double radius) const {
    if (!std::isfinite(radius) || radius <= 0.0)
        return {PlanStatus::InvalidParameters, {}};
    const long double cell_radius = static_cast<long double>(radius) / grid.resolution();
    if (!std::isfinite(cell_radius))
        return {PlanStatus::InvalidParameters, {}};
    if (static_cast<std::size_t>(start.direction) >= direction_count || !inside(grid, start.cell) ||
        !can_traverse(grid, start.cell, start.cell, cell_radius))
        return {PlanStatus::InvalidStart, {}};
    if (!inside(grid, goal) || !can_traverse(grid, goal, goal, cell_radius))
        return {PlanStatus::InvalidGoal, {}};

    const auto width = grid.width();
    const auto cells = width * grid.height();
    if (cells > std::vector<std::size_t>{}.max_size() / direction_count)
        return {PlanStatus::InvalidParameters, {}};
    const auto count = cells * direction_count;
    const auto start_index = state_index(start, width);
    constexpr auto unseen = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> costs(count, unseen);
    std::vector<std::size_t> parents(count, unseen);
    std::priority_queue<Entry, std::vector<Entry>, LaterEntry> open;
    costs[start_index] = 0;
    open.push({start_index, 0, heuristic(start.cell, goal)});

    while (!open.empty()) {
        const auto current = open.top();
        open.pop();
        if (current.cost != costs[current.index])
            continue;
        const auto pose = state_pose(current.index, width);
        if (pose.cell.x == goal.x && pose.cell.y == goal.y) {
            std::vector<Pose> path;
            for (auto index = current.index;; index = parents[index]) {
                path.push_back(state_pose(index, width));
                if (index == start_index)
                    break;
            }
            std::reverse(path.begin(), path.end());
            return {PlanStatus::Success, std::move(path), current.cost};
        }

        const auto visit = [&](Pose next) {
            const auto index = state_index(next, width);
            const auto cost = current.cost + 1;
            if (cost >= costs[index])
                return;
            costs[index] = cost;
            parents[index] = current.index;
            open.push({index, cost, cost + heuristic(next.cell, goal)});
        };

        // A circular footprint occupies the same area during an in-place turn.
        const auto direction = static_cast<std::size_t>(pose.direction);
        visit({pose.cell, static_cast<Direction>((direction + 1) % direction_count)});
        visit({pose.cell, static_cast<Direction>((direction + direction_count - 1) % direction_count)});
        if (const auto next = forward_cell(pose, grid); next && can_traverse(grid, pose.cell, *next, cell_radius))
            visit({*next, pose.direction});
    }
    return {PlanStatus::NoPath, {}};
}

} // namespace rooting
