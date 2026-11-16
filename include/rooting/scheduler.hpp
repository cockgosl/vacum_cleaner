#pragma once

#include "mapping/grid.hpp"

#include <vector>

namespace rooting {

enum class PlanStatus {
    Success,
    InvalidParameters,
    InvalidStart,
    InvalidGoal,
    NoPath
};

struct PlanResult {
    PlanStatus status;
    std::vector<lidar::CellIndex> path;
};

class Scheduler {
public:
    // Positions are cell centres; radius is in metres and must be finite and positive.
    // Uses four-neighbour A*. A successful path includes both start and goal.
    // The circular footprint must avoid unknown/occupied cells and the map boundary.
    // Touching is treated as a collision. Consecutive positions are checked continuously.
    PlanResult plan(const lidar::Grid& grid, lidar::CellIndex start, lidar::CellIndex goal, double radius) const;
};

} // namespace rooting
