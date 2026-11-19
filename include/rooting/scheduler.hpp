#pragma once

#include "mapping/grid.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rooting {

enum class PlanStatus {
    Success,
    InvalidParameters,
    InvalidStart,
    InvalidGoal,
    NoPath
};

// North is increasing y; enum order follows counterclockwise quarter-turns.
enum class Direction : std::uint8_t {
    East,
    North,
    West,
    South
};

struct Pose {
    lidar::CellIndex cell;
    Direction direction;
};

struct PlanResult {
    PlanStatus status;
    // Includes start and goal; a turn changes direction while keeping the cell.
    std::vector<Pose> path;
    std::size_t cost = 0; // One unit per forward move or 90-degree turn.
};

class Scheduler {
public:
    // Positions are cell centres; radius is in metres and must be finite and positive.
    // A* states include direction. Actions are forward, turn left, and turn right.
    // Goal orientation is unrestricted; the heuristic ignores turn costs.
    // The circular footprint must avoid unknown/occupied cells and the map boundary.
    // Touching is treated as a collision. Consecutive positions are checked continuously.
    PlanResult plan(const lidar::Grid& grid, Pose start, lidar::CellIndex goal, double radius) const;

    // Tests a straight swept circular footprint between world coordinates in metres.
    // Both endpoint footprints and the entire segment must avoid unknown/occupied
    // cells and the map boundary. Touching is a collision; invalid inputs return false.
    bool can_move(const lidar::Grid& grid, double from_x, double from_y, double to_x, double to_y, double radius) const;
};

} // namespace rooting
