#pragma once

#include "io/stl_parser.hpp"
#include "mapping/grid.hpp"

#include <cstddef>
#include <optional>
#include <span>

namespace lidar {

struct SensorPose {
    double x;   // Metres in the grid frame.
    double y;   // Metres in the grid frame.
    double yaw; // Radians counterclockwise from the grid's positive x-axis.
};

struct ScanUpdate {
    std::size_t mapped = 0;
    std::size_t skipped = 0;  // Valid zero-distance returns.
    std::size_t rejected = 0; // Invalid measurements or endpoints outside the map.
};

class Lidar {
public:
    // Uses Grid::Create to validate dimensions and resolution.
    // Returns nullopt for invalid parameters; allocation failures may throw.
    static std::optional<Lidar> Create(std::size_t width, std::size_t height, double resolution);

    // Coordinates are in metres in the grid frame. The endpoint must be a valid hit.
    // Marks traversed cells Free and the endpoint cell Occupied, including same-cell rays.
    // At an exact corner crossing, advances diagonally without clearing side cells.
    // Returns false without changing the grid if either endpoint is invalid or outside.
    bool update_ray(double start_x, double start_y, double hit_x, double hit_y);

    // A valid return: angle is counterclockwise relative to the sensor, in radians.
    // Distance is in metres and must be positive; every input must be finite.
    // Both sensor and hit must lie inside the grid. Returns false without changing
    // the map for invalid inputs or an out-of-bounds ray; no clipping is performed.
    bool update_measurement(const SensorPose& pose, double angle, double distance);

    // Maps a scan at a known pose. Endpoints remain occupied regardless of ray order.
    // A later scan can clear old obstacles. Zero returns are skipped, not cleared.
    // Returns nullopt without changing the map for an invalid/outside sensor pose.
    // Invalid measurements and outside endpoints are rejected individually; no clipping.
    // Allocation failures may throw before the map is changed.
    std::optional<ScanUpdate> update_scan(const SensorPose& pose, std::span<const io::Measurement> measurements);

    // Read-only access to the owned map without copying.
    const Grid& grid() const noexcept;

private:
    explicit Lidar(Grid grid);

    Grid grid_;
};

} // namespace lidar
