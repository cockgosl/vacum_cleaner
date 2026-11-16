#pragma once

#include "mapping/grid.hpp"

#include <cstddef>
#include <optional>

namespace lidar {

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

    // Read-only access to the owned map without copying.
    const Grid& grid() const noexcept;

private:
    explicit Lidar(Grid grid);

    Grid grid_;
};

} // namespace lidar
