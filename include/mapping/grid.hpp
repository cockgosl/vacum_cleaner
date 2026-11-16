#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace lidar {

enum class CellState : std::uint8_t {
    Unknown,
    Free,
    Occupied
};

struct CellIndex {
    std::size_t x;
    std::size_t y;
};

class Grid {
public:
    // Returns nullopt for invalid dimensions, resolution, or storage capacity.
    // Allocation failures may throw std::bad_alloc.
    static std::optional<Grid> Create(std::size_t width, std::size_t height, double resolution);

    std::size_t width() const noexcept;
    std::size_t height() const noexcept;
    double resolution() const noexcept; // Metres per cell.

    // Metres from the bottom-left corner: x points right, y points up.
    // The lower edges are included; the upper edges are excluded.
    // Returns nullopt for non-finite coordinates or points outside the grid.
    std::optional<CellIndex> world_to_cell(double x, double y) const;

    // Coordinates are cell indices: x is the column, y is the row.
    // Both operations throw std::out_of_range outside the grid.
    CellState at(std::size_t x, std::size_t y) const;
    void set(std::size_t x, std::size_t y, CellState state);

private:
    Grid(std::size_t width, std::size_t height, double resolution);

    std::size_t index(std::size_t x, std::size_t y) const;

    std::size_t width_;
    std::size_t height_;
    double resolution_;
    std::vector<CellState> cells_;
};

} // namespace lidar
