#include "lidar/grid.hpp"

#include <cstddef>
#include <iostream>

int main() {
    auto grid_result = lidar::Grid::Create(10, 6, 0.05);
    if (!grid_result) {
        std::cerr << "Invalid grid parameters\n";
        return 1;
    }
    auto& grid = *grid_result;

    // Simulate one observed ray: free cells followed by a wall.
    for (std::size_t x = 1; x < 6; ++x)
        grid.set(x, 2, lidar::CellState::Free);
    grid.set(6, 2, lidar::CellState::Occupied);

    std::cout << "Grid: " << grid.width() << " x " << grid.height() << ", resolution: " << grid.resolution() << " m/cell\n";
    std::cout << "? unknown, . free, # occupied; rows shown in increasing y\n";

    for (std::size_t y = 0; y < grid.height(); ++y) {
        for (std::size_t x = 0; x < grid.width(); ++x) {
            switch (grid.at(x, y)) {
            case lidar::CellState::Unknown:
                std::cout << '?';
                break;
            case lidar::CellState::Free:
                std::cout << '.';
                break;
            case lidar::CellState::Occupied:
                std::cout << '#';
                break;
            }
        }
        std::cout << '\n';
    }
}
