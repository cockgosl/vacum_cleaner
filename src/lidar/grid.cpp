#include "lidar/grid.hpp"

#include <cmath>
#include <stdexcept>

namespace lidar {

std::optional<Grid> Grid::Create(std::size_t width, std::size_t height, double resolution) {
    if (width == 0 || height == 0)
        return std::nullopt;
    if (!std::isfinite(resolution) || resolution <= 0.0)
        return std::nullopt;
    if (width > std::vector<CellState>{}.max_size() / height)
        return std::nullopt;

    return Grid(width, height, resolution);
}

Grid::Grid(std::size_t width, std::size_t height, double resolution)
    : width_(width), height_(height), resolution_(resolution), cells_(width * height, CellState::Unknown) {}

std::size_t Grid::width() const noexcept {
    return width_;
}

std::size_t Grid::height() const noexcept {
    return height_;
}

double Grid::resolution() const noexcept {
    return resolution_;
}

CellState Grid::at(std::size_t x, std::size_t y) const {
    return cells_[index(x, y)];
}

void Grid::set(std::size_t x, std::size_t y, CellState state) {
    cells_[index(x, y)] = state;
}

std::size_t Grid::index(std::size_t x, std::size_t y) const {
    if (x >= width_ || y >= height_)
        throw std::out_of_range("Cell coordinates are outside the grid");
    return y * width_ + x;
}

} // namespace lidar
