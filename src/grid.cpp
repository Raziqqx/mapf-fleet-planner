#include "mapf/grid.hpp"

#include <algorithm>
#include <stdexcept>

namespace mapf {

Grid::Grid(int width, int height, std::vector<std::uint8_t> blocked)
    : width_(width), height_(height), free_count_(0), blocked_(std::move(blocked)) {
    if (width <= 0 || height <= 0) throw std::invalid_argument("grid dimensions must be positive");
    if (static_cast<long>(width) * height > kMaxCells)
        throw std::invalid_argument("grid has more than 2^21 cells");
    if (blocked_.size() != static_cast<std::size_t>(width) * height)
        throw std::invalid_argument("blocked mask size does not match the grid dimensions");
    free_count_ = static_cast<int>(std::count(blocked_.begin(), blocked_.end(), 0));
}

int Grid::neighbours(Cell c, std::array<Cell, 4>& out) const {
    const int x = x_of(c);
    const int y = y_of(c);
    int n = 0;
    if (x > 0 && is_free(c - 1)) out[n++] = c - 1;
    if (x + 1 < width_ && is_free(c + 1)) out[n++] = c + 1;
    if (y > 0 && is_free(c - width_)) out[n++] = c - width_;
    if (y + 1 < height_ && is_free(c + width_)) out[n++] = c + width_;
    return n;
}

std::vector<int> bfs_distances(const Grid& grid, Cell source) {
    std::vector<int> dist(static_cast<std::size_t>(grid.size()), -1);
    if (!grid.is_free(source)) return dist;
    std::vector<Cell> queue{source};
    queue.reserve(static_cast<std::size_t>(grid.free_count()));
    dist[static_cast<std::size_t>(source)] = 0;
    std::array<Cell, 4> nbrs{};
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const Cell c = queue[head];
        const int n = grid.neighbours(c, nbrs);
        for (int i = 0; i < n; ++i) {
            auto& d = dist[static_cast<std::size_t>(nbrs[i])];
            if (d >= 0) continue;
            d = dist[static_cast<std::size_t>(c)] + 1;
            queue.push_back(nbrs[i]);
        }
    }
    return dist;
}

}  // namespace mapf
