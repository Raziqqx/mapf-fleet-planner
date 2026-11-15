#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace mapf {

// A cell is addressed by its linear index y * width + x. Plain ints keep the
// hot loops and hash keys cheap.
using Cell = int;

// A 4-connected grid map. Agents may move to an adjacent free cell or wait.
class Grid {
public:
    // Largest supported cell count: cell ids must fit the 21 bits that the
    // packed hash keys in the planners reserve for them.
    static constexpr int kMaxCells = 1 << 21;

    // `blocked` has width * height entries in row-major order.
    Grid(int width, int height, std::vector<std::uint8_t> blocked);

    int width() const { return width_; }
    int height() const { return height_; }
    int size() const { return width_ * height_; }
    int free_count() const { return free_count_; }

    bool in_bounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }
    bool is_free(Cell c) const { return blocked_[static_cast<std::size_t>(c)] == 0; }
    Cell cell(int x, int y) const { return y * width_ + x; }
    int x_of(Cell c) const { return c % width_; }
    int y_of(Cell c) const { return c / width_; }

    // Writes the free 4-neighbours of `c` into `out` and returns how many there are.
    int neighbours(Cell c, std::array<Cell, 4>& out) const;

private:
    int width_;
    int height_;
    int free_count_;
    std::vector<std::uint8_t> blocked_;
};

// Breadth-first distances from `source` to every cell; -1 marks unreachable or
// blocked cells. Used as the exact (perfect single-agent) heuristic.
std::vector<int> bfs_distances(const Grid& grid, Cell source);

}  // namespace mapf
