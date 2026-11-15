#pragma once
// Internal: space-time keys, the run budget, and the two tables the
// single-agent search consults (hard constraints and soft conflict counts).

#include <chrono>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mapf/planner.hpp"

namespace mapf::detail {

// Packed hash keys. Cells take 21 bits (Grid::kMaxCells), time the rest.
inline std::uint64_t vertex_key(Cell c, int t) {
    return (static_cast<std::uint64_t>(t) << 21) | static_cast<std::uint64_t>(c);
}
// A move from `from` to `to` that arrives at time t.
inline std::uint64_t edge_key(Cell from, Cell to, int t) {
    return (static_cast<std::uint64_t>(t) << 42) | (static_cast<std::uint64_t>(from) << 21) |
           static_cast<std::uint64_t>(to);
}

// Wall-clock and node budget shared by all levels of one planner call.
class Deadline {
public:
    explicit Deadline(const Budget& budget)
        : start_(Clock::now()),
          end_(start_ + std::chrono::duration_cast<Clock::duration>(
                            std::chrono::duration<double>(budget.time_limit_s))) {}
    // Reading the clock costs ~20 ns; callers poll this every few hundred expansions.
    bool expired() const { return Clock::now() >= end_; }
    double elapsed_s() const {
        return std::chrono::duration<double>(Clock::now() - start_).count();
    }

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point start_;
    Clock::time_point end_;
};

// Hard constraints for one agent: forbidden (cell, t) vertices, forbidden
// moves, and cells that become blocked forever from some time on (another
// agent parked at its goal, used by prioritized planning).
class ConstraintTable {
public:
    void add_vertex(Cell c, int t);
    void add_edge(Cell from, Cell to, int t);
    void add_permanent(Cell c, int from_t);

    bool vertex_blocked(Cell c, int t) const;
    bool edge_blocked(Cell from, Cell to, int t) const {
        return !edges_.empty() && edges_.count(edge_key(from, to, t)) != 0;
    }
    // Earliest time from which the agent may sit at `goal` forever, or -1 if
    // the goal is blocked permanently at some point.
    int min_goal_time(Cell goal) const;
    // The latest time at which any constraint applies; after it only the
    // permanent blocks matter.
    int last_constraint_time() const { return last_time_; }

private:
    std::unordered_set<std::uint64_t> vertices_;
    std::unordered_set<std::uint64_t> edges_;
    std::unordered_map<Cell, int> permanent_;           // cell -> blocked from this time on
    std::unordered_map<Cell, int> latest_vertex_time_;  // cell -> latest vertex constraint
    int last_time_ = 0;
};

// Soft information for the focal searches: how many other agents occupy a
// vertex, traverse an edge, or sit parked at their goal at a given time.
class ConflictAvoidanceTable {
public:
    void add_path(const Path& path);
    // Conflicts caused by moving from `from` to `to`, arriving at time t.
    int count(Cell from, Cell to, int t) const;
    bool empty() const { return vertices_.empty() && parked_.empty(); }

private:
    std::unordered_map<std::uint64_t, int> vertices_;
    std::unordered_map<std::uint64_t, int> edges_;
    std::unordered_map<Cell, std::vector<int>> parked_;  // cell -> times agents park there
};

struct LowLevelResult {
    bool found = false;
    bool out_of_time = false;
    Path path;
    // Minimum f in the open list when the path was returned: a lower bound on
    // this agent's optimal cost under the constraints (equals the path cost when w == 1).
    int lower_bound = 0;
};

// Space-time focal search for one agent. With w == 1 this is A* that breaks
// f ties by fewest conflicts with the CAT (then deeper first); with w > 1 it
// returns a path of cost <= w * optimal that greedily avoids conflicts.
// `h` holds the BFS distances to `goal`. `cat` may be null.
LowLevelResult focal_search(const Grid& grid, const std::vector<int>& h, Cell start, Cell goal,
                            const ConstraintTable& constraints, const ConflictAvoidanceTable* cat,
                            double w, const Deadline& deadline, Stats& stats);

// Shared input checks: distinct starts and goals, every goal reachable.
// Returns false (instance infeasible) otherwise and fills the distance tables.
bool precheck(const Instance& instance, std::vector<std::vector<int>>& goal_distances);

// Fills sum_of_costs/makespan/runtime for a solved result.
void finish(Result& result, const Deadline& deadline);

}  // namespace mapf::detail
