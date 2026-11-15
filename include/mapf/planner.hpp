#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "mapf/instance.hpp"

namespace mapf {

// path[t] is the agent's cell at timestep t. After its last entry the agent
// stays at its goal forever (the standard "stay at target" MAPF semantics), so
// a path never ends with redundant waits at the goal.
using Path = std::vector<Cell>;

enum class Status {
    Solved,
    Infeasible,  // proved unsolvable (an agent cannot reach its goal, or duplicate starts/goals)
    Failed,      // an incomplete planner (prioritized) gave up; the instance may be solvable
    TimeLimit,   // the wall-clock budget ran out
    NodeLimit,   // the high-level node budget ran out
};

const char* to_string(Status status);

// Resource limits for one planner call. The planners check the clock every few
// hundred expansions, so they overshoot the time limit by well under a millisecond.
struct Budget {
    double time_limit_s = 60.0;
    std::uint64_t max_hl_nodes = std::numeric_limits<std::uint64_t>::max();
};

struct Stats {
    double runtime_s = 0.0;
    std::uint64_t hl_expanded = 0;   // CBS/ECBS constraint-tree nodes expanded
    std::uint64_t hl_generated = 0;  // ... and generated
    std::uint64_t ll_calls = 0;      // single-agent searches run
    std::uint64_t ll_expanded = 0;   // states expanded by those searches
};

struct Result {
    Status status = Status::Failed;
    std::vector<Path> paths;  // one per agent when solved, otherwise empty
    long sum_of_costs = 0;
    int makespan = 0;
    // A proven lower bound on the optimal sum of costs (sum of single-agent
    // distances for prioritized planning; the open-list minimum for CBS/ECBS).
    long lower_bound = 0;
    Stats stats;
};

// Cost of one path: the timestep at which the agent reaches its goal for good.
inline int path_cost(const Path& path) { return static_cast<int>(path.size()) - 1; }
long sum_of_costs(const std::vector<Path>& paths);
int makespan(const std::vector<Path>& paths);

// Implementation switches for the single-agent search, all chosen at run time
// so before/after profiles come from one binary (see scripts/profile.py).
// Every combination expands exactly the same states in the same order (the
// ordering keys are total), so only speed and memory differ, never the result.
enum class OpenList {
    Set,         // original: std::set for OPEN (by f) and FOCAL (by conflicts, f, -t, id)
    BinaryHeap,  // binary heaps with lazy deletion for OPEN, the not-yet-focal nodes and FOCAL
    Bucket,      // OPEN as buckets indexed by the integer f; FOCAL stays a binary heap
};
enum class NodeLayout {
    AoS,  // one vector of node structs
    SoA,  // one vector per node field
};
struct SearchTuning {
    OpenList open_list = OpenList::Bucket;
    NodeLayout layout = NodeLayout::AoS;
    // Allocate search nodes, the (cell, t) index and the queues from a per-thread
    // arena that is reused across searches, instead of the global heap.
    bool arena = true;
};
// Process-wide setting. Set it before starting planners; it is read once per
// single-agent search, so changing it while planners run is a data race.
void set_search_tuning(const SearchTuning& tuning);
SearchTuning search_tuning();
const char* to_string(OpenList open_list);
const char* to_string(NodeLayout layout);

// Baseline: agents are planned one at a time in scenario order, each with a
// space-time A* that avoids the paths reserved by the agents before it.
// Fast but incomplete: it returns Status::Failed when an agent finds no path.
Result plan_prioritized(const Instance& instance, const Budget& budget);

// Conflict-Based Search: optimal sum of costs (vertex + edge conflicts).
Result plan_cbs(const Instance& instance, const Budget& budget);

// Enhanced CBS: bounded-suboptimal, sum of costs <= w * optimal (w >= 1).
// Focal search at both levels, preferring nodes/paths with fewer conflicts.
Result plan_ecbs(const Instance& instance, double w, const Budget& budget);

}  // namespace mapf
