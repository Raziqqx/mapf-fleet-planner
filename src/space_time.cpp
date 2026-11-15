#include "space_time.hpp"

#include <algorithm>
#include <unordered_set>

namespace mapf {

const char* to_string(Status status) {
    switch (status) {
        case Status::Solved: return "solved";
        case Status::Infeasible: return "infeasible";
        case Status::Failed: return "failed";
        case Status::TimeLimit: return "time_limit";
        case Status::NodeLimit: return "node_limit";
    }
    return "unknown";
}

long sum_of_costs(const std::vector<Path>& paths) {
    long soc = 0;
    for (const auto& p : paths) soc += path_cost(p);
    return soc;
}

int makespan(const std::vector<Path>& paths) {
    int m = 0;
    for (const auto& p : paths) m = std::max(m, path_cost(p));
    return m;
}

}  // namespace mapf

namespace mapf::detail {

void ConstraintTable::add_vertex(Cell c, int t) {
    vertices_.insert(vertex_key(c, t));
    auto [it, inserted] = latest_vertex_time_.try_emplace(c, t);
    if (!inserted) it->second = std::max(it->second, t);
    last_time_ = std::max(last_time_, t);
}

void ConstraintTable::add_edge(Cell from, Cell to, int t) {
    edges_.insert(edge_key(from, to, t));
    last_time_ = std::max(last_time_, t);
}

void ConstraintTable::add_permanent(Cell c, int from_t) {
    auto [it, inserted] = permanent_.try_emplace(c, from_t);
    if (!inserted) it->second = std::min(it->second, from_t);
    last_time_ = std::max(last_time_, from_t);
}

bool ConstraintTable::vertex_blocked(Cell c, int t) const {
    if (!permanent_.empty()) {
        auto it = permanent_.find(c);
        if (it != permanent_.end() && t >= it->second) return true;
    }
    return !vertices_.empty() && vertices_.count(vertex_key(c, t)) != 0;
}

int ConstraintTable::min_goal_time(Cell goal) const {
    if (permanent_.count(goal) != 0) return -1;
    auto it = latest_vertex_time_.find(goal);
    return it == latest_vertex_time_.end() ? 0 : it->second + 1;
}

void ConflictAvoidanceTable::add_path(const Path& path) {
    // Occupied vertices before arrival, the moves, then the goal from arrival on.
    const int last = path_cost(path);
    const auto at = [&path](int t) { return path[static_cast<std::size_t>(t)]; };
    for (int t = 0; t < last; ++t) ++vertices_[vertex_key(at(t), t)];
    for (int t = 1; t <= last; ++t)
        if (at(t) != at(t - 1)) ++edges_[edge_key(at(t - 1), at(t), t)];
    parked_[path.back()].push_back(last);
}

int ConflictAvoidanceTable::count(Cell from, Cell to, int t) const {
    int n = 0;
    if (auto it = vertices_.find(vertex_key(to, t)); it != vertices_.end()) n += it->second;
    if (auto it = parked_.find(to); it != parked_.end())
        for (int since : it->second) n += since <= t ? 1 : 0;
    // A swap: someone else moves to -> from over the same step.
    if (from != to)
        if (auto it = edges_.find(edge_key(to, from, t)); it != edges_.end()) n += it->second;
    return n;
}

bool precheck(const Instance& instance, std::vector<std::vector<int>>& goal_distances) {
    std::unordered_set<Cell> starts;
    std::unordered_set<Cell> goals;
    goal_distances.clear();
    for (const auto& a : instance.agents) {
        if (!starts.insert(a.start).second || !goals.insert(a.goal).second) return false;
        goal_distances.push_back(bfs_distances(instance.grid, a.goal));
        if (goal_distances.back()[static_cast<std::size_t>(a.start)] < 0) return false;
    }
    return true;
}

void finish(Result& result, const Deadline& deadline) {
    result.stats.runtime_s = deadline.elapsed_s();
    if (result.status != Status::Solved) {
        result.paths.clear();
        return;
    }
    result.sum_of_costs = sum_of_costs(result.paths);
    result.makespan = makespan(result.paths);
}

}  // namespace mapf::detail
