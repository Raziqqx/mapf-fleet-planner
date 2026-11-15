// Single-agent space-time focal search (see focal_search in space_time.hpp).
#include <climits>
#include <set>
#include <tuple>

#include "space_time.hpp"

namespace mapf::detail {
namespace {

struct Node {
    Cell cell;
    int t;  // also g: every action, waiting included, costs 1
    int f;
    int conflicts;  // soft conflicts accumulated along the best known path here
    int parent;     // index into the node pool, -1 for the root
    bool closed;
};

// Poll the clock once per this many expansions.
constexpr std::uint64_t kClockPollMask = 255;

}  // namespace

LowLevelResult focal_search(const Grid& grid, const std::vector<int>& h, Cell start, Cell goal,
                            const ConstraintTable& constraints, const ConflictAvoidanceTable* cat,
                            double w, const Deadline& deadline, Stats& stats) {
    ++stats.ll_calls;
    LowLevelResult result;
    const int min_goal_t = constraints.min_goal_time(goal);
    if (min_goal_t < 0 || constraints.vertex_blocked(start, 0)) return result;
    if (cat != nullptr && cat->empty()) cat = nullptr;

    // Beyond the last constraint the space-time graph repeats, so any goal
    // that is reachable at all is reached within free_count further steps.
    const int horizon =
        std::max(constraints.last_constraint_time(), min_goal_t) + grid.free_count() + 1;
    // Admissible and consistent: the agent cannot finish before min_goal_t.
    const auto heuristic = [&](Cell c, int t) {
        return std::max(h[static_cast<std::size_t>(c)], min_goal_t - t);
    };

    std::vector<Node> nodes;
    std::unordered_map<std::uint64_t, int> index;  // vertex_key(cell, t) -> node
    // open: every unexpanded node by f. focal: those with f <= w * f_min, by
    // fewest conflicts, then smaller f, then deeper (larger t) to reach the goal sooner.
    using OpenKey = std::pair<int, int>;              // (f, id)
    using FocalKey = std::tuple<int, int, int, int>;  // (conflicts, f, -t, id)
    std::set<OpenKey> open;
    std::set<FocalKey> focal;
    const auto focal_key = [&nodes](int id) {
        const Node& n = nodes[static_cast<std::size_t>(id)];
        return FocalKey{n.conflicts, n.f, -n.t, id};
    };

    nodes.push_back(Node{start, 0, heuristic(start, 0), 0, -1, false});
    index.emplace(vertex_key(start, 0), 0);
    open.emplace(nodes[0].f, 0);
    focal.insert(focal_key(0));
    double bound = w * nodes[0].f;

    std::array<Cell, 4> nbrs{};
    std::uint64_t expanded = 0;
    while (!open.empty()) {
        if ((++expanded & kClockPollMask) == 0 && deadline.expired()) {
            result.out_of_time = true;
            break;
        }
        // Raise the focal bound if f_min grew, pulling the newly eligible nodes in.
        const int f_min = open.begin()->first;
        if (w * f_min > bound) {
            const double new_bound = w * f_min;
            for (auto it = open.upper_bound({static_cast<int>(bound), INT_MAX});
                 it != open.end() && it->first <= new_bound; ++it)
                focal.insert(focal_key(it->second));
            bound = new_bound;
        }
        const int id = std::get<3>(*focal.begin());
        focal.erase(focal.begin());
        Node cur = nodes[static_cast<std::size_t>(id)];
        open.erase({cur.f, id});
        nodes[static_cast<std::size_t>(id)].closed = true;
        ++stats.ll_expanded;

        if (cur.cell == goal && cur.t >= min_goal_t) {
            result.found = true;
            result.lower_bound = f_min;
            result.path.resize(static_cast<std::size_t>(cur.t) + 1);
            for (int i = id; i >= 0; i = nodes[static_cast<std::size_t>(i)].parent)
                result.path[static_cast<std::size_t>(nodes[static_cast<std::size_t>(i)].t)] =
                    nodes[static_cast<std::size_t>(i)].cell;
            break;
        }
        if (cur.t >= horizon) continue;

        const int t = cur.t + 1;
        const int n = grid.neighbours(cur.cell, nbrs);
        // k == n is the wait action.
        for (int k = 0; k <= n; ++k) {
            const Cell next = k < n ? nbrs[static_cast<std::size_t>(k)] : cur.cell;
            if (h[static_cast<std::size_t>(next)] < 0) continue;
            if (constraints.vertex_blocked(next, t) || constraints.edge_blocked(cur.cell, next, t))
                continue;
            const int conflicts = cur.conflicts + (cat ? cat->count(cur.cell, next, t) : 0);
            const auto [it, inserted] =
                index.try_emplace(vertex_key(next, t), static_cast<int>(nodes.size()));
            if (!inserted) {
                // Same (cell, t) means same g and f; only a path with fewer
                // conflicts is an improvement, and only while still open.
                Node& old = nodes[static_cast<std::size_t>(it->second)];
                if (old.closed || conflicts >= old.conflicts) continue;
                const bool in_focal = focal.erase(focal_key(it->second)) != 0;
                old.conflicts = conflicts;
                old.parent = id;
                if (in_focal) focal.insert(focal_key(it->second));
                continue;
            }
            const int f = t + heuristic(next, t);
            nodes.push_back(Node{next, t, f, conflicts, id, false});
            const int new_id = it->second;
            open.emplace(f, new_id);
            if (f <= bound) focal.insert(focal_key(new_id));
        }
    }
    return result;
}

}  // namespace mapf::detail
