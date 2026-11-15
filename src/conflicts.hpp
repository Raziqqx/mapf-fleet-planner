#pragma once
// Internal: conflict detection between the agents' paths of one CBS node.

#include <vector>

#include "mapf/planner.hpp"

namespace mapf::detail {

// Vertex conflict: a1 and a2 are both at cell c1 at time t (either may be
// parked at its goal). Edge conflict: a1 moves c1 -> c2 while a2 moves
// c2 -> c1, both arriving at time t.
struct Conflict {
    int a1 = -1;
    int a2 = -1;
    bool edge = false;
    Cell c1 = -1;
    Cell c2 = -1;
    int t = -1;
};

struct ConflictSummary {
    int count = 0;      // number of colliding (vertex, time) / (edge, time) events
    Conflict earliest;  // valid when count > 0
};

// Hash-based detection, linear in the total path length.
ConflictSummary find_conflicts(const std::vector<const Path*>& paths);

}  // namespace mapf::detail
