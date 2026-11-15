#pragma once
// Independent MAPF solution checker. It deliberately shares no code or types
// with the planners (only the standard library), so a bug in the planners'
// grid, conflict or cost logic cannot hide itself here.

#include <string>
#include <vector>

namespace mapf_validator {

struct Pos {
    int x;
    int y;
    bool operator==(const Pos& o) const { return x == o.x && y == o.y; }
};

struct Problem {
    int width = 0;
    int height = 0;
    // rows[y][x] is true where there is an obstacle.
    std::vector<std::vector<bool>> obstacle;
    std::vector<Pos> starts;
    std::vector<Pos> goals;
};

struct Report {
    bool valid = true;
    std::vector<std::string> errors;  // human-readable, at most the first 20
    long sum_of_costs = 0;            // computed independently of the planner
    int makespan = 0;
};

// Checks that `plans` (plans[i][t] = agent i's position at time t) is a
// solution of `problem` under the "stay at goal" semantics:
//  - one non-empty plan per agent, starting at its start and ending at its goal;
//  - every position is on the map and obstacle-free;
//  - each step moves to a 4-adjacent cell or waits in place;
//  - no two agents share a cell at the same time, including an agent that
//    already finished and sits at its goal (vertex conflict);
//  - no two agents swap cells over one step (edge conflict).
// An agent's cost is the time it reaches its goal and never leaves again.
Report validate(const Problem& problem, const std::vector<std::vector<Pos>>& plans);

}  // namespace mapf_validator
