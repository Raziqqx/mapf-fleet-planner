// Baseline: prioritized planning with a space-time reservation table.
#include "space_time.hpp"

namespace mapf {

Result plan_prioritized(const Instance& instance, const Budget& budget) {
    detail::Deadline deadline(budget);
    Result result;
    std::vector<std::vector<int>> dist;
    if (!detail::precheck(instance, dist)) {
        result.status = Status::Infeasible;
        detail::finish(result, deadline);
        return result;
    }
    for (std::size_t i = 0; i < instance.agents.size(); ++i)
        result.lower_bound += dist[i][static_cast<std::size_t>(instance.agents[i].start)];

    // Everything planned so far is a hard constraint for the agents after it.
    detail::ConstraintTable reserved;
    result.status = Status::Solved;
    for (std::size_t i = 0; i < instance.agents.size(); ++i) {
        if (deadline.expired()) {
            result.status = Status::TimeLimit;
            break;
        }
        const Agent& agent = instance.agents[i];
        auto ll = detail::focal_search(instance.grid, dist[i], agent.start, agent.goal, reserved,
                                       nullptr, 1.0, deadline, result.stats);
        if (!ll.found) {
            result.status = ll.out_of_time ? Status::TimeLimit : Status::Failed;
            break;
        }
        const Path& p = ll.path;
        const int last = path_cost(p);
        for (int t = 0; t < last; ++t) reserved.add_vertex(p[static_cast<std::size_t>(t)], t);
        // Reserving the reverse of every move forbids swapping through it.
        for (int t = 1; t <= last; ++t)
            if (p[static_cast<std::size_t>(t)] != p[static_cast<std::size_t>(t) - 1])
                reserved.add_edge(p[static_cast<std::size_t>(t)],
                                  p[static_cast<std::size_t>(t) - 1], t);
        reserved.add_permanent(p.back(), last);
        result.paths.push_back(std::move(ll.path));
    }
    detail::finish(result, deadline);
    return result;
}

}  // namespace mapf
