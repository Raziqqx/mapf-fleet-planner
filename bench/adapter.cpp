#include "adapter.hpp"

namespace mapf {

mapf_validator::Problem to_validator_problem(const Instance& instance) {
    const Grid& g = instance.grid;
    mapf_validator::Problem p;
    p.width = g.width();
    p.height = g.height();
    p.obstacle.assign(static_cast<std::size_t>(g.height()),
                      std::vector<bool>(static_cast<std::size_t>(g.width()), false));
    for (int y = 0; y < g.height(); ++y)
        for (int x = 0; x < g.width(); ++x)
            p.obstacle[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] =
                !g.is_free(g.cell(x, y));
    for (const Agent& a : instance.agents) {
        p.starts.push_back({g.x_of(a.start), g.y_of(a.start)});
        p.goals.push_back({g.x_of(a.goal), g.y_of(a.goal)});
    }
    return p;
}

std::vector<std::vector<mapf_validator::Pos>> to_validator_plans(const Grid& grid,
                                                                 const std::vector<Path>& paths) {
    std::vector<std::vector<mapf_validator::Pos>> plans;
    for (const Path& path : paths) {
        auto& plan = plans.emplace_back();
        for (Cell c : path) plan.push_back({grid.x_of(c), grid.y_of(c)});
    }
    return plans;
}

mapf_validator::Report validate_result(const Instance& instance, const Result& result) {
    return mapf_validator::validate(to_validator_problem(instance),
                                    to_validator_plans(instance.grid, result.paths));
}

}  // namespace mapf
