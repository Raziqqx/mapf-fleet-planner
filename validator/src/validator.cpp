#include "mapf_validator/validator.hpp"

#include <algorithm>
#include <cstdlib>
#include <sstream>

namespace mapf_validator {
namespace {

constexpr std::size_t kMaxErrors = 20;

class ErrorLog {
public:
    explicit ErrorLog(Report& report) : report_(report) {}
    template <typename... Parts>
    void add(const Parts&... parts) {
        report_.valid = false;
        if (report_.errors.size() >= kMaxErrors) return;
        std::ostringstream os;
        (os << ... << parts);
        report_.errors.push_back(os.str());
    }

private:
    Report& report_;
};

std::string str(const Pos& p) {
    return "(" + std::to_string(p.x) + "," + std::to_string(p.y) + ")";
}

// Position of agent `plan` at time t: its last position once the plan ends.
const Pos& at(const std::vector<Pos>& plan, std::size_t t) {
    return t < plan.size() ? plan[t] : plan.back();
}

}  // namespace

Report validate(const Problem& problem, const std::vector<std::vector<Pos>>& plans) {
    Report report;
    ErrorLog log(report);
    const std::size_t n = problem.starts.size();
    if (problem.goals.size() != n)
        log.add("problem has ", n, " starts but ", problem.goals.size(), " goals");
    if (plans.size() != n) log.add("expected ", n, " plans, got ", plans.size());
    bool map_ok = problem.obstacle.size() == static_cast<std::size_t>(problem.height);
    for (const auto& row : problem.obstacle)
        map_ok = map_ok && row.size() == static_cast<std::size_t>(problem.width);
    if (!map_ok) log.add("obstacle mask does not match the map size");
    if (!report.valid) return report;

    const auto on_free_cell = [&](const Pos& p) {
        return p.x >= 0 && p.y >= 0 && p.x < problem.width && p.y < problem.height &&
               !problem.obstacle[static_cast<std::size_t>(p.y)][static_cast<std::size_t>(p.x)];
    };

    std::size_t horizon = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& plan = plans[i];
        if (plan.empty()) {
            log.add("agent ", i, ": empty plan");
            continue;
        }
        horizon = std::max(horizon, plan.size());
        if (!(plan.front() == problem.starts[i]))
            log.add("agent ", i, ": starts at ", str(plan.front()), ", expected ",
                    str(problem.starts[i]));
        if (!(plan.back() == problem.goals[i]))
            log.add("agent ", i, ": ends at ", str(plan.back()), ", expected ",
                    str(problem.goals[i]));
        for (std::size_t t = 0; t < plan.size(); ++t) {
            if (!on_free_cell(plan[t]))
                log.add("agent ", i, ": t=", t, " on a blocked or off-map cell ", str(plan[t]));
            if (t > 0) {
                const int d =
                    std::abs(plan[t].x - plan[t - 1].x) + std::abs(plan[t].y - plan[t - 1].y);
                if (d > 1)
                    log.add("agent ", i, ": t=", t, " jumps from ", str(plan[t - 1]), " to ",
                            str(plan[t]));
            }
        }
        // Cost: the last time the agent is not at its goal, plus one.
        std::size_t cost = plan.size() - 1;
        while (cost > 0 && plan[cost - 1] == plan.back()) --cost;
        report.sum_of_costs += static_cast<long>(cost);
        report.makespan = std::max(report.makespan, static_cast<int>(cost));
    }
    if (!report.valid) return report;

    // Pairwise, over every time step until the last agent stops moving
    // (afterwards nobody moves, so nothing new can collide). Quadratic in the
    // number of agents on purpose: simple enough to trust.
    for (std::size_t t = 0; t < horizon; ++t) {
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                if (at(plans[i], t) == at(plans[j], t))
                    log.add("vertex conflict: agents ", i, " and ", j, " at ", str(at(plans[i], t)),
                            " t=", t);
                if (t > 0 && at(plans[i], t) == at(plans[j], t - 1) &&
                    at(plans[j], t) == at(plans[i], t - 1) &&
                    !(at(plans[i], t) == at(plans[i], t - 1)))
                    log.add("edge conflict: agents ", i, " and ", j, " swap ",
                            str(at(plans[i], t - 1)), " <-> ", str(at(plans[i], t)),
                            " arriving t=", t);
            }
        }
    }
    return report;
}

}  // namespace mapf_validator
