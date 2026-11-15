#pragma once
// Helpers shared by the test files.

#include <algorithm>
#include <random>
#include <string>
#include <vector>

#include "mapf/planner.hpp"

namespace mapf::test {

// Builds a grid from rows of '.' (free) and '@' (obstacle).
inline Grid grid_from_rows(const std::vector<std::string>& rows) {
    std::vector<std::uint8_t> blocked;
    for (const auto& row : rows)
        for (char ch : row) blocked.push_back(ch == '.' ? 0 : 1);
    return Grid(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()),
                std::move(blocked));
}

// A random grid with obstacle density `density` and `num_agents` agents whose
// distinct starts and goals all lie in the largest connected free region, so
// each agent can reach its goal on its own.
inline Instance random_instance(std::mt19937& rng, int width, int height, double density,
                                int num_agents) {
    std::bernoulli_distribution obstacle(density);
    std::vector<std::uint8_t> blocked(static_cast<std::size_t>(width) * height);
    for (auto& b : blocked) b = obstacle(rng) ? 1 : 0;
    blocked[0] = 0;  // at least one free cell
    Grid grid(width, height, blocked);

    // Largest connected component by repeated BFS.
    std::vector<int> component(static_cast<std::size_t>(grid.size()), -1);
    std::vector<std::vector<Cell>> members;
    for (Cell c = 0; c < grid.size(); ++c) {
        if (!grid.is_free(c) || component[static_cast<std::size_t>(c)] >= 0) continue;
        const auto dist = bfs_distances(grid, c);
        members.emplace_back();
        for (Cell d = 0; d < grid.size(); ++d) {
            if (dist[static_cast<std::size_t>(d)] < 0) continue;
            component[static_cast<std::size_t>(d)] = static_cast<int>(members.size()) - 1;
            members.back().push_back(d);
        }
    }
    std::vector<Cell> cells = members.front();
    for (const auto& m : members)
        if (m.size() > cells.size()) cells = m;

    std::vector<Cell> starts = cells;
    std::vector<Cell> goals = cells;
    std::shuffle(starts.begin(), starts.end(), rng);
    std::shuffle(goals.begin(), goals.end(), rng);
    std::vector<Agent> agents;
    const int n = std::min<int>(num_agents, static_cast<int>(cells.size()));
    for (int i = 0; i < n; ++i)
        agents.push_back({starts[static_cast<std::size_t>(i)], goals[static_cast<std::size_t>(i)]});
    return Instance{std::move(grid), std::move(agents)};
}

}  // namespace mapf::test
