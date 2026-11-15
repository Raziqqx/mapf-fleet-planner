#pragma once
// Reference optimal solver for tiny instances (a few agents, <= 16 cells),
// written independently of the planners: Dijkstra over joint states.
//
// The joint state is every agent's cell plus a "finished" bit per agent. An
// agent standing on its goal may finish (a free transition); from then on it
// must stay there. Each timestep costs one per unfinished agent, so an agent
// that finishes at time t has paid exactly t, its "stay at goal" path cost.
// Vertex and swap conflicts are forbidden.

#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "mapf/instance.hpp"

namespace mapf::test {

// Returns the optimal sum of costs, or -1 if the instance has no solution.
inline long brute_force_soc(const Instance& inst) {
    const Grid& g = inst.grid;
    const int n = static_cast<int>(inst.agents.size());
    const std::uint64_t base = static_cast<std::uint64_t>(g.size());
    struct State {
        std::vector<Cell> pos;
        unsigned finished;
    };
    const auto encode = [&](const State& s) {
        std::uint64_t key = 0;
        for (Cell c : s.pos) key = key * base + static_cast<std::uint64_t>(c);
        return (key << n) | s.finished;
    };
    const unsigned all = (1u << n) - 1u;

    using Entry = std::pair<long, std::uint64_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> pq;
    std::unordered_map<std::uint64_t, long> best;
    std::unordered_map<std::uint64_t, State> states;
    const auto relax = [&](const State& s, long cost) {
        const auto key = encode(s);
        auto it = best.find(key);
        if (it != best.end() && it->second <= cost) return;
        best[key] = cost;
        states[key] = s;
        pq.emplace(cost, key);
    };
    State start{{}, 0};
    for (const auto& a : inst.agents) start.pos.push_back(a.start);
    relax(start, 0);

    while (!pq.empty()) {
        const auto [cost, key] = pq.top();
        pq.pop();
        if (best[key] < cost) continue;
        const State s = states[key];
        if (s.finished == all) return cost;
        // Free transitions: finish any agent standing on its goal.
        for (int i = 0; i < n; ++i)
            if (!(s.finished >> i & 1u) &&
                s.pos[static_cast<std::size_t>(i)] == inst.agents[static_cast<std::size_t>(i)].goal)
                relax(State{s.pos, s.finished | (1u << i)}, cost);
        // One timestep: every unfinished agent waits or moves to a free neighbour.
        const long step = n - __builtin_popcount(s.finished);
        std::vector<std::vector<Cell>> options(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            auto& opt = options[static_cast<std::size_t>(i)];
            const Cell c = s.pos[static_cast<std::size_t>(i)];
            opt.push_back(c);
            if (s.finished >> i & 1u) continue;
            std::array<Cell, 4> nb{};
            const int k = g.neighbours(c, nb);
            opt.insert(opt.end(), nb.begin(), nb.begin() + k);
        }
        std::vector<Cell> next(static_cast<std::size_t>(n));
        std::function<void(int)> choose = [&](int i) {
            if (i == n) {
                for (int a = 0; a < n; ++a)
                    for (int b = a + 1; b < n; ++b) {
                        const auto ua = static_cast<std::size_t>(a),
                                   ub = static_cast<std::size_t>(b);
                        if (next[ua] == next[ub]) return;
                        if (next[ua] == s.pos[ub] && next[ub] == s.pos[ua]) return;
                    }
                relax(State{next, s.finished}, cost + step);
                return;
            }
            for (Cell c : options[static_cast<std::size_t>(i)]) {
                next[static_cast<std::size_t>(i)] = c;
                choose(i + 1);
            }
        };
        choose(0);
    }
    return -1;
}

}  // namespace mapf::test
