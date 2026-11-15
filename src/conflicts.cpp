#include "conflicts.hpp"

#include <unordered_map>

#include "space_time.hpp"

namespace mapf::detail {

ConflictSummary find_conflicts(const std::vector<const Path*>& paths) {
    ConflictSummary summary;
    const auto record = [&summary](const Conflict& c) {
        if (summary.count++ == 0 || c.t < summary.earliest.t) summary.earliest = c;
    };

    struct Parked {
        int agent;
        int since;
    };
    std::unordered_map<Cell, Parked> parked;
    std::size_t total = 0;
    for (std::size_t a = 0; a < paths.size(); ++a) {
        parked.emplace(paths[a]->back(), Parked{static_cast<int>(a), path_cost(*paths[a])});
        total += paths[a]->size();
    }
    std::unordered_map<std::uint64_t, int> vertices;  // first agent seen at (cell, t)
    std::unordered_map<std::uint64_t, int> edges;     // first agent seen making a move
    vertices.reserve(total);
    edges.reserve(total);

    for (std::size_t ai = 0; ai < paths.size(); ++ai) {
        const int a = static_cast<int>(ai);
        const Path& p = *paths[ai];
        const int last = path_cost(p);
        for (int t = 0; t < last; ++t) {
            const Cell c = p[static_cast<std::size_t>(t)];
            const auto [it, inserted] = vertices.try_emplace(vertex_key(c, t), a);
            if (!inserted) record(Conflict{it->second, a, false, c, -1, t});
            if (auto pk = parked.find(c);
                pk != parked.end() && pk->second.agent != a && pk->second.since <= t)
                record(Conflict{pk->second.agent, a, false, c, -1, t});
        }
        for (int t = 1; t <= last; ++t) {
            const Cell from = p[static_cast<std::size_t>(t) - 1];
            const Cell to = p[static_cast<std::size_t>(t)];
            if (from == to) continue;
            // Whoever is processed second finds the first one's opposite move.
            if (auto it = edges.find(edge_key(to, from, t)); it != edges.end())
                record(Conflict{it->second, a, true, to, from, t});
            edges.try_emplace(edge_key(from, to, t), a);
        }
    }
    return summary;
}

}  // namespace mapf::detail
