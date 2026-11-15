#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include "mapf/grid.hpp"

namespace mapf {

struct Agent {
    Cell start;
    Cell goal;
};

// One MAPF problem: a map plus one start/goal pair per agent.
struct Instance {
    Grid grid;
    std::vector<Agent> agents;
};

// Parses a MovingAI .map file ("type", "height", "width", "map", rows).
// Throws std::runtime_error with a message naming the problem on bad input.
Grid parse_map(std::istream& in);
Grid load_map(const std::string& path);

// Parses a MovingAI .scen file and keeps its first `num_agents` entries.
// Throws std::runtime_error if the file has fewer entries, a coordinate is out
// of range or on an obstacle, or the scenario's map size disagrees with `grid`.
std::vector<Agent> parse_scenario(std::istream& in, const Grid& grid, int num_agents);
Instance load_instance(const std::string& map_path, const std::string& scen_path, int num_agents);

}  // namespace mapf
