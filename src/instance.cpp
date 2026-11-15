#include "mapf/instance.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace mapf {
namespace {

std::ifstream open_or_throw(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path);
    return in;
}

// '.', 'G' and 'S' are passable terrain in the MovingAI format; everything
// else ('@', 'O', 'T', 'W') is treated as an obstacle.
bool is_passable(char ch) { return ch == '.' || ch == 'G' || ch == 'S'; }

}  // namespace

Grid parse_map(std::istream& in) {
    int width = -1;
    int height = -1;
    std::string key;
    // Header: "type <t>", "height <h>", "width <w>" in any order, then "map".
    while (in >> key && key != "map") {
        if (key == "type") {
            in >> key;
        } else if (key == "height") {
            in >> height;
        } else if (key == "width") {
            in >> width;
        } else {
            throw std::runtime_error("map: unexpected header token '" + key + "'");
        }
    }
    if (key != "map") throw std::runtime_error("map: missing 'map' line");
    if (width <= 0 || height <= 0) throw std::runtime_error("map: missing or bad width/height");
    if (static_cast<long>(width) * height > Grid::kMaxCells)
        throw std::runtime_error("map: too large");

    std::vector<std::uint8_t> blocked;
    blocked.reserve(static_cast<std::size_t>(width) * height);
    std::string row;
    for (int y = 0; y < height; ++y) {
        if (!(in >> row))
            throw std::runtime_error("map: expected " + std::to_string(height) + " rows, got " +
                                     std::to_string(y));
        if (static_cast<int>(row.size()) != width)
            throw std::runtime_error("map: row " + std::to_string(y) + " has length " +
                                     std::to_string(row.size()) + ", expected " +
                                     std::to_string(width));
        for (char ch : row) blocked.push_back(is_passable(ch) ? 0 : 1);
    }
    return Grid(width, height, std::move(blocked));
}

Grid load_map(const std::string& path) {
    auto in = open_or_throw(path);
    try {
        return parse_map(in);
    } catch (const std::runtime_error& e) {
        throw std::runtime_error(path + ": " + e.what());
    }
}

std::vector<Agent> parse_scenario(std::istream& in, const Grid& grid, int num_agents) {
    if (num_agents < 0) throw std::runtime_error("scen: negative agent count");
    std::string line;
    if (!std::getline(in, line) || line.rfind("version", 0) != 0)
        throw std::runtime_error("scen: missing 'version' line");

    std::vector<Agent> agents;
    agents.reserve(static_cast<std::size_t>(num_agents));
    int line_no = 1;
    while (static_cast<int>(agents.size()) < num_agents && std::getline(in, line)) {
        ++line_no;
        if (line.empty()) continue;
        // bucket, map name, map width, map height, sx, sy, gx, gy, optimal length.
        // Split on tabs: the map name column could in principle contain spaces.
        std::vector<std::string> cols;
        std::stringstream ss(line);
        for (std::string col; std::getline(ss, col, '\t');) cols.push_back(col);
        const std::string where = "scen line " + std::to_string(line_no) + ": ";
        if (cols.size() < 8) throw std::runtime_error(where + "expected 9 tab-separated columns");
        int v[6];
        try {
            for (int i = 0; i < 6; ++i) v[i] = std::stoi(cols[static_cast<std::size_t>(i) + 2]);
        } catch (const std::exception&) {
            throw std::runtime_error(where + "non-integer field");
        }
        if (v[0] != grid.width() || v[1] != grid.height())
            throw std::runtime_error(where + "map size does not match the map file");
        if (!grid.in_bounds(v[2], v[3]) || !grid.in_bounds(v[4], v[5]))
            throw std::runtime_error(where + "coordinate out of bounds");
        const Agent a{grid.cell(v[2], v[3]), grid.cell(v[4], v[5])};
        if (!grid.is_free(a.start) || !grid.is_free(a.goal))
            throw std::runtime_error(where + "start or goal is on an obstacle");
        agents.push_back(a);
    }
    if (static_cast<int>(agents.size()) < num_agents)
        throw std::runtime_error("scen: requested " + std::to_string(num_agents) +
                                 " agents, file has " + std::to_string(agents.size()));
    return agents;
}

Instance load_instance(const std::string& map_path, const std::string& scen_path, int num_agents) {
    Grid grid = load_map(map_path);
    auto in = open_or_throw(scen_path);
    try {
        auto agents = parse_scenario(in, grid, num_agents);
        return Instance{std::move(grid), std::move(agents)};
    } catch (const std::runtime_error& e) {
        throw std::runtime_error(scen_path + ": " + e.what());
    }
}

}  // namespace mapf
