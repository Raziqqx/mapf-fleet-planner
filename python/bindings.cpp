// pybind11 module `mapf_py`: load MovingAI maps/scenarios, plan with
// PP/CBS/ECBS, and check plans with the independent validator. Positions cross
// the boundary as (x, y) tuples and paths as lists of them, so Python callers
// never see the planner's internal linear cell ids.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "adapter.hpp"
#include "mapf/planner.hpp"

namespace py = pybind11;
using XY = std::pair<int, int>;

namespace {

std::vector<std::vector<XY>> to_xy(const mapf::Grid& grid, const std::vector<mapf::Path>& paths) {
    std::vector<std::vector<XY>> out;
    out.reserve(paths.size());
    for (const auto& path : paths) {
        auto& p = out.emplace_back();
        p.reserve(path.size());
        for (mapf::Cell c : path) p.emplace_back(grid.x_of(c), grid.y_of(c));
    }
    return out;
}

mapf::Cell to_cell(const mapf::Grid& grid, XY xy) {
    if (!grid.in_bounds(xy.first, xy.second))
        throw std::out_of_range("position (" + std::to_string(xy.first) + ", " +
                                std::to_string(xy.second) + ") is outside the map");
    return grid.cell(xy.first, xy.second);
}

// Builds an instance from rows of '.' (free) and '@'/'T' (obstacle).
mapf::Instance make_instance(const std::vector<std::string>& rows,
                             const std::vector<std::pair<XY, XY>>& agents) {
    if (rows.empty()) throw std::invalid_argument("map has no rows");
    std::vector<std::uint8_t> blocked;
    for (const auto& row : rows) {
        if (row.size() != rows.front().size()) throw std::invalid_argument("ragged map rows");
        for (char ch : row) blocked.push_back(ch == '.' || ch == 'G' || ch == 'S' ? 0 : 1);
    }
    mapf::Grid grid(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()),
                    std::move(blocked));
    std::vector<mapf::Agent> list;
    for (const auto& [s, g] : agents) {
        const mapf::Agent a{to_cell(grid, s), to_cell(grid, g)};
        if (!grid.is_free(a.start) || !grid.is_free(a.goal))
            throw std::invalid_argument("agent start or goal on an obstacle");
        list.push_back(a);
    }
    return mapf::Instance{std::move(grid), std::move(list)};
}

py::dict plan(const mapf::Instance& inst, const std::string& planner, double w,
              double time_limit_s) {
    mapf::Budget budget;
    budget.time_limit_s = time_limit_s;
    mapf::Result r;
    {
        // The planners touch no Python state, so other Python threads may run.
        py::gil_scoped_release release;
        if (planner == "pp")
            r = mapf::plan_prioritized(inst, budget);
        else if (planner == "cbs")
            r = mapf::plan_cbs(inst, budget);
        else if (planner == "ecbs")
            r = mapf::plan_ecbs(inst, w, budget);
        else
            throw std::invalid_argument("planner must be 'pp', 'cbs' or 'ecbs'");
    }
    py::dict d;
    d["status"] = mapf::to_string(r.status);
    d["solved"] = r.status == mapf::Status::Solved;
    d["paths"] = to_xy(inst.grid, r.paths);
    d["sum_of_costs"] = r.sum_of_costs;
    d["makespan"] = r.makespan;
    d["lower_bound"] = r.lower_bound;
    d["runtime_s"] = r.stats.runtime_s;
    d["hl_expanded"] = r.stats.hl_expanded;
    d["ll_expanded"] = r.stats.ll_expanded;
    return d;
}

py::dict validate(const mapf::Instance& inst, const std::vector<std::vector<XY>>& paths) {
    std::vector<std::vector<mapf_validator::Pos>> plans;
    for (const auto& path : paths) {
        auto& p = plans.emplace_back();
        for (const auto& [x, y] : path) p.push_back({x, y});
    }
    const auto report = mapf_validator::validate(mapf::to_validator_problem(inst), plans);
    py::dict d;
    d["valid"] = report.valid;
    d["errors"] = report.errors;
    d["sum_of_costs"] = report.sum_of_costs;
    d["makespan"] = report.makespan;
    return d;
}

}  // namespace

PYBIND11_MODULE(mapf_py, m) {
    m.doc() = "Multi-agent path finding: prioritized planning, CBS and ECBS on grid maps.";

    py::class_<mapf::Grid>(m, "Grid")
        .def_property_readonly("width", &mapf::Grid::width)
        .def_property_readonly("height", &mapf::Grid::height)
        .def_property_readonly("free_count", &mapf::Grid::free_count)
        .def(
            "is_free",
            [](const mapf::Grid& g, int x, int y) {
                return g.in_bounds(x, y) && g.is_free(g.cell(x, y));
            },
            py::arg("x"), py::arg("y"))
        .def("rows", [](const mapf::Grid& g) {
            std::vector<std::string> rows;
            for (int y = 0; y < g.height(); ++y) {
                std::string row;
                for (int x = 0; x < g.width(); ++x) row += g.is_free(g.cell(x, y)) ? '.' : '@';
                rows.push_back(std::move(row));
            }
            return rows;
        });

    py::class_<mapf::Instance>(m, "Instance")
        .def_readonly("grid", &mapf::Instance::grid)
        .def_property_readonly("num_agents",
                               [](const mapf::Instance& i) { return i.agents.size(); })
        .def_property_readonly("starts",
                               [](const mapf::Instance& i) {
                                   std::vector<XY> v;
                                   for (const auto& a : i.agents)
                                       v.emplace_back(i.grid.x_of(a.start), i.grid.y_of(a.start));
                                   return v;
                               })
        .def_property_readonly("goals", [](const mapf::Instance& i) {
            std::vector<XY> v;
            for (const auto& a : i.agents) v.emplace_back(i.grid.x_of(a.goal), i.grid.y_of(a.goal));
            return v;
        });

    m.def("load_map", &mapf::load_map, py::arg("path"), "Load a MovingAI .map file.");
    m.def("load_instance", &mapf::load_instance, py::arg("map_path"), py::arg("scen_path"),
          py::arg("num_agents"), "Load a .map and the first num_agents entries of a .scen.");
    m.def("make_instance", &make_instance, py::arg("rows"), py::arg("agents"),
          "Build an instance from map rows ('.' free, '@' obstacle) and "
          "[((sx, sy), (gx, gy)), ...] agents.");
    m.def("plan", &plan, py::arg("instance"), py::arg("planner") = "cbs", py::arg("w") = 1.5,
          py::arg("time_limit_s") = 60.0,
          "Plan with 'pp', 'cbs' or 'ecbs' (bound w). Returns a dict with status, paths "
          "(lists of (x, y), one per agent; empty unless solved), costs and search stats.");
    m.def("validate", &validate, py::arg("instance"), py::arg("paths"),
          "Check paths with the independent validator (vertex/edge conflicts, moves, "
          "obstacles, start/goal). Returns valid, errors, sum_of_costs, makespan.");
}
