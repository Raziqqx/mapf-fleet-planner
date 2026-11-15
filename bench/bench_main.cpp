// Benchmark runner: map x agent count x planner x scenario -> one CSV row each.
//
// Every solved plan is checked by the independent validator before it is
// recorded. For each (map, planner) pair agent counts run in increasing order;
// once a planner solves none of the scenarios at some count, larger counts are
// recorded as "skipped" (counted as failures) instead of burning the time limit.
// (map, planner) groups run in parallel on --jobs threads.
#include <algorithm>
#include <atomic>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "adapter.hpp"
#include "mapf/planner.hpp"

namespace {

struct PlannerSpec {
    std::string name;  // "pp", "cbs" or "ecbs"
    double w = 1.0;
    std::string label() const {
        if (name != "ecbs") return name;
        std::ostringstream os;
        os << "ecbs_w" << w;
        return os.str();
    }
};

struct Config {
    std::string data_dir = "data";
    std::vector<std::string> maps = {"empty-32-32", "random-32-32-20", "room-32-32-4"};
    std::vector<int> agent_counts;
    std::vector<int> scens = {1, 2, 3, 4, 5};
    std::vector<PlannerSpec> planners = {{"pp", 1.0}, {"cbs", 1.0}, {"ecbs", 1.5}};
    double time_limit_s = 10.0;
    unsigned jobs = 6;
    std::string out = "results/bench.csv";
};

std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> parts;
    std::stringstream ss(s);
    for (std::string p; std::getline(ss, p, sep);)
        if (!p.empty()) parts.push_back(p);
    return parts;
}

// "a:b:step" or "a,b,c".
std::vector<int> parse_ints(const std::string& s) {
    std::vector<int> v;
    if (s.find(':') != std::string::npos) {
        const auto p = split(s, ':');
        if (p.size() != 3) throw std::invalid_argument("range must be from:to:step");
        for (int i = std::stoi(p[0]); i <= std::stoi(p[1]); i += std::stoi(p[2])) v.push_back(i);
    } else {
        for (const auto& x : split(s, ',')) v.push_back(std::stoi(x));
    }
    return v;
}

void usage() {
    std::cerr << "usage: mapf_bench [--data DIR] [--maps m1,m2] [--agents 10:100:10]\n"
                 "                  [--scens 1:5:1] [--planners pp,cbs,ecbs:1.5]\n"
                 "                  [--time-limit SECONDS] [--jobs N] [--out FILE]\n";
}

Config parse_args(int argc, char** argv) {
    Config c;
    c.agent_counts = parse_ints("10:100:10");
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (i + 1 >= argc) throw std::invalid_argument("missing value for " + arg);
        const std::string val = argv[++i];
        if (arg == "--data")
            c.data_dir = val;
        else if (arg == "--maps")
            c.maps = split(val, ',');
        else if (arg == "--agents")
            c.agent_counts = parse_ints(val);
        else if (arg == "--scens")
            c.scens = parse_ints(val);
        else if (arg == "--time-limit")
            c.time_limit_s = std::stod(val);
        else if (arg == "--jobs")
            c.jobs = static_cast<unsigned>(std::stoul(val));
        else if (arg == "--out")
            c.out = val;
        else if (arg == "--planners") {
            c.planners.clear();
            for (const auto& p : split(val, ',')) {
                const auto parts = split(p, ':');
                PlannerSpec spec{parts[0], parts.size() > 1 ? std::stod(parts[1]) : 1.0};
                if (spec.name != "pp" && spec.name != "cbs" && spec.name != "ecbs")
                    throw std::invalid_argument("unknown planner " + spec.name);
                if (spec.name == "ecbs" && parts.size() < 2) spec.w = 1.5;
                c.planners.push_back(spec);
            }
        } else {
            throw std::invalid_argument("unknown option " + arg);
        }
    }
    if (c.jobs == 0) c.jobs = 1;
    return c;
}

mapf::Result run_planner(const PlannerSpec& p, const mapf::Instance& inst,
                         const mapf::Budget& budget) {
    if (p.name == "pp") return mapf::plan_prioritized(inst, budget);
    if (p.name == "cbs") return mapf::plan_cbs(inst, budget);
    return mapf::plan_ecbs(inst, p.w, budget);
}

}  // namespace

int main(int argc, char** argv) {
    Config cfg;
    try {
        cfg = parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        usage();
        return 2;
    }

    std::ofstream csv(cfg.out);
    if (!csv) {
        std::cerr << "error: cannot write " << cfg.out << "\n";
        return 1;
    }
    csv << "map,scen,agents,planner,w,status,runtime_s,sum_of_costs,makespan,lower_bound,"
           "hl_expanded,ll_expanded,valid\n";

    struct Group {
        std::string map;
        PlannerSpec planner;
    };
    std::vector<Group> groups;
    for (const auto& m : cfg.maps)
        for (const auto& p : cfg.planners) groups.push_back({m, p});

    std::mutex out_mutex;
    std::atomic<std::size_t> next{0};
    std::atomic<int> invalid{0};
    std::atomic<int> errors{0};
    mapf::Budget budget;
    budget.time_limit_s = cfg.time_limit_s;
    budget.max_hl_nodes = 2'000'000;  // memory guard; at 10 s CBS stays well below it

    const auto worker = [&] {
        for (std::size_t g = next++; g < groups.size(); g = next++) {
            const Group& group = groups[g];
            const std::string map_path = cfg.data_dir + "/maps/" + group.map + ".map";
            bool give_up = false;
            for (int agents : cfg.agent_counts) {
                int solved_here = 0;
                for (int scen : cfg.scens) {
                    std::ostringstream row;
                    row << group.map << ',' << scen << ',' << agents << ',' << group.planner.label()
                        << ',' << group.planner.w << ',';
                    if (give_up) {
                        row << "skipped,,,,,,,\n";
                    } else {
                        const std::string scen_path = cfg.data_dir + "/scen/" + group.map +
                                                      "-random-" + std::to_string(scen) + ".scen";
                        try {
                            const auto inst = mapf::load_instance(map_path, scen_path, agents);
                            const auto r = run_planner(group.planner, inst, budget);
                            row << mapf::to_string(r.status) << ',' << r.stats.runtime_s << ',';
                            if (r.status == mapf::Status::Solved) {
                                ++solved_here;
                                const auto report = mapf::validate_result(inst, r);
                                if (!report.valid) {
                                    ++invalid;
                                    std::lock_guard lock(out_mutex);
                                    std::cerr << "INVALID PLAN " << group.map << " scen " << scen
                                              << " agents " << agents << ' '
                                              << group.planner.label() << ": "
                                              << report.errors.front() << "\n";
                                }
                                row << r.sum_of_costs << ',' << r.makespan << ',' << r.lower_bound
                                    << ',';
                                row << r.stats.hl_expanded << ',' << r.stats.ll_expanded << ','
                                    << (report.valid ? 1 : 0) << '\n';
                            } else {
                                row << ",,," << r.stats.hl_expanded << ',' << r.stats.ll_expanded
                                    << ",\n";
                            }
                        } catch (const std::exception& e) {
                            ++errors;
                            std::lock_guard lock(out_mutex);
                            std::cerr << "error: " << e.what() << "\n";
                            continue;
                        }
                    }
                    std::lock_guard lock(out_mutex);
                    csv << row.str() << std::flush;
                }
                if (!give_up) {
                    std::lock_guard lock(out_mutex);
                    std::cout << group.map << ' ' << group.planner.label() << ' ' << agents
                              << " agents: " << solved_here << '/' << cfg.scens.size() << " solved"
                              << std::endl;
                }
                give_up = give_up || solved_here == 0;
            }
        }
    };
    std::vector<std::thread> threads;
    for (unsigned i = 0; i < std::min<std::size_t>(cfg.jobs, groups.size()); ++i)
        threads.emplace_back(worker);
    for (auto& t : threads) t.join();

    if (invalid > 0 || errors > 0) {
        std::cerr << invalid << " invalid plans, " << errors << " errors\n";
        return 1;
    }
    return 0;
}
