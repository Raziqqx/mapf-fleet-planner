// Profiling driver: runs one fixed instance group with one SearchTuning and
// prints a single CSV row. scripts/profile.py calls it once per (group,
// tuning, rep) in a fresh process, so peak RSS is per run and nothing is warm.
//
//   mapf_profile --group pp|cbs|ecbs --open set|heap|bucket --layout aos|soa
//                --arena 0|1 [--data DIR]
#include <sys/resource.h>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "adapter.hpp"
#include "mapf/planner.hpp"

namespace {

struct Case {
    const char* map;
    int scen;
    int agents;
};

// Chosen from the run-A benchmark: each instance is solved in roughly
// 0.05-1 s, deterministic under a node limit, and together they cover all maps.
const std::vector<Case> kPP = {
    {"empty-32-32", 1, 100},
    {"empty-32-32", 2, 100},
    {"random-32-32-20", 1, 100},
    {"random-32-32-20", 2, 100},
    {"room-32-32-4", 1, 100},
    {"room-32-32-4", 2, 100},
    {"maze-32-32-2", 1, 100},
    {"maze-32-32-2", 2, 100},
    {"warehouse-10-20-10-2-1", 1, 100},
    {"warehouse-10-20-10-2-1", 2, 100},
};
const std::vector<Case> kCBS = {
    {"room-32-32-4", 6, 20}, {"random-32-32-20", 7, 20},        {"empty-32-32", 4, 60},
    {"maze-32-32-2", 4, 10}, {"warehouse-10-20-10-2-1", 1, 40},
};
const std::vector<Case> kECBS = {
    // w = 1.2
    {"maze-32-32-2", 3, 70},    {"room-32-32-4", 8, 80}, {"warehouse-10-20-10-2-1", 2, 100},
    {"random-32-32-20", 9, 90}, {"empty-32-32", 8, 100},
};

int fail(const std::string& msg) {
    std::cerr << "error: " << msg << "\n"
              << "usage: mapf_profile --group pp|cbs|ecbs --open set|heap|bucket "
                 "--layout aos|soa --arena 0|1 [--data DIR] [--reps-inner N]\n";
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    std::string group = "pp", data = "data";
    int inner = 1;  // repeat the group inside the process (for gprof sampling)
    mapf::SearchTuning tuning;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string arg = argv[i], val = argv[i + 1];
        if (arg == "--group")
            group = val;
        else if (arg == "--data")
            data = val;
        else if (arg == "--reps-inner")
            inner = std::stoi(val);
        else if (arg == "--arena")
            tuning.arena = val == "1";
        else if (arg == "--layout" && (val == "aos" || val == "soa"))
            tuning.layout = val == "aos" ? mapf::NodeLayout::AoS : mapf::NodeLayout::SoA;
        else if (arg == "--open" && val == "set")
            tuning.open_list = mapf::OpenList::Set;
        else if (arg == "--open" && val == "heap")
            tuning.open_list = mapf::OpenList::BinaryHeap;
        else if (arg == "--open" && val == "bucket")
            tuning.open_list = mapf::OpenList::Bucket;
        else
            return fail("bad option " + arg + " " + val);
    }
    const std::vector<Case>* cases = group == "pp"     ? &kPP
                                     : group == "cbs"  ? &kCBS
                                     : group == "ecbs" ? &kECBS
                                                       : nullptr;
    if (cases == nullptr) return fail("unknown group " + group);
    mapf::set_search_tuning(tuning);

    mapf::Budget budget;
    budget.time_limit_s = 1e9;      // node limit only: identical work in every run
    budget.max_hl_nodes = 200'000;  // never reached by these instances
    double wall = 0;
    std::uint64_t ll = 0, hl = 0;
    long soc = 0;
    int solved = 0, invalid = 0;
    for (int rep = 0; rep < inner; ++rep) {
        for (const Case& c : *cases) {
            const std::string base = data + "/";
            const auto inst = mapf::load_instance(
                base + "maps/" + c.map + ".map",
                base + "scen/" + c.map + "-random-" + std::to_string(c.scen) + ".scen", c.agents);
            const auto t0 = std::chrono::steady_clock::now();
            const mapf::Result r = group == "pp"    ? mapf::plan_prioritized(inst, budget)
                                   : group == "cbs" ? mapf::plan_cbs(inst, budget)
                                                    : mapf::plan_ecbs(inst, 1.2, budget);
            wall += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            ll += r.stats.ll_expanded;
            hl += r.stats.hl_expanded;
            if (r.status == mapf::Status::Solved) {
                ++solved;
                soc += r.sum_of_costs;
                if (!mapf::validate_result(inst, r).valid) ++invalid;
            }
        }
    }
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    // group,open,layout,arena,wall_s,ll_expanded,hl_expanded,solved,soc,peak_rss_kb
    std::cout << group << ',' << mapf::to_string(tuning.open_list) << ','
              << mapf::to_string(tuning.layout) << ',' << (tuning.arena ? 1 : 0) << ',' << wall
              << ',' << ll << ',' << hl << ',' << solved << ',' << soc << ',' << usage.ru_maxrss
              << '\n';
    return invalid == 0 ? 0 : 1;
}
