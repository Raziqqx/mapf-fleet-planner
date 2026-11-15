// The profiling switches (open list, node layout, arena) must change speed
// only: every combination has to expand the same states and return the same
// plan as the original std::set / AoS / heap-allocated search.
#include <gtest/gtest.h>

#include "test_util.hpp"

using namespace mapf;

namespace {

std::vector<SearchTuning> all_tunings() {
    std::vector<SearchTuning> out;
    for (auto open : {OpenList::Set, OpenList::BinaryHeap, OpenList::Bucket})
        for (auto layout : {NodeLayout::AoS, NodeLayout::SoA})
            for (bool arena : {false, true}) out.push_back({open, layout, arena});
    return out;
}

// Restores the process-wide setting even if an assertion fails.
struct TuningGuard {
    SearchTuning saved = search_tuning();
    ~TuningGuard() { set_search_tuning(saved); }
};

}  // namespace

TEST(Tuning, AllVariantsExpandIdenticallyAndReturnSamePlans) {
    TuningGuard guard;
    std::mt19937 rng(777);
    Budget budget;
    budget.time_limit_s = 1e9;  // node limit only, so runs are deterministic
    budget.max_hl_nodes = 2000;
    for (int i = 0; i < 25; ++i) {
        const int side = 8 + i % 6;
        const Instance inst = test::random_instance(rng, side, side, 0.2, 3 + i % 8);
        SCOPED_TRACE("instance " + std::to_string(i));
        std::vector<Result> reference;  // pp, cbs, ecbs 1.2, ecbs 2.0 under the original setting
        for (const SearchTuning& tuning : all_tunings()) {
            set_search_tuning(tuning);
            std::vector<Result> results = {plan_prioritized(inst, budget), plan_cbs(inst, budget),
                                           plan_ecbs(inst, 1.2, budget),
                                           plan_ecbs(inst, 2.0, budget)};
            if (reference.empty()) {
                reference = std::move(results);
                continue;
            }
            for (std::size_t p = 0; p < results.size(); ++p) {
                SCOPED_TRACE(std::string(to_string(tuning.open_list)) + "/" +
                             to_string(tuning.layout) + "/arena=" + std::to_string(tuning.arena) +
                             " planner " + std::to_string(p));
                EXPECT_EQ(results[p].status, reference[p].status);
                EXPECT_EQ(results[p].paths, reference[p].paths);
                EXPECT_EQ(results[p].stats.ll_expanded, reference[p].stats.ll_expanded);
                EXPECT_EQ(results[p].stats.hl_expanded, reference[p].stats.hl_expanded);
            }
        }
    }
}
