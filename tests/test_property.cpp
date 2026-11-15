// Randomized properties over many random grids:
//  - every returned plan passes the independent validator, and the planner's
//    reported costs equal the validator's;
//  - CBS is optimal: no other planner beats it, and ECBS stays within w of it;
//  - the planners agree on solvability where CBS finishes: if CBS solves an
//    instance, ECBS does too (both are complete), and no planner "solves" an
//    instance that CBS proves infeasible.
#include <gtest/gtest.h>

#include "adapter.hpp"
#include "test_util.hpp"

using namespace mapf;

namespace {

void expect_valid(const Instance& inst, const Result& r, const char* who) {
    if (r.status != Status::Solved) return;
    const auto report = validate_result(inst, r);
    EXPECT_TRUE(report.valid) << who << ": "
                              << (report.errors.empty() ? "" : report.errors.front());
    EXPECT_EQ(report.sum_of_costs, r.sum_of_costs) << who;
    EXPECT_EQ(report.makespan, r.makespan) << who;
    EXPECT_LE(r.lower_bound, r.sum_of_costs) << who;
}

}  // namespace

TEST(Property, RandomGridsAllPlannersConsistent) {
    std::mt19937 rng(12345);
    Budget budget;
    budget.time_limit_s = 1.0;
    budget.max_hl_nodes = 5000;
    int cbs_solved = 0;
    int pp_failed = 0;
    for (int i = 0; i < 120; ++i) {
        const int side = 6 + i % 7;                   // 6..12
        const double density = 0.1 + 0.05 * (i % 5);  // 0.10..0.30
        const int agents = 2 + i % 9;                 // 2..10
        const Instance inst = test::random_instance(rng, side, side, density, agents);
        SCOPED_TRACE("instance " + std::to_string(i));

        const Result pp = plan_prioritized(inst, budget);
        const Result cbs = plan_cbs(inst, budget);
        expect_valid(inst, pp, "pp");
        expect_valid(inst, cbs, "cbs");
        if (pp.status == Status::Failed) ++pp_failed;

        for (double w : {1.1, 1.5, 2.0}) {
            const Result ecbs = plan_ecbs(inst, w, budget);
            expect_valid(inst, ecbs, "ecbs");
            if (cbs.status == Status::Solved) {
                ASSERT_EQ(ecbs.status, Status::Solved) << "w=" << w;
                EXPECT_GE(ecbs.sum_of_costs, cbs.sum_of_costs);
                EXPECT_LE(ecbs.sum_of_costs, w * cbs.sum_of_costs + 1e-9) << "w=" << w;
                EXPECT_LE(ecbs.lower_bound, cbs.sum_of_costs);
            }
            if (cbs.status == Status::Infeasible) {
                EXPECT_NE(ecbs.status, Status::Solved);
            }
        }
        if (cbs.status == Status::Solved) {
            ++cbs_solved;
            if (pp.status == Status::Solved) {
                EXPECT_GE(pp.sum_of_costs, cbs.sum_of_costs);
            }
        }
        if (cbs.status == Status::Infeasible) {
            EXPECT_NE(pp.status, Status::Solved);
        }
    }
    EXPECT_GT(cbs_solved, 100);
    std::cout << "[ info ] property: CBS solved " << cbs_solved << "/120, PP failed " << pp_failed
              << "\n";
}

// Dense instances where prioritized planning often fails: whenever it does
// return a plan, the plan must still be valid.
TEST(Property, PrioritizedPlansAreValidUnderCongestion) {
    std::mt19937 rng(99);
    Budget budget;
    budget.time_limit_s = 2.0;
    int solved = 0;
    for (int i = 0; i < 60; ++i) {
        const Instance inst = test::random_instance(rng, 8, 8, 0.2, 12 + i % 8);
        const Result pp = plan_prioritized(inst, budget);
        expect_valid(inst, pp, "pp");
        solved += pp.status == Status::Solved ? 1 : 0;
    }
    EXPECT_GT(solved, 0);
}
