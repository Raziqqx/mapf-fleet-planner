// CBS must match an exhaustive joint-state search on tiny random instances.
#include <gtest/gtest.h>

#include "adapter.hpp"
#include "brute_force.hpp"
#include "test_util.hpp"

using namespace mapf;

TEST(CbsOptimality, MatchesBruteForceOnTinyInstances) {
    std::mt19937 rng(2024);
    int solved = 0;
    int unsolvable = 0;
    int cbs_gave_up = 0;
    for (int i = 0; i < 300; ++i) {
        const int side = 3 + i % 2;          // 3x3 or 4x4
        const int agents = 2 + (i / 2) % 2;  // 2 or 3 agents
        const Instance inst = test::random_instance(rng, side, side, 0.25, agents);
        const long optimum = test::brute_force_soc(inst);
        Budget budget;
        budget.max_hl_nodes = 3000;  // unsolvable instances never terminate in CBS
        const Result r = plan_cbs(inst, budget);
        SCOPED_TRACE("instance " + std::to_string(i));
        if (optimum < 0) {
            ++unsolvable;
            EXPECT_NE(r.status, Status::Solved);
            continue;
        }
        ++solved;
        // CBS is exponential in the number of conflicts: a few tightly coupled
        // instances need millions of nodes (one here needs > 2M nodes, optimal SoC
        // 30 vs 13 for the independent paths). Giving up cleanly is allowed;
        // a wrong answer is not.
        if (r.status == Status::NodeLimit) {
            ++cbs_gave_up;
            continue;
        }
        ASSERT_EQ(r.status, Status::Solved);
        EXPECT_EQ(r.sum_of_costs, optimum);
        EXPECT_EQ(r.lower_bound, optimum);
        EXPECT_TRUE(validate_result(inst, r).valid);

        // ECBS stays within its bound of the optimum.
        const Result e = plan_ecbs(inst, 1.5, budget);
        ASSERT_EQ(e.status, Status::Solved);
        EXPECT_LE(e.sum_of_costs, 1.5 * optimum + 1e-9);
        EXPECT_LE(e.lower_bound, optimum);
    }
    // Make sure the generator exercises both outcomes.
    EXPECT_GT(solved, 200);
    EXPECT_GT(unsolvable, 0);
    EXPECT_LE(cbs_gave_up, 3);
    std::cout << "[ info ] brute force: " << solved << " solvable, " << unsolvable
              << " unsolvable instances, CBS hit the node limit on " << cbs_gave_up << "\n";
}
