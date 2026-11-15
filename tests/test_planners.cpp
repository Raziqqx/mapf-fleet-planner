// Known-answer and failure-mode tests for the three planners.
#include <gtest/gtest.h>

#include "adapter.hpp"
#include "brute_force.hpp"
#include "test_util.hpp"

using namespace mapf;

namespace {

Result run(const std::string& planner, const Instance& inst, Budget budget = {}) {
    if (planner == "pp") return plan_prioritized(inst, budget);
    if (planner == "cbs") return plan_cbs(inst, budget);
    return plan_ecbs(inst, 1.5, budget);
}

void expect_valid(const Instance& inst, const Result& r) {
    ASSERT_EQ(r.status, Status::Solved);
    const auto report = validate_result(inst, r);
    EXPECT_TRUE(report.valid) << (report.errors.empty() ? "" : report.errors.front());
    EXPECT_EQ(report.sum_of_costs, r.sum_of_costs);
    EXPECT_EQ(report.makespan, r.makespan);
}

class AllPlanners : public ::testing::TestWithParam<std::string> {};

}  // namespace

TEST_P(AllPlanners, AgentsAlreadyAtGoalsCostNothing) {
    const Grid g = test::grid_from_rows({"...", "..."});
    const Instance inst{g, {{0, 0}, {4, 4}}};
    const Result r = run(GetParam(), inst);
    expect_valid(inst, r);
    EXPECT_EQ(r.sum_of_costs, 0);
}

TEST_P(AllPlanners, IndependentAgentsGetShortestPaths) {
    const Grid g = test::grid_from_rows({".....", ".....", "....."});
    const Instance inst{g, {{g.cell(0, 0), g.cell(4, 0)}, {g.cell(0, 2), g.cell(4, 2)}}};
    const Result r = run(GetParam(), inst);
    expect_valid(inst, r);
    EXPECT_EQ(r.sum_of_costs, 8);
    EXPECT_EQ(r.makespan, 4);
}

TEST_P(AllPlanners, UnreachableGoalIsInfeasible) {
    const Grid g = test::grid_from_rows({"..@.."});
    const Instance inst{g, {{0, 4}}};
    EXPECT_EQ(run(GetParam(), inst).status, Status::Infeasible);
}

TEST_P(AllPlanners, DuplicateGoalsAreInfeasible) {
    const Grid g = test::grid_from_rows({"....."});
    const Instance inst{g, {{0, 2}, {4, 2}}};
    EXPECT_EQ(run(GetParam(), inst).status, Status::Infeasible);
}

TEST_P(AllPlanners, ZeroTimeBudgetFailsCleanly) {
    std::mt19937 rng(3);
    const Instance inst = test::random_instance(rng, 20, 20, 0.2, 30);
    Budget budget;
    budget.time_limit_s = 0.0;
    const Result r = run(GetParam(), inst, budget);
    EXPECT_EQ(r.status, Status::TimeLimit);
    EXPECT_TRUE(r.paths.empty());
}

INSTANTIATE_TEST_SUITE_P(Planners, AllPlanners, ::testing::Values("pp", "cbs", "ecbs"));

// Two agents swap ends of a corridor that has one side pocket: one of them
// must step into the pocket. Optimal SoC is 11 (5 + 6), makespan 6.
TEST(Cbs, CorridorWithPocketKnownOptimum) {
    const Grid g = test::grid_from_rows({".....", "@@.@@"});
    const Instance inst{g, {{g.cell(0, 0), g.cell(4, 0)}, {g.cell(4, 0), g.cell(0, 0)}}};
    const Result r = plan_cbs(inst, {});
    expect_valid(inst, r);
    EXPECT_EQ(r.sum_of_costs, 11);
    EXPECT_EQ(r.makespan, 6);
    EXPECT_EQ(r.lower_bound, 11);
    EXPECT_EQ(test::brute_force_soc(inst), 11);
}

// Prioritized planning is incomplete: here the first agent parks on the only
// way through, so the second cannot pass, although CBS finds a solution.
TEST(Prioritized, FailsWhereCbsSucceeds) {
    const Grid g = test::grid_from_rows({"....", "@.@@"});
    // Agent 0: (0,0) -> (1,0), right in the corridor agent 1 must use.
    // Agent 1: (1,1) -> (3,0); it has to pass through (1,0).
    const Instance inst{g, {{g.cell(0, 0), g.cell(1, 0)}, {g.cell(1, 1), g.cell(3, 0)}}};
    EXPECT_EQ(plan_prioritized(inst, {}).status, Status::Failed);
    const Result r = plan_cbs(inst, {});
    expect_valid(inst, r);
    EXPECT_EQ(r.sum_of_costs, test::brute_force_soc(inst));
}

// Two agents swapping ends of a plain corridor: no solution exists, but CBS
// cannot prove it; the node budget must stop it with a clean failure.
TEST(Cbs, UnsolvableCorridorHitsNodeLimit) {
    const Grid g = test::grid_from_rows({"...."});
    const Instance inst{g, {{0, 3}, {3, 0}}};
    Budget budget;
    budget.max_hl_nodes = 500;
    const Result cbs = plan_cbs(inst, budget);
    EXPECT_EQ(cbs.status, Status::NodeLimit);
    EXPECT_TRUE(cbs.paths.empty());
    EXPECT_EQ(cbs.stats.hl_expanded, 500u);
    EXPECT_EQ(plan_ecbs(inst, 2.0, budget).status, Status::NodeLimit);
    EXPECT_EQ(plan_prioritized(inst, {}).status, Status::Failed);
    EXPECT_EQ(test::brute_force_soc(inst), -1);
}

TEST(Ecbs, RejectsWBelowOne) {
    const Grid g = test::grid_from_rows({".."});
    EXPECT_THROW(plan_ecbs(Instance{g, {{0, 1}}}, 0.9, {}), std::invalid_argument);
}
