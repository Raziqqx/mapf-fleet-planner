#include <gtest/gtest.h>

#include "mapf_validator/validator.hpp"

using mapf_validator::Pos;
using mapf_validator::Problem;
using mapf_validator::validate;

namespace {

// 3x3 open grid with a centre obstacle.
Problem ring(std::vector<Pos> starts, std::vector<Pos> goals) {
    Problem p;
    p.width = 3;
    p.height = 3;
    p.obstacle = {{false, false, false}, {false, true, false}, {false, false, false}};
    p.starts = std::move(starts);
    p.goals = std::move(goals);
    return p;
}

}  // namespace

TEST(Validator, AcceptsValidPlanAndComputesCosts) {
    const Problem p = ring({{0, 0}, {2, 2}}, {{2, 0}, {2, 2}});
    // Agent 1 starts on its goal (cost 0); trailing waits do not add cost.
    const auto r = validate(p, {{{0, 0}, {1, 0}, {2, 0}, {2, 0}}, {{2, 2}}});
    EXPECT_TRUE(r.valid) << (r.errors.empty() ? "" : r.errors.front());
    EXPECT_EQ(r.sum_of_costs, 2);
    EXPECT_EQ(r.makespan, 2);
}

TEST(Validator, CostCountsLeavingTheGoalAndComingBack) {
    const Problem p = ring({{0, 0}}, {{1, 0}});
    const auto r = validate(p, {{{0, 0}, {1, 0}, {2, 0}, {1, 0}}});
    EXPECT_TRUE(r.valid);
    EXPECT_EQ(r.sum_of_costs, 3);
}

TEST(Validator, DetectsVertexConflict) {
    const Problem p = ring({{0, 0}, {2, 0}}, {{2, 0}, {0, 0}});
    const auto r = validate(p, {{{0, 0}, {1, 0}, {2, 0}}, {{2, 0}, {1, 0}, {0, 0}}});
    EXPECT_FALSE(r.valid);
}

TEST(Validator, DetectsEdgeSwapConflict) {
    const Problem p = ring({{0, 0}, {1, 0}}, {{1, 0}, {0, 0}});
    const auto r = validate(p, {{{0, 0}, {1, 0}}, {{1, 0}, {0, 0}}});
    ASSERT_FALSE(r.valid);
    EXPECT_NE(r.errors.front().find("edge"), std::string::npos);
}

TEST(Validator, DetectsCollisionWithAgentParkedAtGoal) {
    // Agent 0 finishes at (1,0) at t=1; agent 1 passes through it at t=2.
    const Problem p = ring({{0, 0}, {0, 1}}, {{1, 0}, {2, 0}});
    const auto r = validate(p, {{{0, 0}, {1, 0}}, {{0, 1}, {0, 0}, {1, 0}, {2, 0}}});
    EXPECT_FALSE(r.valid);
}

TEST(Validator, AllowsFollowingIntoAJustVacatedCell) {
    const Problem p = ring({{0, 0}, {1, 0}}, {{1, 0}, {2, 0}});
    const auto r = validate(p, {{{0, 0}, {1, 0}}, {{1, 0}, {2, 0}}});
    EXPECT_TRUE(r.valid);
}

TEST(Validator, RejectsJumpsObstaclesAndWrongEndpoints) {
    const Problem p = ring({{0, 0}}, {{2, 0}});
    EXPECT_FALSE(validate(p, {{{0, 0}, {2, 0}}}).valid);                  // jump
    EXPECT_FALSE(validate(p, {{{0, 0}, {1, 1}, {2, 1}, {2, 0}}}).valid);  // diagonal + obstacle
    EXPECT_FALSE(validate(p, {{{0, 1}, {0, 0}, {1, 0}, {2, 0}}}).valid);  // wrong start
    EXPECT_FALSE(validate(p, {{{0, 0}, {1, 0}}}).valid);                  // wrong goal
    EXPECT_FALSE(validate(p, {{{0, 0}, {-1, 0}, {0, 0}, {1, 0}, {2, 0}}}).valid);  // off map
    EXPECT_FALSE(validate(p, {{}}).valid);                                         // empty
    EXPECT_FALSE(validate(p, {}).valid);                                           // missing plan
}
