// Single-agent space-time search and conflict detection (internal headers).
#include <gtest/gtest.h>

#include "conflicts.hpp"
#include "space_time.hpp"
#include "test_util.hpp"

using namespace mapf;
using namespace mapf::detail;

namespace {

struct Corridor {
    Grid grid = test::grid_from_rows({"....."});
    Cell start = 0;
    Cell goal = 4;
    std::vector<int> h = bfs_distances(grid, goal);
    Budget budget;
    Deadline deadline{budget};
    Stats stats;

    LowLevelResult search(const ConstraintTable& ct, const ConflictAvoidanceTable* cat = nullptr,
                          double w = 1.0) {
        return focal_search(grid, h, start, goal, ct, cat, w, deadline, stats);
    }
};

}  // namespace

TEST(LowLevel, UnconstrainedPathIsShortest) {
    Corridor c;
    const auto r = c.search(ConstraintTable{});
    ASSERT_TRUE(r.found);
    EXPECT_EQ(r.path, (Path{0, 1, 2, 3, 4}));
    EXPECT_EQ(r.lower_bound, 4);
}

TEST(LowLevel, VertexConstraintForcesOneWait) {
    Corridor c;
    ConstraintTable ct;
    ct.add_vertex(2, 2);
    const auto r = c.search(ct);
    ASSERT_TRUE(r.found);
    EXPECT_EQ(path_cost(r.path), 5);
    EXPECT_NE(r.path[2], 2);
}

TEST(LowLevel, EdgeConstraintForcesOneWait) {
    Corridor c;
    ConstraintTable ct;
    ct.add_edge(1, 2, 2);
    const auto r = c.search(ct);
    ASSERT_TRUE(r.found);
    EXPECT_EQ(path_cost(r.path), 5);
}

TEST(LowLevel, LateGoalConstraintDelaysArrival) {
    Corridor c;
    ConstraintTable ct;
    ct.add_vertex(4, 7);  // someone passes the goal at t=7: arrive at t >= 8
    const auto r = c.search(ct);
    ASSERT_TRUE(r.found);
    EXPECT_EQ(path_cost(r.path), 8);
    EXPECT_EQ(r.lower_bound, 8);
}

TEST(LowLevel, PermanentBlockOnTheWayMakesGoalUnreachable) {
    Corridor c;
    ConstraintTable ct;
    ct.add_permanent(2, 0);
    const auto r = c.search(ct);
    EXPECT_FALSE(r.found);
    EXPECT_FALSE(r.out_of_time);
}

TEST(LowLevel, ConflictAvoidanceBreaksTiesWithoutLosingOptimality) {
    // 2x3 open grid; two equally short routes from (0,0) to (2,1).
    const Grid grid = test::grid_from_rows({"...", "..."});
    const auto h = bfs_distances(grid, grid.cell(2, 1));
    Budget budget;
    Deadline deadline(budget);
    Stats stats;
    ConflictAvoidanceTable cat;
    cat.add_path(Path{grid.cell(1, 1), grid.cell(1, 0), grid.cell(2, 0)});  // goes along row 0
    const auto r = focal_search(grid, h, grid.cell(0, 0), grid.cell(2, 1), ConstraintTable{}, &cat,
                                1.0, deadline, stats);
    ASSERT_TRUE(r.found);
    EXPECT_EQ(path_cost(r.path), 3);
    // Row 0 at t=1 is (1,0) at t=1 -> conflict-free only by going down first.
    EXPECT_EQ(r.path[1], grid.cell(0, 1));
}

TEST(LowLevel, FocalPathIsWithinW) {
    std::mt19937 rng(7);
    for (int i = 0; i < 30; ++i) {
        const Instance inst = test::random_instance(rng, 10, 10, 0.2, 1);
        const auto& a = inst.agents[0];
        const auto h = bfs_distances(inst.grid, a.goal);
        Budget budget;
        Deadline deadline(budget);
        Stats stats;
        const auto r = focal_search(inst.grid, h, a.start, a.goal, ConstraintTable{}, nullptr, 1.5,
                                    deadline, stats);
        ASSERT_TRUE(r.found);
        const int opt = h[static_cast<std::size_t>(a.start)];
        EXPECT_LE(r.lower_bound, opt);
        EXPECT_LE(path_cost(r.path), 1.5 * opt + 1e-9);
    }
}

TEST(Conflicts, FindsVertexEdgeAndParkedConflicts) {
    const Path a{0, 1, 2};
    const Path b{2, 1, 0};  // meets a at cell 1, t=1
    EXPECT_EQ(find_conflicts({&a, &b}).earliest.t, 1);

    const Path c{0, 1};
    const Path d{1, 0};  // swap
    const auto swap = find_conflicts({&c, &d});
    ASSERT_EQ(swap.count, 1);
    EXPECT_TRUE(swap.earliest.edge);

    const Path parked{1};
    const Path passer{0, 1, 2};  // walks through the parked agent at t=1
    const auto pk = find_conflicts({&parked, &passer});
    ASSERT_EQ(pk.count, 1);
    EXPECT_EQ(pk.earliest.a1, 0);
    EXPECT_EQ(pk.earliest.c1, 1);

    const Path follow1{0, 1};
    const Path follow2{1, 2};  // moves away as the other arrives: allowed
    EXPECT_EQ(find_conflicts({&follow1, &follow2}).count, 0);
}
