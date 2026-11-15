#include <gtest/gtest.h>

#include <sstream>

#include "mapf/instance.hpp"

using namespace mapf;

namespace {

const char* kMap =
    "type octile\n"
    "height 3\n"
    "width 4\n"
    "map\n"
    "..@.\n"
    ".T..\n"
    "G..S\n";

Grid small_grid() {
    std::istringstream in(kMap);
    return parse_map(in);
}

}  // namespace

TEST(MapParser, ReadsDimensionsAndObstacles) {
    const Grid g = small_grid();
    EXPECT_EQ(g.width(), 4);
    EXPECT_EQ(g.height(), 3);
    EXPECT_EQ(g.free_count(), 10);
    EXPECT_FALSE(g.is_free(g.cell(2, 0)));  // '@'
    EXPECT_FALSE(g.is_free(g.cell(1, 1)));  // 'T' (trees are obstacles)
    EXPECT_TRUE(g.is_free(g.cell(0, 2)));   // 'G' is passable
    EXPECT_TRUE(g.is_free(g.cell(3, 2)));   // 'S' is passable
}

TEST(MapParser, NeighboursSkipObstaclesAndEdges) {
    const Grid g = small_grid();
    std::array<Cell, 4> nb{};
    EXPECT_EQ(g.neighbours(g.cell(0, 0), nb), 2);  // right and down
    EXPECT_EQ(g.neighbours(g.cell(1, 0), nb), 1);  // (2,0) and (1,1) blocked
}

TEST(MapParser, BfsDistancesRouteAroundObstacles) {
    const Grid g = small_grid();
    const auto d = bfs_distances(g, g.cell(0, 0));
    EXPECT_EQ(d[static_cast<std::size_t>(g.cell(3, 0))], 7);  // around the '@' and the 'T'
    EXPECT_EQ(d[static_cast<std::size_t>(g.cell(2, 0))], -1);
}

TEST(MapParser, RejectsMalformedInput) {
    for (const char* bad :
         {"type octile\nheight 2\nwidth 2\nmap\n..\n", "height 2\nwidth 2\n..\n",
          "type octile\nheight 1\nwidth 3\nmap\n..\n", "type octile\nfoo 1\nmap\n"}) {
        std::istringstream in(bad);
        EXPECT_THROW(parse_map(in), std::runtime_error) << bad;
    }
}

TEST(ScenarioParser, ReadsFirstNAgents) {
    const Grid g = small_grid();
    std::istringstream in(
        "version 1\n"
        "0\tm.map\t4\t3\t0\t0\t3\t2\t5.0\n"
        "0\tm.map\t4\t3\t3\t0\t0\t2\t5.0\n"
        "0\tm.map\t4\t3\t0\t1\t2\t2\t5.0\n");
    const auto agents = parse_scenario(in, g, 2);
    ASSERT_EQ(agents.size(), 2u);
    EXPECT_EQ(agents[0].start, g.cell(0, 0));
    EXPECT_EQ(agents[0].goal, g.cell(3, 2));
    EXPECT_EQ(agents[1].start, g.cell(3, 0));
}

TEST(ScenarioParser, RejectsBadEntries) {
    const Grid g = small_grid();
    const char* cases[] = {
        "0\tm.map\t4\t3\t0\t0\t3\t2\t5\n",             // no version line
        "version 1\n0\tm.map\t5\t3\t0\t0\t3\t2\t5\n",  // size mismatch
        "version 1\n0\tm.map\t4\t3\t2\t0\t3\t2\t5\n",  // start on '@'
        "version 1\n0\tm.map\t4\t3\t0\t9\t3\t2\t5\n",  // out of bounds
        "version 1\n0\tm.map\t4\t3\tx\t0\t3\t2\t5\n",  // not a number
        "version 1\n0\tm.map\t4\t3\t0\t0\t3\t2\t5\n",  // only 1 of 2 agents
    };
    for (const char* bad : cases) {
        std::istringstream in(bad);
        EXPECT_THROW(parse_scenario(in, g, 2), std::runtime_error) << bad;
    }
}
