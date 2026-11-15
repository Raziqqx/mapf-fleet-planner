# Decisions

Decisions taken while building, with the reason for each. Newest at the bottom of each section.

## Problem definition
- **4-connected grid, unit-cost moves and waits, "stay at goal" semantics.** An agent that reaches
  its goal for the last time stays there forever and blocks the cell; its cost is that arrival time.
  This is the standard MovingAI/CBS-literature definition, so sum-of-costs numbers are comparable
  with papers. The `.scen` optimal-length column is octile (8-connected) and is ignored.
- **Conflicts: vertex and edge (swap) only.** Following (moving into a cell another agent leaves on
  the same step) is allowed, as in the standard definition.
- **Distinct starts and distinct goals are required.** Duplicates make the instance infeasible
  under these semantics; every planner reports `Status::Infeasible` for them instead of searching.
- **Status values distinguish why a planner stopped**: `Solved`, `Infeasible` (proved: unreachable
  goal, duplicate start/goal, or an exhausted CBS tree), `Failed` (prioritized planning gave up; the
  instance may still be solvable), `TimeLimit`, `NodeLimit`. Unsolved results never carry paths.

## Algorithms
- **One engine for CBS and ECBS** (`src/cbs.cpp`). ECBS keeps OPEN by lower bound and FOCAL (cost
  <= w * LB_min) ordered by number of conflicts. With w = 1 the low level is optimal, so FOCAL holds
  exactly the cheapest nodes and the engine is plain CBS with "fewest conflicts" tie-breaking.
  Alternative considered: two separate implementations. Rejected: twice the code to test, and the
  w = 1 case doubles as a check of the ECBS machinery (the brute-force test exercises it).
- **One low-level search** (`src/low_level.cpp`): space-time focal search. With w = 1 it is A* that
  breaks f-ties by fewest soft conflicts with a conflict-avoidance table (CAT), then deeper first.
  Prioritized planning uses it with w = 1 and no CAT.
- **Heuristic**: exact BFS distance to the goal, maxed with `min_goal_time - t` (the agent cannot
  finish before the last vertex constraint on its goal). Both terms are consistent, so is the max.
- **Low-level horizon**: `max(last constraint time, min goal time) + free cells + 1`. After the last
  constraint the space-time graph repeats, so a reachable goal is reached within `free cells` more
  steps; the bound makes "no path under these constraints" terminate instead of searching forever.
- **Branching on the earliest conflict.** No cardinal-conflict prioritization, bypassing or
  symmetry reasoning (ICBS/CBSH). They are the standard next improvements and would change CBS's
  scaling a lot; out of scope for the baseline-vs-bounded-suboptimal comparison asked for here.
- **Constraint-tree nodes store only the replanned agent's path and constraint**; full state is
  rebuilt by walking to the root (depth is small). Nodes live in a `std::deque` so pointers to their
  paths stay valid. Storing all paths per node would cost ~100x more memory at 100 agents.
- **Conflict detection is hash-based** (linear in the total path length) and recomputed fully per
  node. Counting is "number of colliding events", used as the ECBS focal key.
- **Prioritized planning uses scenario order** as the priority order (no restarts, no
  distance-based ordering). It is the simplest standard baseline; ordering heuristics are a
  possible improvement.
- **Lifelong mode skipped** (optional in the brief, not trivial to do properly).

## Validator
- **Separate library with no planner code or types** (`validator/`, standard library only). It
  uses (x, y) positions, its own adjacency and obstacle checks, a plain pairwise O(agents² · T)
  conflict check and its own cost computation. `bench/adapter.cpp` is the only bridge. The tests
  assert that the planner's reported sum of costs and makespan equal the validator's.

## Testing
- **GoogleTest 1.15.2 via FetchContent**; tests run under ASan + UBSan (`asan` preset).
- **Brute-force reference** (`tests/brute_force.hpp`): Dijkstra over joint states (positions +
  "finished" bit per agent, each step costs one per unfinished agent). This models the stay-at-goal
  cost exactly with a Markov state, so it is a true optimum on 3x3/4x4 maps with 2-3 agents.
- **CBS may give up on a few tiny instances**: one of 257 solvable brute-force instances needs more
  than 2 M CBS nodes (optimal SoC 30 vs 13 for independent paths, a tightly coupled 4x4 map). The
  test allows a clean `NodeLimit` on at most 3 instances but never a wrong cost. This is CBS's known
  exponential blow-up, not a bug (confirmed by running that instance to 1.9 M nodes).
- Test budgets were cut (node limit 3000 in the brute-force test, 1 s / 5000 nodes in the property
  test) so the whole suite runs in ~25 s under the sanitizers.

## Data
- **MovingAI maps are downloaded, not committed** (`scripts/download_movingai.sh`, ~4 MB, ODC-By
  licence, see `data/README.md`). The script extracts with `python3 -m zipfile` because `unzip` is
  not installed everywhere.

## Benchmark (`scripts/run_benchmark.sh`, results in `results/`)
- **Grid: 5 maps x 10..100 agents (step 10) x 10 "random" scenarios x 4 planners** (pp, cbs,
  ecbs w=1.2, ecbs w=1.5), 10 s per instance: 2,000 rows, ~7 min wall time. 10 s instead of the
  literature's usual 60 s keeps the run near the ~15 min budget; success rates at 60 s would be
  somewhat higher for CBS and ECBS w=1.2.
- **Early stop per (map, planner)**: after a count where a planner solves 0/10, larger counts are
  written as `skipped` and counted as failures (MAPF success rates are monotone in practice, and
  this saves most of the time-limit burn). Visible in the CSV, stated in the summary.
- **(map, planner) groups run in parallel** (10 threads on a 20-thread i5-14500HX). Runtimes are
  therefore measured under load and on a mix of P- and E-cores: compare planners by success rate
  and by order of magnitude of runtime, not by small runtime differences. Run B's profiling uses
  single-threaded runs.
- **Every solved plan is re-checked by the validator in the runner**; the runner exits non-zero if
  any plan is invalid. In the recorded run: 0 invalid of 1,416 solved.
- **Summary** (`scripts/summarize.py`, standard library only): success rate and median runtime of
  solved instances per agent count, the largest count with >= 50 % success, and SoC / CBS SoC on
  instances both planners solved (so ratios are only available where CBS finished).
