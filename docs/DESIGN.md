# Design

Full decision log with reasons: `DECISIONS.md`. Numbers below come from `results/`.

## Problem definition

- 4-connected grid, unit-cost moves and waits, "stay at goal" semantics: an agent that reaches its
  goal for the last time stays there and blocks the cell, and its cost is that arrival time. This
  is the standard MovingAI/CBS definition, so sums of costs are comparable with papers.
- Vertex and edge (swap) conflicts only; following another agent is allowed.
- Distinct starts and goals are required; duplicates return `Infeasible` without searching.
- Statuses say why a planner stopped (`Solved`, `Infeasible`, `Failed`, `TimeLimit`,
  `NodeLimit`); unsolved results never carry paths.

## Key decisions and trade-offs

| decision | why | alternative considered |
|---|---|---|
| One engine for CBS and ECBS (`src/cbs.cpp`) | w = 1 makes ECBS exactly CBS with "fewest conflicts" tie-breaking, so the brute-force test also exercises the ECBS machinery | two implementations: twice the code to test |
| One low-level focal search for every planner | one place to optimise and test | separate A* for PP and CBS |
| Heuristic: max(BFS distance, last goal constraint time - t) | both terms are consistent, so the max is too | plain BFS distance (weaker near goal constraints) |
| Horizon: last constraint time + free cells + 1 | the space-time graph repeats after the last constraint, so "no path" terminates | unbounded search, which can loop forever |
| CT nodes store only the replanned agent's path | storing all paths would cost about 100x more memory at 100 agents | full path copy per node |
| Hash-based conflict detection, recomputed per node | linear in total path length, simple | incremental conflict tracking |
| PP uses scenario order | simplest standard baseline | distance-based ordering, random restarts |
| Validator shares no code or types with planners | a shared bug cannot hide in both | reuse planner conflict code |
| Tuning as runtime switches over a templated search | before/after from one binary, identical work | compile-time flags, separate binaries |
| Animated SVG for replay | 36 KiB, standard library only, sharp at any zoom | GIF (several MB), ImGui viewer |
| Maps downloaded, not committed | licence attribution and repo size | vendoring the data |

## Baseline vs improved

**Planners (run A, `results/bench_summary.md`).** Largest agent count with at least 50 % success,
10 s per instance:

| map | PP (baseline) | CBS (optimal) | ECBS w=1.2 | ECBS w=1.5 |
|---|---|---|---|---|
| empty-32-32 | 100 | 40 | 100 | 100 |
| maze-32-32-2 | 20 | 10 | 60 | 100 |
| random-32-32-20 | 100 | 30 | 100 | 100 |
| room-32-32-4 | 50 | 20 | 60 | 100 |
| warehouse-10-20-10-2-1 | 100 | 40 | 100 | 100 |

ECBS w=1.5 is the only planner that reaches 100 agents on all five maps (70 % success on maze at
100). Its cost overhead against CBS is small where both finish: at most 1.0213 (room, 20 agents),
while PP reaches 1.1287 (empty, 60 agents). PP is the fastest but fails on narrow maps (maze 20,
room 50). All 1,416 solved plans passed the validator.

**Low-level search (run B, `results/profile.md`).** Pinned single core, median of 5, same states
expanded in every variant:

| variant | PP wall s | CBS wall s | ECBS wall s | speed-up PP / CBS / ECBS |
|---|---:|---:|---:|---:|
| std::set, AoS, heap allocation (baseline) | 1.329 | 1.319 | 0.901 | 1.00x / 1.00x / 1.00x |
| binary heap | 1.219 | 1.130 | 0.748 | 1.09x / 1.17x / 1.20x |
| bucket queue | 0.988 | 1.089 | 0.764 | 1.35x / 1.21x / 1.18x |
| bucket + arena (default) | 0.842 | 1.000 | 0.725 | 1.58x / 1.32x / 1.24x |
| bucket + SoA + arena | 0.866 | 1.013 | 0.771 | 1.53x / 1.30x / 1.17x |

- The bucket queue works because the heuristic is consistent, so f never decreases and a monotone
  cursor over integer-f buckets replaces the ordered set. FOCAL stays a binary heap (4-tuple key).
- The arena (a per-thread `std::pmr::monotonic_buffer_resource`, 1 MiB doubling up to 256 MiB)
  removes malloc/free from the steady state.
- CBS and ECBS gain less than PP because much of their time is outside the low-level search
  (conflict detection and constraint/CAT tables rebuilt per node): about 0.5 M nodes/s against
  1.89 M for PP.
- Measurement used steady-clock timers, node counters and `ru_maxrss`: perf was not installed
  and gprof attributed samples to functions that never ran under WSL2.

## Known limitations

- CBS has none of the standard enhancements (bypassing, prioritized/cardinal conflicts, CBSH
  heuristics, symmetry reasoning). It branches on the earliest conflict, which is why it stops
  at 10-40 agents.
- Run-A runtimes were measured with 10 parallel (map, planner) groups on mixed P- and E-cores and
  are noisy; success rates close to the 10 s limit may be slightly understated. The run was not
  repeated serially (33 min of summed runtime). Clean timings are in `results/profile.md`.
- 10 s per instance instead of the usual 60 s; CBS and ECBS w=1.2 would solve somewhat more.
- After a planner solves 0/10 at some agent count, larger counts are marked `skipped` and counted
  as failures.
- Struct-of-arrays nodes gave no gain (within noise, slightly worse with the arena): a node is
  24 bytes and the search touches most fields together. Kept as a switch, off by default.
- The arena raises peak memory: PP peak RSS goes from 52.1 MiB (bucket queue) to 75.5 MiB,
  because the buffer is kept between searches. It is bounded at 256 MiB per thread.
- CI is written but has not run on GitHub; the benchmark and profile are local only.
- Lifelong (online task arrival) mode is not implemented.
- Results are tables only; no plots are generated.
- One machine (i5-14500HX under WSL2), 32x32-class maps only.

## Next steps

1. Add ICBS/CBSH enhancements (cardinal conflict priority, bypassing, heuristics) to CBS.
2. Make conflict detection and the constraint/CAT tables incremental, the measured bottleneck
   for CBS/ECBS.
3. Re-run the benchmark serially with a 60 s limit and add plots.
4. Lifelong MAPF with tasks arriving over time.
5. Link into terminal-digital-twin through `add_subdirectory` and `mapf::mapf`.
6. Push to GitHub and get the CI workflow green.
