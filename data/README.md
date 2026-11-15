# Benchmark data

The maps and scenarios come from the **MovingAI MAPF benchmark**
(Stern et al., "Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks", SoCS 2019).

- Source: https://movingai.com/benchmarks/mapf/index.html
  (files fetched from `https://movingai.com/benchmarks/mapf/<map>.map.zip` and
  `<map>.map-scen-random.zip`)
- Licence: Open Data Commons Attribution License (ODC-By), as stated on
  https://movingai.com/benchmarks/mapf.html. Cite the paper above when publishing results.

The files are not committed. Run `scripts/download_movingai.sh` (about 4 MB) to fetch them into
`data/maps/` and `data/scen/`. Maps: `empty-32-32`, `random-32-32-20`, `room-32-32-4`,
`maze-32-32-2`, `warehouse-10-20-10-2-1`, each with its 25 "random" scenario files.

Format notes used by the loaders in `src/io.cpp`:
- `.map`: header `type octile`, `height H`, `width W`, `map`, then H rows. `.`, `G` and `S`
  are free; every other character (`@`, `O`, `T`, `W`) is an obstacle.
- `.scen`: `version 1`, then one tab-separated line per agent:
  `bucket map width height start_x start_y goal_x goal_y optimal_length`. The last column is an
  8-connected octile distance and is ignored here (this library plans on a 4-connected grid).
  The first N lines form an N-agent instance, the usual convention in MAPF papers.
