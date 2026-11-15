#!/usr/bin/env bash
# Reproduces results/bench.csv and results/bench_summary.md:
# 5 MovingAI maps x 10..100 agents x 4 planners x 10 random scenarios, 10 s per instance.
# Takes about 15 minutes on a 20-thread laptop CPU.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAPS="empty-32-32,random-32-32-20,room-32-32-4,maze-32-32-2,warehouse-10-20-10-2-1"
AGENTS="10:100:10"
SCENS="1:10:1"
PLANNERS="pp,cbs,ecbs:1.2,ecbs:1.5"
TIME_LIMIT=10
JOBS=${JOBS:-10}

scripts/download_movingai.sh
cmake --preset release >/dev/null
cmake --build --preset release --target mapf_bench
mkdir -p results
start=$(date +%s)
./build/release/mapf_bench --data data --maps "$MAPS" --agents "$AGENTS" --scens "$SCENS" \
    --planners "$PLANNERS" --time-limit "$TIME_LIMIT" --jobs "$JOBS" --out results/bench.csv
elapsed=$(( $(date +%s) - start ))
python3 -I scripts/summarize.py results/bench.csv results/bench_summary.md \
    --config "maps=$MAPS; agents=$AGENTS; scens=$SCENS (random); planners=$PLANNERS; \
time limit=${TIME_LIMIT}s per instance; jobs=$JOBS parallel (map, planner) groups; \
wall time=${elapsed}s; CPU: $(lscpu | sed -n 's/^Model name: *//p'), $(nproc) threads"
