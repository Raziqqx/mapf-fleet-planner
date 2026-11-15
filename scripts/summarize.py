#!/usr/bin/env python3
"""Turns results/bench.csv into a short Markdown summary (standard library only).

Usage: summarize.py BENCH_CSV OUT_MD [--config TEXT]

Tables per map:
  1. success rate (%) by agent count and planner ("skipped" rows count as failures);
  2. median runtime (s) of the solved instances;
  3. sum-of-costs relative to CBS's optimum, on instances both solved.
"""
import argparse
import csv
import statistics
from collections import defaultdict


def fmt_pct(solved, total):
    return f"{100.0 * solved / total:.0f}" if total else "-"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--config", default="")
    args = ap.parse_args()

    with open(args.csv, newline="") as f:
        rows = list(csv.DictReader(f))

    maps = sorted({r["map"] for r in rows})
    # Baseline first, then the optimal planner, then ECBS by increasing w.
    order = {"pp": 0, "cbs": 1}
    planners = sorted({r["planner"] for r in rows}, key=lambda p: (order.get(p, 2), p))
    agent_counts = sorted({int(r["agents"]) for r in rows})

    total = defaultdict(int)
    solved = defaultdict(int)
    runtimes = defaultdict(list)
    soc = {}  # (map, scen, agents, planner) -> sum of costs
    invalid = 0
    for r in rows:
        key = (r["map"], int(r["agents"]), r["planner"])
        total[key] += 1
        if r["status"] == "solved":
            solved[key] += 1
            runtimes[key].append(float(r["runtime_s"]))
            soc[(r["map"], r["scen"], int(r["agents"]), r["planner"])] = int(r["sum_of_costs"])
            invalid += r["valid"] != "1"

    out = ["# Benchmark summary", ""]
    if args.config:
        out += [f"Configuration: {args.config}", ""]
    out += [
        f"{len(rows)} rows; every solved plan was checked by the independent validator "
        f"({invalid} invalid). \"skipped\" = a larger agent count after the planner solved none "
        "of the scenarios at a smaller one; counted as a failure.",
        "",
    ]
    if args.config and "parallel" in args.config:
        out += [
            "**Note:** runtimes were measured under parallel load and are noisy; success rates "
            "near the time limit may be slightly understated. Clean single-threaded timings are "
            "in `results/profile.md`.",
            "",
        ]

    # Headline: the largest agent count each planner solves in >= 50 % of scenarios.
    out += ["## Largest agent count with >= 50 % success", ""]
    out += ["| map | " + " | ".join(planners) + " |", "|---" * (len(planners) + 1) + "|"]
    for m in maps:
        cells = []
        for p in planners:
            ok = [n for n in agent_counts
                  if total[(m, n, p)] and solved[(m, n, p)] * 2 >= total[(m, n, p)]]
            cells.append(str(max(ok)) if ok else "-")
        out.append(f"| {m} | " + " | ".join(cells) + " |")
    out.append("")

    for m in maps:
        out += [f"## {m}", "", "Success rate (%) / median runtime of solved instances (s):", ""]
        out += ["| agents | " + " | ".join(planners) + " |", "|---" * (len(planners) + 1) + "|"]
        for n in agent_counts:
            cells = []
            for p in planners:
                k = (m, n, p)
                rt = f" / {statistics.median(runtimes[k]):.3f}" if runtimes[k] else ""
                cells.append(fmt_pct(solved[k], total[k]) + rt)
            out.append(f"| {n} | " + " | ".join(cells) + " |")
        out.append("")

        # Suboptimality: SoC / CBS SoC on instances CBS and the planner both solved.
        others = [p for p in planners if p != "cbs"]
        out += ["Mean sum of costs relative to CBS (optimal), on instances both solved "
                "(n = number of such instances):", ""]
        out += ["| agents | " + " | ".join(others) + " |", "|---" * (len(others) + 1) + "|"]
        any_row = False
        for n in agent_counts:
            cells = []
            for p in others:
                ratios = [soc[(m, s, n, p)] / soc[(m, s, n, "cbs")]
                          for (mm, s, nn, pp) in soc
                          if mm == m and nn == n and pp == "cbs" and (m, s, n, p) in soc
                          and soc[(m, s, n, "cbs")] > 0]
                cells.append(f"{statistics.mean(ratios):.4f} (n={len(ratios)})" if ratios else "-")
            if any(c != "-" for c in cells):
                any_row = True
                out.append(f"| {n} | " + " | ".join(cells) + " |")
        if not any_row:
            out.append("| (none) |" + " |" * len(others))
        out.append("")

    with open(args.out, "w") as f:
        f.write("\n".join(out))


if __name__ == "__main__":
    main()
