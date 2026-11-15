#!/usr/bin/env python3
"""Before/after profile of the single-agent search optimisations.

Runs build/release/mapf_profile for every (instance group, SearchTuning) pair
REPS times, each in a fresh single-threaded process pinned to one CPU, with
nothing else running. Writes the raw rows to results/profile_runs.csv and the
medians to results/profile.md. Standard library only.

usage: scripts/profile.py [--reps N] [--cpu K] [--bin PATH] [--data DIR] [--out-dir DIR]
"""
import argparse
import csv
import itertools
import platform
import shutil
import statistics
import subprocess
import sys
from pathlib import Path

GROUPS = ["pp", "cbs", "ecbs"]
OPEN = ["set", "heap", "bucket"]
LAYOUT = ["aos", "soa"]
ARENA = ["0", "1"]
FIELDS = ["group", "open", "layout", "arena", "wall_s", "ll_expanded", "hl_expanded",
          "solved", "soc", "peak_rss_kb"]
BASELINE = ("set", "aos", "0")
# Each optimisation added on top of the previous one, for the headline table.
LADDER = [
    ("baseline: std::set, AoS, heap alloc", ("set", "aos", "0")),
    ("(a) binary heap", ("heap", "aos", "0")),
    ("(a) bucket queue", ("bucket", "aos", "0")),
    ("(a)+(b) bucket + SoA", ("bucket", "soa", "0")),
    ("(a)+(c) bucket + arena", ("bucket", "aos", "1")),
    ("(a)+(b)+(c) all three", ("bucket", "soa", "1")),
]


def cpu_name():
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def run_once(args, group, tuning):
    cmd = [args.bin, "--data", args.data, "--group", group, "--open", tuning[0],
           "--layout", tuning[1], "--arena", tuning[2]]
    if shutil.which("taskset"):
        cmd = ["taskset", "-c", str(args.cpu)] + cmd
    out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout.strip()
    return dict(zip(FIELDS, out.split(",")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--cpu", type=int, default=2)
    ap.add_argument("--bin", default="build/release/mapf_profile")
    ap.add_argument("--data", default="data")
    ap.add_argument("--out-dir", default="results")
    args = ap.parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    tunings = list(itertools.product(OPEN, LAYOUT, ARENA))
    rows = []
    # Interleave reps (rep-major) so slow drift in machine state hits every config alike.
    for rep in range(args.reps):
        for group in GROUPS:
            for tuning in tunings:
                row = run_once(args, group, tuning)
                row["rep"] = rep
                rows.append(row)
        print(f"rep {rep + 1}/{args.reps} done", file=sys.stderr)

    with open(out_dir / "profile_runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["rep"] + FIELDS)
        w.writeheader()
        w.writerows(rows)

    med = {}
    for group in GROUPS:
        for tuning in tunings:
            sel = [r for r in rows if r["group"] == group and (r["open"], r["layout"],
                                                                   r["arena"]) == tuning]
            work = {(r["ll_expanded"], r["hl_expanded"], r["soc"]) for r in sel}
            # The variants must do identical work; anything else is a bug.
            assert len(work) == 1, f"non-deterministic work for {group} {tuning}: {work}"
            ll = int(sel[0]["ll_expanded"])
            walls = [float(r["wall_s"]) for r in sel]
            med[group, tuning] = {
                "wall": statistics.median(walls),
                "nps": statistics.median(ll / x for x in walls),
                "rss": statistics.median(int(r["peak_rss_kb"]) for r in sel) / 1024,
                "ll": ll,
            }
    base_work = {g: med[g, BASELINE]["ll"] for g in GROUPS}
    assert all(med[g, t]["ll"] == base_work[g] for g in GROUPS for t in tunings)

    lines = [
        "# Low-level search profile: before/after",
        "",
        f"CPU: {cpu_name()}; single process pinned to CPU {args.cpu} (`taskset`), no other "
        f"benchmark load; median of {args.reps} reps, each a fresh process. Release build "
        "(-O3). Wall time is the summed planner time over the group (loading excluded); "
        "nodes/s = low-level states expanded / wall time; peak RSS is the whole process "
        "(ru_maxrss). Every variant expands exactly the same states (checked), so the "
        "differences are pure implementation cost.",
        "",
        "Instance groups (fixed, from the run-A benchmark): **pp** = prioritized planning, "
        "100 agents, 2 scenarios on each of the 5 maps; **cbs** = 5 instances with 10-60 "
        "agents; **ecbs** = ECBS w=1.2, 5 instances with 70-100 agents. Work per group: "
        + ", ".join(f"{g} {base_work[g]:,} low-level expansions" for g in GROUPS) + ".",
        "",
        "## Each optimisation added in turn",
        "",
        "| variant | " + " | ".join(f"{g} wall s | {g} Mnodes/s | {g} RSS MiB" for g in GROUPS)
        + " | speed-up pp / cbs / ecbs |",
        "|---|" + "---:|" * (3 * len(GROUPS) + 1),
    ]
    for label, t in LADDER:
        cells = []
        for g in GROUPS:
            m = med[g, t]
            cells += [f"{m['wall']:.3f}", f"{m['nps'] / 1e6:.2f}", f"{m['rss']:.1f}"]
        speed = " / ".join(f"{med[g, BASELINE]['wall'] / med[g, t]['wall']:.2f}x" for g in GROUPS)
        lines.append(f"| {label} | " + " | ".join(cells) + f" | {speed} |")
    lines += ["", "## Full matrix (median wall s)", "",
              "| open | layout | arena | " + " | ".join(GROUPS) + " |",
              "|---|---|---|" + "---:|" * len(GROUPS)]
    for t in tunings:
        lines.append(f"| {t[0]} | {t[1]} | {t[2]} | "
                     + " | ".join(f"{med[g, t]['wall']:.3f}" for g in GROUPS) + " |")
    lines.append("")
    (out_dir / "profile.md").write_text("\n".join(lines))
    print("\n".join(lines))


if __name__ == "__main__":
    main()
