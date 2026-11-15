"""Plot results/bench.csv: success rate, sum-of-costs, makespan and runtime vs number of agents.

Usage: python scripts/plot_bench.py results/bench.csv results/
Writes one PNG per metric (bench_<metric>.png), one subplot per map, one line per planner.
Cost/makespan/runtime are means over *solved* instances only (n shown by the success plot).
Runtimes come from the parallel run-A benchmark and are noisy; see results/profile.md.
"""

import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

METRICS = {
    "success": "success rate",
    "sum_of_costs": "sum of costs (solved)",
    "makespan": "makespan (solved)",
    "runtime_s": "runtime s (solved, log)",
}


def label(row):
    return row["planner"] if row["planner"] != "ecbs" else f"ecbs w={float(row['w']):g}"


def main(csv_path, out_dir):
    rows = list(csv.DictReader(open(csv_path)))
    maps = sorted({r["map"] for r in rows})
    planners = sorted({label(r) for r in rows})
    # (metric, map, planner) -> agents -> list of values
    data = defaultdict(lambda: defaultdict(list))
    for r in rows:
        key, n = (r["map"], label(r)), int(r["agents"])
        solved = r["status"] == "solved"
        data[("success",) + key][n].append(1.0 if solved else 0.0)
        if solved:
            for m in ("sum_of_costs", "makespan", "runtime_s"):
                data[(m,) + key][n].append(float(r[m]))

    out = Path(out_dir)
    for metric, ylabel in METRICS.items():
        fig, axes = plt.subplots(1, len(maps), figsize=(4 * len(maps), 3.4), sharey=metric == "success")
        for ax, mp in zip(axes, maps):
            for pl in planners:
                series = data[(metric, mp, pl)]
                xs = sorted(series)
                if xs:
                    ax.plot(xs, [sum(series[x]) / len(series[x]) for x in xs], marker="o", ms=3, label=pl)
            ax.set_title(mp, fontsize=9)
            ax.set_xlabel("agents")
            if metric == "runtime_s":
                ax.set_yscale("log")
            ax.grid(alpha=0.3)
        axes[0].set_ylabel(ylabel)
        axes[-1].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(out / f"bench_{metric}.png", dpi=110)
        plt.close(fig)
        print(f"wrote {out / f'bench_{metric}.png'}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
