#!/usr/bin/env python3
"""Export an animated SVG (SMIL) replay of one solved MovingAI instance.

The plan is computed through the mapf_py bindings, checked by the independent
validator, and written as a self-contained SVG that loops in any browser.
Each agent is a coloured disc moving one cell per step; its goal is a square
outline in the same colour.

usage: PYTHONPATH=build/release python3 python/examples/replay_svg.py \
           [--map room-32-32-4] [--scen 1] [--agents 20] [--planner ecbs] [--w 1.2] \
           [--out docs/replay.svg]
"""
import argparse
import colorsys
from pathlib import Path

import mapf_py

CELL = 16          # pixels per cell
STEP_S = 0.25      # seconds per timestep
HOLD_STEPS = 6     # pause at the end before looping


def colour(i, n):
    r, g, b = colorsys.hls_to_rgb(i / n, 0.5, 0.75)
    return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


def render(inst, paths, title):
    w, h = inst.grid.width, inst.grid.height
    makespan = max(len(p) for p in paths) - 1
    frames = makespan + 1 + HOLD_STEPS
    dur = frames * STEP_S
    key_times = ";".join(f"{t / (frames - 1):.4f}" for t in range(frames))
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w * CELL} {h * CELL + 24}" '
        f'width="{w * CELL}" height="{h * CELL + 24}" font-family="sans-serif">',
        f"<title>{title}</title>",
        f'<rect width="100%" height="100%" fill="#ffffff"/>',
    ]
    # Obstacles as one path to keep the file small.
    rows = inst.grid.rows()
    blocks = "".join(f"M{x * CELL} {y * CELL}h{CELL}v{CELL}h-{CELL}z"
                     for y, row in enumerate(rows) for x, ch in enumerate(row) if ch != ".")
    out.append(f'<path d="{blocks}" fill="#3a3f47"/>')
    n = len(paths)
    for i, (path, goal) in enumerate(zip(paths, inst.goals)):
        c = colour(i, n)
        out.append(f'<rect x="{goal[0] * CELL + 2}" y="{goal[1] * CELL + 2}" width="{CELL - 4}" '
                   f'height="{CELL - 4}" fill="none" stroke="{c}" stroke-width="1.5"/>')
    for i, path in enumerate(paths):
        c = colour(i, n)
        full = path + [path[-1]] * (frames - len(path))  # agents stay at their goal
        xs = ";".join(str(x * CELL + CELL // 2) for x, _ in full)
        ys = ";".join(str(y * CELL + CELL // 2) for _, y in full)
        out.append(
            f'<circle r="{CELL * 0.4}" fill="{c}" stroke="#000" stroke-width="0.5">'
            f'<animate attributeName="cx" values="{xs}" keyTimes="{key_times}" dur="{dur}s" '
            f'repeatCount="indefinite"/>'
            f'<animate attributeName="cy" values="{ys}" keyTimes="{key_times}" dur="{dur}s" '
            f'repeatCount="indefinite"/></circle>')
    out.append(f'<text x="4" y="{h * CELL + 17}" font-size="12" fill="#222">{title}</text>')
    out.append("</svg>")
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="room-32-32-4")
    ap.add_argument("--scen", default="1")
    ap.add_argument("--agents", type=int, default=20)
    ap.add_argument("--planner", default="cbs", choices=["pp", "cbs", "ecbs"])
    ap.add_argument("--w", type=float, default=1.2)
    ap.add_argument("--data", default=str(Path(__file__).resolve().parents[2] / "data"))
    ap.add_argument("--out", default="docs/replay.svg")
    args = ap.parse_args()

    inst = mapf_py.load_instance(f"{args.data}/maps/{args.map}.map",
                                 f"{args.data}/scen/{args.map}-random-{args.scen}.scen",
                                 args.agents)
    r = mapf_py.plan(inst, args.planner, w=args.w, time_limit_s=30.0)
    if not r["solved"]:
        raise SystemExit(f"not solved: {r['status']}")
    report = mapf_py.validate(inst, r["paths"])
    if not report["valid"]:
        raise SystemExit("validator rejected the plan: " + "; ".join(report["errors"]))
    planner = args.planner.upper() if args.planner != "ecbs" else f"ECBS w={args.w}"
    title = (f"{args.map}, scen {args.scen}, {inst.num_agents} agents, {planner}: "
             f"SoC {r['sum_of_costs']}, makespan {r['makespan']} (validated)")
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(render(inst, r["paths"], title))
    print(f"wrote {out} ({out.stat().st_size / 1024:.0f} KiB): {title}")


if __name__ == "__main__":
    main()
