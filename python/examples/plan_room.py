#!/usr/bin/env python3
"""Plan one MovingAI instance with all three planners and print a comparison.

usage: PYTHONPATH=build/release python3 python/examples/plan_room.py [map] [scen-index] [agents]
"""
import sys
from pathlib import Path

import mapf_py

root = Path(__file__).resolve().parents[2]
name = sys.argv[1] if len(sys.argv) > 1 else "room-32-32-4"
scen = sys.argv[2] if len(sys.argv) > 2 else "1"
agents = int(sys.argv[3]) if len(sys.argv) > 3 else 20

inst = mapf_py.load_instance(str(root / f"data/maps/{name}.map"),
                             str(root / f"data/scen/{name}-random-{scen}.scen"), agents)
print(f"{name} scen {scen}: {inst.num_agents} agents on a "
      f"{inst.grid.width}x{inst.grid.height} map ({inst.grid.free_count} free cells)")
print(f"{'planner':<10} {'status':<10} {'SoC':>6} {'makespan':>8} {'runtime ms':>11} valid")
for planner, w in [("pp", 1.0), ("cbs", 1.0), ("ecbs", 1.2), ("ecbs", 1.5)]:
    r = mapf_py.plan(inst, planner, w=w, time_limit_s=10.0)
    label = planner if planner != "ecbs" else f"ecbs w{w}"
    valid = mapf_py.validate(inst, r["paths"])["valid"] if r["solved"] else "-"
    print(f"{label:<10} {r['status']:<10} {r['sum_of_costs']:>6} {r['makespan']:>8} "
          f"{1e3 * r['runtime_s']:>11.1f} {valid}")
