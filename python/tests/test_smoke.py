"""Smoke test for the mapf_py bindings. Needs the built module on PYTHONPATH
(build/release, or wherever cmake put mapf_py*.so); MovingAI cases are skipped
when data/ has not been downloaded."""
from pathlib import Path

import pytest

import mapf_py

DATA = Path(__file__).resolve().parents[2] / "data"

ROWS = [
    ".....",
    ".@.@.",
    ".....",
]
# Two agents that must pass each other in a 3x5 corridor with pillars.
AGENTS = [((0, 0), (4, 0)), ((4, 0), (0, 0))]


@pytest.mark.parametrize("planner", ["pp", "cbs", "ecbs"])
def test_small_instance_all_planners(planner):
    inst = mapf_py.make_instance(ROWS, AGENTS)
    assert inst.num_agents == 2 and inst.grid.width == 5 and inst.grid.height == 3
    r = mapf_py.plan(inst, planner, w=1.5, time_limit_s=5.0)
    assert r["solved"], r["status"]
    paths = r["paths"]
    assert [p[0] for p in paths] == inst.starts
    assert [p[-1] for p in paths] == inst.goals
    report = mapf_py.validate(inst, paths)
    assert report["valid"], report["errors"]
    assert report["sum_of_costs"] == r["sum_of_costs"]


def test_cbs_is_optimal_here():
    inst = mapf_py.make_instance(ROWS, AGENTS)
    cbs = mapf_py.plan(inst, "cbs")
    # Each agent needs 4 moves alone. One must duck into the middle-row gap at
    # x=2 and wait there, costing 3 extra steps in total: optimum is 11.
    assert cbs["sum_of_costs"] == 11
    ecbs = mapf_py.plan(inst, "ecbs", w=1.5)
    assert cbs["sum_of_costs"] <= ecbs["sum_of_costs"] <= 1.5 * cbs["sum_of_costs"]


def test_validator_rejects_collision():
    inst = mapf_py.make_instance(ROWS, AGENTS)
    # Head-on swap in the top corridor: an edge conflict at t=3.
    bad = [[(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)], [(4, 0), (3, 0), (2, 0), (1, 0), (0, 0)]]
    report = mapf_py.validate(inst, bad)
    assert not report["valid"] and report["errors"]


def test_bad_input_raises():
    with pytest.raises(ValueError):
        mapf_py.plan(mapf_py.make_instance(ROWS, AGENTS), "astar")
    with pytest.raises(IndexError):
        mapf_py.make_instance(ROWS, [((9, 9), (0, 0))])


@pytest.mark.skipif(not (DATA / "maps" / "room-32-32-4.map").exists(),
                    reason="MovingAI data not downloaded")
def test_movingai_room():
    inst = mapf_py.load_instance(str(DATA / "maps/room-32-32-4.map"),
                                 str(DATA / "scen/room-32-32-4-random-1.scen"), 20)
    assert inst.num_agents == 20
    r = mapf_py.plan(inst, "ecbs", w=1.2, time_limit_s=10.0)
    assert r["solved"]
    assert mapf_py.validate(inst, r["paths"])["valid"]
    assert r["lower_bound"] <= r["sum_of_costs"] <= 1.2 * r["lower_bound"] + 1e-9
