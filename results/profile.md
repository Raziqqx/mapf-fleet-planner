# Low-level search profile: before/after

CPU: Intel(R) Core(TM) i5-14500HX; single process pinned to CPU 2 (`taskset`), no other benchmark load; median of 5 reps, each a fresh process. Release build (-O3). Wall time is the summed planner time over the group (loading excluded); nodes/s = low-level states expanded / wall time; peak RSS is the whole process (ru_maxrss). Every variant expands exactly the same states (checked), so the differences are pure implementation cost.

Instance groups (fixed, from the run-A benchmark): **pp** = prioritized planning, 100 agents, 2 scenarios on each of the 5 maps; **cbs** = 5 instances with 10-60 agents; **ecbs** = ECBS w=1.2, 5 instances with 70-100 agents. Work per group: pp 1,588,785 low-level expansions, cbs 573,229 low-level expansions, ecbs 370,721 low-level expansions.

## Each optimisation added in turn

| variant | pp wall s | pp Mnodes/s | pp RSS MiB | cbs wall s | cbs Mnodes/s | cbs RSS MiB | ecbs wall s | ecbs Mnodes/s | ecbs RSS MiB | speed-up pp / cbs / ecbs |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline: std::set, AoS, heap alloc | 1.329 | 1.20 | 49.6 | 1.319 | 0.43 | 14.1 | 0.901 | 0.41 | 14.1 | 1.00x / 1.00x / 1.00x |
| (a) binary heap | 1.219 | 1.30 | 49.5 | 1.130 | 0.51 | 14.1 | 0.748 | 0.50 | 14.1 | 1.09x / 1.17x / 1.20x |
| (a) bucket queue | 0.988 | 1.61 | 52.1 | 1.089 | 0.53 | 14.1 | 0.764 | 0.49 | 14.1 | 1.35x / 1.21x / 1.18x |
| (a)+(b) bucket + SoA | 0.985 | 1.61 | 45.7 | 1.105 | 0.52 | 14.1 | 0.775 | 0.48 | 14.1 | 1.35x / 1.19x / 1.16x |
| (a)+(c) bucket + arena | 0.842 | 1.89 | 75.5 | 1.000 | 0.57 | 14.1 | 0.725 | 0.51 | 14.1 | 1.58x / 1.32x / 1.24x |
| (a)+(b)+(c) all three | 0.866 | 1.84 | 70.6 | 1.013 | 0.57 | 14.1 | 0.771 | 0.48 | 14.1 | 1.53x / 1.30x / 1.17x |

## Full matrix (median wall s)

| open | layout | arena | pp | cbs | ecbs |
|---|---|---|---:|---:|---:|
| set | aos | 0 | 1.329 | 1.319 | 0.901 |
| set | aos | 1 | 1.331 | 1.155 | 0.845 |
| set | soa | 0 | 1.325 | 1.365 | 0.895 |
| set | soa | 1 | 1.315 | 1.200 | 0.824 |
| heap | aos | 0 | 1.219 | 1.130 | 0.748 |
| heap | aos | 1 | 1.136 | 1.052 | 0.698 |
| heap | soa | 0 | 1.248 | 1.163 | 0.767 |
| heap | soa | 1 | 1.122 | 1.062 | 0.720 |
| bucket | aos | 0 | 0.988 | 1.089 | 0.764 |
| bucket | aos | 1 | 0.842 | 1.000 | 0.725 |
| bucket | soa | 0 | 0.985 | 1.105 | 0.775 |
| bucket | soa | 1 | 0.866 | 1.013 | 0.771 |
