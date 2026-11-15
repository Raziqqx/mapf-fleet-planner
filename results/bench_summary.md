# Benchmark summary

Configuration: maps=empty-32-32,random-32-32-20,room-32-32-4,maze-32-32-2,warehouse-10-20-10-2-1; agents=10:100:10; scens=1:10:1 (random); planners=pp,cbs,ecbs:1.2,ecbs:1.5; time limit=10s per instance; jobs=10 parallel (map, planner) groups; wall time=418s; CPU: Intel(R) Core(TM) i5-14500HX, 20 threads

2000 rows; every solved plan was checked by the independent validator (0 invalid). "skipped" = a larger agent count after the planner solved none of the scenarios at a smaller one; counted as a failure.

**Note:** runtimes were measured under parallel load and are noisy; success rates near the time limit may be slightly understated. Clean single-threaded timings are in `results/profile.md`.

## Largest agent count with >= 50 % success

| map | pp | cbs | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|---|
| empty-32-32 | 100 | 40 | 100 | 100 |
| maze-32-32-2 | 20 | 10 | 60 | 100 |
| random-32-32-20 | 100 | 30 | 100 | 100 |
| room-32-32-4 | 50 | 20 | 60 | 100 |
| warehouse-10-20-10-2-1 | 100 | 40 | 100 | 100 |

## empty-32-32

Success rate (%) / median runtime of solved instances (s):

| agents | pp | cbs | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|---|
| 10 | 100 / 0.000 | 100 / 0.001 | 100 / 0.001 | 100 / 0.001 |
| 20 | 100 / 0.001 | 90 / 0.002 | 100 / 0.002 | 100 / 0.002 |
| 30 | 100 / 0.002 | 80 / 0.005 | 100 / 0.004 | 100 / 0.003 |
| 40 | 100 / 0.003 | 60 / 0.010 | 100 / 0.007 | 100 / 0.007 |
| 50 | 100 / 0.003 | 40 / 0.013 | 100 / 0.011 | 100 / 0.012 |
| 60 | 100 / 0.004 | 40 / 0.051 | 100 / 0.017 | 100 / 0.016 |
| 70 | 100 / 0.005 | 0 | 100 / 0.024 | 100 / 0.021 |
| 80 | 100 / 0.006 | 0 | 100 / 0.030 | 100 / 0.033 |
| 90 | 100 / 0.007 | 0 | 100 / 0.044 | 100 / 0.040 |
| 100 | 100 / 0.007 | 0 | 100 / 0.053 | 100 / 0.052 |

Mean sum of costs relative to CBS (optimal), on instances both solved (n = number of such instances):

| agents | pp | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|
| 10 | 1.0129 (n=10) | 1.0000 (n=10) | 1.0000 (n=10) |
| 20 | 1.0327 (n=9) | 1.0010 (n=9) | 1.0010 (n=9) |
| 30 | 1.0558 (n=8) | 1.0025 (n=8) | 1.0025 (n=8) |
| 40 | 1.0848 (n=6) | 1.0042 (n=6) | 1.0042 (n=6) |
| 50 | 1.1081 (n=4) | 1.0065 (n=4) | 1.0065 (n=4) |
| 60 | 1.1287 (n=4) | 1.0071 (n=4) | 1.0071 (n=4) |

## maze-32-32-2

Success rate (%) / median runtime of solved instances (s):

| agents | pp | cbs | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|---|
| 10 | 100 / 0.001 | 100 / 0.020 | 100 / 0.004 | 100 / 0.004 |
| 20 | 70 / 0.004 | 0 | 100 / 0.011 | 100 / 0.011 |
| 30 | 40 / 0.007 | 0 | 100 / 0.040 | 100 / 0.043 |
| 40 | 30 / 0.011 | 0 | 90 / 0.121 | 100 / 0.128 |
| 50 | 0 | 0 | 90 / 0.184 | 100 / 0.190 |
| 60 | 0 | 0 | 50 / 0.275 | 100 / 0.472 |
| 70 | 0 | 0 | 20 / 0.387 | 100 / 0.852 |
| 80 | 0 | 0 | 20 / 4.568 | 100 / 1.234 |
| 90 | 0 | 0 | 0 | 80 / 1.679 |
| 100 | 0 | 0 | 0 | 70 / 2.132 |

Mean sum of costs relative to CBS (optimal), on instances both solved (n = number of such instances):

| agents | pp | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|
| 10 | 1.0413 (n=10) | 1.0043 (n=10) | 1.0043 (n=10) |

## random-32-32-20

Success rate (%) / median runtime of solved instances (s):

| agents | pp | cbs | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|---|
| 10 | 100 / 0.001 | 100 / 0.001 | 100 / 0.001 | 100 / 0.001 |
| 20 | 100 / 0.001 | 100 / 0.006 | 100 / 0.003 | 100 / 0.003 |
| 30 | 100 / 0.002 | 70 / 0.088 | 100 / 0.005 | 100 / 0.004 |
| 40 | 100 / 0.003 | 30 / 0.074 | 100 / 0.008 | 100 / 0.007 |
| 50 | 90 / 0.004 | 0 | 100 / 0.017 | 100 / 0.015 |
| 60 | 80 / 0.006 | 0 | 100 / 0.025 | 100 / 0.023 |
| 70 | 70 / 0.007 | 0 | 100 / 0.039 | 100 / 0.036 |
| 80 | 60 / 0.009 | 0 | 100 / 0.062 | 100 / 0.055 |
| 90 | 60 / 0.012 | 0 | 100 / 0.081 | 100 / 0.073 |
| 100 | 60 / 0.012 | 0 | 100 / 0.121 | 100 / 0.099 |

Mean sum of costs relative to CBS (optimal), on instances both solved (n = number of such instances):

| agents | pp | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|
| 10 | 1.0173 (n=10) | 1.0017 (n=10) | 1.0017 (n=10) |
| 20 | 1.0487 (n=10) | 1.0040 (n=10) | 1.0052 (n=10) |
| 30 | 1.0459 (n=7) | 1.0049 (n=7) | 1.0049 (n=7) |
| 40 | 1.0777 (n=3) | 1.0106 (n=3) | 1.0118 (n=3) |

## room-32-32-4

Success rate (%) / median runtime of solved instances (s):

| agents | pp | cbs | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|---|
| 10 | 100 / 0.001 | 100 / 0.003 | 100 / 0.001 | 100 / 0.001 |
| 20 | 100 / 0.002 | 70 / 0.186 | 100 / 0.004 | 100 / 0.004 |
| 30 | 100 / 0.003 | 0 | 100 / 0.013 | 100 / 0.011 |
| 40 | 100 / 0.006 | 0 | 100 / 0.029 | 100 / 0.022 |
| 50 | 70 / 0.010 | 0 | 100 / 0.060 | 100 / 0.053 |
| 60 | 30 / 0.011 | 0 | 80 / 0.117 | 100 / 0.079 |
| 70 | 20 / 0.015 | 0 | 30 / 0.136 | 100 / 0.154 |
| 80 | 20 / 0.022 | 0 | 30 / 3.411 | 100 / 0.208 |
| 90 | 0 | 0 | 0 | 100 / 0.360 |
| 100 | 0 | 0 | 0 | 100 / 0.572 |

Mean sum of costs relative to CBS (optimal), on instances both solved (n = number of such instances):

| agents | pp | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|
| 10 | 1.0285 (n=10) | 1.0083 (n=10) | 1.0083 (n=10) |
| 20 | 1.0537 (n=7) | 1.0185 (n=7) | 1.0213 (n=7) |

## warehouse-10-20-10-2-1

Success rate (%) / median runtime of solved instances (s):

| agents | pp | cbs | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|---|
| 10 | 100 / 0.002 | 100 / 0.002 | 100 / 0.002 | 100 / 0.002 |
| 20 | 100 / 0.004 | 90 / 0.005 | 100 / 0.006 | 100 / 0.005 |
| 30 | 100 / 0.007 | 70 / 0.028 | 100 / 0.013 | 100 / 0.012 |
| 40 | 100 / 0.011 | 60 / 0.069 | 100 / 0.021 | 100 / 0.021 |
| 50 | 100 / 0.014 | 40 / 0.065 | 100 / 0.031 | 100 / 0.034 |
| 60 | 100 / 0.018 | 30 / 2.126 | 100 / 0.041 | 100 / 0.046 |
| 70 | 100 / 0.022 | 30 / 5.981 | 100 / 0.059 | 100 / 0.058 |
| 80 | 100 / 0.024 | 0 | 100 / 0.084 | 100 / 0.079 |
| 90 | 100 / 0.026 | 0 | 100 / 0.124 | 100 / 0.112 |
| 100 | 100 / 0.037 | 0 | 100 / 0.178 | 100 / 0.172 |

Mean sum of costs relative to CBS (optimal), on instances both solved (n = number of such instances):

| agents | pp | ecbs_w1.2 | ecbs_w1.5 |
|---|---|---|---|
| 10 | 1.0000 (n=10) | 1.0000 (n=10) | 1.0000 (n=10) |
| 20 | 1.0105 (n=9) | 1.0001 (n=9) | 1.0001 (n=9) |
| 30 | 1.0165 (n=7) | 1.0001 (n=7) | 1.0001 (n=7) |
| 40 | 1.0216 (n=6) | 1.0006 (n=6) | 1.0006 (n=6) |
| 50 | 1.0346 (n=4) | 1.0006 (n=4) | 1.0006 (n=4) |
| 60 | 1.0551 (n=3) | 1.0017 (n=3) | 1.0017 (n=3) |
| 70 | 1.0515 (n=3) | 1.0014 (n=3) | 1.0014 (n=3) |
