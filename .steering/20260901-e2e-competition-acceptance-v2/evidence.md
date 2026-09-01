# Evidence

## Root-cause evidence

Before this slice, `run_simulator.bash` redirected stdout to `LOG_DIR` but did
not change AWSIM cwd.  Historic root files in `aichallenge/` had different
modification times for individual domains, while their owning `output/<run>`
directories had no AWSIM result JSON.  `output/latest/d1/result-summary.json`
also pointed at an old 2026-07-15 run.  Therefore historic root JSON could not
be paired safely with a later bag.

## Static verification

```text
bash -n aichallenge/run_simulator.bash                         pass
python3 -m py_compile analyze_e2e_competition.py               pass
targeted analyzer tests                                        16 passed
TinyLidarNet full tests in autoware-command container           94 passed
git diff --check                                                pass
```

The historic six-lap motion-only run `output/20260901-084641` is now classified
as `incomplete` with `missing-result-summary`, rather than being silently called
a competition pass.

## Dynamic verification

- command: `make e2e-single`
- run: `output/20260901-151131`
- AWSIM result files:
  - `output/20260901-151131/result-summary.json`
  - `output/20260901-151131/d1-result-details.json`
- `output/latest/d1` result, bag, analytics, and log links all point to this run
- runtime control mode: `fixed_lidar_brake`
- runtime checkpoint path: packaged `tinylidarnet_weights.npy`
- source/package SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`

Race result:

| lap | time |
|---:|---:|
| 1 | 99.030 s |
| 2 | 88.241 s |
| 3 | 88.956 s |

- Finish: yes, 3/3 laps
- penalty count: 0
- bag distance: 1007.75 m
- bag duration: 296.73 s
- mean forward speed: 3.40 m/s
- longest post-start low speed: 0.00 s
- longest positive-acceleration stall: 0.00 s
- combined competition status: `pass`

This proves the result-provenance correction and single-vehicle acceptance
path.  It does not prove four-peer collision avoidance; candidate3 remains
frozen and the known production four-peer failure remains open.
