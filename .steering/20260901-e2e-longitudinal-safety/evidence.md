# Evidence

## Static validation

- TinyLidarNet controller tests: 21 passed.
- Submit launch contract tests: 5 passed.
- System launch contract tests: 6 passed.
- `make autoware-build`: 25 packages passed.
- Analyzer unit tests: 8 passed.

## Closed-loop runs

### Single vehicle

`output/20260901-053548` ran with candidate2 and `fixed_lidar_brake`.

- distance: 1006.77 m
- duration: 297.63 s
- mean forward speed: 3.385 m/s
- post-start stall: 0 s
- safety activation: 0 in normal free-running sectors

The safety layer caused no measurable normal-course regression relative to the
same candidate without the layer (297.48 s).

### NPC seed 2027, student

`output/20260901-054150` reproduced the student failure.

- distance: 878.89 m
- duration: 329.56 s (run stopped after failure was established)
- post-start low-speed interval: 64.36 s
- low-speed start: 265.19 s
- positive-acceleration stall: 0 s
- stop context: front 0.49 m, right side 0.31 m

The new safety authority correctly replaced `+0.6 m/s2` with braking and stopped
continued acceleration into the obstacle. It did not create a lateral escape,
so this run is a failure. Schema v1 of `analyze_e2e_run.py` incorrectly passed
this case because it measured only positive-acceleration stalls. Schema v2 also
gates all post-start low-speed intervals.

### NPC seed 2027, gap teacher

`output/20260901-054831` used the same world and candidate base network with
`gap_teacher` lateral authority.

- distance: 1020.22 m
- duration: 298.01 s
- mean forward speed: 3.434 m/s
- post-start low-speed interval: 0 s
- front minimum: 1.93 m

The teacher passes the exact seed where the student stalls. Physical feasibility,
the longitudinal safety layer and the teacher policy are therefore not the current
blocker. The remaining blocker is transfer of corrective teacher behavior into
the student training distribution; production weights remain unchanged.
