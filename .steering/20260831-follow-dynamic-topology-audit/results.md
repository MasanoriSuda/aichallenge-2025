# Results

## Frozen comparison

The audit replayed Domain 1 decision `1187` from
`output/20260831-192221` with interaction fingerprint
`13608911548693048044`.

| Arm | Result | Earliest useful boundary |
|---|---|---|
| Persistent `Automatic` | rejected | coupled dynamic-obstacle row, stage 3 |
| Follow `StayBehind` | rejected | effective-progress obstacle row, stage 4 |
| Stateless left/right | rejected | no certified bundle |
| Rough lattice left/right | rejected | all bounded candidates rejected |
| Offline multi-SQP left/right | rejected | no certified bundle |
| Physical nonlinear warm-start oracle | accepted | exact wall/dynamic/terminal proof accepted |

Changing only the Follow topology does not restore feasibility.  The failure
is therefore not evidence for binding production Follow to `StayBehind`.

## Root-cause boundary

The recorded obstacle stages advance from `3.350506 m` to `5.200213 m` over
twenty `0.25 s` stages: about `0.389 m/s`.  The same immutable replay obstacle
has velocity `(-2.531755, 3.015834) m/s`; projection onto the adjacent recorded
course tangents gives about `3.9 m/s` along track.  Lateral evolution has the
same order-of-magnitude discrepancy.

The nonlinear oracle uses the immutable timed world and accepts the controls,
while both convex longitudinal topologies reject.  The earliest unresolved
boundary is therefore the target-tube time/velocity provenance feeding the
convex problem, not Mission resume policy, solver tolerance, clearance, or a
proven physical impossibility.

## Production decision

No production authority, parameter, solver setting, timeout, lease, grace,
fallback, or clearance was changed.  The audit arm is reachable only through
the standalone comparison tool and has no Store, mailbox, command or publisher
API.

MPCC work is intentionally paused at this evidence boundary.  If resumed, the
next Slice must compare a current-world, semantic-time-aligned target tube on
this exact snapshot and consolidate the duplicate prediction producer only if
that arm certifies.

## Verification

- `make autoware-build`: passed, 25 packages built.
- Focused architecture-comparison target: 34/34 tests passed; aggregate
  `colcon test-result` reported 2272 tests, 0 errors and 0 failures (with one
  pre-existing missing `joycon_contract_guard/package.xml` result warning).
- Frozen architecture replay: deterministic rejection for both `Automatic`
  and `StayBehind`.
- `git diff --check`: passed.
