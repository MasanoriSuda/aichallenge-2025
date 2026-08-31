# Results: Stop stationary trajectory contract

## Observed phenomenon

In `output/20260831-184430`, decision 1116 lost Cruise authority at only
0.055985 m/s even though the measured-to-control path and dynamic-obstacle
proof were clear.  The physical Stop successor was rejected with
`exact-trajectory-rejected/invalid-path-distance`.

## Causal chain

1. Maximum braking of -3.0 m/s2 reaches zero speed after approximately
   0.0187 s.
2. The already serialized command must remain represented until the 0.025 s
   publisher boundary.
3. During the remaining approximately 0.0063 s, time and actuator response
   advance but physical path distance remains unchanged.
4. The common exact-trajectory validator required strictly increasing path
   distance for every temporal sample.
5. It rejected the valid stationary suffix, which removed the terminal Stop
   certificate and therefore normal authority.
6. Later `steering-unreachable` and retained Stop loops were downstream
   symptoms after authority had already been lost.

## Root cause

A temporal physical trajectory was validated as an unconditional spatial
parameterization.  Stationary time samples are invalid for a normal advancing
MPCC path, but necessary for a maximum-braking Stop command which reaches the
zero-velocity saturation inside one publisher period.

## Structural repair

- Added an immutable, default-off Stop-only stationary-suffix property to the
  exact physical trajectory.
- Equal adjacent path distances are accepted only when that property is set
  and both samples are inside the sealed stationary-velocity tolerance.
- The maximum-braking Stop builder is the only production owner which enables
  the property.
- Distance regression remains invalid in every case.
- Normal and moving repeated-distance trajectories remain invalid.

No solver, acceleration, wall clearance, timeout, lease, grace or fallback
parameter changed.

## Static verification

- `make autoware-build`: passed, 25 packages built.
- Targeted physical adapter and foundation tests: 2/2 passed.
- Complete `multi_purpose_mpc_ros` CTest suite: 59/59 passed.
- Regression fixture reproduces a 0.055985 m/s Stop crossing zero inside one
  0.025 s publication interval and observes the real repeated-distance suffix.

## Dynamic acceptance

Bounded `make dev2` Acceptance was run in `output/20260831-190352` and stopped
after the repaired contract and the next independent failure were observable.

- `invalid-path-distance`: 0 occurrences.
- `terminal Stop course geometry unavailable`: 0 occurrences.
- `Published Stop successor shadow`: 28 observations; the exact physical
  trajectory remained accepted rather than failing on its stationary suffix.
- The run entered `ShiftOut` and continued publishing certified normal MPCC
  solutions, so Acceptance covered both nominal running and a tactical intent.

The repair is therefore dynamically accepted for the frozen low-speed Stop
failure.  The next independent issue is not folded into this Slice: a tactical
candidate took about 2.38 s to build, and by result consumption its stage wall
proof was rejected (`stage-wall-rejected` / `ShiftOut/Pass path requires wall
clamp`).  That timing/provenance failure requires a separate frozen snapshot
and root-cause Slice.
