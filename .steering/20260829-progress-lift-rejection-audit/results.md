# Results

## Root cause and correction

Retained revalidation compared two different quantities:

- current `BicycleModel::s`: cumulative progress at a discrete associated
  waypoint;
- retained artifact `theta`: continuously integrated course-frame progress.

The artifact also carries a longitudinal lag state.  The physical quantity
represented by the artifact is therefore `theta + lag`, not `theta` alone.
The correction makes both sides mean physical along-course position at the
same control origin:

```text
current = associated-waypoint progress + projected longitudinal lag
artifact = theta + lag
```

The circular lap lift and the 1.5 m continuity tolerance were not changed.

## Verification

- `make autoware-build`: passed, 25 packages.
- focused retained-revalidation test: 49/49 passed.
- source-contract test: 75/75 passed after updating the intentional log
  contract.
- complete `multi_purpose_mpc_ros` test suite: 54/54 CTest targets passed.
- `git diff --check`: passed.

The new regression uses `theta=50.20 m`, `lag=0.80 m` and a current physical
progress of `51.00 m`.  The old theta-only comparison would reject it with a
0.80 m discontinuity under the 0.20 m test tolerance; the corrected physical
comparison accepts it with zero difference.  The independent discontinuity
test still rejects a 9.85 m mismatch.

## Dynamic evidence

Run: `output/20260829-164506`, D1 episode 2, sequence 1074.

- The retained artifact remained accepted while its physical progress delta
  was small; no waypoint-boundary reject/accept oscillation was observed.
- At decision 1710, the first authority loss was
  `terminal-contingency-unavailable`, not `progress-lift-rejected`.
- Its proof reports a terminal Stop suffix wall collision while the current
  artifact was only 0.60 m ahead of the physical control origin.
- Stop then correctly owned the wire and decelerated the vehicle.
- Only after that deceleration did decision 1732 report a genuine physical
  progress difference of -1.586 m and reject progress continuity.

Therefore the coordinate defect is removed, while the integrated run still
fails for a separate upstream reason.  The next Slice must audit why an
already-published ShiftOut loses terminal-successor viability; relaxing
progress tolerance would merely hide the downstream consequence.

## Unchanged boundaries

- No Mission resume rule, lease, grace period, timeout or fallback was added.
- No solver tolerance, speed, wall clearance or lateral-acceleration setting
  was changed.
- Stop authority and legacy three-state MPC coordinates are unchanged.
