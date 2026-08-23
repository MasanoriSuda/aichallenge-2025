# Audit

## Observation

The frozen `8ff1a9e` build was exercised with `make dev2` in
`output/20260824-031752`. Domain 1 produced two typed Overtake episodes:

| Episode | Interval | Transition | First terminal cause |
|---|---:|---|---|
| 1 | 3.855 s | `Idle -> ShiftOut -> FollowPrepare -> Idle` | live corridor loss, then Dynamic Mission wait distance limit |
| 2 | 1.256 s | `Idle -> ShiftOut -> FollowPrepare -> Idle` | final physical wall admission rejected `0.35283 m < 0.4 m` |

The run is therefore sufficient to replace the invalid pre-race Gate at
`output/20260824-005436` with live Overtake evidence.

## Failure decomposition

### Root defect

Overtake does not yet have a canonical production producer lifecycle. The
five-state solve, exact artifact extraction and canonical selector are executed
inside a telemetry-only shadow block. Production independently converts a
successful five-state primal through `convert_extended_solution_to_legacy()`;
on a fresh miss it enters a circuit breaker and solves the three-state/legacy
formulation.

This creates the observed causal chain:

```text
synchronous five-state solve in the 40 Hz callback
-> fresh five-state result may be slow or unavailable
-> shadow canonical chain is not connected to publication
-> circuit/reentry path selects three-state or legacy normal control
-> no same-formulation retained command is available
-> final trace reports legacy-normal-bypass or wall-handoff hold
```

The first emitted five-state result in each episode was physically certified,
but the final trace reported
`canonical=violated, reason=missing-canonical-command-identity`. This is a
symptom of the shadow/production split, not a standalone logging omission.

### Contributors

- The synchronous extended solve caused deadline pressure during the exact
  Overtake intervals: Episode 1 contained 14 callback overruns and Episode 2
  contained 5.
- The extended solver succeeded on 220 of 322 eligible cycles. The remaining
  102 production cycles were recorded as fallback: 92 circuit skips, 4 solve
  failures and 6 requalification cycles.
- Exact canonical fresh construction completed on 198 cycles. Two otherwise
  valid fresh solutions were rejected by the physical wall certificate.
- All 58 retained attempts failed before world certification: 40 expired
  cursors and 18 unavailable current course-frame windows.
- Episode 2 exposed a real planner/execution geometry disagreement: planner
  reserve `1.86212 m`, physical swept-footprint clearance `0.35283 m`, required
  `0.4 m`. The physical admission correctly rejected it; this is not evidence
  for relaxing clearance.

### Masks

- `convert_extended_solution_to_legacy()` makes a certified five-state solve
  look like a usable production solution without carrying canonical command
  identity.
- The circuit-breaker/reentry/three-state fallback keeps publishing normal
  commands when the canonical five-state chain is unavailable, hiding the
  continuity failure.
- Wall handoff states prevent immediate wall contact but obscure which normal
  formulation lost ownership.

### Detection gap

The previous Gate counted solver/certificate outcomes but did not require the
same exact `CanonicalNormalCommand` to reach final publication. The joined
final trace now exposes this as `missing-canonical-command-identity` and
`legacy-normal-bypass`.

## Rejected local repair

Simply copying the fresh shadow command into the publisher is rejected. Fresh
coverage was only 198/256 eligible canonical-shadow cycles and retained
coverage was 0/58. Such a patch would turn the uncovered cycles into repeated
Emergency Stop, not provide one stable MPCC authority.

## Next bounded root slice

Build one Overtake canonical producer lifecycle, analogous in ownership (not
necessarily in implementation) to the accepted asynchronous Follow producer:

1. seal a five-state Overtake problem snapshot with current intent/world
   provenance;
2. solve/certify it outside the 40 Hz publication callback;
3. atomically store the complete immutable canonical plan;
4. select fresh or current-world-revalidated retained command from that one
   store;
5. only after dynamic coverage, connect that selected command to publication
   and delete conversion, circuit/reentry and three-state normal fallback in
   the same authority-promotion slice.

The next slice must begin with deterministic failure coverage for result
identity, stale snapshot rejection, cursor advancement and missing-fresh
same-formulation continuation. No solver, wall-margin or behavior tuning is
authorized.
