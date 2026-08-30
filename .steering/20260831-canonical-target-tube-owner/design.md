# Design: canonical target-tube owner

## Earliest violated invariant

One immutable control epoch must have one target-prediction owner. The
candidate reference, obstacle rows, exact proof input and problem fingerprint
may use different representations, but a stateless path rebuild cannot silently
replace the sealed target tube with another predictor.

## Producer and causal chain

```text
current V2X observation
  -> canonical MpcProblem target tube (current epoch)
  -> seven-state submission snapshot
  -> stateless candidate generator discards canonical tube       [root]
  -> constant-global-velocity projection into finite wall window
  -> projection clamps at terminal progress                      [contributor]
  -> lateral residual grows to -8.143 m
  -> wall and selected-side obstacle rows become contradictory
  -> OSQP maximum iterations                                     [detection]
  -> retained proof unavailable / Emergency Stop                 [mask]
  -> target eventually stale / Recovery                          [recovery]
```

## Architecture comparison and replay boundary

The unchanged A/B/C/D comparator found no certified bundle on fingerprint
`698276355274855471`. The selected negative side has an explicit stage-10 row
contradiction, while the opposite arm solves but fails the unchanged exact wall
proof.

That result is useful for localizing the contradictory rows, but it cannot
validate this repair. The frozen file is a post-candidate failure snapshot: it
already contains `dynamic_obstacle_forced_physical_diagonal=true` and the
candidate's rebuilt target stages. It is not the raw upstream submission from
before `rebuild_target_horizon`. Replaying it after the repair must therefore
preserve the already-corrupted tube. Dynamic validation needs a newly captured
current-epoch submission/candidate pair.

## Repair

Rename `rebuild_target_horizon` to `resolve_canonical_target_horizon` and make
it a validator/copy operation:

1. require the canonical dynamic-obstacle contract to be active;
2. require target ID and observation generation to match ReplayWorld;
3. require exactly one finite, physically valid stage per solver input;
4. return the sealed stage tube unchanged.

Mission lateral and heading references remain rebuilt statelessly. ReplayWorld
remains the immutable source for the dense exact dynamic certificate. Thus the
SQP approximation and physical certificate retain separate roles without a
second hidden target predictor.

## Dynamic validation

Run `output/20260831-063008` exercised Gate A and entered ShiftOut on D1 and
D2. A new D2 ShiftOut failure snapshot at sequence 904 contains a monotonically
advancing target progress and target lateral range `-0.069477..0.207660 m`.
The previous endpoint-pinned `-1.170..-8.143 m` target no longer appears.

The new sequence-904 failure is a `steering-rate-prefix` convergence failure,
not a dynamic-obstacle lateral contradiction. D1 later aborted for actual
footprint wall margin and D2 invalidated a stale/lost target. Those are separate
failure families and are not patched in this Slice.

## Deleted path

- `CourseProjection` and its rejection taxonomy;
- constant-global-velocity projection through `wall_course_frame_knots`;
- endpoint-clamped target reconstruction tests;
- the experiment assumption that canonical target stages are persistent
  Mission geometry.

## Rollback

Rollback commit: `a8b8cbe3`.
