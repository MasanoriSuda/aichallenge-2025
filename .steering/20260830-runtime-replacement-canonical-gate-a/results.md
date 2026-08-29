# Results

## Root-cause result

The active replacement consumer and causal Gate A producer used different
tactical inputs. Active ShiftOut/Pass tried the live same/cross-side runtime
candidate, while Gate A continued to certify the new-entry selection. The
frozen failure therefore retained the old side even though offline production
candidate `G` found an exactly certified opposite-side trajectory.

The implementation now resolves exactly one tactical input owner:

- Idle uses the pre-entry selection;
- active FollowPrepare/ShiftOut/Pass uses the same-side runtime candidate first,
  then the cross-side candidate, matching execution precedence;
- active execution cannot fall back to pre-entry geometry.

The chosen Mission's old physical certificate is cleared and the existing
causal worker rebuilds and certifies the seven-state problem from the current
serialized command. Gate A, exact wall/opponent proof, and publisher authority
were not bypassed or weakened.

## Static verification

- `git diff --check`: passed.
- source-contract pytest: `81 passed`.
- `make autoware-build`: `25 packages` completed.
- package CTest: `54/54` passed.

## Dynamic verification

Run: `output/20260830-035918`

Episode 1 completed the full lifecycle:

1. Gate A accepted `tactical_input=preentry-selection`, side `+1`;
2. `Idle -> ShiftOut` at waypoint 129;
3. `ShiftOut -> Pass` at waypoint 136;
4. `Pass -> Return` at waypoint 146;
5. `Return -> Idle` at waypoint 154.

During active execution, Gate A consumed
`tactical_input=active-same-side`. Candidate absence was no longer the silent
pre-entry/runtime producer disconnect: the log classified the current-world
result as either dynamic blocked (`reserve=-0.003`) or solve rejected. No
cross-side runtime request occurred in this run, so cross-side ownership is
covered by the pure resolver and source-contract tests, while the exact frozen
world supplies the offline acceptance evidence.

Episode 2 is a separate failure class. Gate A accepted a current-world
pre-entry proposal and ShiftOut started at waypoint 307, then approximately
2.1 seconds later the exact runtime monitor reported
`actual footprint wall margin violated` at waypoint 318. This is not the
producer/admission disconnect fixed in this Slice. It must be frozen and
compared independently to determine whether entry proof horizon, trajectory
tracking, or wall-coordinate/model agreement is the first invalid edge.

## Comparison with `.steering/ano`

The reference GMPCC log continuously reports one current solution with
opponent state and solve status, instead of retaining a long tactical geometry
through a disconnected certification path. This Slice narrows that structural
gap: the active tactical request is now the object certified by the current
world Gate A worker. It does not close the quality gap. The current run had one
complete overtake and one wall Recovery, whereas the reference behavior is
characterized by repeated continuous replanning without an equivalent Mission
wall abort in the inspected passages.

## Remaining concern / next Slice

Freeze Episode 2 without changing parameters. Compare persistent, stateless,
rough-candidate, and offline nonlinear/multi-SQP arms on the same immutable
world. The next change is justified only after identifying whether the first
invalid edge is candidate geometry, proof horizon/model mismatch, or live
trajectory execution.
