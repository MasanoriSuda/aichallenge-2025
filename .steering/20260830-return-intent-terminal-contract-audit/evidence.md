# Evidence ledger

## Frozen boundaries

- Review baseline: `831232dcf4d7c1baffab7b38da7f500d7d40b9ee`.
- Root-cause run: `output/20260830-053105`, Domain 1/2.
- Additional performance run: `output/20260830-083733`, Domain 1/2.
- Upper comparison: `.steering/ano/autoware - 2026-08-21T211659.829.log`.

The `083733` run does not contain a commit marker. Its filesystem timestamps
are compatible with the uncommitted Return work, but that does not prove the
binary provenance. It is performance evidence only and is not used as causal
acceptance for this Slice.

## Return root cause and repair mapping

| Item | Evidence |
|---|---|
| Earliest violated invariant | One canonical `Return` intent had two meanings: the supervisor requested racing-line rejoin while the generic stateless producer rewrote the reference to the old pass side. |
| Invalid producer | Generic `build_bounded_candidates(source, execution_side_sign)` Return edge. |
| Visible downstream symptoms | Return authority loss, Emergency alternation, steering-unreachable delay, and eventual static-wall contact. |
| Mask/recovery | Emergency and Recovery preserved safety after the valid Return branch had already been lost; they were not the root cause. |
| Root repair | Dedicated current-world Return relation, preserved Return reference, explicit longitudinal topology, and solved-terminal semantic certificate. |
| Removed production edge | Return no longer uses the generic pass-side population. ShiftOut and Pass remain unchanged. |
| Added exceptional runtime paths | Zero. No timeout, retry, lease, fallback, clearance, weight, or solver setting was added. |

## Static verification

- `make autoware-build`: passed, 25 packages, 2026-08-30.
- Focused CTest: 5/5 passed:
  - `test_mpcc_rate_resolved_dynamic_obstacle`
  - `test_mpcc_rate_resolved_shadow`
  - `test_mpcc_stateless_maneuver`
  - `test_mpcc_rate_resolved_retained_revalidation`
  - `test_mpcc_rate_resolved_command_candidate`
- Full package CTest after restoring the original layout and reapplying the
  semantic delta: 55/55 targets passed.
- `test_single_authority_source_contract` passes without weakening any source
  assertion. The accidental repository-wide formatting divergence is gone.

## Temporal proof defect found during acceptance

The first rebuilt dynamic trial still rejected a physically solved Return as
`terminal-intent-not-reached`.  The Return contract describes the endpoint of
the complete 20-stage solve, but the validator was comparing it with
`ExecutionArtifact.predicted_states.back()`.  That state is only the end of
the short publisher prefix and is deliberately earlier than the Return
endpoint.

This was a proof-scope mismatch, not a solver or clearance failure.  The
artifact now carries an immutable terminal certificate read from the complete
solve primal.  Validation requires its solved horizon to equal the problem
horizon and checks the Return contract against that full-horizon endpoint.
The executable prefix remains short and unchanged.

- Cause: full-horizon semantic contract checked at prefix time.
- Repair: full-horizon producer and validator now use the same temporal scope.
- Added timeout, lease, retry, fallback, tolerance or clearance: none.
- Regression test: a two-stage publisher prefix is accepted only when its
  20-stage source solve certificate reaches the Return endpoint.

## Provenance-bound dynamic verification

Run: `output/20260830-101331`, rebuilt Domain 1/2 artifact.

Two independent episodes completed the canonical chain:

| Episode | Chain | Elapsed | Minimum speed | Minimum wall reserve | Result |
|---|---|---:|---:|---:|---|
| 1 | ShiftOut -> Pass -> Return -> Idle | 4.89 s | 3.04 m/s | 6.02 m | `return handoff converged` |
| 3 | ShiftOut -> Pass -> Return -> Idle | 6.22 s | 2.88 m/s | 5.22 m | `return handoff converged` |

The run contains two `Return -> Idle` transitions and zero occurrences of
`terminal-intent-not-reached`, `actual footprint wall margin violated`, or
`actual footprint intersects static wall`.  This accepts the frozen Return
failure family dynamically.

Episode 2 is deliberately not counted as a Return regression.  It changed
`FollowPrepare -> Recovery` before Pass with reason
`static wall clearance margin infeasible`.  It is an independent ShiftOut
admission/wall-feasibility failure and receives no patch in this Slice.

Replaying the old sequence-1599 snapshot continues to reject.  That is
expected: its serialized upstream Return request already contains the old
pass-side reference.  The replay proves the validator was not weakened; the
repair replaces the live producer so new current-world snapshots no longer
carry that polluted semantic request.

## Lap and overtake loss ledger

### Current run (`083733`, Domain 1)

- Laps: `100.260`, `85.892`, `85.023`, `95.617` s.
- Best / median / mean: `85.023 / 90.755 / 91.698` s.
- Overtake entries: 9 `Idle -> ShiftOut` and 9 `ShiftOut -> Pass`.
- Rear-clear transitions: 5 `Pass -> Return`.
- Clean `Return -> Idle` handoffs: 2.
- Clean complete rate: 2/9 (22.2%).

Observed terminal paths included Pass-entry wall-gate rejection, actual wall
margin violation, SafeSeparation time limit, missing current-side prefix,
SafetyBrake, and external Recovery. Therefore no current lap is a valid
free-running controller benchmark.

### Upper run (`.steering/ano`)

- Laps: `54.067`, `59.849`, `48.256`, `39.959`, `55.393`, `56.763` s.
- Best / median / mean: `39.959 / 54.730 / 52.381` s.
- Current best is `45.064` s slower (2.128x).
- Current median is `36.025` s slower (1.658x).

The gap is too large to defer until fine tuning. It must first be partitioned
into free-running loss and traffic/authority/fallback loss; tuning before that
partition would mix structurally different causes.

## Follow-up, outside this Slice

1. Classify the independent `FollowPrepare -> Recovery` wall-feasibility
   episode without changing Return.
2. Audit one-cycle normal-authority gaps visible outside the completed Return
   episodes.
3. Open Pass-wall and alternate-branch work only as separate frozen Slices.
4. Run a longer race acceptance before parameter tuning; this short run is
   causal evidence, not a lap-performance benchmark.
