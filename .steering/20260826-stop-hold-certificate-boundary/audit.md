# Audit

## Expected and observed behavior

Expected: once odometry and the race session are active, stationary Track or
Cruise produces a certified six-state plan and a positive normal acceleration.

Observed in `output/20260826-095203`: both domains solve continuously but every
result is `artifact-rejected/invalid-control-stage`.  The certified store stays
empty and production publishes the typed Emergency Stop.

## Authority graph at the incident

```text
odometry / course / intent
  -> six-state semantic request: available
  -> rate-resolved adapter: accepted
  -> OSQP: solved and finite
  -> steering sample: accepted
  -> execution artifact validation: invalid-control-stage  <-- first failure
  -> physical wall proof: not evaluated
  -> candidate/executed store: empty
  -> current-world join: missing plan
  -> Emergency Stop: published
```

## Root, contributor, mask, and detection gap

- Root: virtual-progress bound is revalidated with zero tolerance after a
  solver-certified singleton row.
- Contributor: stationary launch creates `[0,0]` stop/hold stages, so the
  mismatch occurs on every solve rather than occasionally.
- Mask: Emergency Stop correctly prevents uncertified execution but makes the
  symptom look like a launch/control-mode failure.
- Detection gap: the adapter singleton tests stop before solver artifact
  validation; the artifact test explicitly encoded the incompatible
  exact-zero assumption.
- Recovery: not involved; race launch confirmation times out because motion
  never begins.

## Hypotheses considered

1. AWSIM start/control-mode handshake failure: falsified by `Ready`, active race
   session, fresh odometry, a subscriber to the engage request, and continuous
   solver submissions.
2. OSQP infeasibility: falsified by solved/finite results and accepted
   actuation sampling before artifact rejection.
3. Steering-rate physical violation: low confidence and inconsistent with the
   logged rate remaining inside the physical +/-0.7 rad/s envelope.
4. Singleton virtual-progress residual mismatch: high confidence from the
   exact reject location, `83edbba` bound change, and the stale strict test.

## Pre-fix replay/test

Create an artifact with a virtual-progress singleton `[0,0]`, a small
row-certified residual inside `physical_global_tolerance`, and consistent
progress dynamics.  It must be accepted.  A second value beyond that tolerance
must remain rejected.

The new regression failed against `992be2a` at the inside-tolerance case:

```text
MpccRateResolvedPhysicalAdapter.AppliesCertifiedToleranceToInternalProgressInput
Expected: RejectReason::None
Actual:   RejectReason::InvalidArtifact
```

The same test passes after making artifact validation use the sealed physical
tolerance for the internal progress input.  The beyond-tolerance case remains
rejected as `InvalidControlStage`.

## Static verification

- `make autoware-build`: passed, 25 packages.
- focused virtual-progress regression: passed, 1/1.
- focused predicted-velocity regressions: passed, foundation 1/1 and adapter 2/2.
- complete foundation suite: passed, 35/35.
- complete physical-adapter suite: passed, 7/7.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- `colcon test-result --verbose`: 1909 tests, 0 errors, 0 failures,
  0 skipped.
- single-authority source contract: passed, 56/56.

## Dynamic pre-start evidence

Run `output/20260826-101100` was started with `make dev2`.  Before the AWSIM
race Start transition:

- neither domain emitted the previous `artifact-rejected` /
  `invalid-control-stage` signature;
- d2 reported `production_reason:available`, `emergency:0`, and positive
  canonical acceleration from the six-state formulation;
- d1 passed physical certification and current-world joining after the
  Track-to-Cruise transition (`joined=1`, sequence 320).

The standard `/admin/awsim/start` publisher could not find a subscriber in
this first dev session, so that pre-start inspection alone could not close
moving acceptance.  The subsequent manual Start replay below completed it;
no launch workaround or alternate controller was added to this Slice.

## Predicted-stop failure exposed by the first replay

After manual Start, both vehicles initially moved.  d1 later stopped when
fresh Follow/Cruise artifacts were rejected one boundary later:

```text
solver outcome=solved
artifact_valid=1
physical=adapter-rejected
exact=invalid-velocity/stage=17
retained cursor=exhausted
production=Emergency Stop
```

The rejected velocity was a solver-certified lower-bound residual at a future
stop stage.  `ExactPhysicalExecutionTrajectory` had no field with which to
carry that certificate, although the raw execution artifact and the progress
state already supported certified residuals.  This was the same root contract
defect, not a Recovery or tuning problem.

The failure-first foundation test admitted `-1e-8 m/s` with a sealed
`1e-7 m/s` certificate.  Before the validator repair it failed as
`InvalidVelocity`; after repair it passes, while a `1e-9 m/s` certificate still
rejects the same raw state.  The adapter test additionally proves that values
beyond the artifact's measured solver violation are rejected upstream.

## Moving acceptance after both repairs

`output/20260826-103853` was run with `make dev2` and AWSIM Start:

- old `artifact-rejected/invalid-control-stage`: 0 in d1 and d2;
- old `exact=invalid-velocity`: 0 in d1 and d2;
- retained certified publications observed: d1 71, d2 100 at inspection;
- measured moving speeds: d1 `0.627 m/s`, d2 `3.103 m/s`.

Both domains therefore produced and executed physically certified six-state
plans instead of remaining at zero.  Intermittent command rejection and an
unwanted Recovery episode remain separate integration-quality evidence for a
later Slice; they did not recreate either certificate-boundary defect.
