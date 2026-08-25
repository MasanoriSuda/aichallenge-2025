# Validation

## Evidence boundary

- Branch: `develop_july`
- Baseline: `490f095 refactor(mpcc): preserve rate-resolved source context`
- First production run: `output/20260825-094733`
- Corrected production run: `output/20260825-100454`
- Domains: d1 and d2
- User-owned `aichallenge/result-summary.json` was not modified or staged by
  this Slice.

## Observed failure before the root fix

The first production run proved that a six-state retained proof reached the
publisher, but the exact canonical command identity was rejected. The logs
contained `canonical normal command mutated before publication` four times in
d1 and 1,885 times in d2.

The first differing field was acceleration. For example, the certified solver
result was approximately `1.3700004119 m/s2`, while the publisher's
unconditional physical clamp changed it to `1.37 m/s2`. A second boundary case
allowed a microscopic negative virtual-progress input inside OSQP's accepted
row residual although the original physical lower bound was zero.

## Causal chain

```text
physical actuator/input envelope
  -> identical solver row at the exact physical boundary
  -> OSQP accepts a small row residual by the existing certificate tolerance
  -> immutable execution artifact retains the slightly out-of-envelope value
  -> publisher clamps the certified acceleration
  -> final command no longer matches the certified solution
  -> exact identity guard rejects the normal authority
  -> explicit EmergencyStop
```

The root cause was therefore a producer/consumer boundary mismatch: a
residual-tolerant numerical certificate and an exact physical publication
contract shared the same boundary. The publisher clamp was a downstream mask,
not the correct owner of the repair.

## Structural fix

- Solver-facing velocity, acceleration, steering-rate and virtual-progress
  rows are inset by the maximum residual already authorized by the existing
  physical OSQP tolerance.
- Original physical acceleration and virtual-progress bounds are preserved in
  the execution artifact and validated exactly.
- The canonical publisher bypasses acceleration and steering post-processing;
  any future mutation remains visible through the unchanged exact identity
  guard.
- Noncanonical and Recovery output clamps are unchanged.
- No physical parameter, OSQP setting, timeout, fallback flag or ROS contract
  was changed.

## Static validation

- `make autoware-build`: passed, 25 packages.
- Package CTest: 49 targets, 1,870 tests, zero errors/failures/skips.
- `test_single_authority_source_contract.py`: 33 passed.
- `git diff --check`: passed.

Failure-first coverage includes:

- six-state retained proof to exact canonical production command;
- final execution trace accepting the exact six-state formulation identity;
- exact rejection of acceleration and virtual-progress values outside the
  original physical envelope;
- rejection when the physical interval is too narrow for the certified solver
  inset;
- source-contract proof that Track/Cruise cannot call the five-state normal
  evaluator and that a canonical command is not post-processed.

## Dynamic validation after the root fix

In `output/20260825-100454`:

- publisher mutation count was zero in d1 and d2;
- d2 repeatedly reported `attempted=81/available=81/production_canonical=81`;
- production command deltas were exactly zero for speed, acceleration and
  steering;
- final decision traces reported
  `formulation=velocity-steering-progress-6state`,
  `authority=certified-normal-solution` and `canonical=satisfied`;
- d1 used the six-state owner during Track/Cruise and transferred to the
  unchanged five-state Follow owner when it caught d2;
- no fatal error or segmentation fault was observed.

This is sufficient to accept the Track/Cruise authority migration itself.

## Residual risks kept visible

1. An isolated Track/Cruise cycle failed closed because the current dynamic
   world observation was unavailable. Age-only reuse is prohibited. A later
   Slice must define provenance and uncertainty/reachability for a retained
   dynamic observation before increasing availability.
2. d1 recorded 12 callback-overrun detail events while in traffic. The
   examples attribute most time to `MPC::get_control()` plus Recovery
   evaluation. d2 recorded none. This timing tail is not evidence of a
   command-identity or postprocessor defect and is not masked in this Slice.
3. The old five-state Track/Cruise helper implementation remains physically in
   the translation unit but has no reachable Track/Cruise production owner.
   Its physical deletion belongs to Slice 6 and must not restore a normal
   fallback.

## Next acceptance checks

- Search must continue to show no Track/Cruise call to the five-state normal
  evaluator.
- A repeated run must keep canonical mutation and cross-formulation fallback
  at zero.
- Dynamic-observation rejection and callback timing must be measured as their
  own causes, not converted to retained-by-age authority or parameter tuning.
