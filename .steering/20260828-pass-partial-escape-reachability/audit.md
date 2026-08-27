# Audit: Pass partial-escape reachability

## Observed phenomenon

In `output/20260828-011708`, Pass snapshot 1594 is rejected by the
dynamic-obstacle refinement before the later SafetyBrake/Recovery sequence.
Warm and cold replay fail identically.

## Causal chain

1. The pre-pass stay-behind branch has an empty intersection with the current
   state box, so the selected physical side must continue laterally.
2. The wall-only seven-state solution is physically delayed by steering and
   yaw response and initially loses about 3.5 cm of signed separation.
3. The partial-escape producer replaces that demonstrated value with
   `max(initial separation, wall-only separation)`.
4. The generated obstacle row therefore requires an instantaneous
   non-decrease that the witness trajectory does not demonstrate.
5. Exact LP isolation is infeasible with the six obstacle rows and feasible
   when they are removed. The live QP consequently has no valid Pass artifact,
   and the persistent Mission later reaches Emergency/Recovery.

## A--D evidence

- A warm/cold: both fail at the same dynamic-obstacle refinement.
- Independent affine feasibility: full problem infeasible; removing only the
  dynamic-obstacle rows restores feasibility.
- D bounded exact-dynamics search: the best 64-start result still violates a
  hard row by 3.54 cm. This is not a physical-infeasibility proof, but it
  agrees that the recorded instantaneous envelope is unreachable.
- B/C lifecycle and candidate comparisons remain open for the later
  current-Mission retention failure; this repair does not classify or mask
  that separate defect.

## Root cause

The constraint producer did not implement its documented witness contract.
It combined the wall-only reachable envelope with a stronger measured-state
monotonicity assumption. The measured stage-zero separation is not a
reachability certificate for stage one.

## Repair

Use the wall-only signed separation itself until it reaches the unchanged full
physical separation. The wall-only trajectory remains the sole witness for
the convexified side branch. No clearance, solver setting, tolerance, horizon,
lease, timeout, fallback or authority is changed.

## Remaining risk

The same episode later preserves a Mission after current-world viability has
reported a wall/target conflict. Hold and Return are still schema-only in the
Race MPCC shadow. That lifecycle/candidate question must be evaluated in the
next architecture comparison rather than hidden by this producer repair.

## Verification

- Focused dynamic-obstacle test: 10/10 passed, including an initial
  steering-lag regression where the witness separation decreases.
- `make autoware-build`: 25 packages built successfully.
- Full package suite: 49/49 targets, 2,005 tests, zero failure. The existing
  stale `joycon_contract_guard/package.xml` result warning remains unrelated.
- `make dev2`: `output/20260828-014301`.
  - partial-escape contracts with 20 rows reached `solved=1`;
  - the old sequence-1594 instantaneous-envelope failure was not reproduced;
  - no Pass/Return was reached;
  - later episodes repeatedly reported `physical target separation conflicts
    with wall bounds` while `canonical_action=preserve-mission` remained;
  - Hold and Return still reported
    `schema-ready/*-solve-not-yet-unified`.
- dev3 was not run because the two-vehicle integration Gate remains open.

The dynamic result confirms the repaired producer but also proves that the
next blocker is not another separation parameter. The next Slice must compare
a current-world stateless continuation/Hold/Return bundle against preserving
the persistent Mission.
