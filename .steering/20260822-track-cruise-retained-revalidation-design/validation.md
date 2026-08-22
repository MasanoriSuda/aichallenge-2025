# Validation

## Observed phenomenon

No retained runtime failure was claimed or patched in this slice.  The source audit found a
structural proof gap before retained execution is connected: the retained candidate API can name
an exact old plan/window, but it cannot represent the current physical world which supposedly
revalidated that window.

## Problem propagation

```text
complete retained five-state plan exists
-> exact time cursor identifies remaining control stages
-> current geometry and obstacles have advanced independently
-> revalidation summary has no current observation/geometry/pose identity
-> old/new stages can be paired by index instead of progress and time
-> a PhysicalCertificate summary can certify the wrong physical horizon
-> future retained authority could publish a stale command
```

## Root cause

`CanonicalExecutionRevalidation` is a summary/window contract, not a sealed
current-observation physical proof.  Static spatial geometry and dynamic temporal occupancy also
lack a required dual-axis alignment contract.

## Source evidence

- Complete retained state/control plan: `canonical_execution_plan.hpp:18-43`.
- Exact non-clamping cursor: `canonical_execution_plan.cpp:142-180`.
- Incomplete revalidation type: `canonical_execution_plan.hpp:156-163`.
- Candidate accepts plan/window plus physical booleans: `canonical_execution_plan.cpp:326-376`.
- Current context already owns ego/geometry/target generations:
  `mpcc_execution_contract.hpp:173-189` and `mpc_controller_cpp.cpp:23322-23365`.
- Current course-frame sampler fails closed by absolute progress:
  `mpc_stage_geometry.cpp:73-145`.
- Fresh world-footprint and swept-wall proof:
  `mpc_controller_cpp.cpp:20265-20595`.
- Dynamic obstacle occupancy is predicted using current age and per-stage horizon time:
  `mpc_controller_cpp.cpp:4268-4355`.
- Those current obstacle corridors are later intersected into current stage bounds:
  `mpc_controller_cpp.cpp:17673-17713`.
- Fresh shadow proof/candidate construction occurs within one callback only:
  `mpc_controller_cpp.cpp:21286-21472`.

History confirms the type was introduced as plan-store scaffolding in `56d8a39`; later commits
added fresh admission and solved-progress wall fidelity, but no current retained-world producer was
added.  This is an unfinished migration boundary, not evidence that old certificates are reusable.

## Existing patch relationship

- `valid_until_sec` bounds age but does not prove the world and observation are unchanged.
- `execution_certificate_decision_id` proves that somebody asserted a current decision; without a
  sealed input fingerprint it does not prove what was checked.
- `stage_geometry_id` in the old problem fingerprint proves old solve geometry, not current
  progress alignment.
- Existing fresh swept-wall validation is useful implementation material, but its current call
  begins from a controller member pose and consumes same-horizon arrays; it cannot be invoked on a
  retained window by merely slicing indices.
- Shadow-only selection (`authority=shadow, selected=0`) currently masks this from final output and
  is intentionally preserved.

## Chosen correction

Adopt a typed, sealed dual-axis retained revalidation proof:

- exact elapsed-time cursor and partial first-stage duration;
- current ego/target/geometry/control-pose provenance;
- progress-aligned current course/wall sampling;
- time-aligned current dynamic-obstacle tube sampling;
- measured-to-predicted delay-prefix plus predicted-to-horizon swept proof;
- exact proof fingerprint consumed by candidate construction.

No code implementation is performed until dynamic Gate A establishes that the fresh canonical
pipeline is complete under a real run.  This avoids hiding an upstream fresh-path defect behind a
new retained fallback.

## Checks performed in this design slice

- Source and history audit completed.
- Options and causal chain recorded.
- Ten deterministic failure-first cases defined.
- `git diff --check`: passed.
- Build/tests: not required because this slice changes documentation only.
- Runtime/authority/configuration delta: none.

## Remaining concerns

- Fresh canonical dynamic counters after `5bae30b` remain unmeasured.
- The exact current dynamic-obstacle tube API must be factored from the existing gap-planner
  prediction without duplicating its motion model.
- Circular progress lifting needs deterministic ambiguity limits; it must not silently hide a
  localization discontinuity.
- The command-application pose identity is not currently a first-class generation and must be
  introduced with the pure proof contract.
- Track/Cruise final publishing remains legacy until explicit Gate C approval.

## Next run and decision

The next user-started run is Gate A, not a parameter experiment.  Run clean `make dev` for at least
two laps and verify:

```text
physically certified == canonical extracted == canonical stored
                     == cursor available == candidate accepted
                     == fresh authority ready == actuation extracted
actuation_diff == 0
authority=shadow
selected=0
```

Also record solve/certificate/callback p95, p99 and maximum.  Any count mismatch is repaired in the
fresh producer before retained revalidation implementation begins.
