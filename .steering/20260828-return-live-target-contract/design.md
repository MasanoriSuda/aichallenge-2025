# Design

## Root cause

The shared canonical identity currently treats `Return` exactly like
`Follow`, `ShiftOut` and `Pass`: a non-empty target ID and a current target
observation generation are both mandatory.

That contract is correct before rear-clear, but wrong after a successful pass.
Return still needs the immutable encounter identity, Mission generation and
homotopy so phase ownership cannot be borrowed. It no longer needs the passed
vehicle to remain the tactical active target. The current-world dynamic proof
already checks every presently observed peer independently of that tactical
label.

The visible stop was downstream:

```text
rear-clear
-> behavior drops completed target
-> Return problem identity has target ID but no target generation
-> fresh Return submission rejected
-> one previously certified Return artifact exhausts
-> no canonical Return authority
-> Emergency Stop / stuck Recovery
```

The initial Return solve and physical certificate prove that neither candidate
generation nor physical infeasibility caused this event.

## Repair

Split the target contract into two predicates:

1. `canonical_normal_intent_requires_target()` means that a stable encounter
   ID is part of the semantic identity. It remains true for Follow, ShiftOut,
   Pass and Return.
2. `canonical_normal_intent_requires_target_observation()` means that the
   identity is incomplete without a current target-obstacle generation. It is
   true for Follow, ShiftOut and Pass, but false for Return.

`problem_context_complete()` uses the first predicate for target ID and the
second for observation-generation completeness. `make_problem_context()`
continues to copy Return's target ID from the canonical authority trace and
copies a target generation only when current provenance exists.

No dynamic obstacle is removed from the current-world proof. Return artifacts
remain subject to current wall, current peer footprint, progress, actuator and
course-frame validation.

## Existing patch relationship

The 2026-08-23 canonical-intent Slice correctly prevented ShiftOut/Pass plans
from being targetless, but generalized that live-observation rule to Return.
Later Return fallback patches masked the resulting finite-artifact lifetime.
This Slice corrects the upstream identity contract rather than extending the
artifact lifetime or adding another Return fallback.
