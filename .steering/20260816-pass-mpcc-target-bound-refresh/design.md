# Design

## Current gap

The rolling DP refresh is stitched to the active prefix and then checked for:

- complete horizon coverage,
- static-wall feasibility,
- lateral-acceleration feasibility,
- target identity/continuity and current body separation.

It does not prove that the candidate lateral samples remain on the committed side of the
time-aligned predicted target while their longitudinal footprints overlap. The later receding
horizon validator detects that mismatch, but by then the candidate has already replaced the
previous DP execution reference.

## Change

Add a pure `FrenetDpTargetBoundHorizon` validator. For each active target-overlap sample:

```text
positive pass side: candidate_y >= target_y + physical_separation
negative pass side: candidate_y <= target_y - physical_separation
```

The controller builds the active mask and predicted target lateral samples with the existing
`resolve_receding_horizon_target_prediction()` model. Rear-clear samples use the existing bounds
release policy and therefore do not keep an obsolete opponent constraint.

Atomic promotion gains a `target_bound_horizon_feasible` input. Promotion occurs only when:

1. the stitched reference covers the current control horizon,
2. wall and lateral-acceleration validation succeeds,
3. the target-bound validator succeeds,
4. existing target continuity and hard-fault gates succeed.

The active path is modified only after all four checks. A failed candidate stays pending and the
last feasible execution reference remains untouched.

## Safety boundary

The validator uses physical center separation as the hard floor. Robust separation remains a
planning preference handled by the existing receding-horizon optimizer. This change therefore
prevents candidate paths from crossing the target body without turning a soft robust reserve into
a new hard rejection.

## Dynamic verification

For `make dev2`, compare:

- `ShiftOut -> Pass`, `Pass -> Return`, and `Pass -> Recovery` counts,
- `target_bound=0` rolling-candidate rejections,
- `optimized horizon escaped target separation bounds`,
- wall contact and SafetyBrake counts.
