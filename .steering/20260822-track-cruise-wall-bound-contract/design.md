# Design

## Initial causal chain under test

```text
safety_margin_scale = 0
  -> raw path lb/ub become centre-point QP bounds
  -> five-state solve can legally place e_y near either raw boundary
  -> non-zero e_psi projects front/rear body length laterally
  -> exact oriented-footprint certificate finds occupied wall cells
  -> candidate is rejected despite a large scalar e_y reserve
```

This is an interface-contract limitation, not evidence that OSQP violated its accepted lateral rows.
The A/B did not remove the failures, so it is not the primary defect fixed by this slice.

## A/B experiment

The first experiment changes both local and cloud YAML from `safety_margin_scale: 0.0` to `1.0`,
rebuilds, and runs the same single-car `make dev` scenario. This is a temporary diagnostic change:
the Track/Cruise five-state output remains shadow-only, while production legacy MPC also sees the
narrower path and therefore lap/contact behavior must be recorded rather than inferred.

After the run the temporary YAML change is removed before any structural implementation is chosen.

## Measured progress-frame defect

Failure diagnostics were extended with nominal reference progress, solved progress and their delta.
The failing states were certified in a course frame 1.0--1.9 m ahead of the frame selected by the
five-state solution:

```text
five-state QP state (e_y, e_psi, theta)
  -> theta selects the physical course frame
  -> old certificate discards theta and uses ref_wp_id + stage
  -> the same e_y/e_psi is attached to a different curved-track pose
  -> false candidate hard-wall contact
```

## Adopted structural design

`EffectiveStageGeometry` is converted into a typed, strictly increasing set of
`CourseFrameKnot`s. Physical proof samples world `x/y/yaw` at every solved progress value and then
applies solved `e_y/e_psi`. The certificate fails closed with `CourseFrameUnavailable` when the
mapping is missing, non-finite, non-monotonic or outside the represented window.

OSQP may accept an equality residual within its reported tolerance. The course-frame sampler uses
that same accepted metre-domain tolerance only to clamp a solved progress value just outside an
endpoint; it does not introduce an unrelated magic epsilon.

The fixed scalar QP bounds remain explicitly a centre-path constraint in this slice. The exact
oriented-footprint certificate remains the physical oracle. Moving a nonlinear footprint envelope
inside the optimizer is not smuggled into this fix.

The current production pose to the first solved pose remains a separate swept reachability proof.
After the progress-frame fix it is the remaining candidate-side failure at wp260 and is deliberately
not hidden by relaxing the wall certificate.

## Rejected structural directions

### A. Canonical body-envelope bounds

Keep legacy path bounds and their aggression setting unchanged. Construct a separately named
canonical lateral-bound vector for the five-state problem from the physical vehicle envelope. The
problem context/fingerprint and diagnostics must identify that schema. The exact oriented-footprint
certificate remains the final oracle.

### B. Global safety margin config

Set the existing scale above zero for both legacy and canonical control. This is simple but mixes a
legacy performance parameter with the canonical safety proof and changes production behavior before
Track/Cruise authority promotion. Use only as diagnostic evidence, not as the default architecture.

### C. Exact nonlinear footprint inside the optimizer

This is the most direct model but is outside the current linear QP and two-week migration scope.
Earlier local linearizations were neither conservative nor real-time safe. Retain as future work.

## Acceptance discipline

No threshold is adjusted solely to make the certificate percentage look better. The selected design
must explain which physical region every QP lateral bound proves and must preserve reason-separated
current-pose and swept-path failure provenance.
