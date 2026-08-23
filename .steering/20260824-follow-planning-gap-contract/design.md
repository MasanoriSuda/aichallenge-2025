# Design

## Observed failure

The run `output/20260824-051821` contains 611 Follow retained
`stage-gap-violation` outcomes. Of these, 600 reject at stage 19, 9 at stage 18
and 2 at stage 12. The rejected minimum gap averages 1.943 m while the physical
hard gap is configured as 2.05 m. At the same decisions the measured front gap
is commonly 9.7--14.0 m.

This refutes a current-measurement hard-gap violation and an async compute-age
failure. Worker result age is normally 0.020--0.035 s and compute time is a few
milliseconds. The failure is created at the end of the planned horizon.

## Root cause

`FollowLongitudinalContractRequest` has two distinct distances:

- `desired_gap_m`: the nominal Follow objective (4.0 m);
- `hard_gap_m`: the physical fail-closed boundary (2.05 m).

The reference uses `desired_gap_m`, but both the progress upper bound and the
explicit five-state Follow constraint use `hard_gap_m`. Terminal progress
reward can therefore consume all nominal reserve and solve on the physical
boundary. The next current target tube moves by roughly 0.1 m and the retained
plan correctly fails the unchanged hard certificate. Because Follow production
is asynchronous, that correct rejection immediately becomes emergency
authority, creating the observed accelerate/brake alternation.

The problem is not that retained proof is too strict. It is that planning and
physical failure limits share one semantic field.

## Structural change

Add `planning_gap_m` to `FollowLongitudinalContract`:

- set it to the existing `desired_gap_m` when that reserve is available, or
  the current observed gap when already inside the nominal gap;
- keep theta-only progress bounds generic and encode Follow separation once as
  the explicit `theta + e_lag <= target_progress - planning_gap_m` constraint;
- construct the explicit QP Follow inequality with `planning_gap_m`;
- continue evaluating `FollowEffectiveGapCertificate` and retained
  `FollowDynamicObstacleObservation` with `hard_gap_m`.

Thus the solver plans at the configured nominal distance while the independent
physical certificate remains fail-closed at the configured hard distance.
No new tuning surface is introduced.

The target tube is also rebased once at contract creation from the ego-relative
V2X gap to the MPCC progress origin by adding the initial Frenet lag. Retained
proof receives both that origin-relative tube and the separately sealed
ego-relative current gap; it no longer overloads one vector with both meanings.

## Rejected alternatives

- Add tolerance to retained proof: hides an actual physical violation.
- Lower the hard gap: parameter tuning and reduced safety, not root repair.
- Ignore terminal stages: destroys horizon certification.
- Reuse the old plan after rejection: stale authority without evidence.
- Add a retry/fallback: preserves the contract defect and adds another owner.

## Data flow after repair

V2X ego-relative target gap + initial Frenet lag
→ one origin-relative target tube
→ Follow contract (`planning_gap_m`, `hard_gap_m`)
→ QP feasibility at planning gap
→ fresh physical certificate at hard gap
→ immutable canonical plan
→ current target-tube revalidation at hard gap
→ retained authority only if still physically valid.
