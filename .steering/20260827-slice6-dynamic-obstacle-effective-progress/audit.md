# Audit: Slice 6 dynamic-obstacle effective progress

## Observed phenomenon

In `output/20260827-204645/d1/autoware.log`, a valid Cruise artifact was
temporarily retained after behavior changed to Follow.  The first Follow
dynamic-obstacle solve selected a full-horizon partial lateral escape, reached
the solver iteration limit, and left no normal authority when the Cruise cursor
expired.  The visible result was Emergency braking from about 3.35 m/s.

## Causal chain

1. The canonical wall-only Follow solution had raw virtual progress
   `theta=0.743 m` and a negative lag state.
2. Dynamic-obstacle branch selection compared `theta` alone with the target
   longitudinal boundary and reported `behind_margin=-0.592 m`.
3. The same formulation defines physical ego progress as `theta + e_lag`.
   Ignoring the negative lag therefore created a false longitudinal overlap.
4. The refinement replaced the longitudinal Follow branch with 20 lateral
   partial-escape rows.
5. That unrequested homotopy did not solve before the retained Cruise cursor
   expired, so the external Emergency supervisor became the only authority.

## Root cause

The Slice 6 dynamic-obstacle module reintroduced a retired coordinate contract:
it used virtual progress `theta` both for branch classification and for the QP
longitudinal row.  The canonical Follow gap certificate, retained physical
revalidation, and vehicle geometry use physical progress `theta + e_lag`.

## Evidence

- `race_mpcc_foundation` documents and enforces
  `theta + e_lag <= target_progress - planning_gap`.
- Retained current-world revalidation reconstructs physical course progress as
  `course_origin + progress + lag`.
- The failure-first unit case uses rising raw `theta` with `e_lag=-1.0 m`.
  The old rule selects a partial lateral escape; the physical rule proves a
  four-stage stay-behind branch.
- `output/20260827-211306/d1/autoware.log`, after race Start, contains 38
  dynamic-obstacle Follow contracts: 38 solved, 0 rejected, 0 Emergency, and 0
  normal fallback.  Diagnostics independently show raw and effective progress.

## Relationship to existing patches

Cruise retention, receding-prefix revalidation, solver retry, timing grace, or
another Follow fallback would only have prolonged the preceding authority.
The partial-escape policy itself remains valid for a physically overlapping
state.  This Slice removes only the theta-only longitudinal interpretation; it
does not weaken obstacle, wall, or lateral-separation protection.

## Repair

- Replaced the representable `Progress` dynamic-obstacle axis with
  `EffectiveProgress`.
- Assembled its affine row with unit coefficients on both `theta` and `e_lag`.
- Applied the same physical coordinate to branch classification and state-box
  feasibility checks.
- Logged raw `theta` and effective progress separately.
- Added deterministic classification and QP-row regression tests.

## Removed or consolidated processing

The theta-only obstacle row and its row-semantic type were physically removed.
No compatibility alias, fallback, timeout, lease, solver tuning, or parameter
exception was retained.

## Verification

- `make autoware-build`: 25 packages passed.
- Focused QP and dynamic-obstacle tests: 2/2 passed.
- Package CTest: 47/47 targets passed, 1,987 tests with no failure.
- `make dev`, `output/20260827-211056`: post-Start normal Cruise authority,
  no Emergency, solver fallback, recovery, or callback overrun.
- `make dev2`, `output/20260827-211306`: d1 post-Start dynamic contracts
  38/38 solved; no Emergency, fallback, or callback overrun.  d2 had no dynamic
  obstacle episode and recorded three isolated callback overruns.
- All containers were stopped after both gates.

## Remaining concerns

- Before the recorded race-Start edge, a real near-field SafetyBrake episode
  returned to Follow for one decision without a certified normal artifact.  Its
  obstacle margin was physically negative, so this is not the coordinate defect
  repaired here.  It remains a separate authority-transition audit.
- Domain 2 recorded three isolated 25 ms callback overruns.  No command failure
  followed, but callback-tail quality remains an integration-gate item.
- This bounded run did not exercise every ShiftOut, Pass, and Return admission.

## Next-run observations

- Freeze the first post-SafetyBrake Follow decision and identify whether the
  missing artifact starts at snapshot production, async completion, retained
  proof, or final authority selection.
- Continue recording raw/effective progress and require every longitudinal
  obstacle row to report the same physical margin as Follow revalidation.
- Re-run multi-vehicle gates long enough to cover ShiftOut, Pass, and Return;
  classify any failure before changing parameters.
