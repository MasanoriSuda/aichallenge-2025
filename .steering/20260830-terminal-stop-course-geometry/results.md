# Results

## Root cause confirmed

The normal `ExecutionArtifact` is intentionally limited to a short executable
prefix.  Terminal maximum-braking proof nevertheless sampled the complete
Stop from that prefix and clamped every query after its end to the last prefix
stage.  This conflated two different horizons:

- command horizon: the immutable prefix which may receive normal authority;
- contingency horizon: the physical course support needed to reach zero
  speed.

The observed Emergency Stop was downstream.  The upstream failure was an
expired tactical curvature/lateral interval being treated as physical course
geometry.

## Implemented correction

- Added `StopCourseGeometry` containing full planning-horizon progress,
  curvature and physical lateral support.
- Sealed that geometry into the immutable physical-wall snapshot assembled
  from the same current solver problem and wall profile as the normal plan.
- Changed terminal Stop synthesis to consume the separate geometry and reject
  progress outside its support instead of extrapolating the final prefix
  stage.
- Routed retained revalidation and the frozen A/B/C/D architecture comparison
  through the same physical source.
- Made an incomplete terminal geometry invalidate the physical snapshot.
- Removed the old Stop sampler's dependency on artifact predicted-state
  progress and tactical lateral bounds.

No authority rule, clearance, solver tolerance, braking limit, Mission rule,
lease, timeout or fallback was changed.

## Static verification

- `make autoware-build`: passed, 25 packages built.
- Direct physical-adapter test: 20/20 passed, including:
  - braking beyond the short prefix with full course support;
  - fail-closed when the full support ends before stopping;
  - rejection when the Stop exits full physical lateral support.
- Complete package CTest: 54/54 passed.
- Source contract verifies that terminal Stop no longer samples
  `artifact.predicted_states` as course geometry.
- `git diff --check`: passed.

The first complete CTest run exposed one useful integration defect in the
architecture-comparison harness: its physical proof snapshot had not been
given the new terminal geometry.  The harness now reconstructs it only from
the immutable captured problem, matching production.  The comparison test and
then all 54 tests passed.

## Dynamic acceptance

Command:

```text
make dev2
```

Baseline: `output/20260830-011957/d1/autoware.log`  
Candidate: `output/20260830-014758/d1/autoware.log`

The logs are similar in size (3233 versus 3309 lines), so raw event counts are
useful for this focused comparison:

| Metric | Baseline | Candidate |
|---|---:|---:|
| terminal contingency unavailable | 50 | 15 |
| Stop exact invalid-lateral-bounds | 4 | 2 |
| certified terminal Stop | 66 | 77 |
| certified normal authority | 93 | 157 |
| Emergency authority | 57 | 43 |

The candidate published only `certified-normal-solution` or explicit
`emergency-override` execution contracts.  No uncertified normal authority was
observed.

Representative continuation exact-bound failures now commonly retain normal
authority because the independent terminal Stop is certified.  For example,
decisions 1693, 1741, 1758 and 1782 report continuation
`invalid-lateral-bounds` while terminal Stop is `exact:accepted`, wall-clear
and dynamic-clear.

## Remaining failures are no longer hidden

Three distinct residual failures remain and are intentionally fail-closed:

- decision 1313: terminal exact model succeeds, but the authoritative swept
  wall proof reports a real collision;
- decision 3800: the full physical lateral support rejects exact sample 76;
- decision 5442: at 7.80 m/s the full physical lateral support rejects exact
  sample 215.

These are not repaired by restoring prefix extrapolation or widening bounds.
They are evidence for later physical course/candidate-quality work.  Several
other terminal attempts are skipped because the retained artifact itself is
already invalid; that is a separate lifecycle/progress-lift family.

The run also observed `Idle -> ShiftOut` four times, but no
`ShiftOut -> Pass`.  The recurring ShiftOut physical revalidation/DynamicWait
failure is an independent overtake-quality blocker and remains outside this
Slice.

## Acceptance decision

Accepted for commit as a structural root-cause correction:

- it materially reduces false terminal authority loss;
- it preserves fail-closed physical proof;
- residual events now separate exact course-support, wall collision and
  invalid-artifact causes;
- it deletes the obsolete geometry ownership rather than adding another
  exception path.
