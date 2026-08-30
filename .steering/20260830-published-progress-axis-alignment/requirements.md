# Requirements: published progress-to-path alignment

## Objective

Remove the coordinate mismatch which rejects a still-published certified
Overtake trajectory on curves.  Measured course progress must be projected
onto the certified trajectory's monotonic path coordinate before the lateral
profile is resampled.

## Frozen evidence

Run `output/20260830-162637/d1/autoware.log` repeatedly reports
`published trajectory resampling failed` during committed Pass while the
publication cursor remains available and certified normal commands continue to
cross the publisher.  The failures begin in curved, laterally displaced
execution and precede one longitudinal-progress Recovery.

## Invariants

- Keep production authority, Mission state transitions and Emergency ownership
  unchanged.
- Do not add a lease, grace period, timeout, fallback or parameter change.
- Keep physical path distance and course progress as distinct named values.
- Course progress may plateau during lateral motion and therefore may not be
  used directly as a one-dimensional resampling axis.
- The publication clock disambiguates multiple path locations which share the
  same course progress; the mapping is owned once by the execution source.
- Projection failure remains a rejection, not a fallback to another axis.

## Acceptance

- Unit tests use deliberately different coordinates and a course-progress
  plateau to prove projection onto the publication-clock-consistent path point.
- Published Overtake alignment resamples and computes remaining coverage on
  the monotonic path axis only.
- Source-contract, build and package tests pass.
- In dev2, the old `published trajectory resampling failed` signature is absent
  or any remaining occurrence is classified independently of the former
  coordinate mismatch.
