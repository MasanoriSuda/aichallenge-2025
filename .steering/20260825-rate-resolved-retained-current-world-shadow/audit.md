# Root-cause Audit

## Observation

Fresh six-state Track/Cruise solutions can be solved, converted into an exact
physical trajectory and accepted by swept wall proof.  A monotonic retained
store now preserves that accepted artifact/result pair.

## Root cause

The store loses the physical snapshot which gave the result meaning.  In
particular, the wall-grid shared owner, footprint and course-frame knots are
not recoverable from `PhysicalWall::Result`.  Result identity alone proves
which solve was checked, not which static world was checked.

## Symptom if promoted prematurely

A fresh worker miss would invite an age-only retained fallback.  That fallback
could extract valid controls from the correct six-state artifact while using
an obsolete wall/course proof and an unreachable state join.  The eventual
wall stop would be observed downstream, hiding the missing proof boundary.

## Rejected alternatives

- Accept by age and intent: does not prove the current world or state join.
- Compare rolling `stage_geometry_id`: a new rolling horizon legitimately has
  a different ID and an equal ID still does not identify the wall raster.
- Re-run the full suffix wall proof synchronously: duplicates certified static
  work and risks the callback overruns already observed in an earlier Slice.
- Convert to the five-state retained plan: changes formulation and preserves
  the migration path this project intends to remove.

## Chosen correction

Retain the exact proof source.  At consumption, keep the old immutable suffix
certificate and prove only the current dynamic boundary: current empty-world
semantics, branch-continuous progress, reachable actuation, delay prefix and
connector on the identical static world.
