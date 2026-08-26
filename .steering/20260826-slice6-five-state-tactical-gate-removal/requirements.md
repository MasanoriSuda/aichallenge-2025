# Requirements

## Objective

Remove the last live five-state Overtake tactical Gate and make the prospective
six-state left/right evidence the sole branch-selection and Mission-admission
source.  A five-state solver result must not suppress, replace, cache, or
reinterpret a six-state candidate.

## Root evidence

`output/20260826-153933/d1/autoware.log` shows that each tactical side still
runs both formulations.  At `1787726416.184` the old five-state branch reports
`progress-regressed` and emits `dual execution entry held`, while the six-state
pipeline is independently solving and later admits the same episode.  This is
a live parallel Gate, not dead compatibility code.

## Invariants

- Normal actuation and prospective Overtake admission use only
  `VelocitySteeringProgress6State` proof.
- Tactical Mission geometry may seed a prospective solve but cannot own a
  physical execution certificate.
- Left/right selection is computed once from the six-state branch evaluations.
- No five-state pre-entry plan, retained entry-plan cache, or five-state
  current-world certificate revalidator remains.
- Emergency and Recovery authority boundaries are unchanged.
- No solver, wall, clearance, speed, horizon, timeout, or lease parameter is
  changed.
- Generated result JSON files are not staged.

## Exit criteria

- Source contracts reject every residual five-state Overtake Gate surface.
- One async tactical snapshot performs one prospective solve per valid side,
  not a six-state plus five-state pair.
- A rejected six-state branch keeps the current proven path; an accepted branch
  reaches the causal Gate A worker without a second-formulation veto.
- Build and all package tests pass.
- Moving acceptance contains no five-state branch decision or pre-entry plan
  cache and records only six-state Gate A admission or fail-closed hold.
