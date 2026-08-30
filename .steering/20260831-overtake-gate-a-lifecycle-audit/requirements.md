# Requirements

## Objective

Trace the causal `Follow -> Overtake -> Gate A -> ShiftOut` entry path and
identify the first boundary which drops an otherwise valid current-world
overtake candidate.

## Constraints

- Do not change production authority.
- Do not add a Mission resume rule, lease, grace period, timeout or fallback.
- Do not change solver tolerances, wall clearance or vehicle clearance.
- Keep the existing seven-state formulation and exact physical certificates.
- Logging must identify draft construction, publication-boundary admission,
  worker submission and Gate A consumption as separate events.

## Evidence motivating this slice

The `20260831-051051` run contains several tactical `Follow -> Overtake`
transitions but no canonical `ShiftOut`, no Gate A consumption trace and no
entry commit.  Existing logs only expose later cycles as
`causal pre-entry homotopy unavailable`, so they cannot distinguish a draft
construction failure from a publication/worker lifecycle failure.

