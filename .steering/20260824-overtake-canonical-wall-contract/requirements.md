# Requirements

## Objective

Make fresh and retained canonical Overtake candidates prove the same physical
wall-clearance contract as the current production five-state Overtake path
before any production-authority promotion is considered.

## Root-cause hypothesis

The canonical fresh shadow currently calls the exact swept-wall validator with
`0.0 m`, while production requires
`MpcProblem::progress_execution_required_wall_clearance_m` (normally
`0.40 m`).  Retained current-world revalidation also checks the raw footprint
without carrying that required clearance.  A canonical candidate can therefore
be labelled physically certified under a weaker contract than the command it
would replace.

## Constraints

- Observation/certification Slice only; do not promote Overtake authority.
- Do not tune wall margins, solver settings, weights, timeouts or leases.
- Do not add a fallback, grace period, feature flag or alternate command owner.
- Reject invalid or missing clearance provenance; do not silently substitute
  zero.
- Preserve the user's `aichallenge/result-summary.json` modification.

## Acceptance

- Fresh canonical Overtake physical proof uses the production problem's exact
  required wall clearance.
- Retained current-world proof receives and applies the same clearance.
- Failure-first tests prove a path accepted at zero clearance is rejected at a
  positive required clearance, and invalid clearance is rejected.
- Static build/tests pass.
- A rebuilt `make dev2` run shows whether current-world canonical coverage
  survives the corrected contract.

