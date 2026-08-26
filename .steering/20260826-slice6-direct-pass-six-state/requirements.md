# Requirements

## Objective

Complete the remaining Slice 6 normal-entry migration by making fresh Direct
Pass use the same prospective steering-rate six-state Gate A as ShiftOut, then
remove the production dependency on the five-state pre-entry artifact.

## Constraints

- Keep one `VelocitySteeringProgress6State` normal authority.
- Do not change weights, margins, timeout, horizon or solver settings.
- Mission state must not mutate before the exact six-state Pass or ShiftOut
  artifact passes physical proof, current-world join and identity admission.
- Do not retain the five-state Gate A as a production fallback.
- Preserve Emergency and Recovery authority boundaries.
- Preserve the user-owned `aichallenge/result-summary.json`.

## Exit criteria

- Both fresh ShiftOut and fresh Direct Pass require a complete prospective
  six-state Gate A proposal.
- The entry commit path contains no five-state pre-entry resolver or canonical
  plan requirement.
- Rejected six-state entries keep the preceding proven normal intent through
  the atomic-intent contract.
- Build, package tests and moving `dev2` validation pass.
