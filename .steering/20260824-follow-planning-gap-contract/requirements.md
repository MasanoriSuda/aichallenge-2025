# Requirements

## Objective

Remove the structural Follow authority oscillation in which a fresh canonical
MPCC plan consumes the physical hard-gap boundary, then fails current-world
revalidation at the terminal horizon after a small target-prediction update.

## Scope

- Separate the nominal/planning Follow gap from the physical hard gap in the
  five-state MPCC contract.
- Keep the physical hard gap as the fail-closed fresh and retained obstacle
  certificate.
- Preserve canonical authority, async identity, ROS interfaces and configured
  distance values.
- Add failure-first source tests for the exact contract wiring.

## Constraints

- Do not tune YAML parameters.
- Do not add a fallback, retry, timeout, lease, tolerance or feature flag.
- Do not relax retained current-world proof below the physical hard gap.
- Do not promote Overtake authority in this Slice.
- Preserve the user's unrelated `aichallenge/result-summary.json` change.

## Acceptance criteria

- The Follow QP upper bound uses the nominal planning gap.
- Fresh and retained physical certificates still use the hard gap.
- A contract with target gap 10 m, planning gap 4 m and hard gap 2.05 m
  exposes both identities without conflating them.
- Existing package tests and build pass.
- In `make dev2`, terminal `stage-gap-violation` and emergency-authority
  alternation materially decrease without accepting a gap below 2.05 m.
