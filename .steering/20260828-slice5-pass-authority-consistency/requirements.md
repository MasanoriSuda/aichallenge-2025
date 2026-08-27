# Requirements

## Objective

Complete the next Slice 5 integration repair without tuning parameters: make
the longitudinal authority applied during Pass consistent with the dynamic
footprint decision made in the same control cycle.

## Frozen evidence

- Baseline: `b6da7ebb`
- Structural model/proof repair: `a3926734`
- Dynamic run: `output/20260828-022111/d1/autoware.log`
- The third Overtake episode reached `ShiftOut -> Pass`.
- A confirmed predicted footprint overlap re-applied the front cap and entered
  pre-contact lateral escape, but the target-bound replan prefix subsequently
  restored the retained speed floor in the same cycle.
- The following cycle escalated to Emergency Stop.

## Constraints

- Do not change solver tolerances, wall/vehicle clearance, timing leases,
  Mission resume rules, or production authority.
- Do not add a scenario-specific fallback.
- Preserve user-owned generated result files.
- Add a pure policy test for the authority priority.
- Preserve the actual artifact construction rejection reason instead of
  reporting it as a physical-proof rejection.

## Definition of done

- Target-bound speed retention is permitted only while the current dynamic
  target corridor remains certified and the front cap remains released.
- Confirmed predicted overlap or pre-contact escape cannot be overwritten by
  retained-speed ownership later in the same cycle.
- Artifact construction failures are classified as artifact failures.
- Focused tests, package build, and package tests pass.
- A dynamic Gate run confirms the same-cycle contradiction is absent.
