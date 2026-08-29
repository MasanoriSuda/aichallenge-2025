# Requirements: Follow authority semantic alignment

## Objective

Prevent a tactical `Behavior::Follow` label from selecting the canonical
Follow QP when no Follow longitudinal constraint owns control.  In that case
the canonical seven-state controller shall remain Cruise while retaining the
current target tube as a dynamic-obstacle constraint.

## Root-cause evidence

- In `output/20260829-143616/d1`, the first stop occurs while
  `action=follow` but both lateral and longitudinal owners are `racing-line`.
  The front vehicle is 12.84 m ahead and no Follow cap is active.
- That semantic mismatch selects the bounded Follow escape population.  New
  Follow QPs fail, the published cursor exhausts, Emergency Stop persists for
  3 s, and Stuck Recovery begins.
- D2 contains 25 `action=follow/longitudinal_owner=racing-line` trace samples;
  20 report Emergency output.
- The unchanged dynamic-obstacle contract already keeps the current target
  tube active for Cruise.  Removing the false Follow owner does not remove
  opponent avoidance.
- The upper-rank `.steering/ano` controller runs one opponent-aware GMPCC and
  does not switch to a separate Follow formulation merely because an opponent
  is detected.

## Constraints

- No solver setting, fallback, lease, grace, timeout, clearance or behavior
  parameter change.
- Follow remains canonical when `follow_cap_active` is true.
- Cruise must retain the current-world target tube and exact dynamic proof.
- SafetyBrake, ShiftOut, Pass, Return, DynamicWait and Recovery precedence are
  unchanged.
- Delete the behavior-label-only Follow authority in the same Slice.

## Definition of done

1. `Behavior::Follow` without a Follow cap resolves to Cruise/racing-line.
2. A Follow cap resolves to Follow/FollowCap even if the tactical behavior
   label is not Follow.
3. Cruise with a current target tube keeps the dynamic-obstacle contract.
4. Unit, source-contract, package and workspace build checks pass.
5. `make dev2` shows no `action=follow/longitudinal_owner=racing-line` trace.
6. Any remaining Emergency is classified by its actual owner and frozen for
   the next architecture audit.
