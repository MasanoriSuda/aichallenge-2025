# Requirements

## Objective

Remove the post-publication ownership contradiction where an exact-certified
stateless Overtake Bundle becomes canonical authority, but V2X Behavior falls
back to Follow because the rejected frozen Mission geometry was retired.

## Frozen evidence

Run `output/20260831-112650/d1`, target `d2`, generation 1:

- sequence 703 was published and atomically adopted from side `-1` to `+1`;
- the serialized command held canonical seven-state ShiftOut authority;
- the legacy Mission candidate for side `+1` was rejected by its independent
  wall-clamp preflight;
- the next Behavior evaluation changed `Overtake -> Follow` while
  OvertakeLine remained in ShiftOut;
- sequence 711 subsequently solved and owned the same current-world side.

Architecture classification:

- A persistent/frozen Mission: fails candidate preflight;
- B stateless current-world Bundle with the same seven-state SQP: succeeds and
  is published;
- C rough candidate and D offline multi-SQP are not limiting because B already
  succeeds.

Exit class: **Mission/supervisor lifecycle defect**.

## Constraints

- Do not restore rejected Mission path samples or corridor geometry.
- Do not add a resume rule, lease, timeout, grace period or fallback.
- Do not change solver, clearance, wall, velocity, weight or horizon settings.
- Behavior continuity alone may not manufacture command authority.
- Target identity, side, phase, body-clear handoff and existing hard guards
  remain mandatory.
- Only an actually published stateless certified artifact may replace the
  frozen-Mission execution-source fact.

## Definition of Done

- ShiftOut and Pass behavior ownership accept either a validated fixed Mission
  source or an actually published stateless current-world source.
- The pure ownership contract no longer encodes `fixed line` as the only
  possible certified execution producer.
- Candidate-quality rejection alone cannot demote a published stateless
  ShiftOut/Pass to Follow.
- Existing target, wall, waypoint, emergency, overlap and solver hard guards
  still release ownership.
- Build, full package tests and bounded dynamic acceptance pass.
