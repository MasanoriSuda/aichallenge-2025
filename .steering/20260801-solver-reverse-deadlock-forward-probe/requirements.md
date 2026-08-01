# Requirements

## Purpose

Recover from the simulation-only deadlock where a solver-triggered Reverse-only episode is
already in mixed wall contact and every Reverse rollout worsens contact.

## Evidence

In `output/20260801-085442/d2/autoware.log`, P2 repeatedly reported:

- `reverse_only=1`
- `wall=mixed`
- `static=contact_worsened`
- `direction=Unknown`
- `maneuver_distance=0.000 m`

The supervisor executed 175 aggressive retry cycles without selecting or actuating any maneuver.
The existing Forward fallback cannot unlock because it requires a wall-free, contact-free pose,
which a mixed-contact deadlock cannot reach without first moving.

## Constraints

- Simulation race mode only.
- Do not disable footprint, course-progress, V2X, boost, or gear safety checks.
- Do not execute Forward merely because Reverse is unavailable; only add Forward rollouts to the
  existing bounded candidate comparison.
- Require an established solver Reverse-only episode, mixed wall contact, an active Recovery,
  course rejoin guard, and at least one completed aggressive retry.
- Preserve ROS interfaces and all existing configuration values.
- Preserve the user's `wp_id_offset` and generated `result-summary.json` changes.

## Definition of Done

- The P2 deadlock context permits a bounded Forward candidate probe.
- Normal Reverse preference, non-simulation operation, first recovery attempt, rear-wall handling,
  and on-course recovery remain unchanged.
- A Forward maneuver is selected only after the existing static contact-reduction/course checks
  and V2X rollout-separation validation succeed.
- Build and package tests pass.
