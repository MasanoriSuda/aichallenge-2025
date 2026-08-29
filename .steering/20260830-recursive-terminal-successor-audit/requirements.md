# Requirements

## Baseline

- `d7d31a14 refactor(mpcc): certify active replacement intent`
- Dynamic evidence: `output/20260830-035918/d1/autoware.log`

## Root-cause evidence

The second Overtake encounter admitted a current-world `ShiftOut` bundle at
decision 3906. At decision 3975 the retained normal prefix was still clear for
the next publisher interval, but the exact maximum-braking Stop successor hit
the wall at sample 39. Production therefore returned
`terminal-contingency-unavailable`, published Emergency Stop, and later entered
Recovery.

The same terminal failure also occurred during the first encounter's Pass and
Return. The architecture comparison currently marks a bundle accepted when a
declarative `ContingencyStopIntent` exists; it does not synthesize or prove that
Stop trajectory. The snapshot recorder only persists solver/QP failures, so it
cannot freeze this proof-boundary failure.

## Objective

Make recursive terminal-successor feasibility observable and comparable before
changing production authority:

1. Freeze a complete immutable current-world source snapshot when retained
   execution fails with `TerminalContingencyUnavailable`, even when no QP was
   rejected in that callback.
2. For every architecture arm which otherwise succeeds, synthesize the exact
   one-publisher-interval command followed by the production maximum-braking
   path-tracking Stop suffix.
3. Prove the Stop suffix against the same wall grid, footprint and dynamic
   obstacle observation as the normal trajectory.
4. Report normal-trajectory success and terminal-successor failure as
   `TerminalSuccessorRejected`, never `Accepted`.
5. Keep the comparison and capture path observation-only. It must not publish,
   replace a certified plan, change a Mission, or alter normal authority.

## Constraints

- Do not change solver tolerances, weights, horizons, wall clearance, vehicle
  clearance or acceleration limits.
- Do not add a lease, grace period, timeout, retry, resume rule or fallback.
- Do not change production authority in this Slice.
- The Stop policy, braking envelope and current-world inputs must be sealed in
  the interaction fingerprint; they may not be loaded from the current config
  while replaying an old snapshot.
- Existing v2 snapshots remain loadable for their previous comparison scope,
  but must explicitly report that an exact terminal-successor certificate is
  unavailable rather than silently using present-day defaults.
- Generated rosbag/output/snapshot artifacts remain untracked.

## Exit classification

- A fails, B succeeds: persistent Mission lifecycle defect.
- A/B fail, C succeeds: candidate-generation defect.
- A/B/C fail, D succeeds: single-SQP limitation.
- All arms fail the exact Stop proof: physical infeasibility or entry-timing
  defect.
- Normal solve succeeds but Stop proof fails: model/certificate mismatch or
  missing recursive-feasibility contract.
- Offline arm succeeds while the same live arm fails: scheduling/lifecycle
  defect.
