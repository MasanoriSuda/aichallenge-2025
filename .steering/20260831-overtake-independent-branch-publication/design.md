# Design: independent Overtake branch publication

## Root cause

`evaluate_rate_resolved_active_overtake_population()` evaluates both sides in
parallel but publishes the observation-only branch bank only after joining the
two solves.  A completed sibling is therefore invisible while the other side
continues candidate generation/SQP refinement.  Offline success combined with
live failure classifies this as a scheduling/lifecycle defect, not physical
infeasibility or a clearance problem.

## Repair

Add a same-epoch `merge_branch()` operation to the observation-only Overtake
branch bank.

- The first completion for a newer source sequence replaces the entire older
  snapshot and publishes only its certified side.
- A second completion with the exact same immutable source identity fills the
  other side.
- An older completion is rejected as stale.
- A sequence collision with a different source identity is rejected.
- `nullptr` records a completed but uncertified side; it never removes the
  other certified side from the same epoch.

Each branch worker calls `merge_branch()` immediately after its own exact proof.
The outer population still joins both workers before returning its selected
pipeline and updating the selected certified-plan store.  Thus command
authority is unchanged; only observation evidence becomes available earlier.

## Rejected alternatives

- Allow cross-side adoption after no-return: contradicts the existing tactical
  commitment invariant and does not address the observed pre-no-return delay.
- Increase leases/timeouts or retain an older bank: masks scheduling latency
  and risks using a different world epoch.
- Relax wall proof: the opposite branch already passes the existing proof.
